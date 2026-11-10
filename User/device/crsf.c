/* ExpressLRS CRSF receiver on the connector marked UART2 (STM32 USART1). */

#include "crsf.h"

#include <string.h>

#include "bsp/uart.h"

#define CRSF_FRAMETYPE_LINK_STATISTICS (0x14u)
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED (0x16u)
#define CRSF_RC_PAYLOAD_SIZE (22u)
#define CRSF_CHANNEL_MIN (172u)
#define CRSF_CHANNEL_MID (992u)
#define CRSF_CHANNEL_MAX (1811u)
#define CRSF_CHANNEL_DEADBAND (16u)
#define CRSF_SWITCH_LOW_MAX (650u)
#define CRSF_SWITCH_HIGH_MIN (1330u)

static CRSF_t *active_crsf;
static osThreadId_t receiver_thread;

/* 接收快照与拼帧缓冲由遥控任务独占，DMA 仅写原始接收缓冲 */
static uint8_t receive_snapshot[CRSF_RX_BUFFER_SIZE];
static uint8_t frame_buffer[CRSF_RX_BUFFER_SIZE * 2U];
static uint16_t frame_buffer_length;

static uint8_t CRSF_Crc8(const uint8_t *data, uint16_t length) {
  uint8_t crc = 0u;
  for (uint16_t i = 0u; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0u; bit < 8u; bit++)
      crc = (crc & 0x80u) != 0u ? (uint8_t)((crc << 1u) ^ 0xD5u)
                                : (uint8_t)(crc << 1u);
  }
  return crc;
}

static bool CRSF_AddressValid(uint8_t address) {
  return address == 0xC8u || address == 0xEAu || address == 0xECu ||
         address == 0xEEu;
}

static void CRSF_UnpackChannels(const uint8_t *payload, uint16_t *channel) {
  uint32_t bit_buffer = 0u;
  uint8_t bits_available = 0u;
  uint8_t payload_index = 0u;

  for (uint8_t ch = 0u; ch < CRSF_CHANNEL_COUNT; ch++) {
    while (bits_available < 11u) {
      bit_buffer |= (uint32_t)payload[payload_index++] << bits_available;
      bits_available += 8u;
    }
    channel[ch] = (uint16_t)(bit_buffer & 0x07FFu);
    bit_buffer >>= 11u;
    bits_available -= 11u;
  }
}

static float CRSF_NormalizeChannel(uint16_t value) {
  if (value >= CRSF_CHANNEL_MID - CRSF_CHANNEL_DEADBAND &&
      value <= CRSF_CHANNEL_MID + CRSF_CHANNEL_DEADBAND)
    return 0.0f;

  float normalized;
  if (value >= CRSF_CHANNEL_MID)
    normalized = (float)(value - CRSF_CHANNEL_MID) /
                 (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID);
  else
    normalized = -(float)(CRSF_CHANNEL_MID - value) /
                 (float)(CRSF_CHANNEL_MID - CRSF_CHANNEL_MIN);
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f;
  return normalized;
}

/* Low is the safe position. This maps low to CMD_SW_UP because the existing
 * right-switch logic uses UP for SHOOT_MODE_SAFE. */
static CMD_SwitchPos_t CRSF_ChannelToSwitch(uint16_t value) {
  if (value <= CRSF_SWITCH_LOW_MAX) return CMD_SW_UP;
  if (value >= CRSF_SWITCH_HIGH_MIN) return CMD_SW_DOWN;
  return CMD_SW_MID;
}

static CMD_SwitchPos_t CRSF_ChannelToArmSwitch(uint16_t value) {
  return value >= CRSF_SWITCH_HIGH_MIN ? CMD_SW_UP : CMD_SW_ERR;
}

static void CRSF_CopyToCommand(const CRSF_t *crsf, CMD_RC_t *rc) {
  memset(rc, 0, sizeof(*rc));
  /* Default radio order is AETR: roll, pitch, throttle, yaw. */
  rc->ch_r_x = CRSF_NormalizeChannel(crsf->channel[0]);
  rc->ch_r_y = CRSF_NormalizeChannel(crsf->channel[1]);
  rc->ch_l_y = CRSF_NormalizeChannel(crsf->channel[2]);
  rc->ch_l_x = CRSF_NormalizeChannel(crsf->channel[3]);
  /* CH5 is a positive arm gate. CH8 (Pocket SC) selects chassis rotation direction. Keeping sw_r at UP
   * while armed makes the current command layer enable the gimbal but retain
   * SHOOT_MODE_SAFE. */
  rc->sw_r = CRSF_ChannelToArmSwitch(crsf->channel[4]);
  rc->sw_l = CRSF_ChannelToSwitch(crsf->channel[7]);
  /* ch_res 保持为零，底盘旋转使用固定速度，不依赖 CH10。 */
}

static bool CRSF_ParseFrames(CRSF_t *crsf, uint16_t received) {
  bool rc_frame_found = false;
  uint16_t offset = 0u;

  /* 保留上一接收块末尾的半帧，再追加本次 DMA 快照 */
  if (frame_buffer_length + received > sizeof(frame_buffer)) {
    frame_buffer_length = 0U;
  }
  memcpy(frame_buffer + frame_buffer_length, receive_snapshot, received);
  frame_buffer_length += received;
  received = frame_buffer_length;

  while (offset + 4u <= received) {
    if (!CRSF_AddressValid(frame_buffer[offset])) {
      offset++;
      continue;
    }

    const uint8_t frame_length = frame_buffer[offset + 1u];
    const uint16_t total_length = (uint16_t)frame_length + 2u;
    if (frame_length < 2u || total_length > CRSF_RX_BUFFER_SIZE) {
      offset++;
      continue;
    }
    if (offset + total_length > received) break;

    const uint8_t *type = &frame_buffer[offset + 2u];
    const uint8_t expected_crc = frame_buffer[offset + total_length - 1u];
    if (CRSF_Crc8(type, (uint16_t)frame_length - 1u) != expected_crc) {
      crsf->crc_error_count++;
      offset++;
      continue;
    }

    crsf->valid_frame_count++;
    if (type[0] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
        frame_length == CRSF_RC_PAYLOAD_SIZE + 2u) {
      CRSF_UnpackChannels(&type[1], crsf->channel);
      rc_frame_found = true;
    } else if (type[0] == CRSF_FRAMETYPE_LINK_STATISTICS &&
               frame_length >= 5u) {
      crsf->link_quality = type[3];
      crsf->link_statistics_seen = 1u;
    }
    offset += total_length;
  }
  frame_buffer_length = received - offset;
  memmove(frame_buffer, frame_buffer + offset, frame_buffer_length);
  return rc_frame_found;
}

static void CRSF_RxEventCallback(uint16_t size) {
  if (active_crsf == NULL || receiver_thread == NULL) return;
  active_crsf->rx_length = size <= CRSF_RX_BUFFER_SIZE ? size : 0u;
  osThreadFlagsSet(receiver_thread, SIGNAL_CRSF_RAW_READY);
}

static void CRSF_ErrorCallback(void) {
  if (active_crsf == NULL || receiver_thread == NULL) return;
  active_crsf->rx_length = 0u;
  osThreadFlagsSet(receiver_thread, SIGNAL_CRSF_RAW_READY);
}

int8_t CRSF_Init(CRSF_t *crsf) {
  if (crsf == NULL) return DEVICE_ERR_NULL;
  if (active_crsf != NULL) return DEVICE_ERR_INITED;
  receiver_thread = osThreadGetId();
  if (receiver_thread == NULL) return DEVICE_ERR;

  memset(crsf, 0, sizeof(*crsf));
  frame_buffer_length = 0U;
  active_crsf = crsf;
  BSP_UART_RegisterRxEventCallback(BSP_UART_ELRS, CRSF_RxEventCallback);
  BSP_UART_RegisterCallback(BSP_UART_ELRS, BSP_UART_ERROR_CB,
                            CRSF_ErrorCallback);
  return CRSF_StartReceive(crsf);
}

int8_t CRSF_StartReceive(CRSF_t *crsf) {
  if (crsf == NULL) return DEVICE_ERR_NULL;
  UART_HandleTypeDef *uart = BSP_UART_GetHandle(BSP_UART_ELRS);
  if (uart == NULL) return DEVICE_ERR_NO_DEV;

  if (uart->RxState != HAL_UART_STATE_READY) HAL_UART_AbortReceive(uart);
  crsf->rx_length = 0u;
  if (HAL_UARTEx_ReceiveToIdle_DMA(uart, crsf->rx_buffer,
                                   sizeof(crsf->rx_buffer)) != HAL_OK)
    return DEVICE_ERR;
  __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
  return DEVICE_OK;
}

bool CRSF_WaitRC(CRSF_t *crsf, CMD_RC_t *rc, uint32_t timeout) {
  if (crsf == NULL || rc == NULL) return false;
  const uint32_t start = osKernelGetTickCount();

  while ((uint32_t)(osKernelGetTickCount() - start) < timeout) {
    const uint32_t elapsed = (uint32_t)(osKernelGetTickCount() - start);
    if (elapsed >= timeout) return false;
    const uint32_t remaining = timeout - elapsed;
    if (osThreadFlagsWait(SIGNAL_CRSF_RAW_READY, osFlagsWaitAny, remaining) !=
        SIGNAL_CRSF_RAW_READY)
      return false;

    const uint16_t received = crsf->rx_length;
    if (received == 0U) {
      frame_buffer_length = 0U;
      /* Let Task_RC restart reception and yield after an UART error. */
      return false;
    }
    /* 先复制稳定快照并重启 DMA，再执行协议解析以缩短接收空窗 */
    memcpy(receive_snapshot, crsf->rx_buffer, received);
    if (CRSF_StartReceive(crsf) != DEVICE_OK) return false;
    const bool valid_rc = CRSF_ParseFrames(crsf, received);
    if (valid_rc &&
        (crsf->link_statistics_seen == 0u || crsf->link_quality != 0u)) {
      CRSF_CopyToCommand(crsf, rc);
      return true;
    }
    if (valid_rc) return false;
  }
  return false;
}

void CRSF_HandleOffline(CMD_RC_t *rc) {
  if (rc == NULL) return;
  memset(rc, 0, sizeof(*rc));
  rc->sw_l = CMD_SW_ERR;
  rc->sw_r = CMD_SW_ERR;
}
