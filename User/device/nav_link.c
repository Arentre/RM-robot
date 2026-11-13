#include "device/nav_link.h"

#include <string.h>

#include "bsp/uart.h"
#include "task/user_task.h"

#define NAV_SYNC_0 (0xAAU)
#define NAV_SYNC_1 (0x55U)
#define NAV_TYPE_CMD (0x01U)
#define NAV_TYPE_TELEMETRY (0x02U)
#define NAV_PAYLOAD_SIZE (12U)
#define NAV_FRAME_SIZE (19U)
#define NAV_RX_BUFFER_SIZE (64U)
#define NAV_RX_STREAM_SIZE (128U)
#define NAV_TX_PERIOD_MS (20U)

static uint8_t rx_dma[NAV_RX_BUFFER_SIZE];
static uint8_t rx_stream[NAV_RX_STREAM_SIZE];
static uint16_t rx_stream_len;
static uint8_t tx_frame[NAV_FRAME_SIZE];
static osThreadId_t nav_thread;
static volatile uint32_t rx_size;
static volatile uint8_t rx_ready;
static volatile Nav_Velocity_t nav_command;
static volatile uint32_t nav_command_tick;
static volatile uint8_t nav_command_valid;
static volatile Nav_Velocity_t measured_velocity;
static uint8_t tx_sequence;

static uint16_t Nav_Crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static int32_t Nav_ReadI32LE(const uint8_t *data) {
  const uint32_t raw = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                       ((uint32_t)data[2] << 16U) |
                       ((uint32_t)data[3] << 24U);
  int32_t value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static void Nav_WriteI32LE(uint8_t *data, int32_t value) {
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  data[0] = (uint8_t)raw;
  data[1] = (uint8_t)(raw >> 8U);
  data[2] = (uint8_t)(raw >> 16U);
  data[3] = (uint8_t)(raw >> 24U);
}

static void Nav_PackFrame(uint8_t type, uint8_t sequence,
                          const Nav_Velocity_t *velocity) {
  tx_frame[0] = NAV_SYNC_0;
  tx_frame[1] = NAV_SYNC_1;
  tx_frame[2] = type;
  tx_frame[3] = sequence;
  tx_frame[4] = NAV_PAYLOAD_SIZE;
  Nav_WriteI32LE(&tx_frame[5], velocity->vx_mm_s);
  Nav_WriteI32LE(&tx_frame[9], velocity->vy_mm_s);
  Nav_WriteI32LE(&tx_frame[13], velocity->wz_mrad_s);
  const uint16_t crc = Nav_Crc16(&tx_frame[2], 15U);
  tx_frame[17] = (uint8_t)crc;
  tx_frame[18] = (uint8_t)(crc >> 8U);
}

static bool Nav_DecodeCommand(const uint8_t *frame, Nav_Velocity_t *velocity) {
  if (frame[0] != NAV_SYNC_0 || frame[1] != NAV_SYNC_1 ||
      frame[2] != NAV_TYPE_CMD || frame[4] != NAV_PAYLOAD_SIZE) {
    return false;
  }

  const uint16_t expected =
      (uint16_t)frame[17] | ((uint16_t)frame[18] << 8U);
  if (Nav_Crc16(&frame[2], 15U) != expected) return false;

  velocity->vx_mm_s = Nav_ReadI32LE(&frame[5]);
  velocity->vy_mm_s = Nav_ReadI32LE(&frame[9]);
  velocity->wz_mrad_s = Nav_ReadI32LE(&frame[13]);
  return true;
}

static void Nav_ParseStream(void) {
  uint16_t offset = 0U;
  while ((uint16_t)(rx_stream_len - offset) >= 5U) {
    if (rx_stream[offset] != NAV_SYNC_0 ||
        rx_stream[offset + 1U] != NAV_SYNC_1 ||
        rx_stream[offset + 4U] != NAV_PAYLOAD_SIZE) {
      ++offset;
      continue;
    }
    if ((uint16_t)(rx_stream_len - offset) < NAV_FRAME_SIZE) break;

    Nav_Velocity_t command;
    if (Nav_DecodeCommand(&rx_stream[offset], &command)) {
      const int32_t lock_state = osKernelLock();
      nav_command.vx_mm_s = command.vx_mm_s;
      nav_command.vy_mm_s = command.vy_mm_s;
      nav_command.wz_mrad_s = command.wz_mrad_s;
      nav_command_tick = osKernelGetTickCount();
      nav_command_valid = 1U;
      osKernelRestoreLock(lock_state);
      offset = (uint16_t)(offset + NAV_FRAME_SIZE);
    } else {
      ++offset;
    }
  }

  if (offset != 0U) {
    rx_stream_len = (uint16_t)(rx_stream_len - offset);
    memmove(rx_stream, &rx_stream[offset], rx_stream_len);
  }
  if (rx_stream_len >= sizeof(rx_stream)) rx_stream_len = 0U;
}

static void Nav_RxEvent(uint16_t size) {
  rx_size = size <= sizeof(rx_dma) ? size : 0U;
  rx_ready = 1U;
  if (nav_thread != NULL) (void)osThreadFlagsSet(nav_thread, 1U);
}

static void Nav_Error(void) {
  rx_size = 0U;
  rx_ready = 1U;
  if (nav_thread != NULL) (void)osThreadFlagsSet(nav_thread, 1U);
}

static bool Nav_StartReceive(void) {
  UART_HandleTypeDef *uart = BSP_UART_GetHandle(BSP_UART_NAV);
  if (uart == NULL || uart->hdmarx == NULL) return false;
  if (HAL_UARTEx_ReceiveToIdle_DMA(uart, rx_dma, sizeof(rx_dma)) != HAL_OK)
    return false;
  __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
  return true;
}

void NavLink_Init(void) {
  nav_thread = osThreadGetId();
  rx_stream_len = 0U;
  rx_ready = 0U;
  nav_command_valid = 0U;
  (void)BSP_UART_RegisterRxEventCallback(BSP_UART_NAV, Nav_RxEvent);
  (void)BSP_UART_RegisterCallback(BSP_UART_NAV, BSP_UART_ERROR_CB, Nav_Error);
}

bool NavLink_GetCommand(Nav_Velocity_t *velocity, uint32_t *last_rx_tick) {
  if (velocity == NULL || last_rx_tick == NULL) return false;

  const int32_t lock_state = osKernelLock();
  const bool valid = nav_command_valid != 0U;
  if (valid) {
    velocity->vx_mm_s = nav_command.vx_mm_s;
    velocity->vy_mm_s = nav_command.vy_mm_s;
    velocity->wz_mrad_s = nav_command.wz_mrad_s;
    *last_rx_tick = nav_command_tick;
  }
  osKernelRestoreLock(lock_state);
  return valid;
}

void NavLink_SetMeasured(const Nav_Velocity_t *velocity) {
  if (velocity == NULL) return;

  const int32_t lock_state = osKernelLock();
  measured_velocity.vx_mm_s = velocity->vx_mm_s;
  measured_velocity.vy_mm_s = velocity->vy_mm_s;
  measured_velocity.wz_mrad_s = velocity->wz_mrad_s;
  osKernelRestoreLock(lock_state);
}

void Task_NavLink(void *argument) {
  (void)argument;
  NavLink_Init();

  uint32_t next_tx = osKernelGetTickCount();
  const uint32_t tx_period =
      (NAV_TX_PERIOD_MS * osKernelGetTickFreq()) / 1000U;
  bool rx_active = Nav_StartReceive();

  while (1) {
    const uint32_t now = osKernelGetTickCount();
    if (rx_ready != 0U) {
      const uint16_t size = (uint16_t)rx_size;
      rx_ready = 0U;
      if (size != 0U && rx_stream_len + size <= sizeof(rx_stream)) {
        memcpy(&rx_stream[rx_stream_len], rx_dma, size);
        rx_stream_len = (uint16_t)(rx_stream_len + size);
        Nav_ParseStream();
      } else if (size != 0U) {
        rx_stream_len = 0U;
      }

      UART_HandleTypeDef *uart = BSP_UART_GetHandle(BSP_UART_NAV);
      if (uart != NULL && uart->RxState != HAL_UART_STATE_READY)
        (void)HAL_UART_AbortReceive(uart);
      rx_active = false;
    }
    if (!rx_active) rx_active = Nav_StartReceive();

    if ((int32_t)(now - next_tx) >= 0) {
      Nav_Velocity_t telemetry;
      const int32_t lock_state = osKernelLock();
      telemetry.vx_mm_s = measured_velocity.vx_mm_s;
      telemetry.vy_mm_s = measured_velocity.vy_mm_s;
      telemetry.wz_mrad_s = measured_velocity.wz_mrad_s;
      osKernelRestoreLock(lock_state);

      UART_HandleTypeDef *uart = BSP_UART_GetHandle(BSP_UART_NAV);
      if (uart != NULL && uart->gState == HAL_UART_STATE_READY) {
        Nav_PackFrame(NAV_TYPE_TELEMETRY, tx_sequence, &telemetry);
        if (HAL_UART_Transmit_DMA(uart, tx_frame, sizeof(tx_frame)) == HAL_OK)
          ++tx_sequence;
      }
      next_tx += tx_period;
    }
    (void)osThreadFlagsWait(1U, osFlagsWaitAny, 1U);
  }
}
