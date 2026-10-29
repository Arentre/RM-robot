/* Compile with stubs first, then User, in the include search path.
 * Exercises the actual CAN driver; no STM32 hardware or RTOS is simulated. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "component/gimbal_safety.h"
#include "../../User/device/can.c"

static CAN_HandleTypeDef handles[2];
static uint8_t transmitted[8];
static CAN_TxHeaderTypeDef transmitted_header;
uint32_t osKernelGetTickCount(void) { return 999U; }
osThreadId_t osThreadGetId(void) { return (void *)1; }
osMessageQueueId_t osMessageQueueNew(uint32_t n, uint32_t s, const void *a) {
  (void)n; (void)s; (void)a; return (void *)1;
}
osStatus_t osMessageQueuePut(osMessageQueueId_t q, const void *i,
                             uint8_t p, uint32_t t) {
  (void)q; (void)i; (void)p; (void)t; return osOK;
}
CAN_HandleTypeDef *BSP_CAN_GetHandle(BSP_CAN_t bus) { return &handles[bus]; }
int8_t BSP_CAN_RegisterCallback(BSP_CAN_t b, BSP_CAN_Callback_t t,
                                void (*cb)(void)) {
  (void)b; (void)t; (void)cb; return 0;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t f,
    CAN_RxHeaderTypeDef *r, uint8_t *d) {
  (void)h; (void)f; (void)r; (void)d; return HAL_ERROR;
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f) {
  (void)h; (void)f; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t f) {
  (void)h; (void)f; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,
    CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox) {
  (void)h; *mailbox = 1U; transmitted_header = *header;
  memcpy(transmitted, data, sizeof(transmitted)); return HAL_OK;
}

int main(void) {
  const CAN_Params_t params = {.gimbal = BSP_CAN_2};
  CAN_t feedback = {0};
  feedback.param = &params;
  CAN_RawRx_t frame = {0};
  frame.bus = BSP_CAN_2;
  frame.rx_header.StdId = CAN_GM6020_PIT_ID;
  frame.rx_header.DLC = 8U;
  frame.receive_tick = 100U;
  frame.rx_data[0] = 0x10U; /* Valid 180-degree encoder sample. */
  frame.rx_data[6] = 42U;
  assert(CAN_StoreMsg(&feedback, &frame) == DEVICE_OK);
  assert(feedback.gimbal_feedback_count[1] == 1U);
  assert(feedback.gimbal_feedback_tick[1] == 100U);
  const CAN_MotorFeedback_t good = feedback.motor.gimbal.as_array[1];

  /* Inject corrupted payloads: they cannot make old feedback look fresh. */
  frame.receive_tick = 200U;
  memset(frame.rx_data, 0xff, sizeof(frame.rx_data));
  assert(CAN_StoreMsg(&feedback, &frame) == DEVICE_ERR);
  assert(feedback.gimbal_invalid_feedback_count[1] == 1U);
  assert(feedback.gimbal_feedback_count[1] == 1U);
  assert(feedback.gimbal_feedback_tick[1] == 100U);
  assert(memcmp(&good, &feedback.motor.gimbal.as_array[1], sizeof(good)) == 0);
  frame.rx_data[0] = 0x20U; frame.rx_data[1] = 0U; /* 8192 is out of range. */
  assert(CAN_StoreMsg(&feedback, &frame) == DEVICE_ERR);
  frame.rx_data[0] = 0x1fU; frame.rx_data[1] = 0xffU; /* 8191 is valid. */
  frame.bus = BSP_CAN_1;
  assert(CAN_StoreMsg(&feedback, &frame) == DEVICE_ERR);
  assert(feedback.gimbal_feedback_tick[1] == 100U);
  frame.bus = BSP_CAN_2;
  assert(CAN_StoreMsg(&feedback, &frame) == DEVICE_OK);
  assert(feedback.gimbal_feedback_tick[1] == 200U);
  assert(feedback.gimbal_feedback_count[0] == 0U);

  CAN_Output_t output = {0};
  output.gimbal.named.yaw = -1.0f; output.gimbal.named.pit = 1.0f;
  assert(CAN_Motor_Control(CAN_MOTOR_GROUT_GIMBAL1, &output, &feedback) == DEVICE_OK);
  assert(transmitted_header.StdId == 0x2feU);
  assert(transmitted[0] == 0xc0U && transmitted[1] == 0U);
  assert(transmitted[2] == 0x40U && transmitted[3] == 0U);

  const float corrupt[] = {NAN, INFINITY, -INFINITY, 1.001f, -1.001f, 1.0e30f};
  for (unsigned axis = 0; axis < 2; axis++) {
    for (unsigned i = 0; i < sizeof(corrupt) / sizeof(corrupt[0]); i++) {
      output.gimbal.named.yaw = 0.2f; output.gimbal.named.pit = -0.2f;
      output.gimbal.as_array[axis] = corrupt[i];
      assert(CAN_Motor_Control(CAN_MOTOR_GROUT_GIMBAL1, &output, &feedback) == DEVICE_OK);
      for (unsigned b = 0; b < 8; b++) assert(transmitted[b] == 0U);
    }
  }
  assert(feedback.gimbal_invalid_output_count == 12U);

  /* A missing producer and tick wrap must both produce a correct sample age. */
  assert(GimbalSafety_AgeMs(121U, 100U, 1000U) > GIMBAL_IMU_TIMEOUT_MS);
  assert(GimbalSafety_AgeMs(4U, UINT32_MAX - 15U, 1000U) == 20U);
  assert(GimbalSafety_AgeMs(5U, UINT32_MAX - 15U, 1000U) == 21U);
  assert(GimbalSafety_AgeMs(6000000U, 0U, 1000U) == 6000000U);
  assert(GimbalSafety_AgeMs(102U, 100U, 100U) == 20U);
  /* Permit real fast motion; reject a finite 30-degree sensor jump in 2 ms. */
  assert(!GimbalSafety_PitchJump(0.0f, 0.03490659f, 2U, 1000U));
  assert(GimbalSafety_PitchJump(0.0f, 0.52359878f, 2U, 1000U));
  assert(GimbalSafety_PitchJump(0.0f, NAN, 2U, 1000U));
  puts("gimbal fault guard: CAN payload/output injections and IMU boundaries passed");
  return 0;
}
