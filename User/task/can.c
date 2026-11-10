/*
  CAN总线数据处理

  处理CAN总线收到的电机电容数据。
*/

/* Includes ----------------------------------------------------------------- */
#include "device/can.h"

#include "task/user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */

#define CAN_RX_BUDGET_PER_TICK (16U)
#define GIMBAL_OUTPUT_TIMEOUT_MS (50U)

GimbalCANTxHealth_t gimbal_can_tx_health = {0};

#ifdef DEBUG
CAN_t can;
CAN_Output_t can_out;
CAN_RawRx_t can_rx;
#else
static CAN_t can;
static CAN_Output_t can_out;
static CAN_RawRx_t can_rx;
#endif

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_Can(void *argument) {
  (void)argument;
  const uint32_t delay_tick = osKernelGetTickFreq() / TASK_FREQ_CAN;

  /* Device Setup */
  CAN_Init(&can, &task_runtime.cfg.robot_param->can);

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  uint32_t gimbal_output_tick = tick;
  CAN_HandleTypeDef *gimbal_bus = BSP_CAN_GetHandle(can.param->gimbal);
  /* Task Setup */
  while (1) {
#ifdef DEBUG
    task_runtime.stack_water_mark.can = osThreadGetStackSpace(osThreadGetId());
#endif
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* Bound RX work so a continuous feedback stream cannot postpone motor
     * transmission until the GM6020 command watchdog removes current. */
    for (uint32_t i = 0U; i < CAN_RX_BUDGET_PER_TICK; i++) {
      if (osMessageQueueGet(can.msgq_raw, &can_rx, 0, 0) != osOK) break;
      CAN_StoreMsg(&can, &can_rx);
    }
    osMessageQueueReset(task_runtime.msgq.can.feedback.chassis);
    osMessageQueuePut(task_runtime.msgq.can.feedback.chassis, &can, 0, 0);

    osMessageQueueReset(task_runtime.msgq.can.feedback.gimbal);
    osMessageQueuePut(task_runtime.msgq.can.feedback.gimbal, &can, 0, 0);

    osMessageQueueReset(task_runtime.msgq.can.feedback.shoot);
    osMessageQueuePut(task_runtime.msgq.can.feedback.shoot, &can, 0, 0);

    if (osMessageQueueGet(task_runtime.msgq.can.output.chassis,
                          &(can_out.chassis), 0, 0) == osOK) {
      CAN_Motor_Control(CAN_MOTOR_GROUT_CHASSIS, &can_out, &can);
    }

    /* Repeat the latest 500 Hz control sample as a 1 kHz CAN heartbeat. */
    CAN_GimbalOutput_t next_gimbal_output;
    if (osMessageQueueGet(task_runtime.msgq.can.output.gimbal,
                          &next_gimbal_output, 0, 0) == osOK) {
      can_out.gimbal = next_gimbal_output;
      gimbal_output_tick = osKernelGetTickCount();
    }
    gimbal_can_tx_health.output_age_ms =
        (uint32_t)(osKernelGetTickCount() - gimbal_output_tick) * 1000U /
        osKernelGetTickFreq();
    if (gimbal_can_tx_health.output_age_ms > GIMBAL_OUTPUT_TIMEOUT_MS) {
      can_out.gimbal.named.yaw = 0.0f;
      can_out.gimbal.named.pit = 0.0f;
    }
    if (CAN_Motor_Control(CAN_MOTOR_GROUT_GIMBAL1, &can_out, &can) ==
        DEVICE_OK) {
      /* HAL_OK means queued, not acknowledged by either motor. */
      gimbal_can_tx_health.last_success_tick = osKernelGetTickCount();
      gimbal_can_tx_health.success_count++;
    } else {
      gimbal_can_tx_health.failure_count++;
    }
    gimbal_can_tx_health.tx_status_register = gimbal_bus->Instance->TSR;
    gimbal_can_tx_health.bus_esr = gimbal_bus->Instance->ESR;
    gimbal_can_tx_health.free_mailboxes =
        HAL_CAN_GetTxMailboxesFreeLevel(gimbal_bus);

    if (osMessageQueueGet(task_runtime.msgq.can.output.shoot, &(can_out.shoot),
                          0, 0) == osOK) {
      CAN_Motor_Control(CAN_MOTOR_GROUT_SHOOT1, &can_out, &can);
    }

    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
}
