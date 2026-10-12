/*
  云台控制任务

  控制云台行为。

  从CAN总线接收底盘电机反馈，从IMU接收欧拉角和角速度，
  根据接收到的控制命令，控制电机输出。
*/

/* Includes ----------------------------------------------------------------- */
#include <math.h>

#include "module\gimbal.h"
#include "task\user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
static CAN_t can;

/* Exposed read-only to the low-priority USB telemetry task. */
Gimbal_t gimbal;
CAN_GimbalOutput_t gimbal_out;

#ifdef DEBUG
CMD_GimbalCmd_t gimbal_cmd;
Referee_GimbalUI_t gimbal_ui;
#else
static CMD_GimbalCmd_t gimbal_cmd;
static Referee_GimbalUI_t gimbal_ui;
#endif

/* Keil Watch中设为1才开启云台，上电默认无电流输出。 */
volatile uint8_t gimbal_test_enable = 0U;
volatile uint8_t gimbal_serial_control_active = 0U;
volatile uint32_t gimbal_serial_last_command_tick = 0U;
volatile float gimbal_yaw_test_origin_rad = 0.0f;
volatile float gimbal_pit_test_origin_rad = 0.0f;
volatile uint8_t gimbal_test_stop_reason = GIMBAL_TEST_STOP_NONE;
volatile uint8_t gimbal_yaw_chirp_active = 0U;
volatile uint8_t gimbal_yaw_chirp_down = 0U;
volatile uint32_t gimbal_yaw_chirp_start_tick = 0U;
extern volatile float gimbal_yaw_target_rad;
extern volatile uint8_t gimbal_yaw_target_valid;
extern volatile float gimbal_pit_target_rad;
extern volatile uint8_t gimbal_pit_target_valid;

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * \brief 控制云台
 *
 * \param argument 未使用
 */
void Task_CtrlGimbal(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / TASK_FREQ_CTRL_GIMBAL;
  /* 初始化云台 */
  Gimbal_Init(&gimbal, &(task_runtime.cfg.robot_param->gimbal),
              task_runtime.cfg.gimbal_limit, (float)TASK_FREQ_CTRL_GIMBAL);

  /* 不启动Task_Command时，上电先保持放松模式。 */
  gimbal_cmd.mode = GIMBAL_MODE_RELAX;
  AHRS_ResetEulr(&gimbal_cmd.delta_eulr);

  /* 等待电机反馈和IMU数据就绪后再开启控制。 */
  osMessageQueueGet(task_runtime.msgq.can.feedback.gimbal, &can, NULL,
                    osWaitForever);
  osMessageQueueGet(task_runtime.msgq.gimbal.eulr_imu,
                    &(gimbal.feedback.eulr.imu), NULL, osWaitForever);
  osMessageQueueGet(task_runtime.msgq.gimbal.accl, &(gimbal.feedback.accl),
                    NULL, osWaitForever);
  osMessageQueueGet(task_runtime.msgq.gimbal.gyro, &(gimbal.feedback.gyro),
                    NULL, osWaitForever);
  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.ctrl_gimbal =
        osThreadGetStackSpace(osThreadGetId());
#endif

    osMessageQueueGet(task_runtime.msgq.can.feedback.gimbal, &can, NULL, 0);
    /* 读取控制指令、姿态、IMU数据 */
    osMessageQueueGet(task_runtime.msgq.gimbal.eulr_imu,
                      &(gimbal.feedback.eulr.imu), NULL, 0);
    osMessageQueueGet(task_runtime.msgq.gimbal.accl, &(gimbal.feedback.accl),
                      NULL, 0);
    osMessageQueueGet(task_runtime.msgq.gimbal.gyro, &(gimbal.feedback.gyro),
                      NULL, 0);
    /* 任务只提交模式请求。进入 ABS 的瞬间由 Gimbal_SetMode()
     * 锁存当前 yaw/pitch，避免任务层和模块层重复改写目标角。 */
    /* The internal chirp is generated at the 500 Hz control rate. */
    if (gimbal_yaw_chirp_active != 0U) {
      const float elapsed =
          (float)(tick - gimbal_yaw_chirp_start_tick) /
          (float)osKernelGetTickFreq();
      if (elapsed >= 20.0f) {
        gimbal_yaw_chirp_active = 0U;
        gimbal_yaw_target_rad = gimbal_yaw_test_origin_rad;
      } else {
        const float f0 = gimbal_yaw_chirp_down != 0U ? 2.0f : 0.2f;
        const float f1 = gimbal_yaw_chirp_down != 0U ? 0.2f : 2.0f;
        const float k = (f1 - f0) / 20.0f;
        const float phase = M_2PI * (f0 * elapsed + 0.5f * k * elapsed * elapsed);
        float target = gimbal_yaw_test_origin_rad;
        CircleAdd(&target, 0.08726646f * sinf(phase), M_2PI); /* 5 deg */
        gimbal_yaw_target_rad = target;
      }
      gimbal_yaw_target_valid = 1U;
    }

    /* Serial targets latch until replaced, explicitly stopped, or a motion
     * protection trips. Silence/USB disconnect does not release the gimbal. */
    uint8_t enable = (gimbal_test_enable != 0U) ? 1U : 0U;
    if (enable != 0U) {
      gimbal_cmd.mode = GIMBAL_MODE_ABSOLUTE;
    } else {
      gimbal_cmd.mode = GIMBAL_MODE_RELAX;
    }

    osKernelLock(); /* 锁住RTOS内核防止控制过程中断，造成错误 */
    Gimbal_UpdateFeedback(&gimbal, &can);

    if (enable != 0U && gimbal_serial_control_active != 0U) {
      uint8_t stop_reason = GIMBAL_TEST_STOP_NONE;
      if (!isfinite(gimbal.feedback.eulr.imu.yaw) ||
          !isfinite(gimbal.feedback.eulr.imu.pit) ||
          !isfinite(gimbal.feedback.gyro.y) ||
          !isfinite(gimbal.feedback.gyro.z))
        stop_reason = GIMBAL_TEST_STOP_INVALID_FEEDBACK;
      else if (fabsf(gimbal.feedback.eulr.imu.pit) > GIMBAL_PITCH_SAFE_LIMIT_RAD)
        stop_reason = GIMBAL_TEST_STOP_PITCH_PROTECT;

      if (stop_reason != GIMBAL_TEST_STOP_NONE) {
        gimbal_test_enable = 0U;
        gimbal_serial_control_active = 0U;
        gimbal_yaw_chirp_active = 0U;
        gimbal_yaw_target_valid = 0U;
        gimbal_pit_target_valid = 0U;
        gimbal_test_stop_reason = stop_reason;
        enable = 0U;
        gimbal_cmd.mode = GIMBAL_MODE_RELAX;
      }
    }
    if (gimbal.mode == GIMBAL_MODE_ABSOLUTE) {
      if (gimbal_yaw_target_valid != 0U)
        gimbal.setpoint.eulr.yaw = gimbal_yaw_target_rad;
      if (enable != 0U && gimbal_pit_target_valid != 0U)
        gimbal.setpoint.eulr.pit = gimbal_pit_target_rad;
    }
    Gimbal_Control(&gimbal, &gimbal_cmd, tick);
    if (enable != 0U && gimbal_serial_control_active != 0U) {
      if (gimbal_ladrc_pit_enable == 0U)
        gimbal.out[GIMBAL_ACTR_PIT_IDX] = 0.0f;
    }
    Gimbal_DumpOutput(&gimbal, &gimbal_out);

    osKernelUnlock();
    osMessageQueueReset(task_runtime.msgq.can.output.gimbal);
    osMessageQueuePut(task_runtime.msgq.can.output.gimbal, &gimbal_out, 0, 0);

    Gimbal_DumpUI(&gimbal, &gimbal_ui);
    osMessageQueueReset(task_runtime.msgq.ui.gimbal);
    osMessageQueuePut(task_runtime.msgq.ui.gimbal, &gimbal_ui, 0, 0);

    tick += delay_tick; /* 计算下一个唤醒时刻 */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
}
