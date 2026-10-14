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
volatile float gimbal_yaw_test_origin_rad = 0.0f;
volatile float gimbal_pit_test_origin_rad = 0.0f;
volatile uint8_t gimbal_test_stop_reason = GIMBAL_TEST_STOP_NONE;
volatile uint8_t gimbal_yaw_chirp_active = 0U;
volatile uint8_t gimbal_yaw_chirp_down = 0U;
volatile uint32_t gimbal_yaw_chirp_start_tick = 0U;

#define GIMBAL_BINARY_TRAJECTORY_TIMEOUT_TICKS (125U) /* 250 ms at 500 Hz */

/* Private function --------------------------------------------------------- */
static void Gimbal_TargetCopyLocked(GimbalTargetInput_t *target) {
  target->yaw_rad = gimbal_target_input.yaw_rad;
  target->pitch_rad = gimbal_target_input.pitch_rad;
  target->yaw_velocity_rad_s = gimbal_target_input.yaw_velocity_rad_s;
  target->yaw_acceleration_rad_s2 =
      gimbal_target_input.yaw_acceleration_rad_s2;
  target->pitch_velocity_rad_s = gimbal_target_input.pitch_velocity_rad_s;
  target->pitch_acceleration_rad_s2 =
      gimbal_target_input.pitch_acceleration_rad_s2;
  target->update_tick = gimbal_target_input.update_tick;
  target->revision = gimbal_target_input.revision;
  target->yaw_valid = gimbal_target_input.yaw_valid;
  target->pitch_valid = gimbal_target_input.pitch_valid;
  target->trajectory_active = gimbal_target_input.trajectory_active;
}

static uint8_t Gimbal_TargetValidate(GimbalTargetInput_t *target) {
  if (target->yaw_valid != 0U) {
    if (!isfinite(target->yaw_rad) || fabsf(target->yaw_rad) > 2.0f * M_2PI)
      return 0U;
    target->yaw_rad = fmodf(target->yaw_rad, M_2PI);
    if (target->yaw_rad < 0.0f) target->yaw_rad += M_2PI;
  }
  if (target->pitch_valid != 0U &&
      (!isfinite(target->pitch_rad) ||
       fabsf(target->pitch_rad) > GIMBAL_PITCH_SAFE_LIMIT_RAD))
    return 0U;
  if (target->trajectory_active != 0U &&
      (target->yaw_valid == 0U || target->pitch_valid == 0U ||
       !isfinite(target->yaw_velocity_rad_s) ||
       !isfinite(target->yaw_acceleration_rad_s2) ||
       !isfinite(target->pitch_velocity_rad_s) ||
       !isfinite(target->pitch_acceleration_rad_s2) ||
       fabsf(target->yaw_velocity_rad_s) > 20.0f ||
       fabsf(target->pitch_velocity_rad_s) > 12.0f ||
       fabsf(target->yaw_acceleration_rad_s2) > 160.0f ||
       fabsf(target->pitch_acceleration_rad_s2) > 120.0f))
    return 0U;
  return 1U;
}

static void Gimbal_TargetDisableLocked(void) {
  gimbal_target_input.yaw_velocity_rad_s = 0.0f;
  gimbal_target_input.yaw_acceleration_rad_s2 = 0.0f;
  gimbal_target_input.pitch_velocity_rad_s = 0.0f;
  gimbal_target_input.pitch_acceleration_rad_s2 = 0.0f;
  gimbal_target_input.yaw_valid = 0U;
  gimbal_target_input.pitch_valid = 0U;
  gimbal_target_input.trajectory_active = 0U;
  gimbal_target_input.revision++;
}

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
  GimbalTargetInput_t target = {0};
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
    osKernelLock(); /* 锁住RTOS内核防止控制过程中断，造成错误 */
    Gimbal_UpdateFeedback(&gimbal, &can);

    /* Generate and publish chirp targets under the same lock as serial target
     * writes.  The revision is committed only after the angle is complete. */
    if (gimbal_yaw_chirp_active != 0U) {
      const float elapsed =
          (float)(tick - gimbal_yaw_chirp_start_tick) /
          (float)osKernelGetTickFreq();
      float yaw_target = gimbal_yaw_test_origin_rad;
      if (elapsed >= 20.0f) {
        gimbal_yaw_chirp_active = 0U;
      } else {
        const float f0 = gimbal_yaw_chirp_down != 0U ? 2.0f : 0.2f;
        const float f1 = gimbal_yaw_chirp_down != 0U ? 0.2f : 2.0f;
        const float k = (f1 - f0) / 20.0f;
        const float phase =
            M_2PI * (f0 * elapsed + 0.5f * k * elapsed * elapsed);
        CircleAdd(&yaw_target, 0.08726646f * sinf(phase), M_2PI);
      }
      gimbal_target_input.yaw_rad = yaw_target;
      gimbal_target_input.yaw_valid = 1U;
      gimbal_target_input.update_tick = tick;
      gimbal_target_input.revision++;
    }

    uint8_t stop_reason = GIMBAL_TEST_STOP_NONE;
    if (target.revision != gimbal_target_input.revision) {
      GimbalTargetInput_t candidate;
      Gimbal_TargetCopyLocked(&candidate);
      if (Gimbal_TargetValidate(&candidate) != 0U)
        target = candidate;
      else
        stop_reason = GIMBAL_TEST_STOP_INVALID_TARGET;
    }

    /* Serial targets latch until replaced, explicitly stopped, or a motion
     * protection trips. Silence/USB disconnect does not release the gimbal. */
    uint8_t enable = (gimbal_test_enable != 0U) ? 1U : 0U;
    gimbal_cmd.mode = enable != 0U ? GIMBAL_MODE_ABSOLUTE : GIMBAL_MODE_RELAX;

    if (enable != 0U && gimbal_serial_control_active != 0U) {
      if (stop_reason == GIMBAL_TEST_STOP_NONE &&
          target.trajectory_active != 0U &&
          (uint32_t)(tick - target.update_tick) >
              GIMBAL_BINARY_TRAJECTORY_TIMEOUT_TICKS)
        stop_reason = GIMBAL_TEST_STOP_TIMEOUT;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
          (!isfinite(gimbal.feedback.eulr.imu.yaw) ||
          !isfinite(gimbal.feedback.eulr.imu.pit) ||
          !isfinite(gimbal.feedback.gyro.y) ||
           !isfinite(gimbal.feedback.gyro.z)))
        stop_reason = GIMBAL_TEST_STOP_INVALID_FEEDBACK;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               fabsf(gimbal.feedback.eulr.imu.pit) >
                   GIMBAL_PITCH_SAFE_LIMIT_RAD)
        stop_reason = GIMBAL_TEST_STOP_PITCH_PROTECT;

      float elapsed = 0.0f;
      if (target.trajectory_active != 0U) {
        elapsed = (float)(tick - target.update_tick) /
                  (float)osKernelGetTickFreq();
        if (elapsed < 0.0f) elapsed = 0.0f;
        if (elapsed > 0.20f) elapsed = 0.20f;
        const float predicted_pitch =
            target.pitch_rad + target.pitch_velocity_rad_s * elapsed +
            0.5f * target.pitch_acceleration_rad_s2 * elapsed * elapsed;
        if (!isfinite(predicted_pitch) ||
            fabsf(predicted_pitch) > GIMBAL_PITCH_SAFE_LIMIT_RAD)
          stop_reason = GIMBAL_TEST_STOP_INVALID_TARGET;
      }

      if (stop_reason != GIMBAL_TEST_STOP_NONE) {
        gimbal_test_enable = 0U;
        gimbal_serial_control_active = 0U;
        gimbal_yaw_chirp_active = 0U;
        Gimbal_TargetDisableLocked();
        gimbal_test_stop_reason = stop_reason;
        enable = 0U;
        gimbal_cmd.mode = GIMBAL_MODE_RELAX;
      }
    }
    if (gimbal.mode == GIMBAL_MODE_ABSOLUTE) {
      const float elapsed = target.trajectory_active != 0U
                                ? (float)(tick - target.update_tick) /
                                      (float)osKernelGetTickFreq()
                                : 0.0f;
      if (target.yaw_valid != 0U) {
        float yaw_target = target.yaw_rad;
        if (target.trajectory_active != 0U) {
          yaw_target += target.yaw_velocity_rad_s * elapsed +
                        0.5f * target.yaw_acceleration_rad_s2 * elapsed * elapsed;
          CircleAdd(&yaw_target, 0.0f, M_2PI);
        }
        gimbal.setpoint.eulr.yaw = yaw_target;
      }
      if (enable != 0U && target.pitch_valid != 0U) {
        float pitch_target = target.pitch_rad;
        if (target.trajectory_active != 0U) {
          pitch_target += target.pitch_velocity_rad_s * elapsed +
                          0.5f * target.pitch_acceleration_rad_s2 * elapsed * elapsed;
        }
        gimbal.setpoint.eulr.pit = pitch_target;
      }
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
