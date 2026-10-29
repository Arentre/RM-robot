/*
  云台控制任务

  控制云台行为。

  从CAN总线接收底盘电机反馈，从IMU接收欧拉角和角速度，
  根据接收到的控制命令，控制电机输出。
*/

/* Includes ----------------------------------------------------------------- */
#include <math.h>

#include "component\gimbal_safety.h"
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

static CMD_GimbalCmd_t remote_gimbal_cmd;
static uint32_t remote_gimbal_update_tick;
static uint8_t remote_gimbal_fault_latched;

/* Keil Watch中设为1才开启云台，上电默认无电流输出。 */
volatile uint8_t gimbal_test_enable = 0U;
volatile uint8_t gimbal_serial_control_active = 0U;
volatile float gimbal_yaw_test_origin_rad = 0.0f;
volatile float gimbal_pit_test_origin_rad = 0.0f;
volatile uint8_t gimbal_test_stop_reason = GIMBAL_TEST_STOP_NONE;
volatile uint8_t gimbal_yaw_chirp_active = 0U;
volatile uint8_t gimbal_yaw_chirp_down = 0U;
volatile uint32_t gimbal_yaw_chirp_start_tick = 0U;
volatile uint16_t gimbal_actuator_mismatch_count[GIMBAL_ACTR_NUM] = {0U, 0U};
volatile GimbalProtectionStatus_t gimbal_protection_status;
volatile GimbalFaultSnapshot_t gimbal_fault_snapshot;
static GimbalIMUSample_t imu_sample;
static uint8_t imu_seen;
static float previous_pitch;
static uint32_t previous_imu_tick;
static uint32_t invalid_output_count_seen;
static uint16_t actuator_startup_count;
static uint16_t gyro_overrate_count;

#define GIMBAL_BINARY_TRAJECTORY_TIMEOUT_MS (250U)
#define GIMBAL_CAN_TX_TIMEOUT_MS (100U)
#define GIMBAL_CAN_RX_TIMEOUT_MS (150U)
#define GIMBAL_CURRENT_COMMAND_MIN_A (0.60f)
#define GIMBAL_CURRENT_TRACKING_MIN_RATIO (0.10f)
#define GIMBAL_CURRENT_MISMATCH_TIME_MS (300U)
#define GIMBAL_CURRENT_STARTUP_GRACE_MS (200U)
#define GIMBAL_GYRO_OVERRATE_TIME_MS (10U)
#define GIMBAL_REMOTE_TIMEOUT_MS (150U)

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
  if (target->pitch_valid != 0U) {
    if (!isfinite(target->pitch_rad)) return 0U;
    Clip(&target->pitch_rad, -GIMBAL_PITCH_SAFE_LIMIT_RAD,
         GIMBAL_PITCH_SAFE_LIMIT_RAD);
  }
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

static uint8_t Gimbal_GyroRateUnsafe(const AHRS_Gyro_t *gyro) {
  const float rate_squared = gyro->x * gyro->x + gyro->y * gyro->y +
                             gyro->z * gyro->z;
  const uint16_t limit =
      (GIMBAL_GYRO_OVERRATE_TIME_MS * TASK_FREQ_CTRL_GIMBAL + 999U) / 1000U;
  if (rate_squared >
      GIMBAL_GYRO_SAFE_NORM_RAD_S * GIMBAL_GYRO_SAFE_NORM_RAD_S) {
    if (gyro_overrate_count < limit) gyro_overrate_count++;
  } else {
    gyro_overrate_count = 0U;
  }
  return gyro_overrate_count >= limit;
}

static void Gimbal_UpdateLinkStatus(uint32_t now) {
  const uint32_t freq = osKernelGetTickFreq();
  gimbal_protection_status.tx_age_ms =
      GimbalSafety_AgeMs(now, gimbal_can_tx_health.last_success_tick, freq);
  for (uint8_t i = 0U; i < GIMBAL_ACTR_NUM; i++) {
    gimbal_protection_status.feedback_count[i] = can.gimbal_feedback_count[i];
    gimbal_protection_status.feedback_age_ms[i] =
        can.gimbal_feedback_count[i] != 0U
            ? GimbalSafety_AgeMs(now, can.gimbal_feedback_tick[i], freq)
            : UINT32_MAX;
  }
  gimbal_protection_status.rx_queue_drop_count = can.rx_queue_drop_count;
  gimbal_protection_status.imu_age_ms = imu_seen != 0U
      ? GimbalSafety_AgeMs(now, imu_sample.gyro_tick, freq) : UINT32_MAX;
  gimbal_protection_status.accl_age_ms = imu_seen != 0U
      ? GimbalSafety_AgeMs(now, imu_sample.accl_tick, freq) : UINT32_MAX;
}

static uint8_t Gimbal_CANRxUnsafe(void) {
  for (uint8_t i = 0U; i < GIMBAL_ACTR_NUM; i++)
    if (gimbal_protection_status.feedback_age_ms[i] > GIMBAL_CAN_RX_TIMEOUT_MS)
      return 1U;
  return 0U;
}

static uint8_t Gimbal_CANTxUnsafe(uint32_t now) {
  const uint32_t timeout_ticks =
      (GIMBAL_CAN_TX_TIMEOUT_MS * osKernelGetTickFreq() + 999U) / 1000U;
  return (uint32_t)(now - gimbal_can_tx_health.last_success_tick) >
         timeout_ticks;
}

static uint8_t Gimbal_ActuatorCurrentUnsafe(void) {
  const uint16_t grace =
      (GIMBAL_CURRENT_STARTUP_GRACE_MS * TASK_FREQ_CTRL_GIMBAL + 999U) / 1000U;
  if (actuator_startup_count < grace) {
    actuator_startup_count++;
    return 0U;
  }
  const float command_current[GIMBAL_ACTR_NUM] = {
      gimbal.out[GIMBAL_ACTR_YAW_IDX] * CAN_GM6020_MAX_ABS_CUR,
      gimbal.out[GIMBAL_ACTR_PIT_IDX] * CAN_GM6020_MAX_ABS_CUR};
  const float feedback_current[GIMBAL_ACTR_NUM] = {
      gimbal.feedback.motor.yaw_current,
      gimbal.feedback.motor.pit_current};
  const uint16_t mismatch_limit =
      (uint16_t)((GIMBAL_CURRENT_MISMATCH_TIME_MS *
                  TASK_FREQ_CTRL_GIMBAL + 999U) /
                 1000U);

  uint8_t unsafe = 0U;
  for (uint8_t i = 0U; i < GIMBAL_ACTR_NUM; i++) {
    const float command = command_current[i];
    const float feedback = feedback_current[i];
    const uint8_t tracking =
        isfinite(command) && isfinite(feedback) && command * feedback > 0.0f &&
        fabsf(feedback) >=
            GIMBAL_CURRENT_TRACKING_MIN_RATIO * fabsf(command);
    if (fabsf(command) < GIMBAL_CURRENT_COMMAND_MIN_A || tracking != 0U) {
      gimbal_actuator_mismatch_count[i] = 0U;
    } else if (gimbal_actuator_mismatch_count[i] < mismatch_limit) {
      gimbal_actuator_mismatch_count[i]++;
    }
    if (gimbal_actuator_mismatch_count[i] >= mismatch_limit) unsafe = 1U;
  }
  return unsafe;
}

static void Gimbal_ActuatorGuardReset(void) {
  actuator_startup_count = 0U;
  gyro_overrate_count = 0U;
  for (uint8_t i = 0U; i < GIMBAL_ACTR_NUM; i++)
    gimbal_actuator_mismatch_count[i] = 0U;
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

static void Gimbal_TripLocked(uint8_t reason, uint32_t now) {
  gimbal_test_stop_reason = reason;
  gimbal_protection_status.last_reason = reason;
  gimbal_protection_status.trip_count++;
  remote_gimbal_fault_latched = 1U;
  gimbal_protection_status.fault_latched = 1U;
  gimbal_fault_snapshot.reason = reason;
  gimbal_fault_snapshot.tick = now;
  gimbal_fault_snapshot.link = gimbal_protection_status;
  gimbal_fault_snapshot.can_esr = gimbal_can_tx_health.bus_esr;
  gimbal_fault_snapshot.can_tsr = gimbal_can_tx_health.tx_status_register;
  gimbal_fault_snapshot.output_age_ms = gimbal_can_tx_health.output_age_ms;
  gimbal_fault_snapshot.imu_io_error_count = gimbal_imu_io_error_count;
  gimbal_fault_snapshot.invalid_output_count = can.gimbal_invalid_output_count;
  gimbal_fault_snapshot.pitch_control_enabled = gimbal_ladrc_pit_enable;
  gimbal_fault_snapshot.serial_control_active = gimbal_serial_control_active;
  gimbal_fault_snapshot.attitude = gimbal.feedback.eulr.imu;
  gimbal_fault_snapshot.gyro = gimbal.feedback.gyro;
  gimbal_fault_snapshot.previous_pitch = previous_pitch;
  gimbal_fault_snapshot.pitch_target = gimbal.setpoint.eulr.pit;
  for (uint8_t i = 0U; i < GIMBAL_ACTR_NUM; i++) {
    gimbal_fault_snapshot.output[i] = gimbal.out[i];
    gimbal_fault_snapshot.current[i] =
        can.motor.gimbal.as_array[i].torque_current;
    gimbal_fault_snapshot.temperature[i] = can.motor.gimbal.as_array[i].temp;
    gimbal_fault_snapshot.invalid_feedback_count[i] =
        can.gimbal_invalid_feedback_count[i];
    gimbal_fault_snapshot.mismatch_count[i] = gimbal_actuator_mismatch_count[i];
  }
  gimbal_test_enable = 0U;
  gimbal_serial_control_active = 0U;
  gimbal_yaw_chirp_active = 0U;
  Gimbal_TargetDisableLocked();
  gimbal_cmd.mode = GIMBAL_MODE_RELAX;
  AHRS_ResetEulr(&gimbal_cmd.delta_eulr);
  Gimbal_ActuatorGuardReset();
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
  remote_gimbal_cmd.mode = GIMBAL_MODE_RELAX;
  AHRS_ResetEulr(&remote_gimbal_cmd.delta_eulr);

  /* 等待电机反馈和IMU数据就绪后再开启控制。 */
  osMessageQueueGet(task_runtime.msgq.can.feedback.gimbal, &can, NULL,
                    osWaitForever);
  /* Allow cold-start sensor setup before treating missing IMU as a fault. */
  if (osMessageQueueGet(task_runtime.msgq.gimbal.sample, &imu_sample, NULL,
                        osKernelGetTickFreq()) == osOK) {
    imu_seen = 1U;
    previous_pitch = imu_sample.eulr.pit;
    previous_imu_tick = imu_sample.gyro_tick;
    gimbal.feedback.eulr.imu = imu_sample.eulr;
    gimbal.feedback.gyro = imu_sample.gyro;
    gimbal.feedback.accl = imu_sample.accl;
  }
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
    const uint8_t imu_updated = osMessageQueueGet(
        task_runtime.msgq.gimbal.sample, &imu_sample, NULL, 0) == osOK;
    CMD_GimbalCmd_t received_remote_cmd;
    if (osMessageQueueGet(task_runtime.msgq.cmd.gimbal,
                          &received_remote_cmd, NULL, 0) == osOK) {
      remote_gimbal_cmd = received_remote_cmd;
      remote_gimbal_update_tick = osKernelGetTickCount();
      if (remote_gimbal_cmd.mode == GIMBAL_MODE_RELAX &&
          gimbal_serial_control_active == 0U) {
        remote_gimbal_fault_latched = 0U;
        gimbal_test_stop_reason = GIMBAL_TEST_STOP_NONE;
      }
    }
    osKernelLock(); /* 锁住RTOS内核防止控制过程中断，造成错误 */
    const uint32_t control_now = osKernelGetTickCount();
    uint8_t pitch_jump = 0U;
    if (imu_updated != 0U) {
      if (imu_seen != 0U &&
          GimbalSafety_AgeMs(imu_sample.gyro_tick, previous_imu_tick,
                             osKernelGetTickFreq()) <= GIMBAL_IMU_TIMEOUT_MS)
        pitch_jump = GimbalSafety_PitchJump(previous_pitch, imu_sample.eulr.pit,
            (uint32_t)(imu_sample.gyro_tick - previous_imu_tick),
            osKernelGetTickFreq());
      gimbal.feedback.eulr.imu = imu_sample.eulr;
      gimbal.feedback.accl = imu_sample.accl;
      gimbal.feedback.gyro = imu_sample.gyro;
      imu_seen = 1U;
    }
    Gimbal_UpdateFeedback(&gimbal, &can);
    Gimbal_UpdateLinkStatus(control_now);

    /* Generate and publish chirp targets under the same lock as serial target
     * writes.  The revision is committed only after the angle is complete. */
    if (gimbal_yaw_chirp_active != 0U) {
      const float elapsed =
          (float)(control_now - gimbal_yaw_chirp_start_tick) /
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
      gimbal_target_input.update_tick = control_now;
      gimbal_target_input.revision++;
    }

    uint8_t stop_reason = GIMBAL_TEST_STOP_NONE;
    if (gimbal_serial_control_active != 0U &&
        target.revision != gimbal_target_input.revision) {
      GimbalTargetInput_t candidate;
      Gimbal_TargetCopyLocked(&candidate);
      if (Gimbal_TargetValidate(&candidate) != 0U)
        target = candidate;
      else
        stop_reason = GIMBAL_TEST_STOP_INVALID_TARGET;
    }

    /* 串口轨迹优先，否则使用遥控队列；遥控增量每帧只消费一次 */
    const uint8_t serial_control_active =
        gimbal_serial_control_active != 0U ? 1U : 0U;
    uint8_t enable;
    if (serial_control_active != 0U) {
      enable = gimbal_test_enable != 0U ? 1U : 0U;
      gimbal_cmd.mode =
          enable != 0U ? GIMBAL_MODE_ABSOLUTE : GIMBAL_MODE_RELAX;
      AHRS_ResetEulr(&gimbal_cmd.delta_eulr);
    } else {
      const uint32_t remote_timeout_tick =
          (GIMBAL_REMOTE_TIMEOUT_MS * osKernelGetTickFreq() + 999U) / 1000U;
      if ((uint32_t)(control_now - remote_gimbal_update_tick) >
          remote_timeout_tick) {
        if (remote_gimbal_cmd.mode == GIMBAL_MODE_ABSOLUTE &&
            remote_gimbal_fault_latched == 0U)
          Gimbal_TripLocked(GIMBAL_TEST_STOP_REMOTE_TIMEOUT, control_now);
        remote_gimbal_cmd.mode = GIMBAL_MODE_RELAX;
      }
      gimbal_cmd = remote_gimbal_cmd;
      AHRS_ResetEulr(&remote_gimbal_cmd.delta_eulr);
      enable = gimbal_cmd.mode == GIMBAL_MODE_ABSOLUTE &&
                       remote_gimbal_fault_latched == 0U
                   ? 1U
                   : 0U;
      if (enable == 0U) gimbal_cmd.mode = GIMBAL_MODE_RELAX;
    }

    if (enable != 0U) {
      if (serial_control_active != 0U &&
          stop_reason == GIMBAL_TEST_STOP_NONE &&
          target.trajectory_active != 0U &&
          GimbalSafety_AgeMs(control_now, target.update_tick,
                             osKernelGetTickFreq()) >
              GIMBAL_BINARY_TRAJECTORY_TIMEOUT_MS)
        stop_reason = GIMBAL_TEST_STOP_TIMEOUT;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               (gimbal_protection_status.imu_age_ms > GIMBAL_IMU_TIMEOUT_MS ||
                gimbal_protection_status.accl_age_ms > GIMBAL_IMU_TIMEOUT_MS))
        stop_reason = GIMBAL_TEST_STOP_IMU_TIMEOUT;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
          (!isfinite(gimbal.feedback.eulr.imu.yaw) ||
          !isfinite(gimbal.feedback.eulr.imu.pit) ||
          !isfinite(gimbal.feedback.eulr.imu.rol) ||
          !isfinite(gimbal.feedback.gyro.x) ||
          !isfinite(gimbal.feedback.gyro.y) ||
           !isfinite(gimbal.feedback.gyro.z) ||
           fabsf(gimbal.feedback.eulr.imu.pit) > 1.5718f ||
           fabsf(gimbal.feedback.eulr.imu.yaw) > 3.1426f ||
           fabsf(gimbal.feedback.eulr.imu.rol) > 3.1426f ||
           fabsf(gimbal.feedback.gyro.x) > 21.0f ||
           fabsf(gimbal.feedback.gyro.y) > 21.0f ||
           fabsf(gimbal.feedback.gyro.z) > 21.0f))
        stop_reason = GIMBAL_TEST_STOP_INVALID_FEEDBACK;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE && pitch_jump != 0U)
        stop_reason = GIMBAL_TEST_STOP_FEEDBACK_JUMP;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               can.gimbal_invalid_output_count != invalid_output_count_seen)
        stop_reason = GIMBAL_TEST_STOP_INVALID_OUTPUT;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               (!isfinite(gimbal_cmd.delta_eulr.yaw) ||
                !isfinite(gimbal_cmd.delta_eulr.pit) ||
                fabsf(gimbal_cmd.delta_eulr.yaw) > 0.25f ||
                fabsf(gimbal_cmd.delta_eulr.pit) > 0.15f ||
                (gimbal.mode == GIMBAL_MODE_ABSOLUTE &&
                 (!isfinite(gimbal.setpoint.eulr.yaw) ||
                  !isfinite(gimbal.setpoint.eulr.pit)))))
        stop_reason = GIMBAL_TEST_STOP_INVALID_TARGET;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               (gimbal_can_tx_health.bus_esr & CAN_ESR_BOFF) != 0U)
        stop_reason = GIMBAL_TEST_STOP_CAN_BUS_OFF;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               gimbal.mode == GIMBAL_MODE_ABSOLUTE &&
               gimbal_can_tx_health.output_age_ms > 50U)
        stop_reason = GIMBAL_TEST_STOP_CONTROL_TIMEOUT;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               Gimbal_CANTxUnsafe(control_now) != 0U)
        stop_reason = GIMBAL_TEST_STOP_CAN_TX;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               Gimbal_CANRxUnsafe() != 0U)
        stop_reason = GIMBAL_TEST_STOP_CAN_RX;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               Gimbal_ActuatorCurrentUnsafe() != 0U)
        stop_reason = GIMBAL_TEST_STOP_ACTUATOR_CURRENT;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               Gimbal_GyroRateUnsafe(&gimbal.feedback.gyro) != 0U)
        stop_reason = GIMBAL_TEST_STOP_RATE;
      else if (stop_reason == GIMBAL_TEST_STOP_NONE &&
               fabsf(gimbal.feedback.eulr.imu.pit) >
                   GIMBAL_PITCH_FAULT_LIMIT_RAD)
        stop_reason = GIMBAL_TEST_STOP_PITCH_PROTECT;

      float elapsed = 0.0f;
      if (serial_control_active != 0U && target.trajectory_active != 0U) {
        elapsed = (float)(control_now - target.update_tick) /
                  (float)osKernelGetTickFreq();
        if (elapsed < 0.0f) elapsed = 0.0f;
        if (elapsed > 0.20f) elapsed = 0.20f;
        const float predicted_pitch =
            target.pitch_rad + target.pitch_velocity_rad_s * elapsed +
            0.5f * target.pitch_acceleration_rad_s2 * elapsed * elapsed;
        /* Finite trajectories saturate at the shared soft limit in the module. */
        if (!isfinite(predicted_pitch))
          stop_reason = GIMBAL_TEST_STOP_INVALID_TARGET;
      }

      if (stop_reason != GIMBAL_TEST_STOP_NONE) {
        Gimbal_TripLocked(stop_reason, control_now);
        enable = 0U;
      }
    } else {
      Gimbal_ActuatorGuardReset();
    }
    if (enable != 0U && serial_control_active != 0U &&
        gimbal.mode == GIMBAL_MODE_ABSOLUTE) {
      const float elapsed = target.trajectory_active != 0U
                                ? (float)(control_now - target.update_tick) /
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
    Gimbal_Control(&gimbal, &gimbal_cmd, control_now);
    if (enable != 0U &&
        (!GimbalSafety_OutputValid(gimbal.out[GIMBAL_ACTR_YAW_IDX],
                                    gimbal.out[GIMBAL_ACTR_PIT_IDX]) ||
         !isfinite(gimbal.setpoint.eulr.yaw) ||
         !isfinite(gimbal.setpoint.eulr.pit))) {
      Gimbal_TripLocked(GIMBAL_TEST_STOP_INVALID_OUTPUT, control_now);
      Gimbal_Control(&gimbal, &gimbal_cmd, control_now);
      enable = 0U;
    }
    if (enable == 0U) {
      gimbal.out[GIMBAL_ACTR_YAW_IDX] = 0.0f;
      gimbal.out[GIMBAL_ACTR_PIT_IDX] = 0.0f;
    }
    if (enable != 0U && gimbal_serial_control_active != 0U) {
      if (gimbal_ladrc_pit_enable == 0U)
        gimbal.out[GIMBAL_ACTR_PIT_IDX] = 0.0f;
    }
    Gimbal_DumpOutput(&gimbal, &gimbal_out);
    gimbal_protection_status.fault_latched = remote_gimbal_fault_latched;
    invalid_output_count_seen = can.gimbal_invalid_output_count;
    if (imu_updated != 0U) {
      previous_pitch = imu_sample.eulr.pit;
      previous_imu_tick = imu_sample.gyro_tick;
    }

    osKernelUnlock();
    osMessageQueueReset(task_runtime.msgq.can.output.gimbal);
    osMessageQueuePut(task_runtime.msgq.can.output.gimbal, &gimbal_out, 0, 0);

    Gimbal_DumpUI(&gimbal, &gimbal_ui);
    osMessageQueueReset(task_runtime.msgq.ui.gimbal);
    osMessageQueuePut(task_runtime.msgq.ui.gimbal, &gimbal_ui, 0, 0);

    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* A late task must not issue a burst of catch-up control cycles. */
    if ((int32_t)(tick - osKernelGetTickCount()) <= 0)
      tick = osKernelGetTickCount() + delay_tick;
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
}
