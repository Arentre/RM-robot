/* VOFA+ FireWater USB telemetry task. */

/* Includes ----------------------------------------------------------------- */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/usb.h"
#include "component/crc16.h"
#include "module/gimbal.h"
#include "task/user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
extern Gimbal_t gimbal;
extern volatile uint8_t gimbal_pitch_protect_active;
extern volatile uint8_t gimbal_test_enable;
extern volatile uint8_t gimbal_serial_control_active;
extern volatile float gimbal_yaw_test_origin_rad;
extern volatile float gimbal_pit_test_origin_rad;
extern volatile uint8_t gimbal_test_stop_reason;
extern volatile uint8_t gimbal_yaw_chirp_active;
extern volatile uint8_t gimbal_yaw_chirp_down;
extern volatile uint32_t gimbal_yaw_chirp_start_tick;
extern volatile uint8_t gimbal_yaw_dither_enable;
extern volatile float gimbal_yaw_dither_amplitude;
extern volatile float gimbal_yaw_dither_frequency_hz;
volatile GimbalTargetInput_t gimbal_target_input = {0};

/* Exact sp_vision_25 VisionToGimbal wire layout, 29 bytes packed. */
typedef struct __attribute__((packed)) {
  uint8_t head[2];
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
} VisionToGimbalPacket_t;
typedef char VisionToGimbalPacketSizeCheck[
    sizeof(VisionToGimbalPacket_t) == 29U ? 1 : -1];

static void Gimbal_SerialStop(uint8_t reason);

/* Caller holds the kernel lock.  Incrementing revision last is the commit. */
static void Gimbal_TargetCommitLocked(void) {
  gimbal_target_input.update_tick = osKernelGetTickCount();
  gimbal_target_input.revision++;
}

static void Gimbal_TargetClearTrajectoryLocked(void) {
  gimbal_target_input.yaw_velocity_rad_s = 0.0f;
  gimbal_target_input.yaw_acceleration_rad_s2 = 0.0f;
  gimbal_target_input.pitch_velocity_rad_s = 0.0f;
  gimbal_target_input.pitch_acceleration_rad_s2 = 0.0f;
  gimbal_target_input.trajectory_active = 0U;
}

static void Gimbal_TargetSetHoldLocked(float yaw, float pitch) {
  Gimbal_TargetClearTrajectoryLocked();
  gimbal_target_input.yaw_rad = yaw;
  gimbal_target_input.pitch_rad = pitch;
  gimbal_target_input.yaw_valid = 1U;
  gimbal_target_input.pitch_valid = 1U;
  Gimbal_TargetCommitLocked();
}

static void Gimbal_TargetDisableLocked(void) {
  Gimbal_TargetClearTrajectoryLocked();
  gimbal_target_input.yaw_valid = 0U;
  gimbal_target_input.pitch_valid = 0U;
  Gimbal_TargetCommitLocked();
}

static uint8_t Gimbal_ParseTrajectory(const uint8_t *data, uint16_t length) {
  VisionToGimbalPacket_t packet;
  if (data == NULL || length != sizeof(packet) || data[0] != 'S' ||
      data[1] != 'P')
    return 0U;
  memcpy(&packet, data, sizeof(packet));
  if (CRC16_Calc(data, sizeof(packet) - sizeof(packet.crc16), CRC16_INIT) !=
      packet.crc16) {
    BSP_USB_Printf("err:V2G,crc\r\n");
    return 1U;
  }
  /* Mode 0 is the reference idle command. Mode 2 would imply a fire command,
   * which is not wired to this bench adapter. */
  if (!isfinite(packet.yaw) ||
      !isfinite(packet.yaw_vel) || !isfinite(packet.yaw_acc) ||
      !isfinite(packet.pitch) || !isfinite(packet.pitch_vel) ||
      !isfinite(packet.pitch_acc)) {
    BSP_USB_Printf("err:V2G,range\r\n");
    return 1U;
  }
  if (packet.mode == 0U) {
    Gimbal_SerialStop(GIMBAL_TEST_STOP_COMMAND);
    return 1U;
  }
  if (packet.mode != 1U) {
    BSP_USB_Printf("err:V2G,mode\r\n");
    return 1U;
  }
  if (fabsf(packet.yaw_vel) > 20.0f || fabsf(packet.pitch_vel) > 12.0f ||
      fabsf(packet.yaw_acc) > 160.0f || fabsf(packet.pitch_acc) > 120.0f ||
      fabsf(packet.pitch) > GIMBAL_PITCH_SAFE_LIMIT_RAD) {
    BSP_USB_Printf("err:V2G,limit\r\n");
    return 1U;
  }
  float yaw = packet.yaw;
  CircleAdd(&yaw, 0.0f, M_2PI);
  osKernelLock();
  if (gimbal_serial_control_active == 0U || gimbal_test_enable == 0U) {
    osKernelUnlock();
    BSP_USB_Printf("err:V2G,disabled\r\n");
    return 1U;
  }
  gimbal_target_input.yaw_rad = yaw;
  gimbal_target_input.pitch_rad = packet.pitch;
  gimbal_target_input.yaw_velocity_rad_s = packet.yaw_vel;
  gimbal_target_input.yaw_acceleration_rad_s2 = packet.yaw_acc;
  gimbal_target_input.pitch_velocity_rad_s = packet.pitch_vel;
  gimbal_target_input.pitch_acceleration_rad_s2 = packet.pitch_acc;
  gimbal_target_input.yaw_valid = 1U;
  gimbal_target_input.pitch_valid = 1U;
  gimbal_target_input.trajectory_active = 1U;
  Gimbal_TargetCommitLocked();
  osKernelUnlock();
  return 1U;
}
/* Private function --------------------------------------------------------- */
static float FireWater_Safe(float value) {
  return isfinite(value) ? value : 0.0f;
}

static float Angle_Degrees360(float radians) {
  if (!isfinite(radians)) return 0.0f;
  float degrees = fmodf(radians * M_RAD2DEG_MULT, 360.0f);
  if (degrees < 0.0f) degrees += 360.0f;
  if (degrees >= 360.0f) degrees -= 360.0f;
  return degrees;
}

static void Gimbal_SerialStop(uint8_t reason) {
  osKernelLock();
  gimbal_test_enable = 0U;
  gimbal_serial_control_active = 0U;
  Gimbal_TargetDisableLocked();
  gimbal_ladrc_pit_enable = 0U;
  gimbal_yaw_chirp_active = 0U;
  gimbal_test_stop_reason = reason;
  osKernelUnlock();
}

static uint8_t Gimbal_ParseNumber(const char *text, float *value) {
  char *end;
  *value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(*value);
}

static uint8_t Gimbal_LADRCParamsSafe(const LADRC_Params_t *p,
                                      float max_output) {
  return p->r >= 0.5f && p->r <= 50.0f && p->b0 >= 10.0f &&
         p->b0 <= 2000.0f && p->wc >= 0.2f && p->wc <= 50.0f &&
         p->w0 >= 0.5f && p->w0 <= 200.0f && p->out_limit >= 0.005f &&
         p->out_limit <= max_output;
}

static uint8_t Gimbal_ParseLADRCParams(const char *text, LADRC_Params_t *p) {
  float *value[5] = {&p->r, &p->b0, &p->wc, &p->w0, &p->out_limit};
  const char *cursor = text;
  for (uint8_t k = 0U; k < 5U; k++) {
    char *end;
    *value[k] = strtof(cursor, &end);
    if (end == cursor || !isfinite(*value[k])) return 0U;
    if (k < 4U) {
      if (*end != ',') return 0U;
      cursor = end + 1;
    } else if (*end != '\0') {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t Gimbal_ParseFive(const char *text, float value[5]) {
  const char *cursor = text;
  for (uint8_t k = 0U; k < 5U; k++) {
    char *end;
    value[k] = strtof(cursor, &end);
    if (end == cursor || !isfinite(value[k])) return 0U;
    if (k < 4U) {
      if (*end != ',') return 0U;
      cursor = end + 1;
    } else if (*end != '\0') {
      return 0U;
    }
  }
  return 1U;
}

static void Gimbal_CalibrateGyro(void) {
  enum { sample_count = 750U };
  const float sample_delay_s = 0.020f;
  const uint32_t sample_delay_ticks =
      (uint32_t)(sample_delay_s * (float)osKernelGetTickFreq()) > 0U
          ? (uint32_t)(sample_delay_s * (float)osKernelGetTickFreq())
          : 1U;
  const float rate_limit = 3.0f * M_DEG2RAD_MULT;
  const float noise_limit = 0.5f * M_DEG2RAD_MULT;

  if (gimbal_test_enable != 0U || gimbal_serial_control_active != 0U) {
    BSP_USB_Printf("err:GCAL,stop_first\r\n");
    return;
  }
  if (!isfinite(gimbal_imu_temperature_c) ||
      gimbal_imu_temperature_c < 38.0f ||
      gimbal_imu_temperature_c > 42.0f) {
    BSP_USB_Printf("err:GCAL,temp=%.2f,need=38..42\r\n",
                   gimbal_imu_temperature_c);
    return;
  }

  BSP_USB_Printf("ack:GCAL,start,temp=%.2f,seconds=%.1f\r\n",
                 gimbal_imu_temperature_c,
                 (float)sample_count * sample_delay_s);

  float mean[3] = {0.0f, 0.0f, 0.0f};
  float m2[3] = {0.0f, 0.0f, 0.0f};
  for (uint32_t n = 1U; n <= sample_count; n++) {
    osDelay(sample_delay_ticks);

    AHRS_Gyro_t gyro;
    AHRS_Accl_t accl;
    float temperature;
    osKernelLock();
    gyro = gimbal.feedback.gyro;
    accl = gimbal.feedback.accl;
    temperature = gimbal_imu_temperature_c;
    const uint8_t stopped =
        gimbal_test_enable == 0U && gimbal_serial_control_active == 0U;
    osKernelUnlock();

    const float accl_norm =
        sqrtf(accl.x * accl.x + accl.y * accl.y + accl.z * accl.z);
    if (stopped == 0U || !isfinite(gyro.x) || !isfinite(gyro.y) ||
        !isfinite(gyro.z) || !isfinite(accl_norm) ||
        fabsf(gyro.x) >= rate_limit || fabsf(gyro.y) >= rate_limit ||
        fabsf(gyro.z) >= rate_limit || accl_norm < 0.85f ||
        accl_norm > 1.15f || temperature < 38.0f || temperature > 42.0f) {
      BSP_USB_Printf("err:GCAL,moving_or_temp,temp=%.2f\r\n", temperature);
      return;
    }

    const float value[3] = {gyro.x, gyro.y, gyro.z};
    for (uint8_t axis = 0U; axis < 3U; axis++) {
      const float delta = value[axis] - mean[axis];
      mean[axis] += delta / (float)n;
      m2[axis] += delta * (value[axis] - mean[axis]);
    }
  }

  float deviation[3];
  for (uint8_t axis = 0U; axis < 3U; axis++)
    deviation[axis] = sqrtf(m2[axis] / (float)(sample_count - 1U));
  if (deviation[0] > noise_limit || deviation[1] > noise_limit ||
      deviation[2] > noise_limit) {
    BSP_USB_Printf("err:GCAL,noise=%.3f,%.3f,%.3fdeg_s\r\n",
                   deviation[0] * M_RAD2DEG_MULT,
                   deviation[1] * M_RAD2DEG_MULT,
                   deviation[2] * M_RAD2DEG_MULT);
    return;
  }

  Config_t cfg;
  Config_Get(&cfg);
  /* Measurements are already corrected by the active static offset, so the
   * remaining stationary mean is added to that offset. */
  cfg.cali.bmi088.gyro_offset.x =
      task_runtime.cfg.cali.bmi088.gyro_offset.x + mean[0];
  cfg.cali.bmi088.gyro_offset.y =
      task_runtime.cfg.cali.bmi088.gyro_offset.y + mean[1];
  cfg.cali.bmi088.gyro_offset.z =
      task_runtime.cfg.cali.bmi088.gyro_offset.z + mean[2];
  Config_Set(&cfg);

  Config_t saved_cfg;
  Config_Get(&saved_cfg);
  if (!isfinite(saved_cfg.cali.bmi088.gyro_offset.x) ||
      !isfinite(saved_cfg.cali.bmi088.gyro_offset.y) ||
      !isfinite(saved_cfg.cali.bmi088.gyro_offset.z) ||
      saved_cfg.cali.bmi088.gyro_offset.x != cfg.cali.bmi088.gyro_offset.x ||
      saved_cfg.cali.bmi088.gyro_offset.y != cfg.cali.bmi088.gyro_offset.y ||
      saved_cfg.cali.bmi088.gyro_offset.z != cfg.cali.bmi088.gyro_offset.z) {
    BSP_USB_Printf("err:GCAL,flash_verify\r\n");
    return;
  }

  osKernelLock();
  task_runtime.cfg.cali.bmi088 = saved_cfg.cali.bmi088;
  osKernelUnlock();
  BSP_USB_Printf(
      "ack:GCAL,offset=%.5f,%.5f,%.5fdeg_s,noise=%.3f,%.3f,%.3fdeg_s\r\n",
      cfg.cali.bmi088.gyro_offset.x * M_RAD2DEG_MULT,
      cfg.cali.bmi088.gyro_offset.y * M_RAD2DEG_MULT,
      cfg.cali.bmi088.gyro_offset.z * M_RAD2DEG_MULT,
      deviation[0] * M_RAD2DEG_MULT,
      deviation[1] * M_RAD2DEG_MULT,
      deviation[2] * M_RAD2DEG_MULT);
}

/* Bench commands are ASCII lines terminated by CR or LF. */
static void Gimbal_PrintFault(void) {
  GimbalFaultSnapshot_t fault;
  GimbalProtectionStatus_t live;
  osKernelLock();
  fault = gimbal_fault_snapshot;
  live = gimbal_protection_status;
  osKernelUnlock();
  BSP_USB_Printf(
      "ack:FAULT,reason=%lu,tick=%lu,trips=%lu,latched=%lu,"
      "imu_ms=%lu,acc_ms=%lu,rx_ms=%lu/%lu,tx_ms=%lu,out_ms=%lu,"
      "esr=0x%08lX,tsr=0x%08lX,drop=%lu,imu_err=%lu,"
      "bad_rx=%lu/%lu,bad_out=%lu,pitch_en=%lu,serial=%lu,mismatch=%lu/%lu,"
      "pit_deg=%.3f,prev_deg=%.3f,target_deg=%.3f,"
      "gyro=%.3f/%.3f/%.3f,out=%.4f/%.4f,"
      "current=%.4f/%.4f,temp=%.1f/%.1f,"
      "live_imu_ms=%lu,live_acc_ms=%lu,live_rx_ms=%lu/%lu\r\n",
      (unsigned long)fault.reason, (unsigned long)fault.tick,
      (unsigned long)live.trip_count, (unsigned long)live.fault_latched,
      (unsigned long)fault.link.imu_age_ms,
      (unsigned long)fault.link.accl_age_ms,
      (unsigned long)fault.link.feedback_age_ms[0],
      (unsigned long)fault.link.feedback_age_ms[1],
      (unsigned long)fault.link.tx_age_ms, (unsigned long)fault.output_age_ms,
      (unsigned long)fault.can_esr, (unsigned long)fault.can_tsr,
      (unsigned long)fault.link.rx_queue_drop_count,
      (unsigned long)fault.imu_io_error_count,
      (unsigned long)fault.invalid_feedback_count[0],
      (unsigned long)fault.invalid_feedback_count[1],
      (unsigned long)fault.invalid_output_count,
      (unsigned long)fault.pitch_control_enabled,
      (unsigned long)fault.serial_control_active,
      (unsigned long)fault.mismatch_count[0],
      (unsigned long)fault.mismatch_count[1],
      fault.attitude.pit * M_RAD2DEG_MULT,
      fault.previous_pitch * M_RAD2DEG_MULT,
      fault.pitch_target * M_RAD2DEG_MULT,
      fault.gyro.x, fault.gyro.y, fault.gyro.z,
      fault.output[0], fault.output[1], fault.current[0], fault.current[1],
      fault.temperature[0], fault.temperature[1],
      (unsigned long)live.imu_age_ms, (unsigned long)live.accl_age_ms,
      (unsigned long)live.feedback_age_ms[0],
      (unsigned long)live.feedback_age_ms[1]);
  return;
}

static void Gimbal_ParseCommand(char *cmd) {
  /* Command names are case insensitive; numeric exponents also accept E. */
  for (char *p = cmd; *p != '\0'; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
  }
  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "ENABLE=0") == 0) {
    Gimbal_SerialStop(GIMBAL_TEST_STOP_COMMAND);
    BSP_USB_Printf("ack:STOP\r\n");
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    return;
  }

  if (strcmp(cmd, "PROTO?") == 0) {
    BSP_USB_Printf("ack:PROTO=SPV2,TRAJ,CRC16\r\n");
    return;
  }

  if (strcmp(cmd, "IMU") == 0) {
    AHRS_Gyro_t gyro;
    BMI088_Cali_t cali;
    float temperature;
    float heater_duty;
    osKernelLock();
    gyro = gimbal.feedback.gyro;
    cali = task_runtime.cfg.cali.bmi088;
    temperature = gimbal_imu_temperature_c;
    heater_duty = gimbal_imu_heater_duty;
    osKernelUnlock();
    BSP_USB_Printf(
        "ack:IMU,temp=%.2f,gyro=%.4f,%.4f,%.4fdeg_s,"
        "offset=%.5f,%.5f,%.5fdeg_s,heat=%.3f\r\n",
        temperature, gyro.x * M_RAD2DEG_MULT, gyro.y * M_RAD2DEG_MULT,
        gyro.z * M_RAD2DEG_MULT,
        cali.gyro_offset.x * M_RAD2DEG_MULT,
        cali.gyro_offset.y * M_RAD2DEG_MULT,
        cali.gyro_offset.z * M_RAD2DEG_MULT, heater_duty);
    return;
  }

  if (strcmp(cmd, "GCAL") == 0) {
    Gimbal_CalibrateGyro();
    return;
  }

  if (strcmp(cmd, "ENABLE=1") == 0) {
    const float yaw = gimbal.feedback.eulr.imu.yaw;
    const float pit = gimbal.feedback.eulr.imu.pit;
    if (!isfinite(yaw) || !isfinite(pit)) {
      Gimbal_SerialStop(GIMBAL_TEST_STOP_INVALID_FEEDBACK);
      BSP_USB_Printf("err:ENABLE,feedback\r\n");
      return;
    }
    if (fabsf(pit) > GIMBAL_PITCH_SAFE_LIMIT_RAD) {
      Gimbal_SerialStop(GIMBAL_TEST_STOP_PITCH_PROTECT);
      BSP_USB_Printf("err:ENABLE,pitch_limit\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_test_origin_rad = yaw;
    gimbal_pit_test_origin_rad = pit;
    Gimbal_TargetSetHoldLocked(yaw, pit);
    gimbal_ladrc_yaw_enable = 1U;
    gimbal_ladrc_pit_enable = 0U;
    gimbal_test_stop_reason = GIMBAL_TEST_STOP_NONE;
    gimbal_yaw_chirp_active = 0U;
    gimbal_serial_control_active = 1U;
    gimbal_test_enable = 1U;
    osKernelUnlock();
    BSP_USB_Printf("ack:ENABLE,yaw=%.3f,pit=%.3f\r\n",
                   yaw * M_RAD2DEG_MULT, pit * M_RAD2DEG_MULT);
    return;
  }

  if (strcmp(cmd, "HOLD") == 0) {
    if (gimbal_serial_control_active == 0U) {
      BSP_USB_Printf("err:HOLD,disabled\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_chirp_active = 0U;
    Gimbal_TargetSetHoldLocked(gimbal.feedback.eulr.imu.yaw,
                               gimbal.feedback.eulr.imu.pit);
    osKernelUnlock();
    BSP_USB_Printf("ack:HOLD\r\n");
    return;
  }

  if (strcmp(cmd, "CENTER") == 0) {
    if (gimbal_serial_control_active == 0U) {
      BSP_USB_Printf("err:CENTER,disabled\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_chirp_active = 0U;
    Gimbal_TargetSetHoldLocked(gimbal_yaw_test_origin_rad,
                               gimbal_pit_test_origin_rad);
    osKernelUnlock();
    BSP_USB_Printf("ack:CENTER\r\n");
    return;
  }

  if (strcmp(cmd, "YCHIRP=UP") == 0 ||
      strcmp(cmd, "YCHIRP=DOWN") == 0) {
    if (gimbal_serial_control_active == 0U || gimbal_test_enable == 0U) {
      BSP_USB_Printf("err:YCHIRP,state\r\n");
      return;
    }
    osKernelLock();
    Gimbal_TargetClearTrajectoryLocked();
    gimbal_yaw_chirp_down = strcmp(cmd, "YCHIRP=DOWN") == 0 ? 1U : 0U;
    gimbal_yaw_chirp_start_tick = osKernelGetTickCount();
    gimbal_yaw_chirp_active = 1U;
    Gimbal_TargetCommitLocked();
    osKernelUnlock();
    BSP_USB_Printf("ack:YCHIRP=%s\r\n",
                   gimbal_yaw_chirp_down != 0U ? "DOWN" : "UP");
    return;
  }

  if (strcmp(cmd, "PITCH=0") == 0) {
    osKernelLock();
    Gimbal_TargetClearTrajectoryLocked();
    Gimbal_TargetCommitLocked();
    gimbal_ladrc_pit_enable = 0U;
    osKernelUnlock();
    BSP_USB_Printf("ack:PITCH=0\r\n");
    return;
  }
  if (strcmp(cmd, "PITCH=1") == 0) {
    if (gimbal_serial_control_active == 0U) {
      BSP_USB_Printf("err:PITCH,disabled\r\n");
      return;
    }
    osKernelLock();
    Gimbal_TargetClearTrajectoryLocked();
    gimbal_target_input.pitch_rad = gimbal.feedback.eulr.imu.pit;
    gimbal_target_input.pitch_valid = 1U;
    Gimbal_TargetCommitLocked();
    gimbal_ladrc_pit_enable = 1U;
    osKernelUnlock();
    BSP_USB_Printf("ack:PITCH=1\r\n");
    return;
  }

  if (strcmp(cmd, "FAULT") == 0) {
    Gimbal_PrintFault();
    return;
  }

  if (strcmp(cmd, "GET") == 0) {
    const float yaw_travel =
        CircleError(gimbal.feedback.eulr.imu.yaw, gimbal_yaw_test_origin_rad,
                    M_2PI) *
        M_RAD2DEG_MULT;
    BSP_USB_Printf(
        "ack:GET,en=%u,serial=%u,stop=%u,yaw=%.3f,travel=%.3f,"
        "yp=%.4f,%.4f,%.4f,%.4f,%.4f,"
        "pp=%.4f,%.4f,%.4f,%.4f,%.4f,"
        "yff=%u,%.4f,%.4f,%.3f,%.3f,%.3f,"
        "pff=%u,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,"
        "pgrav=%u,%.4f,%.3f,%.4f,"
        "pit=%.3f,pit_target=%.3f,pitch_enable=%u,"
        "ydead=%.3f,%u,pdead=%.3f,%u,pfilter=%.3f\r\n",
        (unsigned int)gimbal_test_enable,
        (unsigned int)gimbal_serial_control_active,
        (unsigned int)gimbal_test_stop_reason,
        gimbal.feedback.eulr.imu.yaw * M_RAD2DEG_MULT, yaw_travel,
        gimbal_ladrc_yaw_param.r, gimbal_ladrc_yaw_param.b0,
        gimbal_ladrc_yaw_param.wc, gimbal_ladrc_yaw_param.w0,
        gimbal_ladrc_yaw_param.out_limit, gimbal_ladrc_pit_param.r,
        gimbal_ladrc_pit_param.b0, gimbal_ladrc_pit_param.wc,
        gimbal_ladrc_pit_param.w0, gimbal_ladrc_pit_param.out_limit,
        (unsigned int)gimbal_yaw_friction.enable,
        gimbal_yaw_friction.static_output, gimbal_yaw_friction.coulomb_output,
        gimbal_yaw_friction.deadband_rad * M_RAD2DEG_MULT,
        gimbal_yaw_friction.transition_rad_s * M_RAD2DEG_MULT,
        gimbal_yaw_friction.slew_per_s,
        (unsigned int)gimbal_pitch_friction.enable,
        gimbal_pitch_friction.static_output,
        gimbal_pitch_friction.coulomb_output,
        gimbal_pitch_friction.deadband_rad * M_RAD2DEG_MULT,
        gimbal_pitch_friction.transition_rad_s * M_RAD2DEG_MULT,
        gimbal_pitch_friction.slew_per_s, gimbal_pitch_friction.output,
        (unsigned int)gimbal_pitch_gravity.enable,
        gimbal_pitch_gravity.amplitude,
        gimbal_pitch_gravity.phase_rad * M_RAD2DEG_MULT,
        gimbal_pitch_gravity.output,
        gimbal.feedback.eulr.imu.pit * M_RAD2DEG_MULT,
        gimbal.setpoint.eulr.pit * M_RAD2DEG_MULT,
        (unsigned int)gimbal_ladrc_pit_enable,
        gimbal_ladrc_yaw_param.deadband_rad * M_RAD2DEG_MULT,
        (unsigned int)(gimbal.ladrc_active[GIMBAL_ACTR_YAW_IDX] != 0U &&
                       gimbal.mode == GIMBAL_MODE_ABSOLUTE &&
                       gimbal.ladrc[GIMBAL_ACTR_YAW_IDX].deadband_active != 0U),
        gimbal_ladrc_pit_param.deadband_rad * M_RAD2DEG_MULT,
        (unsigned int)(gimbal.ladrc_active[GIMBAL_ACTR_PIT_IDX] != 0U &&
                       gimbal.mode == GIMBAL_MODE_ABSOLUTE &&
                       gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].deadband_active != 0U),
        gimbal_pitch_output_filter_hz);
    return;
  }

  if (strncmp(cmd, "YDEAD=", 6) == 0 || strncmp(cmd, "PDEAD=", 6) == 0) {
    float degree;
    if (Gimbal_ParseNumber(cmd + 6, &degree) == 0U ||
        degree < 0.0f || degree > 3.0f) {
      BSP_USB_Printf("err:%cDEAD,range\r\n", cmd[0]);
      return;
    }
    osKernelLock();
    volatile LADRC_Params_t *p = cmd[0] == 'Y' ? &gimbal_ladrc_yaw_param :
                                                &gimbal_ladrc_pit_param;
    p->deadband_rad = degree * M_DEG2RAD_MULT;
    osKernelUnlock();
    BSP_USB_Printf("ack:%cDEAD=%.3f\r\n", cmd[0], degree);
    return;
  }

  if (strncmp(cmd, "PFILTER=", 8) == 0) {
    float frequency;
    if (Gimbal_ParseNumber(cmd + 8, &frequency) == 0U ||
        !(frequency == 0.0f || (frequency >= 5.0f && frequency <= 100.0f))) {
      BSP_USB_Printf("err:PFILTER,range\r\n");
      return;
    }
    /* Change shaping between trials, never while applying motor current. */
    osKernelLock();
    if (gimbal_test_enable != 0U || gimbal_serial_control_active != 0U) {
      osKernelUnlock();
      BSP_USB_Printf("err:PFILTER,stop_first\r\n");
      return;
    }
    gimbal_pitch_output_filter_hz = frequency;
    osKernelUnlock();
    BSP_USB_Printf("ack:PFILTER=%.3f\r\n", frequency);
    return;
  }

  if (strncmp(cmd, "YPARAM=", 7) == 0) {
    LADRC_Params_t p = {.deadband_rad = gimbal_ladrc_yaw_param.deadband_rad};
    if (Gimbal_ParseLADRCParams(cmd + 7, &p) == 0U ||
        Gimbal_LADRCParamsSafe(&p, GIMBAL_NORMALIZED_OUTPUT_MAX) == 0U) {
      BSP_USB_Printf("err:YPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_ladrc_yaw_param = p;
    osKernelUnlock();
    BSP_USB_Printf("ack:YPARAM\r\n");
    return;
  }

  if (strncmp(cmd, "PPARAM=", 7) == 0) {
    LADRC_Params_t p = {.deadband_rad = gimbal_ladrc_pit_param.deadband_rad};
    if (Gimbal_ParseLADRCParams(cmd + 7, &p) == 0U ||
        Gimbal_LADRCParamsSafe(&p, GIMBAL_NORMALIZED_OUTPUT_MAX) == 0U) {
      BSP_USB_Printf("err:PPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_ladrc_pit_param = p;
    osKernelUnlock();
    BSP_USB_Printf("ack:PPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "YFF=0") == 0 || strcmp(cmd, "YFF=1") == 0) {
    gimbal_yaw_friction.enable = cmd[4] == '1' ? 1U : 0U;
    BSP_USB_Printf("ack:YFF=%u\r\n",
                   (unsigned int)gimbal_yaw_friction.enable);
    return;
  }

  if (strncmp(cmd, "YFFPARAM=", 9) == 0) {
    float value[5];
    const char *cursor = cmd + 9;
    uint8_t valid = 1U;
    for (uint8_t k = 0U; k < 5U; k++) {
      char *end;
      value[k] = strtof(cursor, &end);
      if (end == cursor || !isfinite(value[k])) {
        valid = 0U;
        break;
      }
      if (k < 4U) {
        if (*end != ',') {
          valid = 0U;
          break;
        }
        cursor = end + 1;
      } else if (*end != '\0') {
        valid = 0U;
      }
    }
    if (valid == 0U || value[0] < 0.0f || value[0] > 0.06f ||
        value[1] < 0.0f || value[1] > value[0] || value[2] < 0.2f ||
        value[2] > 3.0f || value[3] < 2.0f || value[3] > 30.0f ||
        value[4] < 0.02f || value[4] > 0.50f) {
      BSP_USB_Printf("err:YFFPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_friction.static_output = value[0];
    gimbal_yaw_friction.coulomb_output = value[1];
    gimbal_yaw_friction.deadband_rad = value[2] * M_DEG2RAD_MULT;
    gimbal_yaw_friction.transition_rad_s = value[3] * M_DEG2RAD_MULT;
    gimbal_yaw_friction.slew_per_s = value[4];
    osKernelUnlock();
    BSP_USB_Printf("ack:YFFPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "PFF=0") == 0 || strcmp(cmd, "PFF=1") == 0) {
    gimbal_pitch_friction.enable = cmd[4] == '1' ? 1U : 0U;
    BSP_USB_Printf("ack:PFF=%u\r\n",
                   (unsigned int)gimbal_pitch_friction.enable);
    return;
  }

  if (strncmp(cmd, "PFFPARAM=", 9) == 0) {
    float value[5];
    if (Gimbal_ParseFive(cmd + 9, value) == 0U || value[0] < 0.0f ||
        value[0] > 0.06f || value[1] < 0.0f || value[1] > value[0] ||
        value[2] < 0.2f || value[2] > 3.0f || value[3] < 2.0f ||
        value[3] > 30.0f || value[4] < 0.02f || value[4] > 0.50f) {
      BSP_USB_Printf("err:PFFPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_pitch_friction.static_output = value[0];
    gimbal_pitch_friction.coulomb_output = value[1];
    gimbal_pitch_friction.deadband_rad = value[2] * M_DEG2RAD_MULT;
    gimbal_pitch_friction.transition_rad_s = value[3] * M_DEG2RAD_MULT;
    gimbal_pitch_friction.slew_per_s = value[4];
    osKernelUnlock();
    BSP_USB_Printf("ack:PFFPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "PGRAV=0") == 0 || strcmp(cmd, "PGRAV=1") == 0) {
    gimbal_pitch_gravity.enable = cmd[6] == '1' ? 1U : 0U;
    BSP_USB_Printf("ack:PGRAV=%u\r\n",
                   (unsigned int)gimbal_pitch_gravity.enable);
    return;
  }

  if (strncmp(cmd, "PGRAVPARAM=", 11) == 0) {
    float value[2];
    const char *cursor = cmd + 11;
    char *end;
    value[0] = strtof(cursor, &end);
    if (end == cursor || *end != ',' || !isfinite(value[0])) {
      BSP_USB_Printf("err:PGRAVPARAM,format\r\n");
      return;
    }
    cursor = end + 1;
    value[1] = strtof(cursor, &end);
    if (end == cursor || *end != '\0' || !isfinite(value[1]) ||
        value[0] < -0.10f || value[0] > 0.10f || value[1] < -180.0f ||
        value[1] > 180.0f) {
      BSP_USB_Printf("err:PGRAVPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_pitch_gravity.amplitude = value[0];
    gimbal_pitch_gravity.phase_rad = value[1] * M_DEG2RAD_MULT;
    osKernelUnlock();
    BSP_USB_Printf("ack:PGRAVPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "YDITHER=0") == 0 || strcmp(cmd, "YDITHER=1") == 0) {
    gimbal_yaw_dither_enable = cmd[8] == '1' ? 1U : 0U;
    BSP_USB_Printf("ack:YDITHER=%u\r\n",
                   (unsigned int)gimbal_yaw_dither_enable);
    return;
  }

  if (strncmp(cmd, "YDITHERPARAM=", 13) == 0) {
    const char *cursor = cmd + 13;
    char *end;
    float amplitude = strtof(cursor, &end);
    if (end == cursor || *end != ',' || !isfinite(amplitude)) {
      BSP_USB_Printf("err:YDITHERPARAM,format\r\n");
      return;
    }
    cursor = end + 1;
    float frequency = strtof(cursor, &end);
    if (end == cursor || *end != '\0' || !isfinite(frequency) ||
        amplitude < 0.0f || amplitude > 0.020f || frequency < 5.0f ||
        frequency > 50.0f) {
      BSP_USB_Printf("err:YDITHERPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_dither_amplitude = amplitude;
    gimbal_yaw_dither_frequency_hz = frequency;
    osKernelUnlock();
    BSP_USB_Printf("ack:YDITHERPARAM\r\n");
    return;
  }

  float degree;
  const uint8_t yaw_relative = strncmp(cmd, "YAWREL=", 7) == 0;
  const uint8_t yaw_absolute = strncmp(cmd, "YAWABS=", 7) == 0;
  if (yaw_relative || yaw_absolute || strncmp(cmd, "YAW=", 4) == 0) {
    const char *name = yaw_relative ? "YAWREL" : (yaw_absolute ? "YAWABS" : "YAW");
    const char *number = cmd + ((yaw_relative || yaw_absolute) ? 7 : 4);
    if (Gimbal_ParseNumber(number, &degree) == 0U) {
      BSP_USB_Printf("err:%s,range\r\n", name);
      return;
    }

    /* Check state and commit under the same lock: a protection stop must
     * not race with this command and leave a new target marked valid. */
    osKernelLock();
    if (gimbal_serial_control_active == 0U || gimbal_test_enable == 0U) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,disabled\r\n", name);
      return;
    }
    const float feedback = gimbal.feedback.eulr.imu.yaw;
    if (!isfinite(feedback)) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,feedback\r\n", name);
      return;
    }
    float target = yaw_relative ? feedback : 0.0f;
    /* Normalize large values before converting to radians and wrapping.
     * Yaw still follows the shortest path to a wrapped orientation. */
    CircleAdd(&target, fmodf(degree, 360.0f) * M_DEG2RAD_MULT, M_2PI);
    gimbal_yaw_chirp_active = 0U;
    Gimbal_TargetClearTrajectoryLocked();
    gimbal_target_input.yaw_rad = target;
    gimbal_target_input.yaw_valid = 1U;
    Gimbal_TargetCommitLocked();
    osKernelUnlock();
    BSP_USB_Printf("ack:%s=%.3f\r\n", name,
                   yaw_relative ? degree : Angle_Degrees360(target));
    return;
  }

  const uint8_t pit_relative = strncmp(cmd, "PITREL=", 7) == 0;
  const uint8_t pit_absolute = strncmp(cmd, "PITABS=", 7) == 0;
  if (pit_relative || pit_absolute || strncmp(cmd, "PIT=", 4) == 0) {
    const char *name = pit_relative ? "PITREL" : (pit_absolute ? "PITABS" : "PIT");
    const char *number = cmd + ((pit_relative || pit_absolute) ? 7 : 4);
    if (Gimbal_ParseNumber(number, &degree) == 0U) {
      BSP_USB_Printf("err:%s,range\r\n", name);
      return;
    }

    osKernelLock();
    if (gimbal_serial_control_active == 0U || gimbal_test_enable == 0U ||
        gimbal_ladrc_pit_enable == 0U) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,disabled\r\n", name);
      return;
    }
    const float feedback = gimbal.feedback.eulr.imu.pit;
    if (!isfinite(feedback)) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,feedback\r\n", name);
      return;
    }
    const float target = (pit_relative ? feedback : 0.0f) +
                         degree * M_DEG2RAD_MULT;
    if (!isfinite(target) || fabsf(target) > GIMBAL_PITCH_SAFE_LIMIT_RAD) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,pitch_limit\r\n", name);
      return;
    }
    Gimbal_TargetClearTrajectoryLocked();
    gimbal_target_input.pitch_rad = target;
    gimbal_target_input.pitch_valid = 1U;
    Gimbal_TargetCommitLocked();
    osKernelUnlock();
    BSP_USB_Printf("ack:%s=%.3f\r\n", name, degree);
    return;
  }

  BSP_USB_Printf("err:UNKNOWN\r\n");
}

/* Exported functions ------------------------------------------------------- */

/**
 * \brief Output gimbal data using the VOFA+ FireWater text protocol.
 *
 * \param argument 未使用
 */
void Task_Info(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / TASK_FREQ_INFO;

  /* Wait for USB enumeration and the first gimbal samples. */
  osDelay(TASK_INIT_DELAY_INFO);
  BSP_USB_ReadyReceive(osThreadGetId());

  char cmd[64] = {0};
  uint8_t cmd_len = 0U;
  uint8_t cmd_overflow = 0U;
  float yaw_fb_continuous = 0.0f;
  float last_yaw_fb = 0.0f;
  uint8_t yaw_fb_ready = 0U;
  uint32_t tick = osKernelGetTickCount();
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.info = osThreadGetStackSpace(osThreadGetId());
#endif
    if (osThreadFlagsWait(SIGNAL_BSP_USB_BUF_RECV, osFlagsWaitAny, 0U) <
        osFlagsError) {
      const uint16_t rx_len = usb_rx_len;
      usb_rx_len = 0U;
      const uint8_t binary_candidate =
          rx_len >= 2U && usb_rx_buf[0] == 'S' && usb_rx_buf[1] == 'P';
      if (!binary_candidate || !Gimbal_ParseTrajectory(usb_rx_buf, rx_len)) {
        for (uint32_t i = 0U; i < rx_len; i++) {
          char ch = (char)usb_rx_buf[i];
          if (ch == '\r' || ch == '\n') {
            cmd[cmd_len] = '\0';
            if (cmd_len != 0U && cmd_overflow == 0U) {
              Gimbal_ParseCommand(cmd);
              /* Long commands such as GCAL must not leave the periodic deadline
               * several seconds behind and cause a telemetry burst. */
              tick = osKernelGetTickCount();
            }
            if (cmd_overflow != 0U) BSP_USB_Printf("err:COMMAND,too_long\r\n");
            cmd_len = 0U;
            cmd_overflow = 0U;
          } else if (cmd_overflow == 0U && cmd_len < sizeof(cmd) - 1U) {
            cmd[cmd_len++] = ch;
          } else {
            cmd_overflow = 1U;
          }
        }
      }
      BSP_USB_ReadyReceive(osThreadGetId());
    }

    float yaw_sp;
    float yaw_fb;
    float yaw_rate;
    float pid_angle_err;
    float yaw_rate_sp;
    float yaw_rate_err;
    float yaw_out;
    float yaw_encoder;
    float pit_sp;
    float pit_fb;
    float pit_rate;
    float pit_ctrl_angle_err;
    float pit_rate_sp;
    float pit_rate_err;
    float pit_out;
    float pit_encoder;
    float pit_control_target;
    float pit_ladrc_output;
    uint8_t pit_limit_active;
    uint8_t pitch_protect;
    uint8_t mode;
    uint8_t enable;
    float yaw_actual_current;
    float pit_actual_current;
    float yaw_temp;
    float pit_temp;
    float yaw_origin;
    uint8_t stop_reason;
    float yaw_friction_ff;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accl_x;
    float accl_y;
    float accl_z;
    float roll_fb;
    float bullet_speed_mps;
    uint32_t can_tx_last_success_tick;
    uint32_t can_tx_failure_count;
    uint16_t yaw_current_mismatch_count;
    uint16_t pit_current_mismatch_count;

    /* Copy one coherent sample; do not print while the kernel is locked. */
    osKernelLock();
    yaw_sp = gimbal.setpoint.eulr.yaw;
    yaw_fb = gimbal.feedback.eulr.imu.yaw;
    yaw_rate = gimbal.feedback.eulr_rate.z;
    pid_angle_err = gimbal.pid[GIMBAL_PID_YAW_ANGLE_IDX].last.err;
    yaw_rate_sp = gimbal.pid[GIMBAL_PID_YAW_ANGLE_IDX].last.out;
    yaw_rate_err = gimbal.pid[GIMBAL_PID_YAW_OMEGA_IDX].last.err;
    yaw_out = gimbal.out[GIMBAL_ACTR_YAW_IDX];
    yaw_encoder = gimbal.feedback.eulr.encoder.yaw;
    if (gimbal_ladrc_yaw_enable != 0U) {
      pid_angle_err = gimbal.ladrc[GIMBAL_ACTR_YAW_IDX].v1 -
                      gimbal.ladrc[GIMBAL_ACTR_YAW_IDX].z1;
      yaw_rate_sp = gimbal.ladrc[GIMBAL_ACTR_YAW_IDX].v2;
      yaw_rate_err = gimbal.ladrc[GIMBAL_ACTR_YAW_IDX].v2 -
                     gimbal.ladrc[GIMBAL_ACTR_YAW_IDX].z2;
    }
    pit_sp = gimbal.setpoint.eulr.pit;
    pit_fb = gimbal.feedback.eulr.imu.pit;
    pit_rate = gimbal.feedback.eulr_rate.y;
    pit_ctrl_angle_err = gimbal.pid[GIMBAL_PID_PIT_ANGLE_IDX].last.err;
    pit_rate_sp = gimbal.pid[GIMBAL_PID_PIT_ANGLE_IDX].last.out;
    pit_rate_err = gimbal.pid[GIMBAL_PID_PIT_OMEGA_IDX].last.err;
    pit_out = gimbal.out[GIMBAL_ACTR_PIT_IDX];
    pit_encoder = gimbal.feedback.eulr.encoder.pit;
    pit_control_target = gimbal.pit_control_target;
    pit_ladrc_output = gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].output;
    pit_limit_active = gimbal.pit_limit_active;
    if (gimbal_ladrc_pit_enable != 0U) {
      pit_ctrl_angle_err = gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].v1 -
                           gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].z1;
      pit_rate_sp = gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].v2;
      pit_rate_err = gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].v2 -
                     gimbal.ladrc[GIMBAL_ACTR_PIT_IDX].z2;
    }
    pitch_protect = gimbal_pitch_protect_active;
    mode = (uint8_t)gimbal.mode;
    enable = gimbal_test_enable;
    yaw_actual_current = gimbal.feedback.motor.yaw_current;
    pit_actual_current = gimbal.feedback.motor.pit_current;
    yaw_temp = gimbal.feedback.motor.yaw_temp;
    pit_temp = gimbal.feedback.motor.pit_temp;
    yaw_origin = gimbal_yaw_test_origin_rad;
    stop_reason = gimbal_test_stop_reason;
    yaw_friction_ff = gimbal_yaw_friction.output;
    gyro_x = gimbal.feedback.gyro.x;
    gyro_y = gimbal.feedback.gyro.y;
    gyro_z = gimbal.feedback.gyro.z;
    accl_x = gimbal.feedback.accl.x;
    accl_y = gimbal.feedback.accl.y;
    accl_z = gimbal.feedback.accl.z;
    roll_fb = gimbal.feedback.eulr.imu.rol;
    bullet_speed_mps = 0.0f; /* No measured bullet-speed source in this build. */
    can_tx_last_success_tick = gimbal_can_tx_health.last_success_tick;
    can_tx_failure_count = gimbal_can_tx_health.failure_count;
    yaw_current_mismatch_count =
        gimbal_actuator_mismatch_count[GIMBAL_ACTR_YAW_IDX];
    pit_current_mismatch_count =
        gimbal_actuator_mismatch_count[GIMBAL_ACTR_PIT_IDX];
    osKernelUnlock();

    const uint32_t telemetry_tick = osKernelGetTickCount();
    const float can_tx_age_ms =
        (float)(uint32_t)(telemetry_tick - can_tx_last_success_tick) * 1000.0f /
        (float)osKernelGetTickFreq();
    const float yaw_current_mismatch_ms =
        (float)yaw_current_mismatch_count * 1000.0f /
        (float)TASK_FREQ_CTRL_GIMBAL;
    const float pit_current_mismatch_ms =
        (float)pit_current_mismatch_count * 1000.0f /
        (float)TASK_FREQ_CTRL_GIMBAL;

    if (isfinite(yaw_fb)) {
      if (yaw_fb_ready == 0U) {
        yaw_fb_continuous = Angle_Degrees360(yaw_fb);
        yaw_fb_ready = 1U;
      } else {
        yaw_fb_continuous +=
            CircleError(yaw_fb, last_yaw_fb, M_2PI) * M_RAD2DEG_MULT;
      }
      last_yaw_fb = yaw_fb;
    }
    const float yaw_angle_error =
        FireWater_Safe(CircleError(yaw_sp, yaw_fb, M_2PI) * M_RAD2DEG_MULT);
    const float yaw_sp_continuous = yaw_fb_continuous + yaw_angle_error;
    /* Pitch Euler feedback is signed (-90..+90 deg); do not wrap it into
     * [0..360) for telemetry. The control loop remains in radians. */
    const float pit_fb_degrees = pit_fb * M_RAD2DEG_MULT;
    const float pit_angle_error =
        FireWater_Safe(CircleError(pit_sp, pit_fb, M_2PI) * M_RAD2DEG_MULT);
    const float pit_sp_degrees = pit_sp * M_RAD2DEG_MULT;

    /* FireWater yaw channels, in order:
     * target/feedback continuous angle (deg), angular speed (deg/s),
     * signed angle error (deg), active controller angle error (deg),
     * target/error speed
     * (deg/s), motor output (unchanged), encoder angle (0..360 deg),
     * pitch_protect, mode, enable. Pitch channels are then appended:
     * target/feedback angle (deg), angular speed (deg/s), signed angle error
     * (deg), controller angle error (deg), target/error speed (deg/s), motor
     * output, encoder angle (0..360 deg), LADRC enable and target valid.
     * Finally: limited Pitch reference (deg), limit active, LADRC output,
     * MCU time (s), actual GM6020 currents (A), temperatures (deg C), stop
     * reason, Yaw enable origin and travel (deg), then filtered BMI088 gyro
     * x/y/z (deg/s), accelerometer x/y/z (g), roll (deg), and measured bullet
     * speed (m/s) for the sp_vision_25 host adapter. CAN TX age (ms),
     * cumulative TX failures, and Yaw/Pitch current mismatch duration (ms)
     * are appended for actuator-path diagnosis.
     */
    BSP_USB_Printf(
        "yaw:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.6f,%u,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.3f,%u,%.3f,%.3f\r\n",
        FireWater_Safe(yaw_sp_continuous), FireWater_Safe(yaw_fb_continuous),
        FireWater_Safe(yaw_rate * M_RAD2DEG_MULT),
        yaw_angle_error,
        FireWater_Safe(pid_angle_err * M_RAD2DEG_MULT),
        FireWater_Safe(yaw_rate_sp * M_RAD2DEG_MULT),
        FireWater_Safe(yaw_rate_err * M_RAD2DEG_MULT), FireWater_Safe(yaw_out),
        Angle_Degrees360(yaw_encoder), (unsigned int)pitch_protect,
        (unsigned int)mode, (unsigned int)enable,
        FireWater_Safe(pit_sp_degrees), FireWater_Safe(pit_fb_degrees),
        FireWater_Safe(pit_rate * M_RAD2DEG_MULT), pit_angle_error,
        FireWater_Safe(pit_ctrl_angle_err * M_RAD2DEG_MULT),
        FireWater_Safe(pit_rate_sp * M_RAD2DEG_MULT),
        FireWater_Safe(pit_rate_err * M_RAD2DEG_MULT), FireWater_Safe(pit_out),
        Angle_Degrees360(pit_encoder),
        (unsigned int)gimbal_ladrc_pit_enable,
        (unsigned int)gimbal_target_input.pitch_valid,
        FireWater_Safe(pit_control_target * M_RAD2DEG_MULT),
        (unsigned int)pit_limit_active, FireWater_Safe(pit_ladrc_output),
        (float)osKernelGetTickCount() / (float)osKernelGetTickFreq(),
        FireWater_Safe(yaw_actual_current), FireWater_Safe(pit_actual_current),
        FireWater_Safe(yaw_temp), FireWater_Safe(pit_temp),
        (unsigned int)stop_reason,
        FireWater_Safe(yaw_origin * M_RAD2DEG_MULT),
        FireWater_Safe(CircleError(yaw_fb, yaw_origin, M_2PI) *
                       M_RAD2DEG_MULT),
        FireWater_Safe(yaw_friction_ff),
        FireWater_Safe(gyro_x * M_RAD2DEG_MULT),
        FireWater_Safe(gyro_y * M_RAD2DEG_MULT),
        FireWater_Safe(gyro_z * M_RAD2DEG_MULT),
        FireWater_Safe(accl_x), FireWater_Safe(accl_y), FireWater_Safe(accl_z),
        FireWater_Safe(roll_fb * M_RAD2DEG_MULT),
        FireWater_Safe(bullet_speed_mps), FireWater_Safe(can_tx_age_ms),
        (unsigned int)can_tx_failure_count,
        FireWater_Safe(yaw_current_mismatch_ms),
        FireWater_Safe(pit_current_mismatch_ms));

    tick += delay_tick;
    osDelayUntil(tick);
  }
}
