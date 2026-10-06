/* VOFA+ FireWater USB telemetry task. */

/* Includes ----------------------------------------------------------------- */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "bsp\usb.h"
#include "module\gimbal.h"
#include "task\user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
extern Gimbal_t gimbal;
extern volatile uint8_t gimbal_pitch_protect_active;
extern volatile uint8_t gimbal_test_enable;
extern volatile uint8_t gimbal_serial_control_active;
extern volatile uint32_t gimbal_serial_last_command_tick;
extern volatile float gimbal_yaw_test_origin_rad;
extern volatile float gimbal_pit_test_origin_rad;
extern volatile uint8_t gimbal_test_stop_reason;
extern volatile uint8_t gimbal_yaw_friction_enable;
extern volatile float gimbal_yaw_friction_static;
extern volatile float gimbal_yaw_friction_coulomb;
extern volatile float gimbal_yaw_friction_deadband_rad;
extern volatile float gimbal_yaw_friction_transition_rad_s;
extern volatile float gimbal_yaw_friction_slew_per_s;
extern volatile float gimbal_yaw_friction_output;
volatile float gimbal_yaw_target_rad = 0.0f;
volatile uint8_t gimbal_yaw_target_valid = 0U;
volatile float gimbal_pit_target_rad = 0.0f;
volatile uint8_t gimbal_pit_target_valid = 0U;
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
  gimbal_yaw_target_valid = 0U;
  gimbal_pit_target_valid = 0U;
  gimbal_ladrc_pit_enable = 0U;
  gimbal_test_stop_reason = reason;
  osKernelUnlock();
}

static void Gimbal_SerialTouch(void) {
  gimbal_serial_last_command_tick = osKernelGetTickCount();
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

/* Bench commands are ASCII lines terminated by CR or LF. */
static void Gimbal_ParseCommand(char *cmd) {
  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "ENABLE=0") == 0) {
    Gimbal_SerialStop(GIMBAL_TEST_STOP_COMMAND);
    BSP_USB_Printf("ack:STOP\r\n");
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    if (gimbal_serial_control_active != 0U) Gimbal_SerialTouch();
    return;
  }

  if (strcmp(cmd, "ENABLE=1") == 0) {
    const float yaw = gimbal.feedback.eulr.imu.yaw;
    const float pit = gimbal.feedback.eulr.imu.pit;
    if (!isfinite(yaw) || !isfinite(pit) || fabsf(pit) > 1.22173048f) {
      Gimbal_SerialStop(GIMBAL_TEST_STOP_INVALID_FEEDBACK);
      BSP_USB_Printf("err:ENABLE,feedback\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_test_origin_rad = yaw;
    gimbal_pit_test_origin_rad = pit;
    gimbal_yaw_target_rad = yaw;
    gimbal_pit_target_rad = pit;
    gimbal_yaw_target_valid = 1U;
    gimbal_pit_target_valid = 1U;
    gimbal_ladrc_yaw_enable = 1U;
    gimbal_ladrc_pit_enable = 0U;
    gimbal_test_stop_reason = GIMBAL_TEST_STOP_NONE;
    gimbal_serial_control_active = 1U;
    Gimbal_SerialTouch();
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
    gimbal_yaw_target_rad = gimbal.feedback.eulr.imu.yaw;
    gimbal_pit_target_rad = gimbal.feedback.eulr.imu.pit;
    gimbal_yaw_target_valid = 1U;
    gimbal_pit_target_valid = 1U;
    Gimbal_SerialTouch();
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
    gimbal_yaw_target_rad = gimbal_yaw_test_origin_rad;
    gimbal_pit_target_rad = gimbal_pit_test_origin_rad;
    gimbal_yaw_target_valid = 1U;
    gimbal_pit_target_valid = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:CENTER\r\n");
    return;
  }

  if (strcmp(cmd, "PITCH=0") == 0) {
    gimbal_ladrc_pit_enable = 0U;
    Gimbal_SerialTouch();
    BSP_USB_Printf("ack:PITCH=0\r\n");
    return;
  }
  if (strcmp(cmd, "PITCH=1") == 0) {
    if (gimbal_serial_control_active == 0U) {
      BSP_USB_Printf("err:PITCH,disabled\r\n");
      return;
    }
    osKernelLock();
    gimbal_pit_target_rad = gimbal.feedback.eulr.imu.pit;
    gimbal_pit_target_valid = 1U;
    gimbal_ladrc_pit_enable = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:PITCH=1\r\n");
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
        "yff=%u,%.4f,%.4f,%.3f,%.3f,%.3f\r\n",
        (unsigned int)gimbal_test_enable,
        (unsigned int)gimbal_serial_control_active,
        (unsigned int)gimbal_test_stop_reason,
        gimbal.feedback.eulr.imu.yaw * M_RAD2DEG_MULT, yaw_travel,
        gimbal_ladrc_yaw_param.r, gimbal_ladrc_yaw_param.b0,
        gimbal_ladrc_yaw_param.wc, gimbal_ladrc_yaw_param.w0,
        gimbal_ladrc_yaw_param.out_limit, gimbal_ladrc_pit_param.r,
        gimbal_ladrc_pit_param.b0, gimbal_ladrc_pit_param.wc,
        gimbal_ladrc_pit_param.w0, gimbal_ladrc_pit_param.out_limit,
        (unsigned int)gimbal_yaw_friction_enable,
        gimbal_yaw_friction_static, gimbal_yaw_friction_coulomb,
        gimbal_yaw_friction_deadband_rad * M_RAD2DEG_MULT,
        gimbal_yaw_friction_transition_rad_s * M_RAD2DEG_MULT,
        gimbal_yaw_friction_slew_per_s);
    return;
  }

  if (strncmp(cmd, "YPARAM=", 7) == 0) {
    LADRC_Params_t p;
    if (Gimbal_ParseLADRCParams(cmd + 7, &p) == 0U ||
        Gimbal_LADRCParamsSafe(&p, GIMBAL_TEST_OUTPUT_MAX) == 0U) {
      BSP_USB_Printf("err:YPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_ladrc_yaw_param = p;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:YPARAM\r\n");
    return;
  }

  if (strncmp(cmd, "PPARAM=", 7) == 0) {
    LADRC_Params_t p;
    if (Gimbal_ParseLADRCParams(cmd + 7, &p) == 0U ||
        Gimbal_LADRCParamsSafe(&p, GIMBAL_TEST_PIT_OUTPUT_MAX) == 0U) {
      BSP_USB_Printf("err:PPARAM,range\r\n");
      return;
    }
    osKernelLock();
    gimbal_ladrc_pit_param = p;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:PPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "YFF=0") == 0 || strcmp(cmd, "YFF=1") == 0) {
    gimbal_yaw_friction_enable = cmd[4] == '1' ? 1U : 0U;
    Gimbal_SerialTouch();
    BSP_USB_Printf("ack:YFF=%u\r\n",
                   (unsigned int)gimbal_yaw_friction_enable);
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
    gimbal_yaw_friction_static = value[0];
    gimbal_yaw_friction_coulomb = value[1];
    gimbal_yaw_friction_deadband_rad = value[2] * M_DEG2RAD_MULT;
    gimbal_yaw_friction_transition_rad_s = value[3] * M_DEG2RAD_MULT;
    gimbal_yaw_friction_slew_per_s = value[4];
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:YFFPARAM\r\n");
    return;
  }

  float degree;
  if (strncmp(cmd, "YAWREL=", 7) == 0) {
    if (gimbal_serial_control_active == 0U ||
        Gimbal_ParseNumber(cmd + 7, &degree) == 0U ||
        fabsf(degree * M_DEG2RAD_MULT) > GIMBAL_TEST_YAW_STEP_MAX_RAD) {
      BSP_USB_Printf("err:YAWREL,state_or_range\r\n");
      return;
    }
    float target = gimbal.feedback.eulr.imu.yaw;
    CircleAdd(&target, degree * M_DEG2RAD_MULT, M_2PI);
    if (fabsf(CircleError(target, gimbal_yaw_test_origin_rad, M_2PI)) >
        GIMBAL_TEST_YAW_TARGET_MAX_RAD) {
      BSP_USB_Printf("err:YAWREL,travel\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_target_rad = target;
    gimbal_yaw_target_valid = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:YAWREL=%.3f\r\n", degree);
    return;
  }

  if (strncmp(cmd, "PITREL=", 7) == 0) {
    if (gimbal_serial_control_active == 0U ||
        gimbal_ladrc_pit_enable == 0U ||
        Gimbal_ParseNumber(cmd + 7, &degree) == 0U ||
        fabsf(degree * M_DEG2RAD_MULT) > GIMBAL_TEST_PIT_STEP_MAX_RAD) {
      BSP_USB_Printf("err:PITREL,state_or_range\r\n");
      return;
    }
    const float target =
        gimbal.feedback.eulr.imu.pit + degree * M_DEG2RAD_MULT;
    if (fabsf(target - gimbal_pit_test_origin_rad) >
            GIMBAL_TEST_PIT_TARGET_MAX_RAD ||
        fabsf(target) > 1.22173048f) {
      BSP_USB_Printf("err:PITREL,travel\r\n");
      return;
    }
    osKernelLock();
    gimbal_pit_target_rad = target;
    gimbal_pit_target_valid = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:PITREL=%.3f\r\n", degree);
    return;
  }

  if (strncmp(cmd, "YAW=", 4) == 0) {
    if (gimbal_serial_control_active == 0U ||
        Gimbal_ParseNumber(cmd + 4, &degree) == 0U || degree < -180.0f ||
        degree > 360.0f) {
      BSP_USB_Printf("err:YAW,state_or_range\r\n");
      return;
    }
    if (degree < 0.0f) degree += 360.0f;
    if (degree >= 360.0f) degree -= 360.0f;
    const float target = degree * M_DEG2RAD_MULT;
    if (fabsf(CircleError(target, gimbal_yaw_test_origin_rad, M_2PI)) >
        GIMBAL_TEST_YAW_TARGET_MAX_RAD) {
      BSP_USB_Printf("err:YAW,travel\r\n");
      return;
    }
    osKernelLock();
    gimbal_yaw_target_rad = target;
    gimbal_yaw_target_valid = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:YAW=%.3f\r\n", degree);
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

  char cmd[32] = {0};
  uint8_t cmd_len = 0U;
  uint8_t cmd_overflow = 0U;
  float yaw_fb_continuous = 0.0f;
  float last_yaw_fb = 0.0f;
  uint8_t yaw_fb_ready = 0U;
  float pit_display_offset = 0.0f;
  uint8_t pit_fb_ready = 0U;
  uint32_t tick = osKernelGetTickCount();
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.info = osThreadGetStackSpace(osThreadGetId());
#endif
    if (osThreadFlagsWait(SIGNAL_BSP_USB_BUF_RECV, osFlagsWaitAny, 0U) <
        osFlagsError) {
      for (uint32_t i = 0;
           i < BSP_USB_MAX_RX_LEN && usb_rx_buf[i] != '\0'; i++) {
        char ch = (char)usb_rx_buf[i];
        if (ch == '\r' || ch == '\n') {
          cmd[cmd_len] = '\0';
          if (cmd_len != 0U && cmd_overflow == 0U) Gimbal_ParseCommand(cmd);
          cmd_len = 0U;
          cmd_overflow = 0U;
        } else if (cmd_overflow == 0U && cmd_len < sizeof(cmd) - 1U) {
          cmd[cmd_len++] = ch;
        } else {
          cmd_overflow = 1U;
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

    /* Copy one coherent sample; do not print while the kernel is locked. */
    osKernelLock();
    yaw_sp = gimbal.setpoint.eulr.yaw;
    yaw_fb = gimbal.feedback.eulr.imu.yaw;
    yaw_rate = gimbal.feedback.gyro.z;
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
    pit_rate = gimbal.feedback.gyro.y;
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
    yaw_friction_ff = gimbal_yaw_friction_output;
    osKernelUnlock();

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
    /* Choose the display turn once. Crossing zero must not move the target
     * trace by 360 deg along with the feedback. Control remains in rad. */
    if (pit_fb_ready == 0U && isfinite(pit_fb)) {
      pit_display_offset = pit_fb < 0.0f ? 360.0f : 0.0f;
      pit_fb_ready = 1U;
    }
    const float pit_fb_degrees = pit_fb * M_RAD2DEG_MULT + pit_display_offset;
    const float pit_angle_error =
        FireWater_Safe(CircleError(pit_sp, pit_fb, M_2PI) * M_RAD2DEG_MULT);
    const float pit_sp_degrees = pit_sp * M_RAD2DEG_MULT + pit_display_offset;

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
     * reason, Yaw enable origin and travel (deg).
     */
    BSP_USB_Printf(
        "yaw:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.6f,%u,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f\r\n",
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
        (unsigned int)gimbal_pit_target_valid,
        FireWater_Safe(pit_control_target * M_RAD2DEG_MULT + pit_display_offset),
        (unsigned int)pit_limit_active, FireWater_Safe(pit_ladrc_output),
        (float)osKernelGetTickCount() / (float)osKernelGetTickFreq(),
        FireWater_Safe(yaw_actual_current), FireWater_Safe(pit_actual_current),
        FireWater_Safe(yaw_temp), FireWater_Safe(pit_temp),
        (unsigned int)stop_reason,
        FireWater_Safe(yaw_origin * M_RAD2DEG_MULT),
        FireWater_Safe(CircleError(yaw_fb, yaw_origin, M_2PI) *
                       M_RAD2DEG_MULT),
        FireWater_Safe(yaw_friction_ff));

    tick += delay_tick;
    osDelayUntil(tick);
  }
}
