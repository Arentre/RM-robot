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
extern volatile uint8_t gimbal_yaw_chirp_active;
extern volatile uint8_t gimbal_yaw_chirp_down;
extern volatile uint32_t gimbal_yaw_chirp_start_tick;
extern volatile uint8_t gimbal_yaw_dither_enable;
extern volatile float gimbal_yaw_dither_amplitude;
extern volatile float gimbal_yaw_dither_frequency_hz;
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
  gimbal_yaw_chirp_active = 0U;
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

/* Bench commands are ASCII lines terminated by CR or LF. */
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
    gimbal_yaw_chirp_active = 0U;
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
    gimbal_yaw_chirp_active = 0U;
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
    gimbal_yaw_chirp_active = 0U;
    gimbal_yaw_target_rad = gimbal_yaw_test_origin_rad;
    gimbal_pit_target_rad = gimbal_pit_test_origin_rad;
    gimbal_yaw_target_valid = 1U;
    gimbal_pit_target_valid = 1U;
    Gimbal_SerialTouch();
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
    gimbal_yaw_chirp_down = strcmp(cmd, "YCHIRP=DOWN") == 0 ? 1U : 0U;
    gimbal_yaw_chirp_start_tick = osKernelGetTickCount();
    gimbal_yaw_chirp_active = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:YCHIRP=%s\r\n",
                   gimbal_yaw_chirp_down != 0U ? "DOWN" : "UP");
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
        "yff=%u,%.4f,%.4f,%.3f,%.3f,%.3f,"
        "pff=%u,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,"
        "pgrav=%u,%.4f,%.3f,%.4f\r\n",
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
        gimbal_pitch_gravity.output);
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
    gimbal_yaw_friction.enable = cmd[4] == '1' ? 1U : 0U;
    Gimbal_SerialTouch();
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
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:YFFPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "PFF=0") == 0 || strcmp(cmd, "PFF=1") == 0) {
    gimbal_pitch_friction.enable = cmd[4] == '1' ? 1U : 0U;
    Gimbal_SerialTouch();
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
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:PFFPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "PGRAV=0") == 0 || strcmp(cmd, "PGRAV=1") == 0) {
    gimbal_pitch_gravity.enable = cmd[6] == '1' ? 1U : 0U;
    Gimbal_SerialTouch();
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
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:PGRAVPARAM\r\n");
    return;
  }

  if (strcmp(cmd, "YDITHER=0") == 0 || strcmp(cmd, "YDITHER=1") == 0) {
    gimbal_yaw_dither_enable = cmd[8] == '1' ? 1U : 0U;
    Gimbal_SerialTouch();
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
    Gimbal_SerialTouch();
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
    if (Gimbal_ParseNumber(number, &degree) == 0U ||
        (yaw_relative && fabsf(degree * M_DEG2RAD_MULT) >
                             GIMBAL_TEST_YAW_STEP_MAX_RAD) ||
        (!yaw_relative && (degree < -180.0f || degree > 360.0f))) {
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
    if (!isfinite(feedback) || !isfinite(gimbal_yaw_test_origin_rad)) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,feedback\r\n", name);
      return;
    }
    float target = yaw_relative ? feedback : 0.0f;
    CircleAdd(&target, degree * M_DEG2RAD_MULT, M_2PI);
    if (fabsf(CircleError(target, gimbal_yaw_test_origin_rad, M_2PI)) >
        GIMBAL_TEST_YAW_TARGET_MAX_RAD) {
      osKernelUnlock();
      BSP_USB_Printf("err:%s,travel\r\n", name);
      return;
    }
    gimbal_yaw_chirp_active = 0U;
    gimbal_yaw_target_rad = target;
    gimbal_yaw_target_valid = 1U;
    Gimbal_SerialTouch();
    osKernelUnlock();
    BSP_USB_Printf("ack:%s=%.3f\r\n", name,
                   yaw_relative ? degree : Angle_Degrees360(target));
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
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accl_x;
    float accl_y;
    float accl_z;

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
    yaw_friction_ff = gimbal_yaw_friction.output;
    gyro_x = gimbal.feedback.gyro.x;
    gyro_y = gimbal.feedback.gyro.y;
    gyro_z = gimbal.feedback.gyro.z;
    accl_x = gimbal.feedback.accl.x;
    accl_y = gimbal.feedback.accl.y;
    accl_z = gimbal.feedback.accl.z;
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
     * reason, Yaw enable origin and travel (deg), then raw BMI088 gyro
     * x/y/z (deg/s) and accelerometer x/y/z (g).
     */
    BSP_USB_Printf(
        "yaw:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.6f,%u,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
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
        FireWater_Safe(accl_x), FireWater_Safe(accl_y), FireWater_Safe(accl_z));

    tick += delay_tick;
    osDelayUntil(tick);
  }
}
