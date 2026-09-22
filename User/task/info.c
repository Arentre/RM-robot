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

/* Commands: YAW=300\n or PIT=-20\n (degrees). */
static void Gimbal_ParseCommand(char *cmd) {
  const uint8_t is_yaw = strncmp(cmd, "YAW=", 4) == 0;
  const uint8_t is_pit = strncmp(cmd, "PIT=", 4) == 0;
  if (is_yaw == 0U && is_pit == 0U) return;

  char *end;
  float degree = strtof(cmd + 4, &end);
  if (end == cmd + 4 || *end != '\0' || !isfinite(degree) ||
      degree < -180.0f || degree > 360.0f)
    return;

  if (is_yaw != 0U) {
    if (degree < 0.0f) degree += 360.0f;
    if (degree >= 360.0f) degree -= 360.0f;
    gimbal_yaw_target_rad = degree * M_DEG2RAD_MULT;
    gimbal_yaw_target_valid = 1U;
  } else {
    /* Accept PIT=350 as -10 deg, but never command the Euler singular area. */
    if (degree > 180.0f) degree -= 360.0f;
    if (degree < -70.0f || degree > 70.0f) return;
    gimbal_pit_target_rad = degree * M_DEG2RAD_MULT;
    gimbal_pit_target_valid = 1U;
  }
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
    uint8_t pitch_protect;
    uint8_t mode;
    uint8_t enable;

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
    const float pit_fb_degrees = Angle_Degrees360(pit_fb);
    const float pit_angle_error =
        FireWater_Safe(CircleError(pit_sp, pit_fb, M_2PI) * M_RAD2DEG_MULT);
    const float pit_sp_degrees = pit_fb_degrees + pit_angle_error;

    /* FireWater yaw channels, in order:
     * target/feedback continuous angle (deg), angular speed (deg/s),
     * signed angle error (deg), active controller angle error (deg),
     * target/error speed
     * (deg/s), motor output (unchanged), encoder angle (0..360 deg),
     * pitch_protect, mode, enable. Pitch channels are then appended:
     * target/feedback angle (deg), angular speed (deg/s), signed angle error
     * (deg), controller angle error (deg), target/error speed (deg/s), motor
     * output, encoder angle (0..360 deg), LADRC enable and target valid.
     */
    BSP_USB_Printf(
        "yaw:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u\r\n",
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
        (unsigned int)gimbal_pit_target_valid);

    tick += delay_tick;
    osDelayUntil(tick);
  }
}
