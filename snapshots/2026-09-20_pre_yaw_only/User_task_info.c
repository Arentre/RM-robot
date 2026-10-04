/* VOFA+ FireWater USB telemetry task. */

/* Includes ----------------------------------------------------------------- */
#include <math.h>

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
/* Private function --------------------------------------------------------- */
static float FireWater_Safe(float value) {
  return isfinite(value) ? value : 0.0f;
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

  uint32_t tick = osKernelGetTickCount();
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.info = osThreadGetStackSpace(osThreadGetId());
#endif
    float pit_sp;
    float pit_fb;
    float pit_rate;
    float pid_angle_err;
    float pit_rate_sp;
    float pit_rate_err;
    float pit_out;
    float pit_encoder;
    uint8_t pitch_protect;
    uint8_t mode;
    uint8_t enable;

    /* Copy one coherent sample; do not print while the kernel is locked. */
    osKernelLock();
    pit_sp = gimbal.setpoint.eulr.pit;
    pit_fb = gimbal.feedback.eulr.imu.pit;
    pit_rate = gimbal.feedback.gyro.y;
    pid_angle_err = gimbal.pid[GIMBAL_PID_PIT_ANGLE_IDX].last.err;
    pit_rate_sp = gimbal.pid[GIMBAL_PID_PIT_ANGLE_IDX].last.out;
    pit_rate_err = gimbal.pid[GIMBAL_PID_PIT_OMEGA_IDX].last.err;
    pit_out = gimbal.out[GIMBAL_ACTR_PIT_IDX];
    pit_encoder = gimbal.feedback.eulr.encoder.pit;
    pitch_protect = gimbal_pitch_protect_active;
    mode = (uint8_t)gimbal.mode;
    enable = gimbal_test_enable;
    osKernelUnlock();

    /* FireWater pitch channels, in order:
     * pit_sp, pit_fb, pit_rate, direct_angle_err, pid_angle_err,
     * pit_rate_sp, pit_rate_err, pit_out, pit_encoder,
     * pitch_protect, mode, enable.
     */
    BSP_USB_Printf(
        "gimbal2:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u\r\n",
        FireWater_Safe(pit_sp), FireWater_Safe(pit_fb),
        FireWater_Safe(pit_rate), FireWater_Safe(pit_sp - pit_fb),
        FireWater_Safe(pid_angle_err), FireWater_Safe(pit_rate_sp),
        FireWater_Safe(pit_rate_err), FireWater_Safe(pit_out),
        FireWater_Safe(pit_encoder), (unsigned int)pitch_protect,
        (unsigned int)mode, (unsigned int)enable);

    tick += delay_tick;
    osDelayUntil(tick);
  }
}

