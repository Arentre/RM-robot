/*
  姿态解算任务。

  控制IMU加热到指定温度防止温漂，收集IMU数据给AHRS算法。

  收集BMI088的数据，解算后得到四元数，转换为欧拉角之后放到消息队列中，
  等待其他任务取用。
*/

/* Includes ----------------------------------------------------------------- */
#include <math.h>
#include <string.h>

#include "bsp/mm.h"
#include "bsp/pwm.h"
#include "bsp/usb.h"
#include "component/ahrs.h"
#include "component/pid.h"
#include "module/control_params.h"
#include "device/bmi088.h"
#include "task/user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */

#ifdef DEBUG
BMI088_t bmi088;

AHRS_t gimbal_ahrs;
AHRS_Eulr_t eulr_to_send;

KPID_t imu_temp_ctrl_pid;
#else
static BMI088_t bmi088;

static AHRS_t gimbal_ahrs;
static AHRS_Eulr_t eulr_to_send;

static KPID_t imu_temp_ctrl_pid;
#endif

volatile float gimbal_imu_temperature_c = 0.0f;
volatile float gimbal_imu_heater_duty = 0.0f;
volatile uint32_t gimbal_imu_io_error_count = 0U;

#define IMU_ACCEL_LOWPASS_CUTOFF_HZ (40.0f)
#define IMU_CONTROL_GYRO_LOWPASS_CUTOFF_HZ (80.0f)
#define IMU_TARGET_TEMPERATURE_C (40.0f)
#define IMU_HEAT_CUTOFF_C (45.0f)

static void AttiEsti_UpdateAccelFilter(AHRS_Accl_t *filtered,
                                       const AHRS_Accl_t *sample,
                                       uint8_t *initialised) {
  if (*initialised == 0U) {
    *filtered = *sample;
    *initialised = 1U;
    return;
  }

  const float dt = 1.0f / BMI088_GetAcclUpdateFreq(&bmi088);
  const float rc = 1.0f / (2.0f * M_PI * IMU_ACCEL_LOWPASS_CUTOFF_HZ);
  const float alpha = dt / (rc + dt);
  filtered->x += alpha * (sample->x - filtered->x);
  filtered->y += alpha * (sample->y - filtered->y);
  filtered->z += alpha * (sample->z - filtered->z);
}

static void AttiEsti_UpdateGyroFilter(AHRS_Gyro_t *filtered,
                                      const AHRS_Gyro_t *sample, float alpha,
                                      uint8_t *initialised) {
  if (*initialised == 0U) {
    *filtered = *sample;
    *initialised = 1U;
    return;
  }

  filtered->x += alpha * (sample->x - filtered->x);
  filtered->y += alpha * (sample->y - filtered->y);
  filtered->z += alpha * (sample->z - filtered->z);
}

/* Private function  -------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * \brief 姿态解算
 *
 * \param argument 未使用
 */
void Task_AttiEsti(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  /* 初始化设备 */
  /* Use the offset stored by GCAL. An erased/corrupt Flash value must never
   * enter the control loop; a valid static offset is comfortably below
   * 10 deg/s on BMI088. */
  BMI088_Cali_t *const cali = &task_runtime.cfg.cali.bmi088;
  const float max_offset = 10.0f * M_DEG2RAD_MULT;
  if (!isfinite(cali->gyro_offset.x) ||
      fabsf(cali->gyro_offset.x) > max_offset)
    cali->gyro_offset.x = 0.0f;
  if (!isfinite(cali->gyro_offset.y) ||
      fabsf(cali->gyro_offset.y) > max_offset)
    cali->gyro_offset.y = 0.0f;
  if (!isfinite(cali->gyro_offset.z) ||
      fabsf(cali->gyro_offset.z) > max_offset)
    cali->gyro_offset.z = 0.0f;
  if (BMI088_Init(&bmi088, cali) != DEVICE_OK) {
    gimbal_imu_io_error_count++;
    /* No valid sample will be published; the control watchdog blocks enable. */
    osThreadExit();
  }

  /* 初始化姿态解算算法 */
  AHRS_Init(&gimbal_ahrs, BMI088_GetUpdateFreq(&bmi088));

  /* 初始化IMU温度控制PID，防止温漂 */
  PID_Init(&imu_temp_ctrl_pid, KPID_MODE_NO_D,
           BMI088_GetAcclUpdateFreq(&bmi088), &imu_temp_ctrl_pid_param);

  /* Begin warm-up immediately. The first valid temperature sample takes over. */
  gimbal_imu_heater_duty = 1.0f;
  BSP_PWM_Set(BSP_PWM_IMU_HEAT, gimbal_imu_heater_duty);

  AHRS_Accl_t filtered_accl = {0.0f, 0.0f, 0.0f};
  AHRS_Gyro_t control_gyro = {0.0f, 0.0f, 0.0f};
  uint8_t accel_filter_initialised = 0U;
  uint8_t gyro_filter_initialised = 0U;
  GimbalIMUSample_t sample;
  memset(&sample, 0, sizeof(sample));
  const float accel_dt = 1.0f / BMI088_GetAcclUpdateFreq(&bmi088);
  const float gyro_filter_alpha =
      1.0f - expf(-M_2PI * IMU_CONTROL_GYRO_LOWPASS_CUTOFF_HZ /
                  BMI088_GetUpdateFreq(&bmi088));

  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.atti_esti =
        osThreadGetStackSpace(osThreadGetId());
#endif
    const uint32_t flags = BMI088_WaitNew();
    if ((flags & osFlagsError) != 0U) {
      gimbal_imu_io_error_count++;
      osDelay(1U);
      continue;
    }
    uint8_t gyro_updated = 0U;

    /* Gyro owns the 1 kHz attitude clock. Read it first because both sensors
     * share one SPI/DMA channel. */
    if ((flags & SIGNAL_BMI088_GYRO_NEW_DATA) != 0U) {
      if (BMI088_GyroStartDmaRecv() != DEVICE_OK ||
          (BMI088_GyroWaitDmaCplt() & osFlagsError) != 0U ||
          BMI088_ParseGyro(&bmi088) != DEVICE_OK) {
        gimbal_imu_io_error_count++;
        continue;
      }
      sample.gyro_tick = osKernelGetTickCount();
      AttiEsti_UpdateGyroFilter(&control_gyro, &bmi088.gyro,
                                gyro_filter_alpha,
                                &gyro_filter_initialised);
      gyro_updated = 1U;
    }

    /* Acceleration and temperature are sampled independently at 800 Hz. */
    if ((flags & SIGNAL_BMI088_ACCL_NEW_DATA) != 0U) {
      if (BMI088_AcclStartDmaRecv() != DEVICE_OK ||
          (BMI088_AcclWaitDmaCplt() & osFlagsError) != 0U ||
          BMI088_ParseAccl(&bmi088) != DEVICE_OK) {
        gimbal_imu_io_error_count++;
        continue;
      }
      sample.accl_tick = osKernelGetTickCount();
      AttiEsti_UpdateAccelFilter(&filtered_accl, &bmi088.accl,
                                 &accel_filter_initialised);
      gimbal_imu_temperature_c = bmi088.temp;
      if (!isfinite(bmi088.temp) || bmi088.temp >= IMU_HEAT_CUTOFF_C) {
        gimbal_imu_heater_duty = 0.0f;
        PID_ResetIntegral(&imu_temp_ctrl_pid);
      } else {
        gimbal_imu_heater_duty =
            PID_Calc(&imu_temp_ctrl_pid, IMU_TARGET_TEMPERATURE_C,
                     bmi088.temp, 0.0f, accel_dt);
        if (gimbal_imu_heater_duty < 0.0f)
          gimbal_imu_heater_duty = 0.0f;
      }
      BSP_PWM_Set(BSP_PWM_IMU_HEAT, gimbal_imu_heater_duty);
    }

    if (gyro_updated == 0U || accel_filter_initialised == 0U) continue;

    if (AHRS_Update(&gimbal_ahrs, &filtered_accl, &bmi088.gyro) != 0 ||
        AHRS_GetEulr(&eulr_to_send, &gimbal_ahrs) != 0) {
      gimbal_imu_io_error_count++;
      continue;
    }

    sample.eulr = eulr_to_send;
    sample.gyro = control_gyro;
    sample.accl = bmi088.accl;
    osMessageQueueReset(task_runtime.msgq.ai.quat);
    osMessageQueuePut(task_runtime.msgq.ai.quat, &gimbal_ahrs.quat, 0, 0);
    osMessageQueueReset(task_runtime.msgq.gimbal.sample);
    osMessageQueuePut(task_runtime.msgq.gimbal.sample, &sample, 0, 0);

    /* 将需要与其他任务分享的数据放到消息队列中 */

  }
}
