#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>

#include "FreeRTOS.h"
#include "module/config.h"
#include "task.h"

/* Exported constants ------------------------------------------------------- */

/* 所有任务都要define一个“任务运行频率”和“初始化延时” */
#define TASK_FREQ_CTRL_CHASSIS (500u)
#define TASK_FREQ_CTRL_GIMBAL (500u)
#define TASK_FREQ_CTRL_SHOOT (500)
#define TASK_FREQ_CTRL_COMMAND (500u)
#define TASK_FREQ_INFO (50u)
#define TASK_FREQ_CAN (1000u)
#define TASK_FREQ_AI (250u)

#define TASK_INIT_DELAY_INFO (500u)
#define TASK_INIT_DELAY_AI (400u)

typedef struct {
  /* Legacy success fields mean HAL enqueue success, NOT motor acknowledgement. */
  volatile uint32_t last_success_tick;
  volatile uint32_t success_count;
  volatile uint32_t failure_count;
  volatile uint32_t tx_status_register; /* Raw TSR snapshot; shared by all groups. */
  volatile uint32_t bus_esr; /* Hardware ESR: bus-off, error counters and LEC. */
  volatile uint32_t free_mailboxes;
  volatile uint32_t output_age_ms;
} GimbalCANTxHealth_t;

extern GimbalCANTxHealth_t gimbal_can_tx_health;

/* One queue item binds attitude/rate to the actual sensor acquisition times. */
typedef struct {
  AHRS_Eulr_t eulr;
  AHRS_Gyro_t gyro;
  AHRS_Accl_t accl;
  uint32_t gyro_tick;
  uint32_t accl_tick;
} GimbalIMUSample_t;

extern volatile uint32_t gimbal_imu_io_error_count;

/* Exported defines --------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */

/* Serial/vision gimbal commands are published as one protected snapshot.
 * Writers update every related field under the RTOS kernel lock, then bump
 * revision.  The 500 Hz control task accepts only complete, validated
 * revisions. */
typedef struct {
  float yaw_rad;
  float pitch_rad;
  float yaw_velocity_rad_s;
  float yaw_acceleration_rad_s2;
  float pitch_velocity_rad_s;
  float pitch_acceleration_rad_s2;
  uint32_t update_tick;
  uint32_t revision;
  uint8_t yaw_valid;
  uint8_t pitch_valid;
  uint8_t trajectory_active;
} GimbalTargetInput_t;

typedef struct {
  /* 各任务，也可以叫做线程 */
  struct {
    osThreadId_t command;
    osThreadId_t ctrl_chassis;
    osThreadId_t ctrl_gimbal;
    osThreadId_t ctrl_shoot;
    osThreadId_t info;
    osThreadId_t can;
    osThreadId_t atti_esti;
    osThreadId_t ai;
    osThreadId_t rc;
  } thread;

  struct {
    struct { osMessageQueueId_t sample; } gimbal;
    struct {
      struct { osMessageQueueId_t host, rc; } raw;
      osMessageQueueId_t chassis, gimbal, shoot, ai;
    } cmd;
    struct {
      struct { osMessageQueueId_t chassis, gimbal, shoot; } output;
      struct { osMessageQueueId_t chassis, gimbal, shoot; } feedback;
    } can;
    struct { osMessageQueueId_t quat; } ai;
  } msgq;

  /* 机器人状态 */
  struct {
    float battery;
    float vbat;
    float cpu_temp;
  } status;

  Config_t cfg; /* 机器人配置 */

#ifdef DEBUG
  struct {
    UBaseType_t command;
    UBaseType_t ctrl_chassis;
    UBaseType_t ctrl_gimbal;
    UBaseType_t ctrl_shoot;
    UBaseType_t info;
    UBaseType_t can;
    UBaseType_t atti_esti;
    UBaseType_t ai;
    UBaseType_t rc;
  } stack_water_mark; /* stack使用 */

  struct {
    float command;
    float ctrl_chassis;
    float ctrl_gimbal;
    float ctrl_shoot;
    float info;
    float can;
    float atti_esti;
    float ai;
    float rc;
  } freq; /* 任务运行频率 */

  struct {
    float command;
    float ctrl_chassis;
    float ctrl_gimbal;
    float ctrl_shoot;
    float info;
    float can;
    float atti_esti;
    float ai;
    float rc;
  } last_up_time; /* 任务最近运行时间 */
#endif
} Task_Runtime_t;

extern Task_Runtime_t task_runtime;
extern volatile float gimbal_imu_temperature_c;
extern volatile float gimbal_imu_heater_duty;
extern volatile GimbalTargetInput_t gimbal_target_input;

extern const osThreadAttr_t attr_init;

extern const osThreadAttr_t attr_command;
extern const osThreadAttr_t attr_ctrl_chassis;
extern const osThreadAttr_t attr_ctrl_gimbal;
extern const osThreadAttr_t attr_ctrl_shoot;
extern const osThreadAttr_t attr_info;
extern const osThreadAttr_t attr_can;
extern const osThreadAttr_t attr_atti_esti;
extern const osThreadAttr_t attr_ai;
extern const osThreadAttr_t attr_rc;

/* Exported functions prototypes -------------------------------------------- */
void Task_Init(void *argument);

void Task_Command(void *argument);
void Task_CtrlChassis(void *argument);
void Task_CtrlGimbal(void *argument);
void Task_CtrlShoot(void *argument);
void Task_Info(void *argument);
void Task_Can(void *argument);
void Task_AttiEsti(void *argument);
void Task_Ai(void *argument);
void Task_RC(void *argument);

#ifdef __cplusplus
}
#endif
