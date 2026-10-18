/* Minimal chassis test adapter: CRSF -> CMD_ChassisCmd_t. */

/* Includes ----------------------------------------------------------------- */
#include <string.h>

#include "device\crsf.h"
#include "task\user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
#ifdef DEBUG
CRSF_t crsf;
CMD_RC_t cmd_rc;
CMD_ChassisCmd_t rc_chassis_cmd;
CMD_GimbalCmd_t rc_gimbal_cmd;
#else
static CRSF_t crsf;
static CMD_RC_t cmd_rc;
static CMD_ChassisCmd_t rc_chassis_cmd;
static CMD_GimbalCmd_t rc_gimbal_cmd;
#endif

/* Private function --------------------------------------------------------- */
#define CHASSIS_TEST_INPUT_SCALE (0.25f)
/* 固定归一化旋转速度，可在此调整；CH8（SC）仅选择方向。 */
#define CHASSIS_TEST_ROTATION_SPEED (2.75f)
/* 底盘排查：置 0 禁用遥控自旋，保留左摇杆平移。 */
#define CHASSIS_TEST_ROTATION_ENABLE (0U)
#define GIMBAL_TEST_YAW_RATE_RAD_S (4.0f)
#define GIMBAL_TEST_PITCH_RATE_RAD_S (2.0f)
#define GIMBAL_TEST_DT_MAX_S (0.05f)

static uint32_t gimbal_last_update_tick;

static void RC_BuildChassisCommand(const CMD_RC_t *rc,
                                   CMD_ChassisCmd_t *command) {
  memset(command, 0, sizeof(*command));
  if (rc->sw_r == CMD_SW_ERR) {
    command->mode = CHASSIS_MODE_RELAX;
    return;
  }

  /* SC 三段开关（CH8）决定 Z 轴旋转方向：低位负转、中位停止、高位正转 */
  command->mode = CHASSIS_MODE_INDENPENDENT;
  command->ctrl_vec.vx = -rc->ch_l_y * CHASSIS_TEST_INPUT_SCALE;
  command->ctrl_vec.vy = rc->ch_l_x * CHASSIS_TEST_INPUT_SCALE;
  float rotation_direction = 0.0f;
  if (rc->sw_l == CMD_SW_UP) {
    rotation_direction = -1.0f;
  }
  else if (rc->sw_l == CMD_SW_DOWN) {
    rotation_direction = 1.0f;
  }
  command->ctrl_vec.wz = CHASSIS_TEST_ROTATION_ENABLE != 0U
                             ? rotation_direction * CHASSIS_TEST_ROTATION_SPEED
                             : 0.0f;
}

static void RC_BuildGimbalCommand(const CMD_RC_t *rc,
                                  CMD_GimbalCmd_t *command,
                                  uint32_t now_tick) {
  memset(command, 0, sizeof(*command));
  if (rc->sw_r == CMD_SW_ERR) {
    command->mode = GIMBAL_MODE_RELAX;
    gimbal_last_update_tick = now_tick;
    return;
  }

  float dt = 0.0f;
  if (gimbal_last_update_tick != 0U) {
    dt = (float)(now_tick - gimbal_last_update_tick) /
         (float)osKernelGetTickFreq();
    if (dt > GIMBAL_TEST_DT_MAX_S) dt = GIMBAL_TEST_DT_MAX_S;
  }
  gimbal_last_update_tick = now_tick;

  command->mode = GIMBAL_MODE_ABSOLUTE;
  command->delta_eulr.yaw = rc->ch_r_x * GIMBAL_TEST_YAW_RATE_RAD_S * dt;
  command->delta_eulr.pit = -rc->ch_r_y * GIMBAL_TEST_PITCH_RATE_RAD_S * dt;
}

/* Exported functions ------------------------------------------------------- */

/**
 * \brief ELRS CRSF receiver
 *
 * \param argument 未使用
 */
void Task_RC(void *argument) {
  (void)argument; /* 未使用，消除警告 */

  if (CRSF_Init(&crsf) != DEVICE_OK) {
    CRSF_HandleOffline(&cmd_rc);
  }

  while (1) {
#ifdef DEBUG
    /*  */
    task_runtime.stack_water_mark.rc = osThreadGetStackSpace(osThreadGetId());
#endif
    if (!CRSF_WaitRC(&crsf, &cmd_rc, 100u)) {
      CRSF_HandleOffline(&cmd_rc);
      CRSF_StartReceive(&crsf);
    }
    RC_BuildChassisCommand(&cmd_rc, &rc_chassis_cmd);
    RC_BuildGimbalCommand(&cmd_rc, &rc_gimbal_cmd, osKernelGetTickCount());
    osMessageQueueReset(task_runtime.msgq.cmd.chassis);
    osMessageQueuePut(task_runtime.msgq.cmd.chassis, &rc_chassis_cmd, 0, 0);
    osMessageQueueReset(task_runtime.msgq.cmd.gimbal);
    osMessageQueuePut(task_runtime.msgq.cmd.gimbal, &rc_gimbal_cmd, 0, 0);
    /* 错误中断连续发生时也必须让出处理器，避免重启接收形成高优先级忙循环 */
    osDelay(1U);
  }
}
