/*
  初始化任务

  根据机器人的FLASH配置，决定生成哪些任务。
*/

/* Includes ----------------------------------------------------------------- */
#include "component/cmd.h"
#include "component/user_math.h"
#include "device/bmi088.h"
#include "device/can.h"
#include "device/nav_link.h"
#include "module/chassis.h"
#include "module/gimbal.h"
#include "module/shoot.h"
#include "task/user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define GIMBAL_MECH_ZERO_YAW_RAD (180.0f * M_DEG2RAD_MULT)
#define GIMBAL_MECH_ZERO_PIT_RAD (240.0f * M_DEG2RAD_MULT)
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * \brief 初始化
 *
 * \param argument 未使用
 */
void Task_Init(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  Config_Get(&task_runtime.cfg); /* 获取机器人配置 */
  /* Fixed mechanical encoder zeros for this chassis/gimbal assembly. */
  task_runtime.cfg.mech_zero.yaw = GIMBAL_MECH_ZERO_YAW_RAD;
  task_runtime.cfg.mech_zero.pit = GIMBAL_MECH_ZERO_PIT_RAD;

  osKernelLock();
  /* Active tasks; shoot remains disabled until its RC mapping is configured. */
  task_runtime.thread.atti_esti =
      osThreadNew(Task_AttiEsti, NULL, &attr_atti_esti);
  task_runtime.thread.ctrl_gimbal =
      osThreadNew(Task_CtrlGimbal, NULL, &attr_ctrl_gimbal);
  task_runtime.thread.can = osThreadNew(Task_Can, NULL, &attr_can);
  task_runtime.thread.info = osThreadNew(Task_Info, NULL, &attr_info);
  task_runtime.thread.rc = osThreadNew(Task_RC, NULL, &attr_rc);
  task_runtime.thread.nav_link =
      osThreadNew(Task_NavLink, NULL, &attr_nav_link);
  task_runtime.thread.ctrl_chassis =
      osThreadNew(Task_CtrlChassis, NULL, &attr_ctrl_chassis);

  /* UART AI and shoot implementations are retained. They were not active in
   * the baseline, so this cleanup does not enable new actuator commands. */
  task_runtime.msgq.ai.quat = osMessageQueueNew(2U, sizeof(AHRS_Quaternion_t), NULL);
  task_runtime.msgq.cmd.ai = osMessageQueueNew(3U, sizeof(CMD_AI_Status_t), NULL);
  task_runtime.msgq.cmd.raw.rc = osMessageQueueNew(3U, sizeof(CMD_RC_t), NULL);
  task_runtime.msgq.cmd.raw.host = osMessageQueueNew(3U, sizeof(CMD_Host_t), NULL);

  /* Mechanism control queues. */
  task_runtime.msgq.can.feedback.chassis = osMessageQueueNew(2U, sizeof(CAN_t), NULL);
  task_runtime.msgq.can.feedback.gimbal = osMessageQueueNew(2U, sizeof(CAN_t), NULL);
  task_runtime.msgq.can.feedback.shoot = osMessageQueueNew(2U, sizeof(CAN_t), NULL);
  task_runtime.msgq.can.output.chassis = osMessageQueueNew(2U, sizeof(CAN_ChassisOutput_t), NULL);
  task_runtime.msgq.can.output.gimbal = osMessageQueueNew(2U, sizeof(CAN_GimbalOutput_t), NULL);
  task_runtime.msgq.can.output.shoot = osMessageQueueNew(2U, sizeof(CAN_ShootOutput_t), NULL);
  task_runtime.msgq.cmd.chassis = osMessageQueueNew(3U, sizeof(CMD_ChassisCmd_t), NULL);
  task_runtime.msgq.cmd.gimbal = osMessageQueueNew(3U, sizeof(CMD_GimbalCmd_t), NULL);
  task_runtime.msgq.cmd.shoot = osMessageQueueNew(3U, sizeof(CMD_ShootCmd_t), NULL);
  task_runtime.msgq.gimbal.sample = osMessageQueueNew(1U, sizeof(GimbalIMUSample_t), NULL);

  osKernelUnlock();
  osThreadTerminate(osThreadGetId()); /* 结束自身 */
}
