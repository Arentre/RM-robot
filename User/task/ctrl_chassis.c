/*
  底盘控制任务

  控制底盘行为。

  从CAN总线接收底盘电机反馈，根据接收到的控制命令，控制电机输出。
*/

/* Includes ----------------------------------------------------------------- */
#include "component/limiter.h"
#include "device/nav_link.h"
#include "module/chassis.h"
#include "module/config.h"
#include "task/user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define CHASSIS_FEEDBACK_TIMEOUT_MS (100U)
#define CHASSIS_WHEEL_RADIUS_M (0.076f)
#define CHASSIS_ROTATION_ARM_M (0.29f)
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
static CAN_t can;
#ifdef DEBUG
CMD_ChassisCmd_t chassis_cmd;
Chassis_t chassis;
CAN_ChassisOutput_t chassis_out;
#else
static CMD_ChassisCmd_t chassis_cmd;
static Chassis_t chassis;
static CAN_ChassisOutput_t chassis_out;
#endif

/* Private function --------------------------------------------------------- */
static bool ChassisFeedbackOnline(const CAN_t *can_feedback,
                                  uint32_t now_tick) {
  if ((can_feedback->recive_flag & CAN_REC_CHASSIS_FINISHED) !=
      CAN_REC_CHASSIS_FINISHED)
    return false;

  const uint32_t timeout_tick =
      CHASSIS_FEEDBACK_TIMEOUT_MS * osKernelGetTickFreq() / 1000U;
  for (uint8_t i = 0U; i < CAN_MOTOR_CHASSIS_NUM; i++) {
    if (now_tick - can_feedback->chassis_feedback_tick[i] > timeout_tick)
      return false;
  }
  return true;
}

/* Exported functions ------------------------------------------------------- */

/**
 * \brief 控制底盘
 *
 * \param argument 未使用
 */
void Task_CtrlChassis(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / TASK_FREQ_CTRL_CHASSIS;
  /* 初始化底盘 */
  Chassis_Init(&chassis, &(task_runtime.cfg.robot_param->chassis),
               &task_runtime.cfg.mech_zero, (float)TASK_FREQ_CTRL_CHASSIS);

  /* 延时一段时间再开启任务 */
  osMessageQueueGet(task_runtime.msgq.can.feedback.chassis, &can, NULL,
                    osWaitForever);
  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  bool nav_mode = false;
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.ctrl_chassis =
        osThreadGetStackSpace(osThreadGetId());
#endif

    osMessageQueueGet(task_runtime.msgq.can.feedback.chassis, &can, NULL, 0);
    /* 读取遥控指令 */
    osMessageQueueGet(task_runtime.msgq.cmd.chassis, &chassis_cmd, NULL, 0);
    Nav_Velocity_t nav_command;
    uint32_t nav_tick;
    if (NavLink_GetCommand(&nav_command, &nav_tick)) {
      nav_mode = true;
      if ((uint32_t)(osKernelGetTickCount() - nav_tick) <=
          NAV_LINK_COMMAND_TIMEOUT_MS * osKernelGetTickFreq() / 1000U) {
        chassis_cmd.mode = CHASSIS_MODE_INDENPENDENT;
        chassis_cmd.ctrl_vec.vx = (float)nav_command.vx_mm_s / 1000.0f;
        chassis_cmd.ctrl_vec.vy = (float)nav_command.vy_mm_s / 1000.0f;
        chassis_cmd.ctrl_vec.wz = (float)nav_command.wz_mrad_s / 1000.0f;
      } else {
        chassis_cmd.mode = CHASSIS_MODE_BREAK;
      }
    } else if (nav_mode) {
      chassis_cmd.mode = CHASSIS_MODE_BREAK;
    }
    osKernelLock(); /* 锁住RTOS内核防止控制过程中断，造成错误 */
    Chassis_UpdateFeedback(&chassis, &can); /* 更新反馈值 */
    const bool feedback_online =
        ChassisFeedbackOnline(&can, osKernelGetTickCount());
    if (feedback_online && chassis.param->type == CHASSIS_TYPE_OMNI_CROSS) {
      const float rpm_to_m_s =
          (M_2PI * CHASSIS_WHEEL_RADIUS_M) /
          (60.0f * CHASSIS_M3508_GEAR_RATIO);
      const float r0 = chassis.feedback.motor_rpm[0] * rpm_to_m_s;
      const float r1 = chassis.feedback.motor_rpm[1] * rpm_to_m_s;
      const float r2 = chassis.feedback.motor_rpm[2] * rpm_to_m_s;
      const float r3 = chassis.feedback.motor_rpm[3] * rpm_to_m_s;
      const Nav_Velocity_t measured = {
          .vx_mm_s = (int32_t)((-r0 - r1 + r2 + r3) * 250.0f),
          .vy_mm_s = (int32_t)((r0 - r1 - r2 + r3) * 250.0f),
          .wz_mrad_s = (int32_t)((r0 + r1 + r2 + r3) *
                                 250.0f / CHASSIS_ROTATION_ARM_M * 1000.0f)};
      NavLink_SetMeasured(&measured);
    } else {
      const Nav_Velocity_t measured = {0};
      NavLink_SetMeasured(&measured);
    }
    if (!feedback_online)
      chassis_cmd.mode = CHASSIS_MODE_RELAX;
    /* 根据遥控器命令计算底盘输出 */
    Chassis_Control(&chassis, &chassis_cmd, tick);
    Chassis_PowerLimit(&chassis); /* 限制输出功率 */
    Chassis_DumpOutput(&chassis, &chassis_out);
    osKernelUnlock();
    /* 将电机输出值发送到CAN */
    osMessageQueueReset(task_runtime.msgq.can.output.chassis);
    osMessageQueuePut(task_runtime.msgq.can.output.chassis, &chassis_out, 0, 0);

    tick += delay_tick; /* 计算下一个唤醒时刻 */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
}
