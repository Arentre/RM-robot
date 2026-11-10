/*
  底盘控制任务

  控制底盘行为。

  从CAN总线接收底盘电机反馈，根据接收到的控制命令，控制电机输出。
*/

/* Includes ----------------------------------------------------------------- */
#include "component/limiter.h"
#include "module/chassis.h"
#include "module/config.h"
#include "task/user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define CHASSIS_FEEDBACK_TIMEOUT_MS (100U)
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
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.ctrl_chassis =
        osThreadGetStackSpace(osThreadGetId());
#endif

    osMessageQueueGet(task_runtime.msgq.can.feedback.chassis, &can, NULL, 0);
    /* 读取遥控指令 */
    osMessageQueueGet(task_runtime.msgq.cmd.chassis, &chassis_cmd, NULL, 0);
    osKernelLock(); /* 锁住RTOS内核防止控制过程中断，造成错误 */
    Chassis_UpdateFeedback(&chassis, &can); /* 更新反馈值 */
    if (!ChassisFeedbackOnline(&can, osKernelGetTickCount()))
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
