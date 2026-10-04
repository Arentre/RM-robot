/*
  云台控制任务

  控制云台行为。

  从CAN总线接收底盘电机反馈，从IMU接收欧拉角和角速度，
  根据接收到的控制命令，控制电机输出。
*/

/* Includes ----------------------------------------------------------------- */
#include "module\gimbal.h"
#include "task\user_task.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
static CAN_t can;

/* Exposed read-only to the low-priority USB telemetry task. */
Gimbal_t gimbal;
CAN_GimbalOutput_t gimbal_out;

#ifdef DEBUG
CMD_GimbalCmd_t gimbal_cmd;
Referee_GimbalUI_t gimbal_ui;
#else
static CMD_GimbalCmd_t gimbal_cmd;
static Referee_GimbalUI_t gimbal_ui;
#endif

/* Keil Watch中设为1才开启云台，上电默认无电流输出。 */
volatile uint8_t gimbal_test_enable = 0U;

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * \brief 控制云台
 *
 * \param argument 未使用
 */
void Task_CtrlGimbal(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  const uint32_t delay_tick = osKernelGetTickFreq() / TASK_FREQ_CTRL_GIMBAL;
  /* 初始化云台 */
  Gimbal_Init(&gimbal, &(task_runtime.cfg.robot_param->gimbal),
              task_runtime.cfg.gimbal_limit, (float)TASK_FREQ_CTRL_GIMBAL);

  /* 不启动Task_Command时，上电先保持放松模式。 */
  gimbal_cmd.mode = GIMBAL_MODE_RELAX;
  AHRS_ResetEulr(&gimbal_cmd.delta_eulr);

  /* 等待电机反馈和IMU数据就绪后再开启控制。 */
  osMessageQueueGet(task_runtime.msgq.can.feedback.gimbal, &can, NULL,
                    osWaitForever);
  osMessageQueueGet(task_runtime.msgq.gimbal.eulr_imu,
                    &(gimbal.feedback.eulr.imu), NULL, osWaitForever);
  osMessageQueueGet(task_runtime.msgq.gimbal.gyro, &(gimbal.feedback.gyro),
                    NULL, osWaitForever);
  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  while (1) {
#ifdef DEBUG
    /* 记录任务所使用的的栈空间 */
    task_runtime.stack_water_mark.ctrl_gimbal =
        osThreadGetStackSpace(osThreadGetId());
#endif

    osMessageQueueGet(task_runtime.msgq.can.feedback.gimbal, &can, NULL, 0);
    /* 读取控制指令、姿态、IMU数据 */
    osMessageQueueGet(task_runtime.msgq.gimbal.eulr_imu,
                      &(gimbal.feedback.eulr.imu), NULL, 0);
    osMessageQueueGet(task_runtime.msgq.gimbal.gyro, &(gimbal.feedback.gyro),
                      NULL, 0);
    /* 任务只提交模式请求。进入 ABS 的瞬间由 Gimbal_SetMode()
     * 锁存当前 yaw/pitch，避免任务层和模块层重复改写目标角。 */
    const uint8_t enable = (gimbal_test_enable != 0U) ? 1U : 0U;
    if (enable != 0U) {
      gimbal_cmd.mode = GIMBAL_MODE_ABSOLUTE;
    } else {
      gimbal_cmd.mode = GIMBAL_MODE_RELAX;
    }

    osKernelLock(); /* 锁住RTOS内核防止控制过程中断，造成错误 */
    Gimbal_UpdateFeedback(&gimbal, &can);
    Gimbal_Control(&gimbal, &gimbal_cmd, tick);
    Gimbal_DumpOutput(&gimbal, &gimbal_out);

    osKernelUnlock();
    osMessageQueueReset(task_runtime.msgq.can.output.gimbal);
    osMessageQueuePut(task_runtime.msgq.can.output.gimbal, &gimbal_out, 0, 0);

    Gimbal_DumpUI(&gimbal, &gimbal_ui);
    osMessageQueueReset(task_runtime.msgq.ui.gimbal);
    osMessageQueuePut(task_runtime.msgq.ui.gimbal, &gimbal_ui, 0, 0);

    tick += delay_tick; /* 计算下一个唤醒时刻 */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
}

