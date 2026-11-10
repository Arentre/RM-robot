/*
 * 底盘模组
 */

/* Includes ----------------------------------------------------------------- */
#include "chassis.h"

#include <stdlib.h>

#include "bsp/mm.h"
#include "cmsis_os2.h"
#include "component/limiter.h"
#include "device/can.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define CHASSIS_STANDALONE_POWER_LIMIT_W (50.0f)

#define CHASSIS_ROTOR_WZ_MIN 0.6f          /* 小陀螺旋转位移下界 */
#define CHASSIS_ROTOR_WZ_MAX 0.8f          /* 小陀螺旋转位移上界 */
#define M_7OVER72PI (M_2PI * 7.0f / 72.0f) /* 三十五度对应弧度值 */
// #define CHASSIS_ROTOR_OMEGA 0.15f        /* 小陀螺转动频率 */
// 随机产生小陀螺转动频率，范围0.001-0.05
//#define CHASSIS_ROTOR_OMEGA rand() % 50 / 10000.0f + 0.001f
#define CHASSIS_ROTOR_OMEGA 0.001

/* 与已验证底盘一致，以下转速均换算为 M3508 转子侧数值 */
#define CHASSIS_SPEED_LIMIT_RPM (300.0f * CHASSIS_M3508_GEAR_RATIO)
#define CHASSIS_RAMP_RATE_RPM_S (200.0f * CHASSIS_M3508_GEAR_RATIO)
#define CHASSIS_REVERSAL_ZERO_SPEED_RPM (10.0f * CHASSIS_M3508_GEAR_RATIO)
#define CHASSIS_CURRENT_LIMIT_NORMALIZED (4.0f / CAN_M3508_MAX_ABS_CUR)
/* Private macro ------------------------------------------------------------ */

/* Private function  -------------------------------------------------------- */
/**
 * \brief 设置底盘模式
 *
 * \param c 包含底盘数据的结构体
 * \param mode 要设置的模式
 *
 * \return 函数运行结果
 */
static int8_t Chassis_SetMode(Chassis_t *c, CMD_ChassisMode_t mode,
                              uint32_t now) {
  if (c == NULL) return CHASSIS_ERR_NULL; /* 主结构体不能为空 */
  if (mode == c->mode) return CHASSIS_OK; /* 模式未改变直接返回 */

  if (mode == CHASSIS_MODE_ROTOR && c->mode != CHASSIS_MODE_ROTOR) {
    srand(now);
    c->wz_multi = (rand() % 2) ? -1 : 1;
  }
  /* 切换模式后重置PID和滤波器 */
  for (uint8_t i = 0; i < c->num_wheel; i++) {
    PID_Reset(c->pid.motor + i);
    LowPassFilter2p_Reset(c->filter.in + i, 0.0f);
    LowPassFilter2p_Reset(c->filter.out + i, 0.0f);
    c->setpoint.ramped_motor_rpm[i] = 0.0f;
  }
  c->mode = mode;

  return CHASSIS_OK;
}
/**
 * @brief 产生小陀螺wz随机速度
 *
 * @param min wz产生最小速度
 * @param max wz产生最大速度
 * @param now ctrl_chassis的tick数
 * @return float
 */
static float Chassis_CalcWz(const float min, const float max, uint32_t now) {
  /* wz在min和max之间，上限0.6f */
  float wz_vary = fabs(0.2f * sinf(CHASSIS_ROTOR_OMEGA * (float)now)) + min;
  return wz_vary > 0.8f ? max : wz_vary;
}

/* Exported functions ------------------------------------------------------- */

/**
 * \brief 初始化底盘
 *
 * \param c 包含底盘数据的结构体
 * \param param 包含底盘参数的结构体指针
 * \param target_freq 任务预期的运行频率
 *
 * \return 函数运行结果
 */
int8_t Chassis_Init(Chassis_t *c, const Chassis_Params_t *param,
                    AHRS_Eulr_t *mech_zero, float target_freq) {
  if (c == NULL) return CHASSIS_ERR_NULL;

  c->param = param;             /* 初始化参数 */
  c->mode = CHASSIS_MODE_RELAX; /* 设置上电后底盘默认模式 */
  c->mech_zero = mech_zero;     /* 设置底盘机械零点 */
  c->control_period_s = 1.0f / target_freq;

  /* 如果电机反装重新计算机械零点 */
  if (param->reverse.yaw) CircleReverse(&(c->mech_zero->yaw));

  /* 根据参数（param）中的底盘型号初始化Mixer */
  Mixer_Mode_t mixer_mode;
  switch (c->param->type) {
    case CHASSIS_TYPE_MECANUM:
      c->num_wheel = 4;
      mixer_mode = MIXER_MECANUM;
      break;

    case CHASSIS_TYPE_PARLFIX4:
      c->num_wheel = 4;
      mixer_mode = MIXER_PARLFIX4;
      break;

    case CHASSIS_TYPE_PARLFIX2:
      c->num_wheel = 2;
      mixer_mode = MIXER_PARLFIX2;
      break;

    case CHASSIS_TYPE_OMNI_CROSS:
      c->num_wheel = 4;
      mixer_mode = MIXER_OMNICROSS;
      break;

    case CHASSIS_TYPE_OMNI_PLUS:
      c->num_wheel = 4;
      mixer_mode = MIXER_OMNIPLUS;
      break;

    case CHASSIS_TYPE_SINGLE:
      c->num_wheel = 1;
      mixer_mode = MIXER_SINGLE;
      break;

    case CHASSIS_TYPE_DRONE:
      /* onboard sdk. */
      return CHASSIS_ERR_TYPE;
  }

  /* 根据底盘型号动态分配控制时使用的变量 */
  c->feedback.motor_rpm =
      BSP_Malloc((size_t)c->num_wheel * sizeof(*c->feedback.motor_rpm));
  if (c->feedback.motor_rpm == NULL) goto error; /* 变量未分配，返回错误 */

  c->feedback.motor_current =
      BSP_Malloc((size_t)c->num_wheel * sizeof(*c->feedback.motor_current));
  if (c->feedback.motor_current == NULL) goto error;

  c->setpoint.motor_rpm =
      BSP_Malloc((size_t)c->num_wheel * sizeof(*c->setpoint.motor_rpm));
  if (c->setpoint.motor_rpm == NULL) goto error;

  c->setpoint.ramped_motor_rpm = BSP_Malloc(
      (size_t)c->num_wheel * sizeof(*c->setpoint.ramped_motor_rpm));
  if (c->setpoint.ramped_motor_rpm == NULL) goto error;

  c->pid.motor = BSP_Malloc((size_t)c->num_wheel * sizeof(*c->pid.motor));
  if (c->pid.motor == NULL) goto error;

  c->out = BSP_Malloc((size_t)c->num_wheel * sizeof(*c->out));
  if (c->out == NULL) goto error;

  c->filter.in = BSP_Malloc((size_t)c->num_wheel * sizeof(*c->filter.in));
  if (c->filter.in == NULL) goto error;

  c->filter.out = BSP_Malloc((size_t)c->num_wheel * sizeof(*c->filter.out));
  if (c->filter.out == NULL) goto error;

  /* 初始化轮子电机控制PID和LPF */
  for (uint8_t i = 0; i < c->num_wheel; i++) {
    PID_Init(c->pid.motor + i, KPID_MODE_NO_D, target_freq,
             &(c->param->motor_pid_param));

    LowPassFilter2p_Init(c->filter.in + i, target_freq,
                         c->param->low_pass_cutoff_freq.in);
    LowPassFilter2p_Init(c->filter.out + i, target_freq,
                         c->param->low_pass_cutoff_freq.out);
    c->setpoint.motor_rpm[i] = 0.0f;
    c->setpoint.ramped_motor_rpm[i] = 0.0f;
    c->out[i] = 0.0f;
  }

  /* 初始化跟随云台的控制PID */
  PID_Init(&(c->pid.follow), KPID_MODE_NO_D, target_freq,
           &(c->param->follow_pid_param));

  Mixer_Init(&(c->mixer), mixer_mode); /* 初始化混合器 */
  return CHASSIS_OK;

error:
  /* 动态内存分配错误时，释放已经分配的内存，返回错误值 */
  BSP_Free(c->feedback.motor_rpm);
  BSP_Free(c->setpoint.motor_rpm);
  BSP_Free(c->setpoint.ramped_motor_rpm);
  BSP_Free(c->pid.motor);
  BSP_Free(c->out);
  BSP_Free(c->filter.in);
  BSP_Free(c->filter.out);
  return CHASSIS_ERR_NULL;
}

/**
 * \brief 更新底盘的反馈信息
 *
 * \param c 包含底盘数据的结构体
 * \param can CAN设备结构体
 *
 * \return 函数运行结果
 */
int8_t Chassis_UpdateFeedback(Chassis_t *c, const CAN_t *can) {
  /* 底盘数据和CAN结构体不能为空 */
  if (c == NULL) return CHASSIS_ERR_NULL;
  if (can == NULL) return CHASSIS_ERR_NULL;

  /* 如果电机反装重新计算正确的反馈值 */
  if (c->param->reverse.yaw) {
    c->feedback.gimbal_yaw_encoder =
        -can->motor.gimbal.named.yaw.rotor_angle + M_2PI;
  } else {
    c->feedback.gimbal_yaw_encoder = can->motor.gimbal.named.yaw.rotor_angle;
  }

  /* 将CAN中的反馈数据写入到feedback中 */
  for (uint8_t i = 0; i < c->num_wheel; i++) {
    c->feedback.motor_rpm[i] = can->motor.chassis.as_array[i].rotor_speed;
    c->feedback.motor_current[i] =
        can->motor.chassis.as_array[i].torque_current;
  }

  return CHASSIS_OK;
}

/**
 * \brief 运行底盘控制逻辑
 *
 * \param c 包含底盘数据的结构体
 * \param c_cmd 底盘控制指令
 * \param dt_sec 两次调用的时间间隔
 *
 * \return 函数运行结果
 */
int8_t Chassis_Control(Chassis_t *c, const CMD_ChassisCmd_t *c_cmd,
                       uint32_t now) {
  /* 底盘数据和控制指令结构体不能为空 */
  if (c == NULL) return CHASSIS_ERR_NULL;
  if (c_cmd == NULL) return CHASSIS_ERR_NULL;

  const float measured_dt =
      (float)(now - c->lask_wakeup) / (float)osKernelGetTickFreq();
  c->dt = c->lask_wakeup != 0U && measured_dt > 0.0f &&
                  measured_dt <= c->control_period_s * 2.0f
              ? measured_dt
              : c->control_period_s;
  c->lask_wakeup = now;

  /* 根据遥控器命令更改底盘模式 */
  Chassis_SetMode(c, c_cmd->mode, now);

  /* ctrl_vec -> move_vec 控制向量和真实的移动向量之间有一个换算关系 */
  /* 计算vx、vy */
  switch (c->mode) {
    case CHASSIS_MODE_BREAK: /* 刹车模式电机停止 */
      c->move_vec.vx = 0.0f;
      c->move_vec.vy = 0.0f;
      break;

    case CHASSIS_MODE_INDENPENDENT: /* 独立模式控制向量与运动向量相等 */
      c->move_vec.vx = c_cmd->ctrl_vec.vx;
      c->move_vec.vy = c_cmd->ctrl_vec.vy;
      break;

    case CHASSIS_MODE_OPEN:
    case CHASSIS_MODE_RELAX:
    case CHASSIS_MODE_FOLLOW_GIMBAL: /* 按照云台方向换算运动向量 */
    case CHASSIS_MODE_FOLLOW_GIMBAL_35:
    case CHASSIS_MODE_ROTOR: {
      float beta = c->feedback.gimbal_yaw_encoder - c->mech_zero->yaw;
      float cos_beta = cosf(beta);
      float sin_beta = sinf(beta);
      c->move_vec.vx =
          cos_beta * c_cmd->ctrl_vec.vx - sin_beta * c_cmd->ctrl_vec.vy;
      c->move_vec.vy =
          sin_beta * c_cmd->ctrl_vec.vx + cos_beta * c_cmd->ctrl_vec.vy;
    }
  }

  /* 计算wz */
  switch (c->mode) {
    case CHASSIS_MODE_RELAX:
    case CHASSIS_MODE_BREAK:
      c->move_vec.wz = 0.0f;
      break;

    case CHASSIS_MODE_INDENPENDENT: /* 独立模式直接使用底盘旋转指令 */
      c->move_vec.wz = c_cmd->ctrl_vec.wz;
      break;

    case CHASSIS_MODE_OPEN:
    case CHASSIS_MODE_FOLLOW_GIMBAL: /* 跟随模式通过PID控制使车头跟随云台 */
      c->move_vec.wz = PID_Calc(&(c->pid.follow), c->mech_zero->yaw,
                                c->feedback.gimbal_yaw_encoder, 0.0f, c->dt);
      break;
    case CHASSIS_MODE_FOLLOW_GIMBAL_35:
      c->move_vec.wz =
          PID_Calc(&(c->pid.follow), c->mech_zero->yaw + M_7OVER72PI,
                   c->feedback.gimbal_yaw_encoder, 0.0f, c->dt);
      break;
    case CHASSIS_MODE_ROTOR: { /* 小陀螺模式使底盘以一定速度旋转 */
      c->move_vec.wz = c->wz_multi * Chassis_CalcWz(CHASSIS_ROTOR_WZ_MIN,
                                                    CHASSIS_ROTOR_WZ_MAX, now);
    }
  }

  /* move_vec -> motor_rpm_set. 通过运动向量计算轮子转速目标值 */
  Mixer_Apply(&(c->mixer), &(c->move_vec), c->setpoint.motor_rpm, c->num_wheel,
              CHASSIS_SPEED_LIMIT_RPM);

  /* 根据轮子转速目标值，利用PID计算电机输出值 */
  for (uint8_t i = 0; i < c->num_wheel; i++) {
    /* 输入滤波. */
    c->feedback.motor_rpm[i] =
        LowPassFilter2p_Apply(c->filter.in + i, c->feedback.motor_rpm[i]);

    /* 换向时先制动到低速，再按固定斜率建立新的目标转速 */
    float effective_target_rpm = c->setpoint.motor_rpm[i];
    if ((c->feedback.motor_rpm[i] > CHASSIS_REVERSAL_ZERO_SPEED_RPM &&
         effective_target_rpm < 0.0f) ||
        (c->feedback.motor_rpm[i] < -CHASSIS_REVERSAL_ZERO_SPEED_RPM &&
         effective_target_rpm > 0.0f)) {
      effective_target_rpm = 0.0f;
      PID_ResetIntegral(c->pid.motor + i);
    }

    const float ramp_step = CHASSIS_RAMP_RATE_RPM_S * c->dt;
    const float ramp_error = effective_target_rpm - c->setpoint.ramped_motor_rpm[i];
    if (ramp_error > ramp_step) {
      c->setpoint.ramped_motor_rpm[i] += ramp_step;
    } else if (ramp_error < -ramp_step) {
      c->setpoint.ramped_motor_rpm[i] -= ramp_step;
    } else {
      c->setpoint.ramped_motor_rpm[i] = effective_target_rpm;
    }

    /* 根据底盘模式计算输出值 */
    switch (c->mode) {
      case CHASSIS_MODE_BREAK:
      case CHASSIS_MODE_FOLLOW_GIMBAL:
      case CHASSIS_MODE_FOLLOW_GIMBAL_35:
      case CHASSIS_MODE_ROTOR:
      case CHASSIS_MODE_INDENPENDENT: /* 独立模式,受PID控制 */
        c->out[i] = PID_Calc(c->pid.motor + i,
                             c->setpoint.ramped_motor_rpm[i],
                             c->feedback.motor_rpm[i], 0.0f, c->dt);
        break;

      case CHASSIS_MODE_OPEN: /* 开环模式,不受PID控制 */
        c->out[i] = c->setpoint.motor_rpm[i] / 9000.0f;
        break;

      case CHASSIS_MODE_RELAX: /* 放松模式,不输出 */
        c->out[i] = 0;
        break;
    }
    /* RELAX is a safety state and must not retain an output-filter tail. */
    if (c->mode == CHASSIS_MODE_RELAX) {
      c->setpoint.ramped_motor_rpm[i] = 0.0f;
      PID_Reset(c->pid.motor + i);
      LowPassFilter2p_Reset(c->filter.out + i, 0.0f);
      c->out[i] = 0.0f;
    } else {
      c->out[i] = LowPassFilter2p_Apply(c->filter.out + i, c->out[i]);
      c->out[i] = AbsClip(c->out[i], CHASSIS_CURRENT_LIMIT_NORMALIZED);
    }
  }

  return CHASSIS_OK;
}

/**
 * \brief 复制底盘输出值
 *
 * \param s 包含底盘数据的结构体
 * \param out CAN设备底盘输出结构体
 */
void Chassis_DumpOutput(Chassis_t *c, CAN_ChassisOutput_t *out) {
  for (uint8_t i = 0; i < c->num_wheel; i++) {
    out->as_array[i] = c->out[i];
  }
}

/**
 * \brief 清空Chassis输出数据
 *
 * \param out CAN设备底盘输出结构体
 */
void Chassis_ResetOutput(CAN_ChassisOutput_t *out) {
  for (uint8_t i = 0; i < 4; i++) {
    out->as_array[i] = 0.0f;
  }
}

int8_t Chassis_PowerLimit(Chassis_t *c) {
  /* Preserve the existing referee-offline power limit. */
  PowerLimit_ChassicOutput(CHASSIS_STANDALONE_POWER_LIMIT_W, c->out,
                           c->feedback.motor_rpm, c->num_wheel);
  return CHASSIS_OK;
}
