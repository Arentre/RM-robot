/*
 * 云台模组
 */

/* Includes ----------------------------------------------------------------- */
#include "gimbal.h"

#include "bsp/mm.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
#define GIMBAL_PITCH_SAFE_LIMIT_RAD (1.22173048f) /* 70 deg */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
volatile uint8_t gimbal_pitch_protect_active = 0U;
volatile uint8_t gimbal_ladrc_yaw_enable = 1U;
volatile uint8_t gimbal_ladrc_pit_enable = 0U;

/* 2026-10-07 实机扫频和两次 30 deg 往返验证后的 Yaw 参数。 */
volatile LADRC_Params_t gimbal_ladrc_yaw_param = {
    .r = 10.0f, .b0 = 500.0f, .wc = 12.0f, .w0 = 48.0f,
    .out_limit = 0.10f};
volatile LADRC_Params_t gimbal_ladrc_pit_param = {
    .r = 2.5f, .b0 = 120.0f, .wc = 1.5f, .w0 = 3.0f,
    .out_limit = 0.03f};
/* Validated on 2 deg and 30 deg hardware moves.  YFF=0 can disable it at
 * runtime for A/B comparison. */
volatile uint8_t gimbal_yaw_friction_enable = 1U;
volatile float gimbal_yaw_friction_static = 0.035f;
volatile float gimbal_yaw_friction_coulomb = 0.020f;
volatile float gimbal_yaw_friction_deadband_rad = 0.00349066f; /* 0.2 deg */
volatile float gimbal_yaw_friction_transition_rad_s =
    0.08726646f; /* 5 deg/s */
volatile float gimbal_yaw_friction_slew_per_s = 0.12f;
volatile float gimbal_yaw_friction_output = 0.0f;
volatile uint8_t gimbal_yaw_dither_enable = 1U;
volatile float gimbal_yaw_dither_amplitude = 0.008f;
volatile float gimbal_yaw_dither_frequency_hz = 25.0f;
volatile float gimbal_yaw_dither_output = 0.0f;
static float gimbal_yaw_dither_phase = 0.0f;
/* Private function  -------------------------------------------------------- */

static float Gimbal_YawFrictionFeedforward(const Gimbal_t *g) {
  if (gimbal_yaw_friction_enable == 0U ||
      gimbal_pitch_protect_active != 0U ||
      g->mode != GIMBAL_MODE_ABSOLUTE)
    return 0.0f;

  const float error = CircleError(g->setpoint.eulr.yaw,
                                  g->feedback.eulr.imu.yaw, M_2PI);
  const float abs_error = fabsf(error);
  if (abs_error <= gimbal_yaw_friction_deadband_rad) return 0.0f;

  float error_scale =
      (abs_error - gimbal_yaw_friction_deadband_rad) /
      (2.0f * gimbal_yaw_friction_deadband_rad);
  if (error_scale > 1.0f) error_scale = 1.0f;

  float speed_ratio = fabsf(g->feedback.gyro.z) /
                      gimbal_yaw_friction_transition_rad_s;
  if (speed_ratio > 1.0f) speed_ratio = 1.0f;
  const float magnitude =
      gimbal_yaw_friction_static * (1.0f - speed_ratio) +
      gimbal_yaw_friction_coulomb * speed_ratio;
  return (error > 0.0f ? 1.0f : -1.0f) * magnitude * error_scale;
}

static uint8_t Gimbal_LADRCParamChanged(
    const LADRC_t *ctrl, const volatile LADRC_Params_t *param) {
  return ctrl->param.r != param->r || ctrl->param.b0 != param->b0 ||
         ctrl->param.wc != param->wc ||
         ctrl->param.w0 != param->w0 ||
         ctrl->param.out_limit != param->out_limit;
}

static float Gimbal_LADRCControl(LADRC_t *ctrl,
                                 const volatile LADRC_Params_t *param,
                                 uint8_t *active, float setpoint,
                                 float feedback, float angular_speed,
                                 float sample_time) {
  if (*active == 0U || Gimbal_LADRCParamChanged(ctrl, param)) {
    if (LADRC_Init(ctrl, param, sample_time) != 0) {
      *active = 0U;
      return 0.0f;
    }
    LADRC_Reset(ctrl, feedback, angular_speed);
    *active = 1U;
  }

  /* 将±pi反馈映射到观测器附近，并把目标映射为最短路径等效角。 */
  const float feedback_continuous =
      ctrl->z1 + CircleError(feedback, ctrl->z1, M_2PI);
  const float reference_continuous =
      feedback_continuous + CircleError(setpoint, feedback, M_2PI);
  return LADRC_Update(ctrl, reference_continuous, feedback_continuous);
}

/**
 * \brief 设置云台模式
 *
 * \param c 包含云台数据的结构体
 * \param mode 要设置的模式
 *
 * \return 函数运行结果
 */
static int8_t Gimbal_SetMode(Gimbal_t *g, CMD_GimbalMode_t mode) {
  if (g == NULL) return -1;
  if (mode == g->mode) return GIMBAL_OK;

  /* 切换模式后重置PID和滤波器 */
  for (uint8_t i = 0; i < GIMBAL_PID_NUM; i++) {
    PID_Reset(g->pid + i);
  }
  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
    LowPassFilter2p_Reset(g->filter_out + i, 0.0f);
  }

  AHRS_ResetEulr(&(g->setpoint.eulr)); /* 切换模式后重置设定值 */
  /* Every transition into absolute mode holds the attitude at that instant. */
  if (mode == GIMBAL_MODE_ABSOLUTE) {
    g->setpoint.eulr.yaw = g->feedback.eulr.imu.yaw;
    g->setpoint.eulr.pit = g->feedback.eulr.imu.pit;
  } else if ((g->mode == GIMBAL_MODE_RELAX) &&
             (mode == GIMBAL_MODE_RELATIVE)) {
    g->setpoint.eulr.yaw = g->feedback.eulr.encoder.yaw;
  }

  g->mode = mode;
  return 0;
}

/* Exported functions ------------------------------------------------------- */

/**
 * \brief 初始化云台
 *
 * \param g 包含云台数据的结构体
 * \param param 包含云台参数的结构体指针
 * \param target_freq 任务预期的运行频率
 *
 * \return 函数运行结果
 */
int8_t Gimbal_Init(Gimbal_t *g, const Gimbal_Params_t *param, float limit_max,
                   float target_freq) {
  if (g == NULL) return -1;

  g->param = param;            /* 初始化参数 */
  g->mode = GIMBAL_MODE_RELAX; /* 设置默认模式 */

  /* 设置软件限位 */
  if (g->param->reverse.pit) CircleReverse(&limit_max);
  g->limit.min = g->limit.max = limit_max;
  CircleAdd(&(g->limit.min), -g->param->pitch_travel_rad, M_2PI);

  /* 初始化云台电机控制PID和LPF */
  PID_Init(&(g->pid[GIMBAL_PID_YAW_ANGLE_IDX]), KPID_MODE_NO_D, target_freq,
           &(g->param->pid[GIMBAL_PID_YAW_ANGLE_IDX]));
  PID_Init(&(g->pid[GIMBAL_PID_YAW_OMEGA_IDX]), KPID_MODE_CALC_D, target_freq,
           &(g->param->pid[GIMBAL_PID_YAW_OMEGA_IDX]));

  PID_Init(&(g->pid[GIMBAL_PID_PIT_ANGLE_IDX]), KPID_MODE_NO_D, target_freq,
           &(g->param->pid[GIMBAL_PID_PIT_ANGLE_IDX]));
  PID_Init(&(g->pid[GIMBAL_PID_PIT_OMEGA_IDX]), KPID_MODE_CALC_D, target_freq,
           &(g->param->pid[GIMBAL_PID_PIT_OMEGA_IDX]));

  const float sample_time = 1.0f / target_freq;
  LADRC_Init(&(g->ladrc[GIMBAL_ACTR_YAW_IDX]), &gimbal_ladrc_yaw_param,
             sample_time);
  LADRC_Init(&(g->ladrc[GIMBAL_ACTR_PIT_IDX]), &gimbal_ladrc_pit_param,
             sample_time);
  g->ladrc_active[GIMBAL_ACTR_YAW_IDX] = 0U;
  g->ladrc_active[GIMBAL_ACTR_PIT_IDX] = 0U;

  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
    LowPassFilter2p_Init(g->filter_out + i, target_freq,
                         g->param->low_pass_cutoff_freq.out);
  }
  return 0;
}

/**
 * \brief 通过CAN设备更新云台反馈信息
 *
 * \param gimbal 云台
 * \param can CAN设备
 *
 * \return 函数运行结果
 */
int8_t Gimbal_UpdateFeedback(Gimbal_t *gimbal, const CAN_t *can) {
  if (gimbal == NULL) return -1;
  if (can == NULL) return -1;

  gimbal->feedback.eulr.encoder.yaw = can->motor.gimbal.named.yaw.rotor_angle;
  gimbal->feedback.eulr.encoder.pit = can->motor.gimbal.named.pit.rotor_angle;
  gimbal->feedback.motor.yaw_current =
      can->motor.gimbal.named.yaw.torque_current;
  gimbal->feedback.motor.pit_current =
      can->motor.gimbal.named.pit.torque_current;
  gimbal->feedback.motor.yaw_temp = can->motor.gimbal.named.yaw.temp;
  gimbal->feedback.motor.pit_temp = can->motor.gimbal.named.pit.temp;

  if (gimbal->param->reverse.yaw)
    CircleReverse(&(gimbal->feedback.eulr.encoder.yaw));
  if (gimbal->param->reverse.pit)
    CircleReverse(&(gimbal->feedback.eulr.encoder.pit));

  return 0;
}

/**
 * \brief 运行云台控制逻辑
 *
 * \param g 包含云台数据的结构体
 * \param fb 云台反馈信息
 * \param g_cmd 云台控制指令
 * \param dt_sec 两次调用的时间间隔
 *
 * \return 函数运行结果
 */
int8_t Gimbal_Control(Gimbal_t *g, CMD_GimbalCmd_t *g_cmd, uint32_t now) {
  if (g == NULL) return -1;
  if (g_cmd == NULL) return -1;

  g->dt = (float)(now - g->lask_wakeup) / 1000.0f;
  g->lask_wakeup = now;

  Gimbal_SetMode(g, g_cmd->mode);

  /* yaw坐标正方向与遥控器操作逻辑相反 */
  g_cmd->delta_eulr.pit = -g_cmd->delta_eulr.pit;
  g_cmd->delta_eulr.yaw = -g_cmd->delta_eulr.yaw;

  /* 处理yaw控制命令 */
  CircleAdd(&(g->setpoint.eulr.yaw), g_cmd->delta_eulr.yaw, M_2PI);

  /* 处理pitch控制命令，软件限位 */
  /* Keep the commanded Pitch independent from IMU feedback.  The stored
   * encoder end stop is not used in this learning build because an invalid
   * calibration generated a false -70 deg reference. */
  const float requested_pitch =
      g->setpoint.eulr.pit + g_cmd->delta_eulr.pit;
  g->pit_control_target = requested_pitch;
  Clip(&g->pit_control_target, -GIMBAL_PITCH_SAFE_LIMIT_RAD,
       GIMBAL_PITCH_SAFE_LIMIT_RAD);
  g->pit_limit_active =
      fabsf(g->pit_control_target - requested_pitch) > 0.00001f;
  g->setpoint.eulr.pit = g->pit_control_target;

  /* 重置输入指令，防止重复处理 */
  AHRS_ResetEulr(&(g_cmd->delta_eulr));

  /* 控制相关逻辑 */
  float yaw_omega_set_point;
  switch (g->mode) {
    case GIMBAL_MODE_RELAX:
      for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->out[i] = 0.0f;
      for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->ladrc_active[i] = 0U;
      break;

    case GIMBAL_MODE_ABSOLUTE:
      /* ZYX Euler angles are singular near pitch +/-90 deg.  Do not run the
       * decoupled yaw/pitch loops in that region. */
      if (fabsf(g->feedback.eulr.imu.pit) > GIMBAL_PITCH_SAFE_LIMIT_RAD) {
        gimbal_pitch_protect_active = 1U;
        for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->out[i] = 0.0f;
        for (uint8_t i = 0; i < GIMBAL_PID_NUM; i++) PID_Reset(g->pid + i);
        for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
          g->ladrc_active[i] = 0U;
          g->ladrc[i].output = 0.0f;
        }
        break;
      }
      gimbal_pitch_protect_active = 0U;

      if (gimbal_ladrc_yaw_enable != 0U) {
        if (g->ladrc_active[GIMBAL_ACTR_YAW_IDX] == 0U ||
            Gimbal_LADRCParamChanged(
                &(g->ladrc[GIMBAL_ACTR_YAW_IDX]),
                &gimbal_ladrc_yaw_param))
          LowPassFilter2p_Reset(
              &(g->filter_out[GIMBAL_ACTR_YAW_IDX]), 0.0f);
        g->out[GIMBAL_ACTR_YAW_IDX] = Gimbal_LADRCControl(
            &(g->ladrc[GIMBAL_ACTR_YAW_IDX]), &gimbal_ladrc_yaw_param,
            &(g->ladrc_active[GIMBAL_ACTR_YAW_IDX]), g->setpoint.eulr.yaw,
            g->feedback.eulr.imu.yaw, g->feedback.gyro.z,
            g->ladrc[GIMBAL_ACTR_YAW_IDX].T);
      } else {
        if (g->ladrc_active[GIMBAL_ACTR_YAW_IDX] != 0U) {
          PID_Reset(&(g->pid[GIMBAL_PID_YAW_ANGLE_IDX]));
          PID_Reset(&(g->pid[GIMBAL_PID_YAW_OMEGA_IDX]));
          LowPassFilter2p_Reset(
              &(g->filter_out[GIMBAL_ACTR_YAW_IDX]), 0.0f);
        }
        g->ladrc_active[GIMBAL_ACTR_YAW_IDX] = 0U;
        yaw_omega_set_point = PID_Calc(
            &(g->pid[GIMBAL_PID_YAW_ANGLE_IDX]), g->setpoint.eulr.yaw,
            g->feedback.eulr.imu.yaw, 0.0f, g->dt);
        g->out[GIMBAL_ACTR_YAW_IDX] = PID_Calc(
            &(g->pid[GIMBAL_PID_YAW_OMEGA_IDX]), yaw_omega_set_point,
            g->feedback.gyro.z, 0.f, g->dt);
      }

      if (gimbal_ladrc_pit_enable != 0U) {
        if (g->ladrc_active[GIMBAL_ACTR_PIT_IDX] == 0U ||
            Gimbal_LADRCParamChanged(
                &(g->ladrc[GIMBAL_ACTR_PIT_IDX]),
                &gimbal_ladrc_pit_param))
          LowPassFilter2p_Reset(
              &(g->filter_out[GIMBAL_ACTR_PIT_IDX]), 0.0f);
        g->out[GIMBAL_ACTR_PIT_IDX] = Gimbal_LADRCControl(
            &(g->ladrc[GIMBAL_ACTR_PIT_IDX]), &gimbal_ladrc_pit_param,
            &(g->ladrc_active[GIMBAL_ACTR_PIT_IDX]), g->pit_control_target,
            g->feedback.eulr.imu.pit, g->feedback.gyro.y,
            g->ladrc[GIMBAL_ACTR_PIT_IDX].T);
      } else {
        /* Yaw-only baseline: ID6 remains zero until Pitch LADRC is enabled. */
        g->ladrc_active[GIMBAL_ACTR_PIT_IDX] = 0U;
        g->out[GIMBAL_ACTR_PIT_IDX] = 0.0f;
      }
      break;

    case GIMBAL_MODE_RELATIVE:
      for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->out[i] = 0.0f;
      for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->ladrc_active[i] = 0U;
      break;
  }

  const float friction_target = Gimbal_YawFrictionFeedforward(g);
  if (friction_target == 0.0f) {
    gimbal_yaw_friction_output = 0.0f;
  } else {
    float delta = friction_target - gimbal_yaw_friction_output;
    const float max_delta = gimbal_yaw_friction_slew_per_s * g->dt;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    gimbal_yaw_friction_output += delta;
  }
  g->out[GIMBAL_ACTR_YAW_IDX] += gimbal_yaw_friction_output;

  const float yaw_error =
      CircleError(g->setpoint.eulr.yaw, g->feedback.eulr.imu.yaw, M_2PI);
  if (gimbal_yaw_dither_enable != 0U &&
      g->mode == GIMBAL_MODE_ABSOLUTE &&
      fabsf(yaw_error) > gimbal_yaw_friction_deadband_rad &&
      fabsf(yaw_error) < 0.02617994f && /* 1.5 deg */
      fabsf(g->feedback.gyro.z) < 0.01745329f) { /* 1 deg/s */
    gimbal_yaw_dither_phase +=
        M_2PI * gimbal_yaw_dither_frequency_hz * g->dt;
    if (gimbal_yaw_dither_phase >= M_2PI)
      gimbal_yaw_dither_phase -= M_2PI;
    gimbal_yaw_dither_output =
        gimbal_yaw_dither_amplitude * sinf(gimbal_yaw_dither_phase);
  } else {
    gimbal_yaw_dither_phase = 0.0f;
    gimbal_yaw_dither_output = 0.0f;
  }
  g->out[GIMBAL_ACTR_YAW_IDX] += gimbal_yaw_dither_output;

  /* 输出滤波 */
  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++)
    g->out[i] = LowPassFilter2p_Apply(g->filter_out + i, g->out[i]);
  if (gimbal_ladrc_pit_enable == 0U) g->out[GIMBAL_ACTR_PIT_IDX] = 0.0f;

  /* 处理电机反装 */
  if (g->param->reverse.yaw)
    g->out[GIMBAL_ACTR_YAW_IDX] = -g->out[GIMBAL_ACTR_YAW_IDX];
  if (g->param->reverse.pit)
    g->out[GIMBAL_ACTR_PIT_IDX] = -g->out[GIMBAL_ACTR_PIT_IDX];

  if (g->out[GIMBAL_ACTR_YAW_IDX] < 0.f) {
    __NOP();
  }

  if (g->out[GIMBAL_ACTR_PIT_IDX] > 0.5f) {
    __NOP();
  }
  return 0;
}

/**
 * \brief 复制云台输出值
 *
 * \param s 包含云台数据的结构体
 * \param out CAN设备云台输出结构体
 */
void Gimbal_DumpOutput(Gimbal_t *g, CAN_GimbalOutput_t *out) {
  out->named.yaw = g->out[GIMBAL_ACTR_YAW_IDX];
  out->named.pit = g->out[GIMBAL_ACTR_PIT_IDX];
}

/**
 * \brief 清空输出值
 *
 * \param output 要清空的结构体
 */
void Gimbal_ResetOutput(CAN_GimbalOutput_t *output) {
  int i = 0;
  for (i = 0; i < 3; i++) {
    output->as_array[i] = 0.0f;
  }
}

/**
 * @brief 导出云台UI数据
 *
 * @param g 云台结构体
 * @param ui UI结构体
 */
void Gimbal_DumpUI(const Gimbal_t *g, Referee_GimbalUI_t *ui) {
  ui->mode = g->mode;
}
