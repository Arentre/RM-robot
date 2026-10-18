/*
 * 云台模组
 */

/* Includes ----------------------------------------------------------------- */
#include "gimbal.h"
#include "component/joint_limit.h"

#include <string.h>

#include "bsp/mm.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
volatile uint8_t gimbal_pitch_protect_active = 0U;
volatile uint8_t gimbal_ladrc_yaw_enable = 1U;
volatile uint8_t gimbal_ladrc_pit_enable = 1U;
/* Runtime Pitch-only output shaping. Bypass until hardware validation. */
volatile float gimbal_pitch_output_filter_hz = 0.0f;

/* 2026-10-07 实机扫频和两次 30 deg 往返验证后的 Yaw 参数。 */
volatile LADRC_Params_t gimbal_ladrc_yaw_param = {
    .r = 10.0f, .b0 = 500.0f, .wc = 12.0f, .w0 = 40.0f,
    .out_limit = 0.30f, .deadband_rad = 0.00349066f}; /* 0.2 deg */
volatile LADRC_Params_t gimbal_ladrc_pit_param = {
    .r = 13.0f, .b0 = 550.0f, .wc = 7.0f, .w0 = 22.0f,
    .out_limit = 0.3f, .deadband_rad = 0.00523599f}; /* 0.3 deg */
volatile uint8_t gimbal_yaw_dither_enable = 0U;
volatile float gimbal_yaw_dither_amplitude = 0.008f;
volatile float gimbal_yaw_dither_frequency_hz = 25.0f;
volatile float gimbal_yaw_dither_output = 0.0f;
static float gimbal_yaw_dither_phase = 0.0f;
/* Private function  -------------------------------------------------------- */

static void Gimbal_UpdateEulerRate(Gimbal_t *g) {
  const float roll = g->feedback.eulr.imu.rol;
  const float pitch = g->feedback.eulr.imu.pit;
  const float p = g->feedback.gyro.x;
  const float q = g->feedback.gyro.y;
  const float r = g->feedback.gyro.z;
  const float cos_roll = cosf(roll);
  const float sin_roll = sinf(roll);
  const float cos_pitch = cosf(pitch);

  if (!isfinite(roll) || !isfinite(pitch) || !isfinite(p) || !isfinite(q) ||
      !isfinite(r) || fabsf(cos_pitch) < 0.1f) {
    g->feedback.eulr_rate = g->feedback.gyro;
    return;
  }

  const float pitch_rate = cos_roll * q - sin_roll * r;
  const float yaw_rate =
      (sin_roll * q + cos_roll * r) / cos_pitch;
  g->feedback.eulr_rate.x =
      p + sinf(pitch) * yaw_rate;
  g->feedback.eulr_rate.y = pitch_rate;
  g->feedback.eulr_rate.z = yaw_rate;
}

static void Gimbal_ApplyFeedforward(Gimbal_t *g) {
  if (g == NULL || g->mode != GIMBAL_MODE_ABSOLUTE ||
      gimbal_pitch_protect_active != 0U) {
    GimbalFriction_Reset(&gimbal_yaw_friction);
    GimbalFriction_Reset(&gimbal_pitch_friction);
    GimbalGravity_Reset(&gimbal_pitch_gravity);
    return;
  }

  const float yaw_error = CircleError(g->setpoint.eulr.yaw,
                                      g->yaw_control_feedback, M_2PI);
  if (gimbal_ladrc_yaw_enable != 0U &&
      g->ladrc[GIMBAL_ACTR_YAW_IDX].deadband_active != 0U) {
    g->out[GIMBAL_ACTR_YAW_IDX] +=
        GimbalFriction_Release(&gimbal_yaw_friction, g->dt);
  } else {
    g->out[GIMBAL_ACTR_YAW_IDX] += GimbalFriction_Update(
        &gimbal_yaw_friction, yaw_error, g->feedback.eulr_rate.z, g->dt);
  }

  if (gimbal_ladrc_pit_enable != 0U) {
    const float pitch_error = CircleError(g->pit_control_target,
                                          g->feedback.eulr.imu.pit, M_2PI);
    if (g->ladrc[GIMBAL_ACTR_PIT_IDX].deadband_active != 0U) {
      g->out[GIMBAL_ACTR_PIT_IDX] +=
          GimbalFriction_Release(&gimbal_pitch_friction, g->dt);
    } else {
      g->out[GIMBAL_ACTR_PIT_IDX] += GimbalFriction_Update(
          &gimbal_pitch_friction, pitch_error, g->feedback.eulr_rate.y,
          g->dt);
    }
    /* A calibrated gravity term still supports Pitch inside the deadband. */
    g->out[GIMBAL_ACTR_PIT_IDX] +=
        GimbalGravity_Update(&gimbal_pitch_gravity,
                             g->feedback.eulr.imu.pit);
  } else {
    GimbalFriction_Reset(&gimbal_pitch_friction);
    GimbalGravity_Reset(&gimbal_pitch_gravity);
  }
}

static uint8_t Gimbal_LADRCParamChanged(
    const LADRC_t *ctrl, const volatile LADRC_Params_t *param) {
  return ctrl->param.r != param->r || ctrl->param.b0 != param->b0 ||
         ctrl->param.wc != param->wc ||
         ctrl->param.w0 != param->w0 ||
         ctrl->param.out_limit != param->out_limit ||
         ctrl->param.deadband_rad != param->deadband_rad;
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

  return LADRC_UpdateAngle(ctrl, setpoint, feedback, angular_speed);
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

  /* Entering absolute mode starts from the current attitude.  Keep the last
   * setpoint while relaxed so STOP cannot appear as an unsolicited 0/360 deg
   * target jump in telemetry. */
  if (mode == GIMBAL_MODE_ABSOLUTE) {
    g->setpoint.eulr.yaw = g->yaw_control_feedback;
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
  memset(g->encoder_track, 0, sizeof(g->encoder_track));
  memset(g->encoder_sample_count, 0, sizeof(g->encoder_sample_count));
  memset(g->encoder_delta, 0, sizeof(g->encoder_delta));
  memset(g->encoder_fresh, 0, sizeof(g->encoder_fresh));
  memset(&g->yaw_assist, 0, sizeof(g->yaw_assist));
  g->yaw_control_feedback = 0.0f;

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

  const float encoder_angle[GIMBAL_ACTR_NUM] = {
      gimbal->feedback.eulr.encoder.yaw, gimbal->feedback.eulr.encoder.pit};
  const uint32_t now = osKernelGetTickCount();
  const uint32_t max_gap = (20U * osKernelGetTickFreq() + 999U) / 1000U;
  for (uint8_t i = 0U; i < GIMBAL_ACTR_NUM; i++) {
    gimbal->encoder_fresh[i] = 0U;
    gimbal->encoder_delta[i] = 0.0f;
    if (can->gimbal_feedback_count[i] == 0U ||
        can->gimbal_feedback_count[i] == gimbal->encoder_sample_count[i])
      continue;
    AngleTrack_t *track = &gimbal->encoder_track[i];
    const uint8_t continuous_sample =
        track->ready != 0U &&
        (uint32_t)(can->gimbal_feedback_tick[i] - track->sample_tick) <= max_gap &&
        (uint32_t)(now - can->gimbal_feedback_tick[i]) <= max_gap;
    const float previous = track->continuous;
    AngleTrack_Update(track, encoder_angle[i], can->gimbal_feedback_tick[i],
                       max_gap);
    if (continuous_sample != 0U) {
      gimbal->encoder_delta[i] = track->continuous - previous;
      gimbal->encoder_fresh[i] = 1U;
    }
    gimbal->encoder_sample_count[i] = can->gimbal_feedback_count[i];
  }

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

  Gimbal_UpdateEulerRate(g);
  const float assist_dt = g->dt > 0.0f && g->dt <= 0.02f ? g->dt : 0.002f;
  const float yaw_continuous = AngleAssist_Update(
      &g->yaw_assist, g->feedback.eulr.imu.yaw,
      g->feedback.eulr_rate.z * assist_dt,
      g->encoder_delta[GIMBAL_ACTR_YAW_IDX],
      g->encoder_fresh[GIMBAL_ACTR_YAW_IDX]);
  g->yaw_control_feedback = fmodf(yaw_continuous, M_2PI);
  if (g->yaw_control_feedback < 0.0f) g->yaw_control_feedback += M_2PI;
  Gimbal_SetMode(g, g_cmd->mode);
  gimbal_pitch_protect_active =
      fabsf(g->feedback.eulr.imu.pit) > GIMBAL_PITCH_FAULT_LIMIT_RAD;

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
  g->pit_control_target = JointLimit_Target(
      requested_pitch, -GIMBAL_PITCH_SAFE_LIMIT_RAD, GIMBAL_PITCH_SAFE_LIMIT_RAD);
  g->pit_limit_active =
      fabsf(g->pit_control_target - requested_pitch) > 0.00001f ||
      fabsf(g->feedback.eulr.imu.pit) >= GIMBAL_PITCH_SAFE_LIMIT_RAD;
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
      if (gimbal_pitch_protect_active != 0U) {
        for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->out[i] = 0.0f;
        for (uint8_t i = 0; i < GIMBAL_PID_NUM; i++) PID_Reset(g->pid + i);
        for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
          g->ladrc_active[i] = 0U;
          g->ladrc[i].output = 0.0f;
        }
        break;
      }

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
            g->yaw_control_feedback, g->feedback.eulr_rate.z,
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
            g->yaw_control_feedback, 0.0f, g->dt);
        g->out[GIMBAL_ACTR_YAW_IDX] = PID_Calc(
            &(g->pid[GIMBAL_PID_YAW_OMEGA_IDX]), yaw_omega_set_point,
            g->feedback.eulr_rate.z, 0.f, g->dt);
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
            g->feedback.eulr.imu.pit, g->feedback.eulr_rate.y,
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

  Gimbal_ApplyFeedforward(g);

  /* Reuse the existing two-pole filter; reconfigure only Pitch when requested.
   * Do this before STOP/deadband handling so neither leaves a filter tail. */
  const float pitch_cutoff = gimbal_pitch_output_filter_hz;
  if (g->filter_out[GIMBAL_ACTR_PIT_IDX].cutoff_freq != pitch_cutoff) {
    LowPassFilter2p_Init(&g->filter_out[GIMBAL_ACTR_PIT_IDX],
                         1.0f / g->ladrc[GIMBAL_ACTR_PIT_IDX].T, pitch_cutoff);
    LowPassFilter2p_Reset(&g->filter_out[GIMBAL_ACTR_PIT_IDX], 0.0f);
  }

  const float yaw_error =
      CircleError(g->setpoint.eulr.yaw, g->yaw_control_feedback, M_2PI);
  if (gimbal_yaw_dither_enable != 0U &&
      g->mode == GIMBAL_MODE_ABSOLUTE &&
      gimbal_pitch_protect_active == 0U &&
      (gimbal_ladrc_yaw_enable == 0U ||
       g->ladrc[GIMBAL_ACTR_YAW_IDX].deadband_active == 0U) &&
      fabsf(yaw_error) > gimbal_yaw_friction.deadband_rad &&
      fabsf(yaw_error) < 0.02617994f && /* 1.5 deg */
      fabsf(g->feedback.eulr_rate.z) < 0.01745329f) { /* 1 deg/s */
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
  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
    /* STOP and abnormal overtravel must bypass all output tails. */
    if (g->mode != GIMBAL_MODE_ABSOLUTE ||
        gimbal_pitch_protect_active != 0U) {
      LowPassFilter2p_Reset(g->filter_out + i, 0.0f);
      g->out[i] = 0.0f;
      continue;
    }
    /* Clear the filter tail as well as the controller/friction residuals.
     * Keep the current gravity feedforward when Pitch is settled. */
    if (g->mode == GIMBAL_MODE_ABSOLUTE && g->ladrc_active[i] != 0U &&
        g->ladrc[i].deadband_active != 0U)
      LowPassFilter2p_Reset(g->filter_out + i, g->out[i]);
    g->out[i] = LowPassFilter2p_Apply(g->filter_out + i, g->out[i]);

    /* Feedforward is part of the actuator command, so it must not bypass the
     * configured LADRC output limit.  The old path allowed a 0.30 limit to
     * become 0.32 after friction compensation during Yaw runaway. */
    float output_limit = GIMBAL_NORMALIZED_OUTPUT_MAX;
    if (i == GIMBAL_ACTR_YAW_IDX && gimbal_ladrc_yaw_enable != 0U)
      output_limit = gimbal_ladrc_yaw_param.out_limit;
    else if (i == GIMBAL_ACTR_PIT_IDX && gimbal_ladrc_pit_enable != 0U)
      output_limit = gimbal_ladrc_pit_param.out_limit;
    if (output_limit > GIMBAL_NORMALIZED_OUTPUT_MAX)
      output_limit = GIMBAL_NORMALIZED_OUTPUT_MAX;
    Clip(&g->out[i], -output_limit, output_limit);
  }
  if (gimbal_ladrc_pit_enable == 0U) g->out[GIMBAL_ACTR_PIT_IDX] = 0.0f;

  /* Normal limit is directional, never a mode change or a latched fault.
   * Apply after feedforward and filtering so neither can push past the limit. */
  const float limited_pitch_output = JointLimit_Output(
      g->feedback.eulr.imu.pit, g->out[GIMBAL_ACTR_PIT_IDX],
      -GIMBAL_PITCH_SAFE_LIMIT_RAD, GIMBAL_PITCH_SAFE_LIMIT_RAD);
  if (limited_pitch_output != g->out[GIMBAL_ACTR_PIT_IDX]) {
    g->out[GIMBAL_ACTR_PIT_IDX] = limited_pitch_output;
    LowPassFilter2p_Reset(&g->filter_out[GIMBAL_ACTR_PIT_IDX], 0.0f);
    LADRC_Reset(&g->ladrc[GIMBAL_ACTR_PIT_IDX],
                 g->feedback.eulr.imu.pit, g->feedback.eulr_rate.y);
    GimbalFriction_Reset(&gimbal_pitch_friction);
  }

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
