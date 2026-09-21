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
volatile uint8_t gimbal_ladrc_pit_enable = 1U;

/* 保守的首次上电参数。b0必须结合云台负载实机辨识。 */
volatile LADRC_Params_t gimbal_ladrc_yaw_param = {
    .r = 4.0f, .b0 = 100.0f, .wc = 1.0f, .out_limit = 0.03f};
volatile LADRC_Params_t gimbal_ladrc_pit_param = {
    .r = 4.0f, .b0 = 100.0f, .wc = 1.0f, .out_limit = 0.03f};
/* Private function  -------------------------------------------------------- */

static uint8_t Gimbal_LADRCParamChanged(
    const LADRC_t *ctrl, const volatile LADRC_Params_t *param) {
  return ctrl->param.r != param->r || ctrl->param.b0 != param->b0 ||
         ctrl->param.wc != param->wc ||
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
  const float delta_max =
      CircleError(g->limit.max,
                  (g->feedback.eulr.encoder.pit + g->setpoint.eulr.pit -
                   g->feedback.eulr.imu.pit),
                  M_2PI);
  const float delta_min =
      CircleError(g->limit.min,
                  (g->feedback.eulr.encoder.pit + g->setpoint.eulr.pit -
                   g->feedback.eulr.imu.pit),
                  M_2PI);
  Clip(&(g_cmd->delta_eulr.pit), delta_min, delta_max);
  g->setpoint.eulr.pit += g_cmd->delta_eulr.pit;

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
        for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->ladrc_active[i] = 0U;
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
            &(g->ladrc_active[GIMBAL_ACTR_PIT_IDX]), g->setpoint.eulr.pit,
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
