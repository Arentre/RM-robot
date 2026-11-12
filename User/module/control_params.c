#include "module/control_params.h"

/* 集中维护 PID、LADRC 和前馈默认参数。 */

/* PID 的输出为归一化指令，电机转速单位为 RPM，角度单位为弧度。
 * Flash 可能选用英雄车配置，因此保留相应参数。 */
/* 依次为增益、比例、积分、微分、积分限幅、输出限幅、微分截止频率和回绕范围。 */
#define PID_PARAMS(k_, p_, i_, d_, integral_, output_, cutoff_, range_) \
  { .k = (k_), .p = (p_), .i = (i_), .d = (d_), .i_limit = (integral_), \
    .out_limit = (output_), .d_cutoff_freq = (cutoff_), .range = (range_) }

#define TUNE_CHASSIS_WHEEL PID_PARAMS(0.01f / CHASSIS_M3508_GEAR_RATIO, 1.0f, 0.0f, 0.0f, 20.0f, 4.0f / CAN_M3508_MAX_ABS_CUR, 20.0f, -1.0f)
#define TUNE_CHASSIS_FOLLOW PID_PARAMS(0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, -1.0f, M_2PI)
#define TUNE_GIMBAL_YAW_OMEGA PID_PARAMS(0.05f, 1.0f, 0.0f, 0.0f, 0.0f, 0.15f, -1.0f, -1.0f)
#define TUNE_GIMBAL_YAW_ANGLE PID_PARAMS(2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2.0f, -1.0f, M_2PI)
#define TUNE_GIMBAL_PIT_OMEGA PID_PARAMS(0.05f, 1.0f, 0.0f, 0.0f, 0.0f, 0.10f, -1.0f, -1.0f)
#define TUNE_GIMBAL_PIT_ANGLE PID_PARAMS(2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, M_2PI)
#define TUNE_FRICTION_SPEED PID_PARAMS(0.001f, 1.0f, 0.2f, 0.01f, 0.5f, 0.5f, -1.0f, 0.0f)
#define TUNE_FEEDER_ANGLE PID_PARAMS(12.0f, 1.0f, 0.0f, 0.0450000018f, 1.0f, 1.0f, -1.0f, M_2PI)
#define TUNE_GIMBAL_PIT_OMEGA_HERO PID_PARAMS(0.03f, 1.0f, 0.0f, 0.0f, 0.0f, 0.10f, -1.0f, -1.0f)
#define TUNE_GIMBAL_PIT_ANGLE_HERO PID_PARAMS(1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, M_2PI)
#define TUNE_FEEDER_ANGLE_HERO PID_PARAMS(10.0f, 1.0f, 0.0f, 0.032f, 1.0f, 1.0f, -1.0f, M_2PI)

/* clang-format off */
#ifdef DEBUG
Config_RobotParam_t param_default = {
#else
static const Config_RobotParam_t param_default = {
#endif
  .model = ROBOT_MODEL_INFANTRY,

  .chassis = { /* 底盘模块参数 */
    .type = CHASSIS_TYPE_OMNI_CROSS,

    .motor_pid_param = TUNE_CHASSIS_WHEEL,

    .follow_pid_param = TUNE_CHASSIS_FOLLOW,

    .low_pass_cutoff_freq = {
      .in = 45.0f,
      .out = -1.0f,
    },

		.reverse = {
			.yaw = false,
		},

  }, /* 底盘参数 */

  .gimbal = { /* 云台模块参数 */
    .pid = {
      [GIMBAL_PID_YAW_OMEGA_IDX] = TUNE_GIMBAL_YAW_OMEGA,
      [GIMBAL_PID_YAW_ANGLE_IDX] = TUNE_GIMBAL_YAW_ANGLE,
      [GIMBAL_PID_PIT_OMEGA_IDX] = TUNE_GIMBAL_PIT_OMEGA,
      [GIMBAL_PID_PIT_ANGLE_IDX] = TUNE_GIMBAL_PIT_ANGLE,
    }, /* 控制环参数 */

    .pitch_travel_rad = 1.05f,

    .low_pass_cutoff_freq = {
      .out = -1.0f,
      .gyro = 1000.0f,
    },

    .reverse = {
      .yaw = false,
      .pit = false,
    },
  }, /* 云台参数 */

  .shoot = { /* 射击模块参数 */

    .fric_pid_param = TUNE_FRICTION_SPEED,
    .trig_pid_param = TUNE_FEEDER_ANGLE,
    .low_pass_cutoff_freq = {
      .in = {
        .fric = -1.0f,
        .trig = -1.0f,
      },
      .out = {
        .fric = -1.0f,
        .trig = -1.0f,
      },
    },
    .num_trig_tooth = 10.0f,
    .trig_gear_ratio = 36.0f,
    .fric_radius = 0.03f,
    .cover_open_duty = 0.10f,
    .cover_close_duty = 0.050f,
    .model = SHOOT_MODEL_17MM,
    .bullet_speed = 25.f,
    .min_shoot_delay = (uint32_t)(1000.0f / 10.0f),
  }, /* 发射机构参数 */

  .can = {
    .chassis = BSP_CAN_1,
    .gimbal = BSP_CAN_2,
    .shoot = BSP_CAN_2,
    }, /* 总线配置 */
}; /* 默认机器人参数 */

static const Config_RobotParam_t param_hero = {
  .model = ROBOT_MODEL_HERO,

  .chassis = { /* 底盘模块参数 */
    .type = CHASSIS_TYPE_OMNI_CROSS,

    .motor_pid_param = TUNE_CHASSIS_WHEEL,

    .follow_pid_param = TUNE_CHASSIS_FOLLOW,

    .low_pass_cutoff_freq = {
      .in = 45.0f,
      .out = -1.0f,
    },

		.reverse = {
			.yaw = true,
		},
  }, /* 底盘参数 */

  .gimbal = { /* 云台模块参数 */
    .pid = {
      [GIMBAL_PID_YAW_OMEGA_IDX] = TUNE_GIMBAL_YAW_OMEGA,
      [GIMBAL_PID_YAW_ANGLE_IDX] = TUNE_GIMBAL_YAW_ANGLE,
      [GIMBAL_PID_PIT_OMEGA_IDX] = TUNE_GIMBAL_PIT_OMEGA_HERO,
      [GIMBAL_PID_PIT_ANGLE_IDX] = TUNE_GIMBAL_PIT_ANGLE_HERO,
    }, /* 控制环参数 */

		.pitch_travel_rad = 1.07685447f,

    .low_pass_cutoff_freq = {
      .out = -1.0f,
      .gyro = 1000.0f,
    },

    .reverse = {
      .yaw = true,
      .pit = false,
    },
  }, /* 云台参数 */

  .shoot = { /* 射击模块参数 */

    .fric_pid_param = TUNE_FRICTION_SPEED,
    .trig_pid_param = TUNE_FEEDER_ANGLE_HERO,
    .low_pass_cutoff_freq = {
      .in = {
        .fric = -1.0f,
        .trig = -1.0f,
      },
      .out = {
        .fric = -1.0f,
        .trig = -1.0f,
      },
    },
    .num_trig_tooth = 6.0f,
    .trig_gear_ratio = 3591.0f / 187.0f,
    .fric_radius = 0.03f,
    .cover_open_duty = 0.125f,
    .cover_close_duty = 0.075f,
    .model = SHOOT_MODEL_42MM,
    .bullet_speed = 16.0f,
    .min_shoot_delay = (uint32_t)(1000.0f / 10.0f),
  }, /* 发射机构参数 */

  .can = {
    .chassis = BSP_CAN_1,
    .gimbal = BSP_CAN_2,
    .shoot = BSP_CAN_2,
    }, /* 总线配置 */
}; /* 英雄机器人参数 */

static const Config_PilotCfg_t cfg_qs = {
  .param = {
		.sens_mouse = 0.06f,
		.sens_rc = 6.0f,
    .map = {
      .key_map[CMD_BEHAVIOR_FORE] = {CMD_ACTIVE_PRESSED, CMD_KEY_W},
      .key_map[CMD_BEHAVIOR_BACK] = {CMD_ACTIVE_PRESSED, CMD_KEY_S},
      .key_map[CMD_BEHAVIOR_LEFT] = {CMD_ACTIVE_PRESSED, CMD_KEY_A},
      .key_map[CMD_BEHAVIOR_RIGHT] = {CMD_ACTIVE_PRESSED, CMD_KEY_D},
      .key_map[CMD_BEHAVIOR_ACCELERATE] = {CMD_ACTIVE_PRESSED, CMD_KEY_SHIFT},
      .key_map[CMD_BEHAVIOR_DECELEBRATE] = {CMD_ACTIVE_PRESSED, CMD_KEY_CTRL},
      .key_map[CMD_BEHAVIOR_FIRE] = {CMD_ACTIVE_PRESSED, CMD_L_CLICK},
      .key_map[CMD_BEHAVIOR_FIRE_MODE] = {CMD_ACTIVE_PRESSING, CMD_R_CLICK},
      .key_map[CMD_BEHAVIOR_FOLLOWGIMBAL35] = {CMD_ACTIVE_PRESSING, CMD_KEY_E},
      .key_map[CMD_BEHAVIOR_OPENCOVER] =  {CMD_ACTIVE_PRESSING, CMD_KEY_F},
      .key_map[CMD_BEHAVIOR_REVTRIG] = {CMD_ACTIVE_PRESSING, CMD_KEY_R},
      .key_map[CMD_BEHAVIOR_ROTOR] = {CMD_ACTIVE_PRESSING, CMD_KEY_G},
    },
    .move = {
      .move_sense = 1.6f,
      .move_fast_sense = 2.4f,
      .move_slow_sense = 0.8f,
    },
    .screen = {
      .height = 1080,
      .width = 1920,
    },
  },
};

static const Config_PilotCfg_t cfg_zyma = {
  .param = {
		.sens_mouse = 0.06f,
		.sens_rc = 6.0f,
    .map = {
      .key_map[CMD_BEHAVIOR_FORE] = {CMD_ACTIVE_PRESSED, CMD_KEY_W},
      .key_map[CMD_BEHAVIOR_BACK] = {CMD_ACTIVE_PRESSED, CMD_KEY_S},
      .key_map[CMD_BEHAVIOR_LEFT] = {CMD_ACTIVE_PRESSED, CMD_KEY_A},
      .key_map[CMD_BEHAVIOR_RIGHT] = {CMD_ACTIVE_PRESSED, CMD_KEY_D},
      .key_map[CMD_BEHAVIOR_ACCELERATE] = {CMD_ACTIVE_PRESSED, CMD_KEY_SHIFT},
      .key_map[CMD_BEHAVIOR_DECELEBRATE] = {CMD_ACTIVE_PRESSED, CMD_KEY_CTRL},
      .key_map[CMD_BEHAVIOR_FIRE] = {CMD_ACTIVE_PRESSED, CMD_L_CLICK},
      .key_map[CMD_BEHAVIOR_FIRE_MODE] = {CMD_ACTIVE_PRESSING, CMD_R_CLICK},
      .key_map[CMD_BEHAVIOR_FOLLOWGIMBAL35] = {CMD_ACTIVE_PRESSING, CMD_KEY_E},
      .key_map[CMD_BEHAVIOR_OPENCOVER] =  {CMD_ACTIVE_PRESSING, CMD_KEY_F},
      .key_map[CMD_BEHAVIOR_REVTRIG] = {CMD_ACTIVE_PRESSING, CMD_KEY_R},
      .key_map[CMD_BEHAVIOR_ROTOR] = {CMD_ACTIVE_PRESSING, CMD_KEY_G},
    },
    .move = {
      .move_sense = 1.6f,
      .move_fast_sense = 2.4f,
      .move_slow_sense = 1.6f,
    },
  },
};

/* clang-format on */

const Config_RobotParamMap_t robot_param_map[] = {
    {"default", &param_default},
    {"infantry", &param_default},
    {"hero", &param_hero},
    {NULL, NULL},
};

const Config_PilotCfgMap_t pilot_cfg_map[] = {
    {"qs", &cfg_qs},
    {"zyma", &cfg_zyma},
    {NULL, NULL},
};

/* 默认使用 LADRC；Yaw 可回退到 PID，关闭 Pitch LADRC 后 Pitch 无输出。 */
volatile uint8_t gimbal_ladrc_yaw_enable = 1U;
volatile uint8_t gimbal_ladrc_pit_enable = 1U;
/* 仅在运行时调整 Pitch 输出滤波，默认旁路。 */
volatile float gimbal_pitch_output_filter_hz = 0.0f;

/* 这些增益来自轻载台架，装车后应重新辨识 b0。装车配置允许使用归一化指令的全量程。 */
volatile LADRC_Params_t gimbal_ladrc_yaw_param = {
    .r = 10.0f, .b0 = 500.0f, .wc = 12.0f, .w0 = 40.0f,
    .out_limit = GIMBAL_NORMALIZED_OUTPUT_MAX,
    .deadband_rad = 0.00872665f}; /* 0.5° */
volatile LADRC_Params_t gimbal_ladrc_pit_param = {
    .r = 13.0f, .b0 = 550.0f, .wc = 7.0f, .w0 = 22.0f,
    .out_limit = GIMBAL_NORMALIZED_OUTPUT_MAX,
    .deadband_rad = 0.00872665f}; /* 0.5° */
/* 低速起动附加助力，设为 0 时关闭对应轴。 */
volatile float gimbal_yaw_breakaway_max = 0.065f;
volatile float gimbal_pitch_breakaway_max = 0.075f;
volatile uint8_t gimbal_yaw_dither_enable = 0U;
volatile float gimbal_yaw_dither_amplitude = 0.008f;
volatile float gimbal_yaw_dither_frequency_hz = 25.0f;
volatile float gimbal_yaw_dither_output = 0.0f;

/* 摩擦和重力前馈的默认参数。 */
volatile GimbalFrictionComp_t gimbal_yaw_friction = {
    .enable = 1U,
    .static_output = 0.060f,
    .coulomb_output = 0.025f,
    .deadband_rad = 0.00349066f,
    .transition_rad_s = 0.08726646f,
    .slew_per_s = 0.20f,
    .output = 0.0f};

volatile GimbalFrictionComp_t gimbal_pitch_friction = {
    .enable = 1U,
    .static_output = 0.070f,
    .coulomb_output = 0.025f,
    .deadband_rad = 0.00523599f,
    .transition_rad_s = 0.08726646f,
    .slew_per_s = 0.20f,
    .output = 0.0f};

/* 在装配完成的 Pitch 机构上辨识补偿方向和幅值之前，保持重力补偿关闭。 */
volatile GimbalGravityComp_t gimbal_pitch_gravity = {
    .enable = 0U, .amplitude = 0.0f, .phase_rad = 0.0f, .output = 0.0f};

/* BMI088 加热控制参数。 */
const KPID_Params_t imu_temp_ctrl_pid_param = {
    .k = 0.15f,
    .p = 1.0f,
    .i = 0.0f,
    .d = 0.0f,
    .i_limit = 1.0f,
    .out_limit = 1.0f,
};

