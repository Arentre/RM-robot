#include "module/control_params.h"

/* Single entry point for PID, LADRC and feedforward tuning. */

/* PID gains: normalized output, motor speed in RPM, angles in radians.
 * HERO differences are retained because Flash may select that hardware. */
/*                            k, p, i, d, integral limit, output limit, D cutoff, wrap range */
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

  }, /* chassis */

  .gimbal = { /* 云台模块参数 */
    .pid = {
      [GIMBAL_PID_YAW_OMEGA_IDX] = TUNE_GIMBAL_YAW_OMEGA,
      [GIMBAL_PID_YAW_ANGLE_IDX] = TUNE_GIMBAL_YAW_ANGLE,
      [GIMBAL_PID_PIT_OMEGA_IDX] = TUNE_GIMBAL_PIT_OMEGA,
      [GIMBAL_PID_PIT_ANGLE_IDX] = TUNE_GIMBAL_PIT_ANGLE,
    }, /* pid */

    .pitch_travel_rad = 1.05f,

    .low_pass_cutoff_freq = {
      .out = -1.0f,
      .gyro = 1000.0f,
    },

    .reverse = {
      .yaw = false,
      .pit = false,
    },
  }, /* gimbal */

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
  }, /* shoot */

  .can = {
    .chassis = BSP_CAN_1,
    .gimbal = BSP_CAN_2,
    .shoot = BSP_CAN_2,
    }, /* can */
}; /* param_default */

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
  }, /* chassis */

  .gimbal = { /* 云台模块参数 */
    .pid = {
      [GIMBAL_PID_YAW_OMEGA_IDX] = TUNE_GIMBAL_YAW_OMEGA,
      [GIMBAL_PID_YAW_ANGLE_IDX] = TUNE_GIMBAL_YAW_ANGLE,
      [GIMBAL_PID_PIT_OMEGA_IDX] = TUNE_GIMBAL_PIT_OMEGA_HERO,
      [GIMBAL_PID_PIT_ANGLE_IDX] = TUNE_GIMBAL_PIT_ANGLE_HERO,
    }, /* pid */

		.pitch_travel_rad = 1.07685447f,

    .low_pass_cutoff_freq = {
      .out = -1.0f,
      .gyro = 1000.0f,
    },

    .reverse = {
      .yaw = true,
      .pit = false,
    },
  }, /* gimbal */

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
  }, /* shoot */

  .can = {
    .chassis = BSP_CAN_1,
    .gimbal = BSP_CAN_2,
    .shoot = BSP_CAN_2,
    }, /* can */
}; /* param_hero */

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

/* Gimbal: LADRC enabled by default; PID is the Yaw fallback.
 * Disabling Pitch LADRC keeps Pitch output at zero. */
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

/* Friction/gravity feedforward defaults. */
volatile GimbalFrictionComp_t gimbal_yaw_friction = {
    .enable = 1U,
    .static_output = 0.035f,
    .coulomb_output = 0.020f,
    .deadband_rad = 0.00349066f,
    .transition_rad_s = 0.08726646f,
    .slew_per_s = 0.12f,
    .output = 0.0f};

volatile GimbalFrictionComp_t gimbal_pitch_friction = {
    .enable = 1U,
    .static_output = 0.020f,
    .coulomb_output = 0.010f,
    .deadband_rad = 0.00523599f,
    .transition_rad_s = 0.08726646f,
    .slew_per_s = 0.12f,
    .output = 0.0f};

/* Gravity compensation stays disabled until its sign and amplitude are
 * identified on the assembled pitch mechanism. */
volatile GimbalGravityComp_t gimbal_pitch_gravity = {
    .enable = 0U, .amplitude = 0.0f, .phase_rad = 0.0f, .output = 0.0f};

/* BMI088 heater control. */
const KPID_Params_t imu_temp_ctrl_pid_param = {
    .k = 0.15f,
    .p = 1.0f,
    .i = 0.0f,
    .d = 0.0f,
    .i_limit = 1.0f,
    .out_limit = 1.0f,
};

