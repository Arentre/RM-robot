#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 前馈使用 GM6020 归一化指令单位。 */
typedef struct {
  uint8_t enable;
  float static_output;
  float coulomb_output;
  float deadband_rad;
  float transition_rad_s;
  float slew_per_s;
  volatile float output;
} GimbalFrictionComp_t;

typedef struct {
  uint8_t enable;
  float amplitude;
  float phase_rad;
  volatile float output;
} GimbalGravityComp_t;

extern volatile GimbalFrictionComp_t gimbal_yaw_friction;
extern volatile GimbalFrictionComp_t gimbal_pitch_friction;
extern volatile GimbalGravityComp_t gimbal_pitch_gravity;

/* 稳定目标只进行一次限幅起动助力；输出使用归一化单位。 */
enum {
  GIMBAL_BREAKAWAY_IDLE,
  GIMBAL_BREAKAWAY_WAIT,
  GIMBAL_BREAKAWAY_ACTIVE,
  GIMBAL_BREAKAWAY_DONE,
};
typedef struct {
  float reference;
  float elapsed_s;
  float output;
  uint8_t phase;
} GimbalBreakaway_t;

extern volatile float gimbal_yaw_breakaway_max;
extern volatile float gimbal_pitch_breakaway_max;
extern volatile GimbalBreakaway_t gimbal_yaw_breakaway;
extern volatile GimbalBreakaway_t gimbal_pitch_breakaway;

void GimbalBreakaway_Reset(volatile GimbalBreakaway_t *state);
float GimbalBreakaway_Update(volatile GimbalBreakaway_t *state,
                             float reference, float error, float speed,
                             float controller_output, float settle_error,
                             float max_output, float dt);

void GimbalFriction_Reset(volatile GimbalFrictionComp_t *comp);
float GimbalFriction_Release(volatile GimbalFrictionComp_t *comp, float dt);
float GimbalFriction_Update(volatile GimbalFrictionComp_t *comp,
                            float angle_error, float angular_speed, float dt);
float GimbalFriction_YawUpdate(volatile GimbalFrictionComp_t *comp,
                               float angle_error, float angular_speed,
                               float controller_output, float dt);
void GimbalGravity_Reset(volatile GimbalGravityComp_t *comp);
float GimbalGravity_Update(volatile GimbalGravityComp_t *comp,
                           float angle_rad);

#ifdef __cplusplus
}
#endif
