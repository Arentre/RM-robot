#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Feedforward values are normalized GM6020 command units. */
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

void GimbalFriction_Reset(volatile GimbalFrictionComp_t *comp);
float GimbalFriction_Release(volatile GimbalFrictionComp_t *comp, float dt);
float GimbalFriction_Update(volatile GimbalFrictionComp_t *comp,
                            float angle_error, float angular_speed, float dt);
void GimbalGravity_Reset(volatile GimbalGravityComp_t *comp);
float GimbalGravity_Update(volatile GimbalGravityComp_t *comp,
                           float angle_rad);

#ifdef __cplusplus
}
#endif
