#include "gimbal_ff.h"

#include <math.h>
#include <stddef.h>

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

void GimbalFriction_Reset(volatile GimbalFrictionComp_t *comp) {
  if (comp != NULL) comp->output = 0.0f;
}

float GimbalFriction_Update(volatile GimbalFrictionComp_t *comp,
                            float angle_error, float angular_speed, float dt) {
  if (comp == NULL || comp->enable == 0U || !isfinite(angle_error) ||
      !isfinite(angular_speed) || !isfinite(dt) || dt <= 0.0f ||
      !isfinite(comp->static_output) || !isfinite(comp->coulomb_output) ||
      !isfinite(comp->deadband_rad) || !isfinite(comp->transition_rad_s) ||
      !isfinite(comp->slew_per_s) || comp->deadband_rad <= 0.0f ||
      comp->transition_rad_s <= 0.0f || comp->slew_per_s <= 0.0f) {
    GimbalFriction_Reset(comp);
    return 0.0f;
  }

  const float abs_error = fabsf(angle_error);
  if (abs_error <= comp->deadband_rad) {
    GimbalFriction_Reset(comp);
    return 0.0f;
  }

  float error_scale =
      (abs_error - comp->deadband_rad) / (2.0f * comp->deadband_rad);
  if (error_scale > 1.0f) error_scale = 1.0f;
  float speed_ratio = fabsf(angular_speed) / comp->transition_rad_s;
  if (speed_ratio > 1.0f) speed_ratio = 1.0f;
  const float magnitude =
      comp->static_output * (1.0f - speed_ratio) +
      comp->coulomb_output * speed_ratio;
  const float target = (angle_error > 0.0f ? 1.0f : -1.0f) * magnitude *
                       error_scale;
  float delta = target - comp->output;
  const float max_delta = comp->slew_per_s * dt;
  if (delta > max_delta) delta = max_delta;
  if (delta < -max_delta) delta = -max_delta;
  comp->output += delta;
  return comp->output;
}

void GimbalGravity_Reset(volatile GimbalGravityComp_t *comp) {
  if (comp != NULL) comp->output = 0.0f;
}

float GimbalGravity_Update(volatile GimbalGravityComp_t *comp,
                           float angle_rad) {
  if (comp == NULL || comp->enable == 0U || !isfinite(angle_rad) ||
      !isfinite(comp->amplitude) || !isfinite(comp->phase_rad)) {
    GimbalGravity_Reset(comp);
    return 0.0f;
  }
  comp->output = comp->amplitude * sinf(angle_rad + comp->phase_rad);
  return comp->output;
}
