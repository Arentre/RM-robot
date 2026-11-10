#include "gimbal_ff.h"

#include <math.h>
#include <stddef.h>

#define GIMBAL_FRICTION_RELEASE_SLEW_MULTIPLIER 4.0f

void GimbalFriction_Reset(volatile GimbalFrictionComp_t *comp) {
  if (comp != NULL) comp->output = 0.0f;
}

static float GimbalFriction_Approach(volatile GimbalFrictionComp_t *comp,
                                     float target, float slew_per_s,
                                     float dt) {
  float delta = target - comp->output;
  const float max_delta = slew_per_s * dt;
  if (delta > max_delta) delta = max_delta;
  if (delta < -max_delta) delta = -max_delta;
  comp->output += delta;
  return comp->output;
}

float GimbalFriction_Release(volatile GimbalFrictionComp_t *comp, float dt) {
  if (comp == NULL || !isfinite(dt) || dt <= 0.0f ||
      !isfinite(comp->slew_per_s) || comp->slew_per_s <= 0.0f ||
      !isfinite(comp->output)) {
    GimbalFriction_Reset(comp);
    return 0.0f;
  }
  /* Release faster than application so a direction reversal does not retain
   * stale feedforward torque for a large part of the next motion cycle. */
  return GimbalFriction_Approach(
      comp, 0.0f,
      comp->slew_per_s * GIMBAL_FRICTION_RELEASE_SLEW_MULTIPLIER, dt);
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
    return GimbalFriction_Release(comp, dt);
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

  /* Once the axis is already moving toward the target at transition speed,
   * static-friction assistance is no longer needed and only adds overshoot.
   * Release it before the target crossing instead of waiting for deadband. */
  if (angle_error * angular_speed > 0.0f &&
      fabsf(angular_speed) >= comp->transition_rad_s)
    return GimbalFriction_Release(comp, dt);

  /* A requested direction reversal must first remove the old feedforward at
   * the fast release rate.  Slewing through zero at the application rate left
   * stale torque for about 0.2 s in the recorded Yaw runaway. */
  if (comp->output * target < 0.0f)
    return GimbalFriction_Release(comp, dt);

  return GimbalFriction_Approach(comp, target, comp->slew_per_s, dt);
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
