#include "gimbal_ff.h"
#include "angle_track.h"

#include <math.h>
#include <stddef.h>

#define GIMBAL_FRICTION_RELEASE_SLEW_MULTIPLIER 4.0f

void GimbalBreakaway_Reset(volatile GimbalBreakaway_t *state) {
  *state = (GimbalBreakaway_t){0};
}

float GimbalBreakaway_Update(volatile GimbalBreakaway_t *state,
                             float reference, float error, float speed,
                             float controller_output, float settle_error,
                             float max_output, float dt) {
  const float min_error = fmaxf(settle_error, 0.00139626f); /* 0.08° */
  const float abs_error = fabsf(error);
  const float abs_speed = fabsf(speed);
  if (!isfinite(reference) || !isfinite(error) || !isfinite(speed) ||
      !isfinite(controller_output) || !isfinite(settle_error) ||
      !isfinite(dt) || dt <= 0.0f || dt > 0.020f ||
      !isfinite(max_output) || max_output <= 0.0f) {
    GimbalBreakaway_Reset(state);
    return 0.0f;
  }
  if (state->phase != GIMBAL_BREAKAWAY_IDLE &&
      fabsf(AngleTrack_Error(reference, state->reference)) > 0.00174533f)
    GimbalBreakaway_Reset(state); /* 目标变化超过 0.1°，允许重新尝试。 */
  if (abs_error <= min_error) {
    if (abs_speed <= 0.00523599f) GimbalBreakaway_Reset(state);
    else if (state->phase != GIMBAL_BREAKAWAY_IDLE) state->phase = GIMBAL_BREAKAWAY_DONE;
    state->output = 0.0f;
    return 0.0f;
  }
  if (state->phase == GIMBAL_BREAKAWAY_DONE) return 0.0f;
  const float speed_limit =
      state->phase == GIMBAL_BREAKAWAY_ACTIVE ? 0.01396263f : 0.00523599f; /* 0.8°/s / 0.3°/s */
  if (abs_error > 0.03490659f || abs_speed > speed_limit ||
      controller_output * error < 0.0f || state->output * error < 0.0f) {
    if (state->phase == GIMBAL_BREAKAWAY_ACTIVE)
      state->phase = GIMBAL_BREAKAWAY_DONE;
    else
      GimbalBreakaway_Reset(state);
    state->output = 0.0f;
    return 0.0f;
  }
  if (state->phase == GIMBAL_BREAKAWAY_IDLE) {
    state->reference = reference;
    state->phase = GIMBAL_BREAKAWAY_WAIT;
  }
  state->elapsed_s += dt;
  if (state->elapsed_s < 0.120f) return 0.0f;
  if (state->elapsed_s >= 0.650f) {
    state->phase = GIMBAL_BREAKAWAY_DONE;
    state->output = 0.0f;
    return 0.0f;
  }
  state->phase = GIMBAL_BREAKAWAY_ACTIVE;
  const float cap = fminf(max_output, 0.08f);
  state->output = copysignf(fminf(cap, fabsf(state->output) + 0.20f * dt),
                            error);
  return state->output;
}

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
  /* 撤力比加力更快，避免换向时残留原方向的前馈力矩。 */
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

  /* 朝目标运动且达到过渡速度后，提前撤掉静摩擦助力，减小过冲。 */
  if (angle_error * angular_speed > 0.0f &&
      fabsf(angular_speed) >= comp->transition_rad_s)
    return GimbalFriction_Release(comp, dt);

  /* 换向时先快速撤掉旧方向的前馈。按加力斜率穿过零点，曾使 Yaw 残留约 0.2 秒的旧方向力矩。 */
  if (comp->output * target < 0.0f)
    return GimbalFriction_Release(comp, dt);

  return GimbalFriction_Approach(comp, target, comp->slew_per_s, dt);
}

void GimbalGravity_Reset(volatile GimbalGravityComp_t *comp) {
  if (comp != NULL) comp->output = 0.0f;
}

float GimbalFriction_YawUpdate(volatile GimbalFrictionComp_t *comp,
                               float angle_error, float angular_speed,
                               float controller_output, float dt) {
  /* 大惯量 Yaw 朝目标运动且控制器已开始刹车时，不让摩擦助力抵消制动力。 */
  if (!isfinite(controller_output) ||
      (fabsf(angular_speed) > 0.01745329f &&
       angle_error * angular_speed > 0.0f &&
       controller_output * angular_speed < 0.0f)) {
    GimbalFriction_Reset(comp);
    return 0.0f;
  }
  return GimbalFriction_Update(comp, angle_error, angular_speed, dt);
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
