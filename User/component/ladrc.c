#include "ladrc.h"
#include "angle_track.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define LADRC_DEADBAND_ENTER_RATE_RAD_S 0.03490659f /* 2 deg/s */
#define LADRC_DEADBAND_EXIT_RATE_RAD_S 0.05235988f  /* 3 deg/s */
#define LADRC_DEADBAND_SETTLE_TIME_S 0.020f

static float LADRC_Limit(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

float LADRC_UpdateAngle(LADRC_t *ctrl, float reference, float feedback,
                        float angular_speed) {
  if (ctrl == NULL) return 0.0f;
  /* Shift position states together; derivative histories are rates, not angles. */
  if (fabsf(ctrl->z1) >= ANGLE_TRACK_TAU) {
    const float offset = truncf(ctrl->z1 / ANGLE_TRACK_TAU) * ANGLE_TRACK_TAU;
    ctrl->z1 -= offset;
    ctrl->v1 -= offset;
  }
  const float continuous_feedback =
      ctrl->z1 + AngleTrack_Error(feedback, ctrl->z1);
  const float continuous_reference =
      continuous_feedback + AngleTrack_Error(reference, feedback);
  return LADRC_Update(ctrl, continuous_reference, continuous_feedback,
                       angular_speed);
}

int LADRC_Init(LADRC_t *ctrl, const volatile LADRC_Params_t *param, float T) {
  if (ctrl == NULL || param == NULL) return -1;

  LADRC_Params_t p = {.r = param->r,
                      .b0 = param->b0,
                      .wc = param->wc,
                      .w0 = param->w0,
                      .out_limit = param->out_limit,
                      .deadband_rad = param->deadband_rad};
  if (!isfinite(T) || T <= 0.0f || !isfinite(p.r) || p.r <= 0.0f ||
      !isfinite(p.b0) || fabsf(p.b0) < 0.000001f ||
      !isfinite(p.wc) || p.wc <= 0.0f ||
      !isfinite(p.w0) || p.w0 <= 0.0f ||
      !isfinite(p.out_limit) || p.out_limit <= 0.0f ||
      !isfinite(p.deadband_rad) || p.deadband_rad < 0.0f) return -1;

  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->param = p;
  ctrl->T = T;
  ctrl->half_T = T * 0.5f;
  ctrl->beta1 = 3.0f * p.w0;
  ctrl->beta2 = 3.0f * p.w0 * p.w0;
  ctrl->beta3 = p.w0 * p.w0 * p.w0;
  ctrl->kp = p.wc * p.wc;
  ctrl->kd = 2.0f * p.wc;
  return 0;
}

void LADRC_Reset(LADRC_t *ctrl, float angle, float angular_speed) {
  if (ctrl == NULL) return;
  if (!isfinite(angle)) angle = 0.0f;
  if (!isfinite(angular_speed)) angular_speed = 0.0f;

  ctrl->v1 = ctrl->z1 = angle;
  ctrl->v2 = ctrl->z2 = angular_speed;
  ctrl->z3 = 0.0f;
  ctrl->v1_i = ctrl->v2_i = 0.0f;
  ctrl->z1_i = ctrl->z2_i = ctrl->z3_i = 0.0f;
  ctrl->output = 0.0f;
  ctrl->deadband_settle_time = 0.0f;
  ctrl->deadband_active = 0U;
}

float LADRC_Update(LADRC_t *ctrl, float reference, float feedback,
                   float angular_speed) {
  if (ctrl == NULL || !isfinite(reference) || !isfinite(feedback)) return 0.0f;

  const float rate_feedback = isfinite(angular_speed) ? angular_speed : ctrl->z2;
  const float abs_error = fabsf(reference - feedback);
  const float abs_rate = fabsf(rate_feedback);

  /* Deadband is a settled-state hold, not an angle-only output switch.
   * A moving chassis can repeatedly carry the feedback through zero error;
   * measured-rate gating keeps derivative braking active during that pass. */
  if (ctrl->param.deadband_rad > 0.0f) {
    if (ctrl->deadband_active != 0U) {
      const uint8_t stay_settled =
          abs_error <= 1.5f * ctrl->param.deadband_rad &&
          abs_rate <= LADRC_DEADBAND_EXIT_RATE_RAD_S;
      if (stay_settled != 0U) {
        LADRC_Reset(ctrl, feedback, 0.0f);
        ctrl->deadband_active = 1U;
        return 0.0f;
      }
      ctrl->deadband_active = 0U;
      ctrl->deadband_settle_time = 0.0f;
    } else if (abs_error <= ctrl->param.deadband_rad &&
               abs_rate <= LADRC_DEADBAND_ENTER_RATE_RAD_S) {
      ctrl->deadband_settle_time += ctrl->T;
      if (ctrl->deadband_settle_time >= LADRC_DEADBAND_SETTLE_TIME_S) {
        LADRC_Reset(ctrl, feedback, 0.0f);
        ctrl->deadband_active = 1U;
        return 0.0f;
      }
    } else {
      ctrl->deadband_settle_time = 0.0f;
    }
  } else {
    ctrl->deadband_settle_time = 0.0f;
  }

  /* LTD: transition the requested angle smoothly. */
  const float fh = -ctrl->param.r * ctrl->param.r * (ctrl->v1 - reference)
                   - 2.0f * ctrl->param.r * ctrl->v2;
  ctrl->v2 += ctrl->half_T * (fh + ctrl->v2_i);
  ctrl->v1 += ctrl->half_T * (ctrl->v2 + ctrl->v1_i);
  ctrl->v2_i = fh;
  ctrl->v1_i = ctrl->v2;

  /* LESO: observe angle, speed and lumped disturbance using last output. */
  const float e = ctrl->z1 - feedback;
  const float dz3 = -ctrl->beta3 * e;
  const float dz2 = ctrl->z3 - ctrl->beta2 * e + ctrl->param.b0 * ctrl->output;
  const float dz1 = ctrl->z2 - ctrl->beta1 * e;
  ctrl->z3 += ctrl->half_T * (dz3 + ctrl->z3_i);
  ctrl->z2 += ctrl->half_T * (dz2 + ctrl->z2_i);
  ctrl->z1 += ctrl->half_T * (dz1 + ctrl->z1_i);
  ctrl->z3_i = dz3;
  ctrl->z2_i = dz2;
  ctrl->z1_i = dz1;

  /* LSEF: use the measured angular speed continuously in the derivative
   * feedback term.  Keep the LESO speed state for disturbance observation;
   * this avoids replacing the observer state with a noisy sensor sample. */
  const float u0 = ctrl->kp * (ctrl->v1 - ctrl->z1) +
                   ctrl->kd * (ctrl->v2 - rate_feedback);
  const float output = (u0 - ctrl->z3) / ctrl->param.b0;
  if (!isfinite(output) || !isfinite(ctrl->z1) || !isfinite(ctrl->z2) ||
      !isfinite(ctrl->z3) || !isfinite(ctrl->v1) || !isfinite(ctrl->v2)) {
    LADRC_Reset(ctrl, feedback, 0.0f);
    return 0.0f;
  }
  ctrl->output = LADRC_Limit(output, ctrl->param.out_limit);
  return ctrl->output;
}
