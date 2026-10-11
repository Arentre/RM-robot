#include "ladrc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float LADRC_Limit(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

int LADRC_Init(LADRC_t *ctrl, const volatile LADRC_Params_t *param, float T) {
  if (ctrl == NULL || param == NULL) return -1;

  LADRC_Params_t p = {.r = param->r,
                      .b0 = param->b0,
                      .wc = param->wc,
                      .w0 = param->w0,
                      .out_limit = param->out_limit};
  if (!isfinite(T) || T <= 0.0f || !isfinite(p.r) || p.r <= 0.0f ||
      !isfinite(p.b0) || fabsf(p.b0) < 0.000001f ||
      !isfinite(p.wc) || p.wc <= 0.0f ||
      !isfinite(p.w0) || p.w0 <= 0.0f ||
      !isfinite(p.out_limit) || p.out_limit <= 0.0f) return -1;

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
}

float LADRC_Update(LADRC_t *ctrl, float reference, float feedback,
                   float angular_speed) {
  if (ctrl == NULL || !isfinite(reference) || !isfinite(feedback)) return 0.0f;

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
  const float rate_feedback = isfinite(angular_speed) ? angular_speed : ctrl->z2;
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
