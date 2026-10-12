#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ladrc.h"

#define DEG_RAD 0.01745329252f

static LADRC_t make_controller(float tolerance_deg, float wc, float w0) {
  LADRC_t ctrl;
  const LADRC_Params_t p = {
      .r = 10.0f, .b0 = 500.0f, .wc = wc, .w0 = w0,
      .out_limit = 0.3f, .deadband_rad = tolerance_deg * DEG_RAD};
  assert(LADRC_Init(&ctrl, &p, 0.002f) == 0);
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  return ctrl;
}

static void check_axis(float tolerance_deg, float wc, float w0) {
  LADRC_t ctrl = make_controller(tolerance_deg, wc, w0);

  /* Stiction left a large disturbance estimate and nonzero motor output.
   * Settled feedback with realistic small noise must stay quiet for 20 s. */
  ctrl.z3 = -100.0f;
  ctrl.output = 0.2f;
  for (int i = 0; i < 10000; ++i) {
    const float feedback = tolerance_deg * DEG_RAD * 0.5f * sinf(i * 0.37f);
    const float rate = 0.4f * DEG_RAD * cosf(i * 0.29f);
    assert(LADRC_Update(&ctrl, 0.0f, feedback, rate) == 0.0f);
    assert(ctrl.deadband_active != 0U);
    assert(ctrl.z3 == 0.0f);
  }

  /* Hysteresis avoids chatter just outside the entry tolerance. */
  assert(LADRC_Update(&ctrl, 0.0f, tolerance_deg * DEG_RAD * 1.2f, 0.0f) == 0.0f);
  assert(ctrl.deadband_active != 0U);
  LADRC_Update(&ctrl, 0.0f, tolerance_deg * DEG_RAD * 1.6f, 0.0f);
  assert(ctrl.deadband_active == 0U);
  LADRC_Update(&ctrl, 0.0f, tolerance_deg * DEG_RAD * 1.2f, 0.0f);
  assert(ctrl.deadband_active == 0U);
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f) == 0.0f);
  assert(ctrl.deadband_active != 0U);

  /* Do not remove braking while passing through the target at speed. */
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, 3.0f * DEG_RAD) < 0.0f);
  assert(ctrl.deadband_active == 0U);
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, -3.0f * DEG_RAD) > 0.0f);

  /* A new 20-degree command exits immediately in either direction. */
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  assert(LADRC_Update(&ctrl, 20.0f * DEG_RAD, 0.0f, 0.0f) > 0.0f);
  assert(ctrl.deadband_active == 0U);
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  assert(LADRC_Update(&ctrl, -20.0f * DEG_RAD, 0.0f, 0.0f) < 0.0f);
  assert(ctrl.deadband_active == 0U);

  /* Unavailable gyro still falls back to the observer's speed. */
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  ctrl.z2 = 3.0f * DEG_RAD;
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, NAN) < 0.0f);
  assert(ctrl.deadband_active == 0U);
}

int main(void) {
  check_axis(0.2f, 12.0f, 40.0f);
  check_axis(0.3f, 8.0f, 30.0f);

  /* Zero tolerance restores the continuous controller. */
  LADRC_t ctrl = make_controller(0.0f, 12.0f, 40.0f);
  assert(LADRC_Update(&ctrl, 0.1f * DEG_RAD, 0.0f, 0.0f) > 0.0f);
  assert(ctrl.deadband_active == 0U);
  assert(LADRC_Update(&ctrl, NAN, 0.0f, 0.0f) == 0.0f);
  LADRC_Params_t invalid = ctrl.param;
  invalid.deadband_rad = -1.0f;
  assert(LADRC_Init(&ctrl, &invalid, 0.002f) != 0);
  invalid.deadband_rad = NAN;
  assert(LADRC_Init(&ctrl, &invalid, 0.002f) != 0);

  puts("PASS: Yaw/Pitch settled noise, hysteresis, braking, +/-20 deg, gyro fallback");
  return 0;
}
