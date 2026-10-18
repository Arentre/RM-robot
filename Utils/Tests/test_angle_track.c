#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "angle_track.h"
#include "ladrc.h"

int main(void) {
  const LADRC_Params_t param = {.r = 10.0f, .b0 = 500.0f, .wc = 12.0f,
      .w0 = 40.0f, .out_limit = 0.3f, .deadband_rad = 0.0f};
  LADRC_t original;
  assert(LADRC_Init(&original, &param, 0.002f) == 0);
  LADRC_Reset(&original, 0.4f, 0.2f);
  original.v1_i = 0.2f;
  original.z1_i = 0.1f;
  LADRC_t baseline = original;
  const float expected = LADRC_UpdateAngle(&baseline, 0.5f, 0.4f, 0.2f);
  /* Regression: old CircleError corrected only one turn, leaving a full turn
   * in the observer feedback mapping on the second/third revolution. */
  for (int turn = -100; turn <= 100; turn++) {
    const float observer = turn * ANGLE_TRACK_TAU + 0.4f;
    assert(fabsf(AngleTrack_Error(0.5f, observer) - 0.1f) < 0.0001f);
    assert(fabsf(AngleTrack_Error(-0.5f, -observer) + 0.1f) < 0.0001f);
    LADRC_t ctrl = original;
    ctrl.z1 += turn * ANGLE_TRACK_TAU;
    ctrl.v1 += turn * ANGLE_TRACK_TAU;
    const float output = LADRC_UpdateAngle(&ctrl, 0.5f, 0.4f, 0.2f);
    assert(fabsf(output - expected) < 0.0001f);
    assert(fabsf(AngleTrack_Error(ctrl.v1, baseline.v1)) < 0.0001f);
    assert(fabsf(AngleTrack_Error(ctrl.z1, baseline.z1)) < 0.0001f);
    assert(fabsf(ctrl.z1) < ANGLE_TRACK_TAU + 0.1f);
  }

  AngleTrack_t encoder = {0};
  AngleTrack_Update(&encoder, 6.27f, 10U, 20U);
  AngleTrack_Update(&encoder, 0.01f, 12U, 20U);
  assert(fabsf(encoder.continuous - (ANGLE_TRACK_TAU + 0.01f)) < 0.00001f);
  AngleTrack_Update(&encoder, 6.27f, 14U, 20U);
  assert(fabsf(encoder.continuous - 6.27f) < 0.00001f);
  const float before_gap = encoder.continuous;
  AngleTrack_Update(&encoder, 3.0f, 100U, 20U);
  assert(encoder.continuous == before_gap);
  AngleTrack_Update(&encoder, 3.01f, 102U, 20U);
  assert(fabsf(encoder.continuous - before_gap - 0.01f) < 0.00001f);

  /* Tick rollover must not look like a disconnection. */
  AngleTrack_t rollover = {0};
  AngleTrack_Update(&rollover, 1.0f, UINT32_MAX - 1U, 20U);
  AngleTrack_Update(&rollover, 1.01f, 1U, 20U);
  assert(fabsf(rollover.continuous - 1.01f) < 0.00001f);

  AngleAssist_t yaw = {0};
  AngleAssist_Update(&yaw, 3.13f, 0.0f, 0.0f, 1U);
  float value = AngleAssist_Update(&yaw, -3.13f, 0.0231853f, 0.0231853f, 1U);
  assert(fabsf(value - (ANGLE_TRACK_TAU - 3.13f)) < 0.00001f);
  const float before_jump = value;
  value = AngleAssist_Update(&yaw, -2.0f, 0.002f, 0.002f, 1U);
  assert(fabsf(value - before_jump - 0.002f) < 0.00001f);
  assert(yaw.rejected_count == 1U);
  value = AngleAssist_Update(&yaw, -1.998f, 0.002f, 0.002f, 1U);
  assert(fabsf(value - before_jump - 0.004f) < 0.00001f);
  /* Real chassis motion is not rejected when the joint encoder stays still. */
  value = AngleAssist_Update(&yaw, -1.978f, 0.02f, 0.0f, 1U);
  assert(fabsf(value - before_jump - 0.024f) < 0.00001f);
  assert(yaw.rejected_count == 1U);
  /* Missing encoder data must never be treated as stationary confirmation. */
  AngleAssist_Update(&yaw, -1.0f, 0.002f, 0.0f, 0U);
  assert(yaw.rejected_count == 1U);
  puts("angle tracking regression tests passed");
  return 0;
}
