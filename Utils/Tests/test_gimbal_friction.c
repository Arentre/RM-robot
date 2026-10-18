#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "gimbal_ff.h"

#define DEG_RAD 0.01745329252f

int main(void) {
  GimbalFrictionComp_t comp = {
      .enable = 1U,
      .static_output = 0.035f,
      .coulomb_output = 0.020f,
      .deadband_rad = 0.2f * DEG_RAD,
      .transition_rad_s = 5.0f * DEG_RAD,
      .slew_per_s = 0.12f,
      .output = 0.020f};

  /* Deadband release follows the configured slew instead of dropping torque. */
  const float released = GimbalFriction_Update(&comp, 0.0f, 0.0f, 0.01f);
  assert(fabsf(released - 0.0152f) < 0.000001f);
  assert(fabsf(GimbalFriction_Release(&comp, 0.01f) - 0.0104f) < 0.000001f);

  for (int i = 0; i < 3; ++i) GimbalFriction_Release(&comp, 0.01f);
  assert(comp.output == 0.0f);

  /* Active compensation still ramps toward the requested direction. */
  assert(GimbalFriction_Update(&comp, 1.0f * DEG_RAD, 0.0f, 0.01f) > 0.0f);

  /* High-speed motion toward the target releases assistance before crossing. */
  comp.output = 0.020f;
  assert(fabsf(GimbalFriction_Update(&comp, 1.0f * DEG_RAD,
                                     6.0f * DEG_RAD, 0.01f) -
               0.0152f) < 0.000001f);

  /* A sign reversal uses the fast release path, never the application slew. */
  comp.output = 0.020f;
  assert(fabsf(GimbalFriction_Update(&comp, -1.0f * DEG_RAD, 0.0f, 0.01f) -
               0.0152f) < 0.000001f);

  /* Moving away from the target may still receive braking compensation. */
  comp.output = 0.0f;
  assert(GimbalFriction_Update(&comp, -1.0f * DEG_RAD,
                               6.0f * DEG_RAD, 0.01f) < 0.0f);
  GimbalFriction_Reset(&comp);
  assert(comp.output == 0.0f);

  puts("PASS: friction release, reversal, and high-speed crossing");
  return 0;
}
