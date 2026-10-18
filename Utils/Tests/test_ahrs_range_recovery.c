#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ahrs.h"

#define DEG_RAD 0.01745329252f

int main(void) {
  AHRS_t ahrs;
  AHRS_Eulr_t euler;
  const AHRS_Accl_t accel = {.x = 0.0f, .y = 0.0f, .z = 1.0f};
  const AHRS_Gyro_t overrange = {
      .x = 0.0f, .y = 0.0f, .z = 1001.0f * DEG_RAD};
  const float yaw = 45.0f * DEG_RAD;

  assert(AHRS_Init(&ahrs, 1000.0f) == 0);

  /* Model a converged runtime attitude before a gyro-range event. */
  ahrs.quat.q0 = cosf(0.5f * yaw);
  ahrs.quat.q1 = 0.0f;
  ahrs.quat.q2 = 0.0f;
  ahrs.quat.q3 = sinf(0.5f * yaw);
  ahrs.initialising = 0U;

  assert(AHRS_Update(&ahrs, &accel, &overrange) == 0);
  assert(ahrs.initialising != 0U);
  assert(ahrs.angular_rate_recovery != 0U);
  assert(AHRS_GetEulr(&euler, &ahrs) == 0);
  assert(euler.yaw > 40.0f * DEG_RAD);

  puts("PASS: runtime gyro-range recovery preserves Yaw heading");
  return 0;
}
