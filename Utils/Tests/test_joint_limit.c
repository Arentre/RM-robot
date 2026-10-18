#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "joint_limit.h"

int main(void) {
  const float limit = 1.22173048f;
  float target = 0.0f;
  /* Sustained outward input cannot build up a hidden target beyond the stop. */
  for (int i = 0; i < 10000; i++)
    target = JointLimit_Target(target + 0.003f, -limit, limit);
  assert(target == limit);
  target = JointLimit_Target(target - 0.003f, -limit, limit);
  assert(target < limit);
  assert(JointLimit_Output(limit, 0.2f, -limit, limit) == 0.0f);
  assert(JointLimit_Output(limit, -0.2f, -limit, limit) == -0.2f);
  assert(JointLimit_Output(limit + 0.01f, -0.2f, -limit, limit) == -0.2f);
  assert(JointLimit_Output(limit - 0.01f, 0.2f, -limit, limit) == 0.2f);

  for (int i = 0; i < 10000; i++)
    target = JointLimit_Target(target - 0.003f, -limit, limit);
  assert(target == -limit);
  target = JointLimit_Target(target + 0.003f, -limit, limit);
  assert(target > -limit);
  assert(JointLimit_Output(-limit, -0.2f, -limit, limit) == 0.0f);
  assert(JointLimit_Output(-limit, 0.2f, -limit, limit) == 0.2f);
  assert(JointLimit_Output(-limit - 0.01f, 0.2f, -limit, limit) == 0.2f);
  puts("joint limit regression tests passed");
  return 0;
}
