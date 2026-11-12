#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "gimbal_ff.h"

#define DEG_RAD 0.01745329252f

static void check_breakaway(int direction, float cap, float deadband) {
  GimbalBreakaway_t state = {0};
  const float target = direction * 1.0f * DEG_RAD;
  float peak = 0.0f;
  for (int i = 0; i < 500; ++i) {
    const float u = direction * GimbalBreakaway_Update(
        &state, target, target, 0.0f, direction * 0.001f,
        1.5f * deadband * DEG_RAD, cap, 0.002f);
    if (i < 59 || i > 326) assert(u == 0.0f);
    assert(u >= 0.0f && u <= cap);
    peak = fmaxf(peak,u);
  }
  assert(fabsf(peak-cap) < 1e-6f);
  assert(state.phase == GIMBAL_BREAKAWAY_DONE);
  const float new_target = target + direction * 0.2f * DEG_RAD;
  for (int i = 0; i < 80; ++i)
    GimbalBreakaway_Update(&state,new_target,target,0.0f,0.0f,
                           1.5f * deadband * DEG_RAD,cap,0.002f);
  assert(direction * state.output > 0.0f);
  assert(GimbalBreakaway_Update(&state,new_target,target,direction * DEG_RAD,
                               0.0f,0.0f,cap,0.002f) == 0.0f);
  assert(state.phase == GIMBAL_BREAKAWAY_DONE);
  GimbalBreakaway_Reset(&state);
  for (int i = 0; i < 80; ++i)
    GimbalBreakaway_Update(&state,target,target,0.0f,0.0f,
                           1.5f * deadband * DEG_RAD,cap,0.002f);
  assert(direction * state.output > 0.0f);
  assert(GimbalBreakaway_Update(&state,target,target,0.0f,
                               -direction * 0.01f,0.0f,cap,0.002f) == 0.0f);
  assert(state.phase == GIMBAL_BREAKAWAY_DONE);
  assert(GimbalBreakaway_Update(&state,target,target,0.0f,
                               0.0f,0.0f,0.0f,0.002f) == 0.0f);
  assert(state.phase == GIMBAL_BREAKAWAY_IDLE);
}

int main(void) {
  GimbalFrictionComp_t comp = {
      .enable = 1U,
      .static_output = 0.035f,
      .coulomb_output = 0.020f,
      .deadband_rad = 0.2f * DEG_RAD,
      .transition_rad_s = 5.0f * DEG_RAD,
      .slew_per_s = 0.12f,
      .output = 0.020f};

  /* 进入死区后按配置斜率撤力，不直接丢弃输出。 */
  const float released = GimbalFriction_Update(&comp, 0.0f, 0.0f, 0.01f);
  assert(fabsf(released - 0.0152f) < 0.000001f);
  assert(fabsf(GimbalFriction_Release(&comp, 0.01f) - 0.0104f) < 0.000001f);

  for (int i = 0; i < 3; ++i) GimbalFriction_Release(&comp, 0.01f);
  assert(comp.output == 0.0f);

  /* 补偿沿目标方向逐步增加。 */
  assert(GimbalFriction_Update(&comp, 1.0f * DEG_RAD, 0.0f, 0.01f) > 0.0f);

  /* 高速接近目标时，在越过目标前撤掉助力。 */
  comp.output = 0.020f;
  assert(fabsf(GimbalFriction_Update(&comp, 1.0f * DEG_RAD,
                                     6.0f * DEG_RAD, 0.01f) -
               0.0152f) < 0.000001f);

  /* 换向时快速释放，不沿用加力斜率。 */
  comp.output = 0.020f;
  assert(fabsf(GimbalFriction_Update(&comp, -1.0f * DEG_RAD, 0.0f, 0.01f) -
               0.0152f) < 0.000001f);

  /* 远离目标时仍允许提供制动方向的补偿。 */
  comp.output = 0.0f;
  assert(GimbalFriction_Update(&comp, -1.0f * DEG_RAD,
                               6.0f * DEG_RAD, 0.01f) < 0.0f);
  GimbalFriction_Reset(&comp);
  assert(comp.output == 0.0f);

  /* 正反两个方向都不能让残余补偿抵消 Yaw 刹车。 */
  for (int direction = -1; direction <= 1; direction += 2) {
    comp.output = direction * 0.020f;
    assert(GimbalFriction_YawUpdate(&comp, direction * DEG_RAD,
               direction * 2.0f * DEG_RAD, direction * -0.015f, 0.01f) == 0.0f);
    assert(comp.output == 0.0f);
    assert(direction * GimbalFriction_YawUpdate(&comp, direction * DEG_RAD,
               0.0f, direction * 0.015f, 0.01f) > 0.0f);
    check_breakaway(direction,0.065f,0.5f);
    check_breakaway(direction,0.075f,0.5f);
  }
  puts("PASS: friction release, braking, and bounded bidirectional breakaway");
  return 0;
}
