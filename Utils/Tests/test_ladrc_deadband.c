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

  /* 静摩擦可能造成扰动估计和输出残留。带少量噪声的到位反馈应持续 20 秒保持安静。 */
  ctrl.z3 = -100.0f;
  ctrl.output = 0.2f;
  for (int i = 0; i < 10000; ++i) {
    const float feedback = tolerance_deg * DEG_RAD * 0.5f * sinf(i * 0.37f);
    const float rate = 0.4f * DEG_RAD * cosf(i * 0.29f);
    const float output = LADRC_Update(&ctrl, 0.0f, feedback, rate);
    if (i >= 10) {
      assert(output == 0.0f);
      assert(ctrl.deadband_active != 0U);
      assert(ctrl.z3 == 0.0f);
    }
  }

  /* 滞回避免误差刚超过进入阈值时反复切换。 */
  assert(LADRC_Update(&ctrl, 0.0f, tolerance_deg * DEG_RAD * 1.2f, 0.0f) == 0.0f);
  assert(ctrl.deadband_active != 0U);
  LADRC_Update(&ctrl, 0.0f, tolerance_deg * DEG_RAD * 1.6f, 0.0f);
  assert(ctrl.deadband_active == 0U);
  LADRC_Update(&ctrl, 0.0f, tolerance_deg * DEG_RAD * 1.2f, 0.0f);
  assert(ctrl.deadband_active == 0U);
  for (int i = 0; i < 10; ++i)
    LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  assert(ctrl.deadband_active != 0U);

  /* 不能只凭角度到位；低速条件必须保持 20 毫秒。 */
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  for (int i = 0; i < 8; ++i)
    LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  LADRC_Update(&ctrl, 0.0f, 0.0f, 2.1f * DEG_RAD);
  assert(ctrl.deadband_active == 0U);
  for (int i = 0; i < 9; ++i) {
    LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
    assert(ctrl.deadband_active == 0U);
  }
  LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  assert(ctrl.deadband_active != 0U);

  /* 2.5°/s 时保持到位状态，超过 3°/s 立即退出。 */
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, 2.5f * DEG_RAD) == 0.0f);
  assert(ctrl.deadband_active != 0U);
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, 3.1f * DEG_RAD) < 0.0f);
  assert(ctrl.deadband_active == 0U);

  /* 快速经过目标时不能撤掉制动。 */
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, 3.0f * DEG_RAD) < 0.0f);
  assert(ctrl.deadband_active == 0U);
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, -3.0f * DEG_RAD) > 0.0f);

  /* 正负方向新的 20° 指令都应立即退出到位状态。 */
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  assert(LADRC_Update(&ctrl, 20.0f * DEG_RAD, 0.0f, 0.0f) > 0.0f);
  assert(ctrl.deadband_active == 0U);
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  LADRC_Update(&ctrl, 0.0f, 0.0f, 0.0f);
  assert(LADRC_Update(&ctrl, -20.0f * DEG_RAD, 0.0f, 0.0f) < 0.0f);
  assert(ctrl.deadband_active == 0U);

  /* 陀螺仪不可用时回退到观测器估计速度。 */
  LADRC_Reset(&ctrl, 0.0f, 0.0f);
  ctrl.z2 = 3.0f * DEG_RAD;
  assert(LADRC_Update(&ctrl, 0.0f, 0.0f, NAN) < 0.0f);
  assert(ctrl.deadband_active == 0U);
}

int main(void) {
  check_axis(0.2f, 12.0f, 40.0f);
  check_axis(0.3f, 8.0f, 30.0f);

  /* 新平台容许小于 1° 的误差，但 1° 阶跃应重新启动控制。 */
  check_axis(0.5f, 12.0f, 40.0f);
  check_axis(0.5f, 7.0f, 22.0f);
  LADRC_t relaxed = make_controller(0.5f,12.0f,40.0f);
  for (int i = 0; i < 16; ++i) LADRC_Update(&relaxed,0.0f,0.0f,0.0f);
  assert(relaxed.deadband_active != 0U);
  assert(LADRC_UpdateAngle(&relaxed,0.6f * DEG_RAD,0.0f,0.0f) == 0.0f);
  assert(LADRC_UpdateAngle(&relaxed,1.0f * DEG_RAD,0.0f,0.0f) > 0.0f);
  assert(relaxed.deadband_active == 0U);

  /* 零容差恢复连续控制。 */
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
