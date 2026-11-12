#pragma once

#include <math.h>
#include <stdint.h>

/* 与 RTOS 和电机驱动无关的反馈及输出合理性检查。 */
#define GIMBAL_IMU_TIMEOUT_MS (20U)

/* 只统计已使能轴的连续过流。 */
static inline uint8_t GimbalSafety_Overcurrent(
    float current, uint8_t enabled, float limit_a, uint16_t limit_cycles,
    volatile uint16_t *count) {
  if (enabled == 0U) {
    *count = 0U;
    return 0U;
  }
  if (!isfinite(current)) return 1U;
  if (fabsf(current) <= limit_a) {
    *count = 0U;
    return 0U;
  }
  if (*count < limit_cycles) (*count)++;
  return *count >= limit_cycles;
}

static inline uint32_t GimbalSafety_AgeMs(uint32_t now, uint32_t sample_tick,
                                         uint32_t tick_frequency) {
  return (uint32_t)((uint64_t)(uint32_t)(now - sample_tick) * 1000U /
                    tick_frequency);
}

static inline uint8_t GimbalSafety_OutputValid(float yaw, float pitch) {
  return isfinite(yaw) && isfinite(pitch) && fabsf(yaw) <= 1.0f &&
         fabsf(pitch) <= 1.0f;
}

static inline uint8_t GimbalSafety_PitchJump(float previous, float current,
                                           uint32_t sample_delta_ticks,
                                           uint32_t tick_frequency) {
  /* 允许 5° 姿态解算修正量和 1000°/s 的真实转动。Pitch 在允许的 ±80° 范围内没有角度回绕。 */
  const float allowance = 0.08726646f + 17.45329252f *
      (float)sample_delta_ticks / (float)tick_frequency;
  return !isfinite(previous) || !isfinite(current) ||
         fabsf(current - previous) > allowance;
}
