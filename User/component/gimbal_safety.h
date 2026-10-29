#pragma once

#include <math.h>
#include <stdint.h>

/* Physical plausibility checks independent of the RTOS and motor driver. */
#define GIMBAL_IMU_TIMEOUT_MS (20U)

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
  /* Allow 5 deg AHRS correction plus 1000 deg/s physical motion. Pitch has
   * no wrap boundary in the permitted +/-80 deg operating region. */
  const float allowance = 0.08726646f + 17.45329252f *
      (float)sample_delta_ticks / (float)tick_frequency;
  return !isfinite(previous) || !isfinite(current) ||
         fabsf(current - previous) > allowance;
}
