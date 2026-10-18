#pragma once

#include <math.h>
#include <stdint.h>

#define ANGLE_TRACK_TAU (6.28318530717958647692f)

/* Works for wrapped inputs and controller states that have accumulated turns. */
static inline float AngleTrack_Error(float target, float actual) {
  float delta = fmodf(target - actual, ANGLE_TRACK_TAU);
  if (delta > ANGLE_TRACK_TAU * 0.5f) delta -= ANGLE_TRACK_TAU;
  if (delta < -ANGLE_TRACK_TAU * 0.5f) delta += ANGLE_TRACK_TAU;
  return delta;
}

typedef struct {
  float previous;
  float continuous;
  uint32_t sample_tick;
  uint8_t ready;
} AngleTrack_t;

/* Dropout/reconnection rebases the single-turn sample without inventing turns.
 * The caller supplies fresh samples only and converts its timeout to ticks. */
static inline void AngleTrack_Update(AngleTrack_t *track, float angle,
                                     uint32_t tick, uint32_t max_gap_ticks) {
  if (!isfinite(angle)) return;
  if (track->ready == 0U) {
    track->continuous = angle;
    track->ready = 1U;
  } else if ((uint32_t)(tick - track->sample_tick) <= max_gap_ticks) {
    track->continuous += AngleTrack_Error(angle, track->previous);
  }
  track->previous = angle;
  track->sample_tick = tick;
}

typedef struct {
  float previous_imu;
  float continuous;
  uint32_t rejected_count;
  uint8_t ready;
} AngleAssist_t;

/* IMU measures world heading, encoder measures joint position. Never substitute
 * encoder angle directly: chassis rotation is real world-heading motion too.
 * Encoder motion can corroborate a gyro prediction when an IMU sample jumps. */
static inline float AngleAssist_Update(AngleAssist_t *assist, float imu,
                                        float gyro_delta, float encoder_delta,
                                        uint8_t encoder_valid) {
  if (!isfinite(imu) || !isfinite(gyro_delta)) return assist->continuous;
  if (assist->ready == 0U) {
    assist->previous_imu = imu;
    assist->continuous = imu;
    assist->ready = 1U;
    return imu;
  }
  const float imu_delta = AngleTrack_Error(imu, assist->previous_imu);
  assist->previous_imu = imu;
  /* 0.12 rad per sample is far above normal motion at 500 Hz. This rejects
   * jumps only when gyro and fresh motor motion both support a small step. */
  if (encoder_valid != 0U && fabsf(imu_delta - gyro_delta) > 0.12f &&
      fabsf(gyro_delta) < 0.06f && fabsf(encoder_delta) < 0.06f) {
    assist->continuous += gyro_delta;
    assist->rejected_count++;
  } else {
    assist->continuous += imu_delta;
  }
  return assist->continuous;
}
