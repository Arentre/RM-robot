#pragma once

#include <math.h>
#include <stdint.h>

#define ANGLE_TRACK_TAU (6.28318530717958647692f)

/* 支持单圈角度与已累计多圈的控制状态。 */
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

/* 反馈断开重连后重新对齐单圈读数，不凭空累计转数。调用方只提交新采样，并将超时时间换算为节拍数。 */
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

/* IMU 测世界坐标下的航向，编码器测关节角。底盘转动也会改变世界航向，因此不能直接用编码器替换 IMU。
 * IMU 跳变时可用编码器运动佐证陀螺仪预测。 */
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
  /* 在 500 Hz 下，单帧 0.12 弧度远超正常运动。
   * 只有陀螺仪与新鲜电机反馈都支持小步运动时，才拒绝该跳变。 */
  if (encoder_valid != 0U && fabsf(imu_delta - gyro_delta) > 0.12f &&
      fabsf(gyro_delta) < 0.06f && fabsf(encoder_delta) < 0.06f) {
    assist->continuous += gyro_delta;
    assist->rejected_count++;
  } else {
    assist->continuous += imu_delta;
  }
  return assist->continuous;
}
