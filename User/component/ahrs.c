/*
 * Fusion AHRS adapter for the existing gimbal interfaces.
 *
 * The update equations are adapted from x-io Technologies Fusion.
 * Copyright (c) 2021 x-io Technologies.
 * https://github.com/xioTechnologies/Fusion
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "ahrs.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define FUSION_INITIAL_GAIN (10.0f)
#define FUSION_INITIALISATION_PERIOD_S (3.0f)
#define FUSION_GAIN (0.5f)
#define FUSION_GYROSCOPE_RANGE_DPS (1000.0f)
#define FUSION_ACCELERATION_REJECTION_DEG (10.0f)
#define FUSION_RECOVERY_PERIOD_S (5.0f)

typedef AHRS_Gyro_t FusionVector_t;

static FusionVector_t Fusion_Vector(float x, float y, float z) {
  const FusionVector_t vector = {.x = x, .y = y, .z = z};
  return vector;
}

static FusionVector_t Fusion_VectorAdd(FusionVector_t a, FusionVector_t b) {
  return Fusion_Vector(a.x + b.x, a.y + b.y, a.z + b.z);
}

static FusionVector_t Fusion_VectorScale(FusionVector_t vector, float scalar) {
  return Fusion_Vector(vector.x * scalar, vector.y * scalar,
                       vector.z * scalar);
}

static FusionVector_t Fusion_VectorCross(FusionVector_t a, FusionVector_t b) {
  return Fusion_Vector(a.y * b.z - a.z * b.y,
                       a.z * b.x - a.x * b.z,
                       a.x * b.y - a.y * b.x);
}

static float Fusion_VectorDot(FusionVector_t a, FusionVector_t b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float Fusion_VectorMagnitudeSquared(FusionVector_t vector) {
  return Fusion_VectorDot(vector, vector);
}

static FusionVector_t Fusion_VectorNormalise(FusionVector_t vector) {
  const float magnitude_squared = Fusion_VectorMagnitudeSquared(vector);
  if (!isfinite(magnitude_squared) || magnitude_squared <= FLT_MIN)
    return Fusion_Vector(0.0f, 0.0f, 0.0f);
  return Fusion_VectorScale(vector, 1.0f / sqrtf(magnitude_squared));
}

static AHRS_Quaternion_t Fusion_QuaternionMultiply(
    AHRS_Quaternion_t a, AHRS_Quaternion_t b) {
  const AHRS_Quaternion_t result = {
      .q0 = a.q0 * b.q0 - a.q1 * b.q1 - a.q2 * b.q2 - a.q3 * b.q3,
      .q1 = a.q0 * b.q1 + a.q1 * b.q0 + a.q2 * b.q3 - a.q3 * b.q2,
      .q2 = a.q0 * b.q2 - a.q1 * b.q3 + a.q2 * b.q0 + a.q3 * b.q1,
      .q3 = a.q0 * b.q3 + a.q1 * b.q2 - a.q2 * b.q1 + a.q3 * b.q0,
  };
  return result;
}

static AHRS_Quaternion_t Fusion_QuaternionMultiplyVector(
    AHRS_Quaternion_t q, FusionVector_t vector) {
  const AHRS_Quaternion_t result = {
      .q0 = -q.q1 * vector.x - q.q2 * vector.y - q.q3 * vector.z,
      .q1 = q.q0 * vector.x + q.q2 * vector.z - q.q3 * vector.y,
      .q2 = q.q0 * vector.y - q.q1 * vector.z + q.q3 * vector.x,
      .q3 = q.q0 * vector.z + q.q1 * vector.y - q.q2 * vector.x,
  };
  return result;
}

static uint8_t Fusion_QuaternionNormalise(AHRS_Quaternion_t *q) {
  const float magnitude_squared = q->q0 * q->q0 + q->q1 * q->q1 +
                                  q->q2 * q->q2 + q->q3 * q->q3;
  if (!isfinite(magnitude_squared) || magnitude_squared <= FLT_MIN) return 0U;
  const float reciprocal = 1.0f / sqrtf(magnitude_squared);
  q->q0 *= reciprocal;
  q->q1 *= reciprocal;
  q->q2 *= reciprocal;
  q->q3 *= reciprocal;
  return 1U;
}

static FusionVector_t Fusion_HalfGravity(const AHRS_t *ahrs) {
  const AHRS_Quaternion_t *q = &ahrs->quat;
  return Fusion_Vector(q->q1 * q->q3 - q->q0 * q->q2,
                       q->q2 * q->q3 + q->q0 * q->q1,
                       q->q0 * q->q0 - 0.5f + q->q3 * q->q3);
}

static FusionVector_t Fusion_Feedback(FusionVector_t sensor,
                                      FusionVector_t reference) {
  FusionVector_t feedback = Fusion_VectorCross(sensor, reference);
  if (Fusion_VectorDot(sensor, reference) < 0.0f)
    feedback = Fusion_VectorNormalise(feedback);
  return feedback;
}

static int32_t Fusion_ClampInt(int32_t value, int32_t minimum,
                               int32_t maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static void Fusion_Reset(AHRS_t *ahrs) {
  ahrs->quat.q0 = 1.0f;
  ahrs->quat.q1 = 0.0f;
  ahrs->quat.q2 = 0.0f;
  ahrs->quat.q3 = 0.0f;
  ahrs->initialising = 1U;
  ahrs->angular_rate_recovery = 0U;
  ahrs->accelerometer_ignored = 0U;
  ahrs->ramped_gain = FUSION_INITIAL_GAIN;
  ahrs->acceleration_recovery_trigger = 0;
  ahrs->acceleration_recovery_timeout = ahrs->recovery_trigger_period;
  ahrs->half_accelerometer_feedback = Fusion_Vector(0.0f, 0.0f, 0.0f);
}

static void Fusion_ZeroHeading(AHRS_t *ahrs) {
  const AHRS_Quaternion_t *q = &ahrs->quat;
  const float yaw = atan2f(q->q0 * q->q3 + q->q1 * q->q2,
                           0.5f - q->q2 * q->q2 - q->q3 * q->q3);
  const float half_yaw = 0.5f * yaw;
  const AHRS_Quaternion_t rotation = {
      .q0 = cosf(half_yaw),
      .q1 = 0.0f,
      .q2 = 0.0f,
      .q3 = -sinf(half_yaw),
  };
  ahrs->quat = Fusion_QuaternionMultiply(rotation, ahrs->quat);
}

int8_t AHRS_Init(AHRS_t *ahrs, float sample_freq) {
  if (ahrs == NULL || !isfinite(sample_freq) || sample_freq <= 0.0f)
    return -1;

  memset(ahrs, 0, sizeof(*ahrs));
  ahrs->inv_sample_freq = 1.0f / sample_freq;
  ahrs->gain = FUSION_GAIN;
  ahrs->gyroscope_range_dps = 0.98f * FUSION_GYROSCOPE_RANGE_DPS;
  const float half_sine =
      0.5f * sinf(FUSION_ACCELERATION_REJECTION_DEG * M_DEG2RAD_MULT);
  ahrs->acceleration_rejection = half_sine * half_sine;
  ahrs->recovery_trigger_period =
      (int32_t)(FUSION_RECOVERY_PERIOD_S * sample_freq);
  ahrs->ramped_gain_step =
      (FUSION_INITIAL_GAIN - ahrs->gain) /
      FUSION_INITIALISATION_PERIOD_S;
  Fusion_Reset(ahrs);
  return 0;
}

int8_t AHRS_Update(AHRS_t *ahrs, const AHRS_Accl_t *accl,
                   const AHRS_Gyro_t *gyro) {
  if (ahrs == NULL || accl == NULL || gyro == NULL) return -1;
  if (!isfinite(accl->x) || !isfinite(accl->y) || !isfinite(accl->z) ||
      !isfinite(gyro->x) || !isfinite(gyro->y) || !isfinite(gyro->z))
    return -1;

  const float dt = ahrs->inv_sample_freq;
  const FusionVector_t gyroscope_dps =
      Fusion_VectorScale(*gyro, M_RAD2DEG_MULT);
  const FusionVector_t accelerometer =
      Fusion_Vector(accl->x, accl->y, accl->z);

  if (fabsf(gyroscope_dps.x) > ahrs->gyroscope_range_dps ||
      fabsf(gyroscope_dps.y) > ahrs->gyroscope_range_dps ||
      fabsf(gyroscope_dps.z) > ahrs->gyroscope_range_dps) {
    const AHRS_Quaternion_t quaternion = ahrs->quat;
    Fusion_Reset(ahrs);
    ahrs->quat = quaternion;
    ahrs->angular_rate_recovery = 1U;
  }

  if (ahrs->initialising != 0U) {
    ahrs->ramped_gain -= ahrs->ramped_gain_step * dt;
    if (ahrs->ramped_gain < ahrs->gain) {
      ahrs->ramped_gain = ahrs->gain;
      ahrs->initialising = 0U;
      ahrs->angular_rate_recovery = 0U;
    }
  }

  FusionVector_t half_accelerometer_feedback =
      Fusion_Vector(0.0f, 0.0f, 0.0f);
  ahrs->accelerometer_ignored = 1U;
  if (Fusion_VectorMagnitudeSquared(accelerometer) > FLT_MIN) {
    ahrs->half_accelerometer_feedback =
        Fusion_Feedback(Fusion_VectorNormalise(accelerometer),
                        Fusion_HalfGravity(ahrs));
    if (ahrs->initialising != 0U ||
        Fusion_VectorMagnitudeSquared(ahrs->half_accelerometer_feedback) <=
            ahrs->acceleration_rejection) {
      ahrs->accelerometer_ignored = 0U;
      ahrs->acceleration_recovery_trigger -= 9;
    } else {
      ahrs->acceleration_recovery_trigger += 1;
    }

    if (ahrs->acceleration_recovery_trigger >
        ahrs->acceleration_recovery_timeout) {
      ahrs->acceleration_recovery_timeout = 0;
      ahrs->accelerometer_ignored = 0U;
    } else {
      ahrs->acceleration_recovery_timeout = ahrs->recovery_trigger_period;
    }
    ahrs->acceleration_recovery_trigger = Fusion_ClampInt(
        ahrs->acceleration_recovery_trigger, 0,
        ahrs->recovery_trigger_period);
    if (ahrs->accelerometer_ignored == 0U)
      half_accelerometer_feedback = ahrs->half_accelerometer_feedback;
  }

  const FusionVector_t half_gyroscope =
      Fusion_VectorScale(gyroscope_dps, 0.5f * M_DEG2RAD_MULT);
  const FusionVector_t adjusted_half_gyroscope =
      Fusion_VectorAdd(half_gyroscope,
                       Fusion_VectorScale(half_accelerometer_feedback,
                                          ahrs->ramped_gain));
  const AHRS_Quaternion_t derivative =
      Fusion_QuaternionMultiplyVector(
          ahrs->quat, Fusion_VectorScale(adjusted_half_gyroscope, dt));
  ahrs->quat.q0 += derivative.q0;
  ahrs->quat.q1 += derivative.q1;
  ahrs->quat.q2 += derivative.q2;
  ahrs->quat.q3 += derivative.q3;
  if (Fusion_QuaternionNormalise(&ahrs->quat) == 0U) {
    Fusion_Reset(ahrs);
    return -1;
  }

  /* Zero heading only during cold-start alignment.  A runtime gyro-range
   * recovery preserves the last quaternion; zeroing its heading here creates
   * a discontinuous Yaw feedback jump while the mechanism is still moving. */
  if (ahrs->initialising != 0U && ahrs->angular_rate_recovery == 0U)
    Fusion_ZeroHeading(ahrs);
  return 0;
}

int8_t AHRS_GetEulr(AHRS_Eulr_t *eulr, const AHRS_t *ahrs) {
  if (eulr == NULL || ahrs == NULL) return -1;
  const AHRS_Quaternion_t *q = &ahrs->quat;
  const float half_minus_q2_squared = 0.5f - q->q2 * q->q2;
  eulr->rol = atan2f(q->q0 * q->q1 + q->q2 * q->q3,
                     half_minus_q2_squared - q->q1 * q->q1);
  float sin_pitch = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
  if (sin_pitch > 1.0f) sin_pitch = 1.0f;
  if (sin_pitch < -1.0f) sin_pitch = -1.0f;
  eulr->pit = asinf(sin_pitch);
  eulr->yaw = atan2f(q->q0 * q->q3 + q->q1 * q->q2,
                     half_minus_q2_squared - q->q3 * q->q3);
  return 0;
}

void AHRS_ResetEulr(AHRS_Eulr_t *eulr) {
  if (eulr != NULL) memset(eulr, 0, sizeof(*eulr));
}
