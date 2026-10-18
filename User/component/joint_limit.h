#pragma once

/* All values use the same joint-positive coordinate, before motor reversal. */
static inline float JointLimit_Target(float requested, float lower, float upper) {
  if (requested > upper) return upper;
  if (requested < lower) return lower;
  return requested;
}

static inline float JointLimit_Output(float position, float output,
                                      float lower, float upper) {
  if (position >= upper && output > 0.0f) return 0.0f;
  if (position <= lower && output < 0.0f) return 0.0f;
  return output;
}
