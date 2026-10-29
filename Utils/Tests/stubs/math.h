/* Use Windows CRT math instead of TCC 0.9.27's x87-only inline fabsf. */
#ifndef GIMBAL_HOST_TEST_MATH_H
#define GIMBAL_HOST_TEST_MATH_H
#include <stdint.h>
double fabs(double);
int _finite(double);
#define isfinite(x) _finite((double)(x))
static inline float fabsf(float x) { return (float)fabs((double)x); }
static inline float HostTest_FloatBits(uint32_t bits) {
  union { uint32_t bits; float value; } result;
  result.bits = bits;
  return result.value;
}
#define NAN HostTest_FloatBits(0x7fc00000U)
#define INFINITY HostTest_FloatBits(0x7f800000U)
#endif
