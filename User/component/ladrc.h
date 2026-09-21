#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Second-order LADRC: angle reference/feedback in rad, output normalized. */
typedef struct {
  float r;          /* tracking differentiator speed factor */
  float b0;         /* estimated plant input gain; sign sets control direction */
  float wc;         /* controller bandwidth, rad/s */
  float out_limit;  /* absolute normalized motor command limit */
} LADRC_Params_t;

typedef struct {
  LADRC_Params_t param;
  float T;
  float half_T;
  float beta1, beta2, beta3;
  float kp, kd;

  float v1, v2;       /* tracking differentiator */
  float z1, z2, z3;   /* extended state observer */
  float v1_i, v2_i;   /* previous derivatives for trapezoidal integration */
  float z1_i, z2_i, z3_i;
  float output;       /* saturated output; also LESO's previous input */
} LADRC_t;

/* Adapted from the supplied Sguan_Ladrc LTD + LESO + LSEF equations. */
int LADRC_Init(LADRC_t *ctrl, const volatile LADRC_Params_t *param, float T);
void LADRC_Reset(LADRC_t *ctrl, float angle, float angular_speed);
float LADRC_Update(LADRC_t *ctrl, float reference, float feedback);

#ifdef __cplusplus
}
#endif
