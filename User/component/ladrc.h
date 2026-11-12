#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 二阶 LADRC：目标和反馈角使用弧度，输出为归一化电机指令。 */
typedef struct {
  float r;          /* 跟踪微分器的响应速度系数 */
  float b0;         /* 估计的对象输入增益，符号决定控制方向 */
  float wc;         /* 控制器带宽，单位为弧度每秒 */
  float w0;         /* 观测器带宽，单位为弧度每秒，可独立于 wc 设置 */
  float out_limit;  /* 归一化电机指令的绝对值上限 */
  float deadband_rad; /* 到位角容差，设为 0 时关闭 */
} LADRC_Params_t;

typedef struct {
  LADRC_Params_t param;
  float T;
  float half_T;
  float beta1, beta2, beta3;
  float kp, kd;

  float v1, v2;       /* 跟踪微分器状态 */
  float z1, z2, z3;   /* 扩张状态观测器状态 */
  float v1_i, v2_i;   /* 梯形积分所需的上次导数 */
  float z1_i, z2_i, z3_i;
  float output;       /* 限幅输出，也是 LESO 的上次输入 */
  float deadband_settle_time; /* 进入到位状态前的小误差、低速持续时间 */
  uint8_t deadband_active;
} LADRC_t;

/* 根据 Sguan_Ladrc 的 LTD、LESO 和 LSEF 方程改写。 */
int LADRC_Init(LADRC_t *ctrl, const volatile LADRC_Params_t *param, float T);
void LADRC_Reset(LADRC_t *ctrl, float angle, float angular_speed);
/* 用实测角速度更新角度环的导数反馈。角度和角速度分别使用弧度和弧度每秒。 */
float LADRC_Update(LADRC_t *ctrl, float reference, float feedback,
                   float angular_speed);
/* 处理循环角度输入，连续转动时对观测器和跟踪微分器的位置状态重新定基。 */
float LADRC_UpdateAngle(LADRC_t *ctrl, float reference, float feedback,
                        float angular_speed);

#ifdef __cplusplus
}
#endif
