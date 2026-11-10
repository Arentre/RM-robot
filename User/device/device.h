#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_OK (0)
#define DEVICE_ERR (-1)
#define DEVICE_ERR_NULL (-2)
#define DEVICE_ERR_INITED (-3)
#define DEVICE_ERR_NO_DEV (-4)

/*
  SIGNAL是有bit位置区分的，一个SIGNAL只能包含一个高bit位。
  不同的SIGNAL不能有相同的高bit位。
*/
#define SIGNAL_BMI088_GYRO_NEW_DATA (1u << 0)
#define SIGNAL_BMI088_ACCL_NEW_DATA (1u << 1)
#define SIGNAL_BMI088_GYRO_RAW_REDY (1u << 2)
#define SIGNAL_BMI088_ACCL_RAW_REDY (1u << 3)

#define SIGNAL_CAN_MOTOR_RECV (1u << 4)



#define SIGNAL_CRSF_RAW_READY (1u << 13)

#define SIGNAL_AI_RAW_REDY (1u << 14)

#ifdef __cplusplus
}
#endif
