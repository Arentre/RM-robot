/* Includes ----------------------------------------------------------------- */
#include "bsp/pwm.h"

#include <stdbool.h>
#include <main.h>
#include <tim.h>

/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
static bool pwm_running[2] = {false, false};
/* Private function  -------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
int8_t BSP_PWM_Start(BSP_PWM_Channel_t ch) {
  if ((uint32_t)ch >= 2U) return BSP_ERR;
  if (pwm_running[ch]) return BSP_OK;

  HAL_StatusTypeDef status;
  switch (ch) {
    case BSP_PWM_IMU_HEAT:
      status = HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
      break;
    case BSP_PWM_SHOOT_SERVO:
      status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      break;
    default:
      return BSP_ERR;
  }
  if (status != HAL_OK) return BSP_ERR;
  pwm_running[ch] = true;
  return BSP_OK;
}

int8_t BSP_PWM_Set(BSP_PWM_Channel_t ch, float duty_cycle) {
  if (duty_cycle > 1.0f) return BSP_ERR;
  if (duty_cycle < 0.0f) duty_cycle = 0.f;

  uint16_t pulse;

  /* 通过PWM通道对应定时器重载值和给定占空比，计算PWM周期值 */
  switch (ch) {
    case BSP_PWM_IMU_HEAT:
      pulse = (uint16_t)(duty_cycle * (float)__HAL_TIM_GET_AUTORELOAD(&htim10));
      break;
    case BSP_PWM_SHOOT_SERVO:
      pulse = (uint16_t)(duty_cycle * (float)__HAL_TIM_GET_AUTORELOAD(&htim1));
      break;
    default:
      return BSP_ERR;
  }

  if (pulse > 0) {
    switch (ch) {
      case BSP_PWM_IMU_HEAT:
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse);
        break;
      case BSP_PWM_SHOOT_SERVO:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        break;
      default:
        return BSP_ERR;
    }
    return BSP_PWM_Start(ch);
  } else {
    return BSP_PWM_Stop(ch);
  }
}

int8_t BSP_PWM_Stop(BSP_PWM_Channel_t ch) {
  if ((uint32_t)ch >= 2U) return BSP_ERR;
  if (!pwm_running[ch]) return BSP_OK;

  switch (ch) {
    case BSP_PWM_IMU_HEAT:
      HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(IMU_HEAT_PWM_GPIO_Port, IMU_HEAT_PWM_Pin,
                        GPIO_PIN_RESET);
      break;
    case BSP_PWM_SHOOT_SERVO:
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      break;
    default:
      return BSP_ERR;
  }
  pwm_running[ch] = false;
  return BSP_OK;
}
