#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <usart.h>

#include "bsp/bsp.h"

/* Exported constants ------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */

/* 要添加使用UART的新设备，需要先在此添加对应的枚举值 */

/* UART实体枚举，与设备对应 */
typedef enum {
  BSP_UART_USART3,
  BSP_UART_USART1,
  BSP_UART_USART6,
  /* BSP_UART_XXX, */
  BSP_UART_NUM,
  BSP_UART_ERR,
} BSP_UART_t;

/* Physical connector UART2 maps to STM32 USART1. */
#define BSP_UART_ELRS BSP_UART_USART1
#define BSP_UART_AI BSP_UART_USART6

/* UART支持的中断回调函数类型，具体参考HAL中定义 */
typedef enum {
  BSP_UART_TX_HALF_CPLT_CB,
  BSP_UART_TX_CPLT_CB,
  BSP_UART_RX_HALF_CPLT_CB,
  BSP_UART_RX_CPLT_CB,
  BSP_UART_ERROR_CB,
  BSP_UART_ABORT_CPLT_CB,
  BSP_UART_ABORT_TX_CPLT_CB,
  BSP_UART_ABORT_RX_CPLT_CB,

  BSP_UART_IDLE_LINE_CB,
  BSP_UART_CB_NUM,
} BSP_UART_Callback_t;

/* Exported functions prototypes -------------------------------------------- */
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);
UART_HandleTypeDef *BSP_UART_GetHandle(BSP_UART_t uart);
int8_t BSP_UART_RegisterCallback(BSP_UART_t uart, BSP_UART_Callback_t type,
                                 void (*callback)(void));
int8_t BSP_UART_RegisterRxEventCallback(
    BSP_UART_t uart, void (*callback)(uint16_t size));

#ifdef __cplusplus
}
#endif
