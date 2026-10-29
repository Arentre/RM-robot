#pragma once
#include <stdint.h>
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp,
                         FilterMatchIndex; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC,
                         TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t FilterBank, FilterIdHigh, FilterIdLow, FilterMode,
                         FilterScale, FilterMaskIdHigh, FilterMaskIdLow,
                         FilterActivation, SlaveStartFilterBank,
                         FilterFIFOAssignment; } CAN_FilterTypeDef;
typedef struct { uint32_t bus; } CAN_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RX_FIFO0 0U
#define CAN_RX_FIFO1 1U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_RX_FIFO1_MSG_PENDING 2U
#define ENABLE 1U
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t,
                                       CAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,
    CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
