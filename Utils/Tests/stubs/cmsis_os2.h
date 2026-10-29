#pragma once
#include <stddef.h>
#include <stdint.h>
typedef void *osThreadId_t;
typedef void *osMessageQueueId_t;
typedef void *osTimerId_t;
typedef int32_t osStatus_t;
#define osOK 0
uint32_t osKernelGetTickCount(void);
osThreadId_t osThreadGetId(void);
osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size,
                                    const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *item,
                             uint8_t priority, uint32_t timeout);
