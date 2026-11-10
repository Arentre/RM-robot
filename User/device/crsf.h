#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os2.h>
#include <stdbool.h>
#include <stdint.h>

#include "component/cmd.h"
#include "device/device.h"

#define CRSF_CHANNEL_COUNT (16u)
#define CRSF_RX_BUFFER_SIZE (64u)

typedef struct {
  uint8_t rx_buffer[CRSF_RX_BUFFER_SIZE];
  uint16_t channel[CRSF_CHANNEL_COUNT];
  volatile uint16_t rx_length;
  uint8_t link_quality;
  uint8_t link_statistics_seen;
  uint32_t valid_frame_count;
  uint32_t crc_error_count;
} CRSF_t;

int8_t CRSF_Init(CRSF_t *crsf);
int8_t CRSF_StartReceive(CRSF_t *crsf);
bool CRSF_WaitRC(CRSF_t *crsf, CMD_RC_t *rc, uint32_t timeout);
void CRSF_HandleOffline(CMD_RC_t *rc);

#ifdef __cplusplus
}
#endif
