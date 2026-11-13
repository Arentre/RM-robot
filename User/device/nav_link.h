#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int32_t vx_mm_s;
  int32_t vy_mm_s;
  int32_t wz_mrad_s;
} Nav_Velocity_t;

#define NAV_LINK_COMMAND_TIMEOUT_MS (250U)

void NavLink_Init(void);
bool NavLink_GetCommand(Nav_Velocity_t *velocity, uint32_t *last_rx_tick);
void NavLink_SetMeasured(const Nav_Velocity_t *velocity);
void Task_NavLink(void *argument);
