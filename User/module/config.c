/*
 * 配置相关
 */

#include "config.h"
#include "module/control_params.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "bsp/flash.h"

/* The deployed 32-bit Flash record contains legacy pointer slots and a
 * reserved magnetometer calibration block. Preserve BMI088 offsets. */
_Static_assert(offsetof(Config_t, cali.bmi088) == 72U, "Flash BMI088 layout changed");
_Static_assert(sizeof(Config_t) == 100U, "Flash config record size changed");

#define CONFIG_BASE_ADDRESS (ADDR_FLASH_SECTOR_11)

/**
 * \brief 从Flash读取配置信息
 *
 * \param cfg 配置信息
 */
void Config_Get(Config_t *cfg) {
  BSP_Flash_ReadBytes(CONFIG_BASE_ADDRESS, (uint8_t *)cfg, sizeof(*cfg));
  cfg->robot_param_name[sizeof(cfg->robot_param_name) - 1U] = '\0';
  cfg->pilot_cfg_name[sizeof(cfg->pilot_cfg_name) - 1U] = '\0';
  cfg->pilot_cfg = Config_GetPilotCfg(cfg->pilot_cfg_name);
  cfg->robot_param = Config_GetRobotParam(cfg->robot_param_name);
  /* 防止第一次烧写后访问NULL指针 */
  if (cfg->robot_param == NULL) cfg->robot_param = robot_param_map[0].param;
  if (cfg->pilot_cfg == NULL) cfg->pilot_cfg = pilot_cfg_map[0].param;
  /* 防止擦除后全为1 */
  if ((uint32_t)(cfg->robot_param) == UINT32_MAX)
    cfg->robot_param = robot_param_map[0].param;
  if ((uint32_t)(cfg->pilot_cfg) == UINT32_MAX) cfg->pilot_cfg = pilot_cfg_map[0].param;
}

/**
 * \brief 将配置信息写入Flash
 *
 * \param cfg 配置信息
 */
void Config_Set(Config_t *cfg) {
  osKernelLock();
  BSP_Flash_EraseSector(11);
  BSP_Flash_WriteBytes(CONFIG_BASE_ADDRESS, (uint8_t *)cfg, sizeof(*cfg));
  osKernelUnlock();
}

/**
 * @brief 通过机器人参数名称获取机器人参数的指针
 *
 * @param robot_param_name 机器人参数名称
 * @return const Config_RobotParam_t* 机器人参数的指针
 */
const Config_RobotParam_t *Config_GetRobotParam(const char *robot_param_name) {
  if (robot_param_name == NULL) return NULL;
  for (size_t j = 0; robot_param_map[j].name != NULL; j++) {
    if (strcmp(robot_param_map[j].name, robot_param_name) == 0) {
      return robot_param_map[j].param;
    }
  }
  return NULL; /* No match. */
}

/**
 * @brief 通过操作手配置名称获取操作手配置的指针
 *
 * @param pilot_cfg_name 操作手配置名称
 * @return const Config_PilotCfg_t* 操作手配置的指针
 */
const Config_PilotCfg_t *Config_GetPilotCfg(const char *pilot_cfg_name) {
  if (pilot_cfg_name == NULL) return NULL;
  for (size_t j = 0; pilot_cfg_map[j].name != NULL; j++) {
    if (strcmp(pilot_cfg_map[j].name, pilot_cfg_name) == 0) {
      return pilot_cfg_map[j].param;
    }
  }
  return NULL; /* No match. */
}

const Config_PilotCfgMap_t *Config_GetPilotNameMap(void) {
  return pilot_cfg_map;
}

const Config_RobotParamMap_t *Config_GetRobotNameMap(void) {
  return robot_param_map;
}
