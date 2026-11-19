/**
  ******************************************************************************
  * @file    tof_vl53l1x.h
  * @brief   Non-blocking multi-sensor VL53L1X manager (official ST ULD firmware
  *          configuration + XSHUT I2C address assignment on I2C2 / LS1..LS4).
  ******************************************************************************
  */

#ifndef INC_TOF_VL53L1X_H_
#define INC_TOF_VL53L1X_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UNIT_TEST_HOST
#include "mock_hal.h"
#else
#include "main.h"
#endif

#include "VL53L1X_api.h"
#include <stdbool.h>
#include <stdint.h>

#define TOF_MAX_SENSORS          4U
#define TOF_CLEAR_DISTANCE_MM    4000U
#define TOF_INVALID_DISTANCE_MM  0xFFFFU

#define TOF_DEFAULT_DIST_MODE    1U   /* 1 = Short (up to 1.3 m, high ambient immunity), 2 = Long (up to 4 m) */
#define TOF_DEFAULT_TB_MS        33U
#define TOF_DEFAULT_IM_MS        40U
#define TOF_POLL_SLOT_MS         5U
#define TOF_SAMPLE_STALE_MS      300U
#define TOF_MAX_CONSEC_I2C_ERRS  5U
#define TOF_RECOVER_BACKOFF_MS   500U

typedef enum
{
  TOF_SENSOR_FRONT = 0U, /* LS1 (PC8) -> 8-bit I2C 0x54 */
  TOF_SENSOR_REAR  = 1U, /* LS2 (PC7) -> 8-bit I2C 0x56 */
  TOF_SENSOR_AUX_L = 2U, /* LS3 (PC6) -> 8-bit I2C 0x58 */
  TOF_SENSOR_AUX_R = 3U  /* LS4 (PB13) -> 8-bit I2C 0x5A */
} ToF_SensorSlot_t;

typedef struct
{
  bool          enabled;
  GPIO_TypeDef *xshut_port;
  uint16_t      xshut_pin;
  uint8_t       addr_8bit;
} ToF_SlotConfig_t;

typedef struct
{
  uint16_t distance_mode;       /* 1 = Short, 2 = Long */
  uint16_t timing_budget_ms;    /* 15, 20, 33, 50, 100, 200, 500 */
  uint32_t inter_meas_ms;       /* >= timing_budget_ms */
  bool     use_2v8_io;          /* true = set 0x2E/0x2F EXTSUP bit for 2.8/3.3V bare pads */
  ToF_SlotConfig_t slots[TOF_MAX_SENSORS];
} ToF_Config_t;

typedef struct
{
  bool             configured;
  bool             online;
  bool             fw_loaded;
  uint8_t          addr_8bit;
  uint8_t          int_pol;
  uint16_t         model_id;
  uint8_t          last_range_status;
  uint16_t         raw_mm;
  uint16_t         filtered_mm;
  uint16_t         median_hist[3];
  uint8_t          median_idx;
  uint8_t          median_count;
  uint8_t          clear_streak;
  uint8_t          consec_errs;
  uint32_t         sample_count;
  uint32_t         last_sample_ms;
  uint32_t         next_recover_ms;
  VL53L1X_Result_t last_result;
} ToF_SensorState_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  ToF_Config_t       cfg;
  ToF_SensorState_t  sensors[TOF_MAX_SENSORS];
  uint8_t            online_mask;
  uint8_t            poll_cursor;
  bool               addr52_conflict;
  uint32_t           last_poll_ms;
  uint32_t           i2c_recoveries;
  uint32_t           sensor_reinits;
} ToF_Manager_t;

void ToF_DefaultConfig(ToF_Config_t *cfg);
uint8_t ToF_Init(ToF_Manager_t *mgr, I2C_HandleTypeDef *hi2c, const ToF_Config_t *cfg);
void ToF_Poll(ToF_Manager_t *mgr, uint32_t now_ms);

bool ToF_IsSensorFresh(const ToF_Manager_t *mgr, ToF_SensorSlot_t slot, uint32_t now_ms);
uint16_t ToF_GetDistanceMm(const ToF_Manager_t *mgr, ToF_SensorSlot_t slot, uint32_t now_ms);
bool ToF_IsDirectionBlocked(const ToF_Manager_t *mgr,
                            bool forward,
                            uint16_t threshold_mm,
                            uint32_t now_ms,
                            uint16_t *out_min_mm);

#ifdef __cplusplus
}
#endif

#endif /* INC_TOF_VL53L1X_H_ */

