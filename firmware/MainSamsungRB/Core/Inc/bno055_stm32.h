#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#ifdef __cplusplus
  extern "C" {
#endif

#ifdef UNIT_TEST_HOST
#include "mock_hal.h"
#else
#include "main.h"
#endif
#include "bno055.h"
#include "bno08x.h"
#include <stdbool.h>
#include <stdint.h>

#define BNO055_I2C_TIMEOUT_MS         25U
#define BNO055_POR_WAIT_MS            1200U
#define BNO055_CONSEC_ERR_LIMIT       3U
#define BNO055_RECOVER_BACKOFF_MS     250U
#define BNO055_CALIB_POLL_DIV         20U
#define BNO055_QUAT_NORM2_MIN         0.85f
#define BNO055_QUAT_NORM2_MAX         1.15f
#define BNO055_MAX_TICK_YAW_STEP_DEG  25.0f
#define BNO055_SPIKE_DT_MS            25U
#define BNO055_DEFAULT_FUSION_MODE    BNO055_OPERATION_MODE_NDOF
#define BNO055_FLASH_CAL_DATA_LEN     22U
#define BNO055_FLASH_CAL_SECTOR_ADDR  0x080E0000UL
#define BNO055_DRIFT_SETTLE_MS        500U
#define BNO055_DRIFT_WINDOW_MS        1000U
#define BNO055_DRIFT_MAX_TICK_DEG     0.35f
#define BNO055_DRIFT_MAX_BIAS_DPS     0.60f
#define BNO055_DRIFT_EMA_ALPHA        0.35f

typedef struct
{
  bno055_vector_t            euler;
  bno055_calibration_state_t calib;
  float                      yaw_raw;
  float                      yaw_prev;
  float                      yaw_total;
  float                      yaw_offset;
  float                      yaw;
  float                      roll_raw;
  float                      roll_prev;
  float                      roll_total;
  float                      roll_offset;
  float                      roll;
  float                      pitch_raw;
  float                      pitch_prev;
  float                      pitch_total;
  float                      pitch_offset;
  float                      pitch;
  bno055_opmode_t            op_mode;
  uint8_t                    i2c_addr;
  uint8_t                    chip_id;
  bool                       connected;
  bool                       initialized;
  bool                       healthy;
  bool                       resync_yaw_prev;
  bool                       stationary_hint;
  bool                       has_prev_sample_ms;
  uint8_t                    consec_errs;
  uint8_t                    calib_div;
  uint32_t                   ok_count;
  uint32_t                   err_count;
  uint32_t                   recoveries;
  uint32_t                   mode_resets;
  uint32_t                   quat_err_count;
  uint32_t                   spike_count;
  uint32_t                   bias_updates;
  uint8_t                    sys_status;
  uint8_t                    sys_err;
  float                      last_bad_qn2;
  float                      last_spike_deg;
  uint32_t                   last_ok_ms;
  uint32_t                   next_recover_ms;
  uint32_t                   prev_sample_ms;
  uint32_t                   still_ms;
  uint32_t                   still_win_ms;
  float                      still_raw_deg;
  float                      yaw_bias_dps;
  bool                       software_drift_comp; /* true = enable software ZRO drift estimator (default false) */
  bool                       has_saved_cal;       /* true = 22B BNO055 cal profile active in RAM/Flash */
  bool                       cal_saved_to_flash;  /* true = 22B BNO055 cal persisted to STM32 Flash Sector 11 */
  uint8_t                    saved_cal_data[BNO055_FLASH_CAL_DATA_LEN];
} BNO055_IMU_t;

extern I2C_HandleTypeDef *_bno055_i2c_port;

/* Low-level STM32 HAL transport glue for bno055.c (thread context only, never call from IRQ prio <= SysTick) */
void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device);
void bno055_assignAddress(uint8_t addr_7bit);
uint8_t bno055_getAddress(void);
HAL_StatusTypeDef bno055_getLastI2CStatus(void);
void bno055_delay(int time);
void bno055_writeData(uint8_t reg, uint8_t data);
void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len);

/* High-level MainRB IMU wrapper (I2C3 J1, continuous 360-deg unwrap + offset tare) */
bool BNO055_IMU_InitWithMode(BNO055_IMU_t *imu, I2C_HandleTypeDef *hi2c, bno055_opmode_t op_mode);
bool BNO055_IMU_Init(BNO055_IMU_t *imu, I2C_HandleTypeDef *hi2c);
void BNO055_IMU_SetStationaryHint(BNO055_IMU_t *imu, bool stationary);
bool BNO055_IMU_Update(BNO055_IMU_t *imu);
void BNO055_IMU_ZeroYaw(BNO055_IMU_t *imu);
void BNO055_IMU_ZeroAll(BNO055_IMU_t *imu);
bool BNO055_IMU_HasFlashCal(void);
bool BNO055_IMU_EnterAutoCalNDOF(BNO055_IMU_t *imu);
bool BNO055_IMU_SaveCalToFlash(BNO055_IMU_t *imu);
bool BNO055_IMU_EraseFlashCal(BNO055_IMU_t *imu);
#ifdef UNIT_TEST_HOST
void BNO055_IMU_FlashCorruptTestWord(uint32_t byte_offset, uint8_t value);
#endif

#ifdef __cplusplus
  }
#endif

#endif  // BNO055_STM32_H_
