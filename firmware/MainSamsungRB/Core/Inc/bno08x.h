/**
  ******************************************************************************
  * @file    bno08x.h
  * @brief   CEVA / Hillcrest Labs BNO080 / BNO085 / BNO086 SHTP driver for STM32.
  *
  * Hardware mapping on MainRB PCB:
  *   - J1 (8-pin IMU header): Pin1=+3.3V, Pin2=GND, Pin3=I2C3_SCL (PA8, R24=4.7k),
  *     Pin4=I2C3_SDA (PC9, R25=4.7k), Pin5=JP1 (SolderJumper_2_Open -> GND).
  *   - Default 7-bit address = 0x4A (BNO08X_I2C_ADDR_LO when SA0 is GND).
  *     Alternate 7-bit address = 0x4B (BNO08X_I2C_ADDR_HI when SA0 is 3V3).
  ******************************************************************************
  */

#ifndef BNO08X_H_
#define BNO08X_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UNIT_TEST_HOST
#include "mock_hal.h"
#else
#include "main.h"
#endif
#include <stdbool.h>
#include <stdint.h>

#define BNO08X_I2C_ADDR_LO          0x4AU
#define BNO08X_I2C_ADDR_HI          0x4BU

#define BNO08X_CHANNEL_COMMAND      0U
#define BNO08X_CHANNEL_EXECUTABLE   1U
#define BNO08X_CHANNEL_CONTROL      2U
#define BNO08X_CHANNEL_REPORTS      3U
#define BNO08X_CHANNEL_WAKE_REPORTS 4U
#define BNO08X_CHANNEL_GYRO_ROTVEC  5U

#define BNO08X_REPORT_ROT_VECTOR    0x05U  /* 9-DOF Geomagnetic Rotation Vector */
#define BNO08X_REPORT_GAME_ROT_VEC  0x08U  /* 6-DOF Gyro+Accel Game Rotation Vector */
#define BNO08X_REPORT_ARVR_GAME_RV  0x29U  /* 6-DOF ARVR-Stabilized Game Rotation Vector (Anti-shock, zero mag interference!) */

#define BNO08X_CMD_REQUEST          0xF2U
#define BNO08X_CMD_SET_FEATURE      0xFDU
#define BNO08X_REPORT_BASE_TS       0xFBU

#define BNO08X_SH2_CMD_SAVE_DCD     0x06U  /* Save Dynamic Calibration Data to internal Flash */
#define BNO08X_SH2_CMD_ME_CALIBRATE 0x07U  /* Configure Accel/Gyro/Mag dynamic calibration */
#define BNO08X_SH2_CMD_CLEAR_DCD    0x0BU  /* Clear Dynamic Calibration Data & reset sensor hub */

typedef struct
{
  float   qw;
  float   qx;
  float   qy;
  float   qz;
  float   yaw_deg;
  float   pitch_deg;
  float   roll_deg;
  uint8_t accuracy;
  bool    has_new_data;
} BNO08X_Rotation_t;

/* Probes bus for BNO08x at 0x4A or 0x4B */
bool BNO08X_Detect(I2C_HandleTypeDef *hi2c, uint8_t *found_addr);

/* Initializes BNO08x and enables Game Rotation Vector (6-DOF, 0 drift) at specified interval */
bool BNO08X_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint32_t report_interval_us);

/* Polls BNO08x for newest rotation vector report. Returns true if new orientation parsed */
bool BNO08X_Poll(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, BNO08X_Rotation_t *rot);

/* Persists current RAM Accel+Gyro+Mag calibration into BNO085 internal non-volatile Flash (DCD) */
bool BNO08X_SaveCalibration(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);

/* Clears old DCD calibration, resets BNO085, and re-enables 9-DOF Rotation Vector + background cal */
bool BNO08X_ClearCalibration(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint32_t report_interval_us);

#ifdef __cplusplus
}
#endif

#endif /* BNO08X_H_ */

