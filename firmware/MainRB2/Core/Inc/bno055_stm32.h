#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include "main.h"
#include "bno055.h"
#include <stdbool.h>
#include <stdint.h>

#define BNO055_I2C_TIMEOUT_MS   20U
#define BNO055_POR_WAIT_MS      750U

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
  uint8_t                    i2c_addr;
  uint8_t                    chip_id;
  bool                       connected;
  bool                       initialized;
  uint32_t                   ok_count;
  uint32_t                   err_count;
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
bool BNO055_IMU_Init(BNO055_IMU_t *imu, I2C_HandleTypeDef *hi2c);
bool BNO055_IMU_Update(BNO055_IMU_t *imu);
void BNO055_IMU_ZeroYaw(BNO055_IMU_t *imu);
void BNO055_IMU_ZeroAll(BNO055_IMU_t *imu);

#ifdef __cplusplus
  }
#endif

#endif  // BNO055_STM32_H_
