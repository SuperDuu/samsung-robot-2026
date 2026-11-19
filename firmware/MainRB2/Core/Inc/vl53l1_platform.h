/**
  ******************************************************************************
  * @file    vl53l1_platform.h
  * @brief   STM32 HAL I2C & host-mockable platform interface for ST VL53L1X ULD.
  ******************************************************************************
  */

#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

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

/* Default unshifted 7-bit address 0x29 -> 8-bit HAL I2C address 0x52 */
#define VL53L1_DEFAULT_ADDR_8BIT   0x52U
#define VL53L1_I2C_TIMEOUT_MS      10U
#define VL53L1_IO_ERROR            (-13)

/* Reference ID expected from VL53L1X_GetSensorId (0x010F..0x0110) */
#define VL53L1X_EXPECTED_MODEL_ID  0xEACCU

typedef struct
{
  uint32_t dummy;
} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

void VL53L1_Platform_BindI2C(I2C_HandleTypeDef *hi2c);
I2C_HandleTypeDef *VL53L1_Platform_GetI2C(void);
bool VL53L1_Platform_IsDeviceReady(uint16_t dev, uint32_t trials);
uint32_t VL53L1_Platform_GetLastHalError(void);

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L1_PLATFORM_H_ */

