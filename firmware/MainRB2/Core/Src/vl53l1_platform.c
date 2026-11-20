/**
  ******************************************************************************
  * @file    vl53l1_platform.c
  * @brief   STM32 HAL I2C2 (and host test mock) transport for ST VL53L1X ULD.
  ******************************************************************************
  */

#include "vl53l1_platform.h"

#ifdef UNIT_TEST_HOST
#define VL53L1_DELAY_MS(ms) HAL_Delay((ms))
#else
#define VL53L1_DELAY_MS(ms) HAL_Delay((ms))
#endif

static I2C_HandleTypeDef *s_vl53l1_hi2c = NULL;
static uint32_t s_last_hal_err = 0U;

void VL53L1_Platform_BindI2C(I2C_HandleTypeDef *hi2c)
{
  s_vl53l1_hi2c = hi2c;
}

I2C_HandleTypeDef *VL53L1_Platform_GetI2C(void)
{
  return s_vl53l1_hi2c;
}

uint32_t VL53L1_Platform_GetLastHalError(void)
{
  return s_last_hal_err;
}

bool VL53L1_Platform_IsDeviceReady(uint16_t dev, uint32_t trials)
{
  if (s_vl53l1_hi2c == NULL)
  {
    return false;
  }
  HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(s_vl53l1_hi2c,
                                               dev,
                                               trials,
                                               VL53L1_I2C_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    s_last_hal_err = HAL_I2C_GetError(s_vl53l1_hi2c);
    return false;
  }
  return true;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  if ((s_vl53l1_hi2c == NULL) || (pdata == NULL) || (count == 0U) || (count > 0xFFFFU))
  {
    return VL53L1_IO_ERROR;
  }

  if (HAL_I2C_Mem_Write(s_vl53l1_hi2c,
                        dev,
                        index,
                        I2C_MEMADD_SIZE_16BIT,
                        pdata,
                        (uint16_t)count,
                        VL53L1_I2C_TIMEOUT_MS) != HAL_OK)
  {
    s_last_hal_err = HAL_I2C_GetError(s_vl53l1_hi2c);
    return VL53L1_IO_ERROR;
  }
  return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  if ((s_vl53l1_hi2c == NULL) || (pdata == NULL) || (count == 0U) || (count > 0xFFFFU))
  {
    return VL53L1_IO_ERROR;
  }

  if (HAL_I2C_Mem_Read(s_vl53l1_hi2c,
                       dev,
                       index,
                       I2C_MEMADD_SIZE_16BIT,
                       pdata,
                       (uint16_t)count,
                       VL53L1_I2C_TIMEOUT_MS) != HAL_OK)
  {
    s_last_hal_err = HAL_I2C_GetError(s_vl53l1_hi2c);
    return VL53L1_IO_ERROR;
  }
  return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  return VL53L1_WriteMulti(dev, index, &data, 1U);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  uint8_t buf[2];
  buf[0] = (uint8_t)((data >> 8U) & 0xFFU);
  buf[1] = (uint8_t)(data & 0xFFU);
  return VL53L1_WriteMulti(dev, index, buf, 2U);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  uint8_t buf[4];
  buf[0] = (uint8_t)((data >> 24U) & 0xFFU);
  buf[1] = (uint8_t)((data >> 16U) & 0xFFU);
  buf[2] = (uint8_t)((data >> 8U) & 0xFFU);
  buf[3] = (uint8_t)(data & 0xFFU);
  return VL53L1_WriteMulti(dev, index, buf, 4U);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
  if (pdata == NULL)
  {
    return VL53L1_IO_ERROR;
  }
  return VL53L1_ReadMulti(dev, index, pdata, 1U);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
  uint8_t buf[2] = {0U, 0U};
  if (pdata == NULL)
  {
    return VL53L1_IO_ERROR;
  }
  if (VL53L1_ReadMulti(dev, index, buf, 2U) != 0)
  {
    return VL53L1_IO_ERROR;
  }
  *pdata = (uint16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
  return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
  uint8_t buf[4] = {0U, 0U, 0U, 0U};
  if (pdata == NULL)
  {
    return VL53L1_IO_ERROR;
  }
  if (VL53L1_ReadMulti(dev, index, buf, 4U) != 0)
  {
    return VL53L1_IO_ERROR;
  }
  *pdata = ((uint32_t)buf[0] << 24U) |
           ((uint32_t)buf[1] << 16U) |
           ((uint32_t)buf[2] << 8U)  |
           ((uint32_t)buf[3]);
  return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
  (void)dev;
  if (wait_ms > 0)
  {
    VL53L1_DELAY_MS((uint32_t)wait_ms);
  }
  return 0;
}

