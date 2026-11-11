/**
  ******************************************************************************
  * @file    bno055_stm32.c
  * @brief   STM32 HAL I2C transport & unwrapped Euler wrapper for BNO055 IMU.
  *
  * Hardware mapping on MainRB PCB:
  *   - J1 (8-pin IMU header): Pin1=+3.3V, Pin2=GND, Pin3=I2C3_SCL (PA8, R24=4.7k),
  *     Pin4=I2C3_SDA (PC9, R25=4.7k), Pin5=JP1 (SolderJumper_2_Open -> GND).
  *   - Default 7-bit address = 0x28 (BNO055_I2C_ADDR_LO); probes 0x29 fallback if
  *     module pulls ADR high while JP1 is open.
  ******************************************************************************
  */

#include "bno055_stm32.h"
#include <string.h>

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

I2C_HandleTypeDef *_bno055_i2c_port = NULL;
static uint8_t _bno055_i2c_addr = BNO055_I2C_ADDR_LO;
static HAL_StatusTypeDef _bno055_last_status = HAL_ERROR;

static float BNO055_WrapDeltaDeg(float delta)
{
  if (delta > 180.0f)
  {
    delta -= 360.0f;
  }
  else if (delta < -180.0f)
  {
    delta += 360.0f;
  }
  return delta;
}

void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device)
{
  _bno055_i2c_port = hi2c_device;
}

void bno055_assignAddress(uint8_t addr_7bit)
{
  _bno055_i2c_addr = addr_7bit;
}

uint8_t bno055_getAddress(void)
{
  return _bno055_i2c_addr;
}

HAL_StatusTypeDef bno055_getLastI2CStatus(void)
{
  return _bno055_last_status;
}

void bno055_delay(int time)
{
#ifdef FREERTOS_ENABLED
  osDelay((uint32_t)time);
#else
  HAL_Delay((uint32_t)time);
#endif
}

void bno055_writeData(uint8_t reg, uint8_t data)
{
  if (_bno055_i2c_port == NULL)
  {
    _bno055_last_status = HAL_ERROR;
    return;
  }

  uint8_t txdata[2] = { reg, data };
  _bno055_last_status = HAL_I2C_Master_Transmit(
      _bno055_i2c_port,
      (uint16_t)(_bno055_i2c_addr << 1U),
      txdata,
      sizeof(txdata),
      BNO055_I2C_TIMEOUT_MS);
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
  if ((_bno055_i2c_port == NULL) || (data == NULL) || (len == 0U))
  {
    _bno055_last_status = HAL_ERROR;
    return;
  }

  HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
      _bno055_i2c_port,
      (uint16_t)(_bno055_i2c_addr << 1U),
      &reg,
      1U,
      BNO055_I2C_TIMEOUT_MS);

  if (st == HAL_OK)
  {
    st = HAL_I2C_Master_Receive(
        _bno055_i2c_port,
        (uint16_t)(_bno055_i2c_addr << 1U),
        data,
        len,
        BNO055_I2C_TIMEOUT_MS);
  }

  _bno055_last_status = st;
}

bool BNO055_IMU_Init(BNO055_IMU_t *imu, I2C_HandleTypeDef *hi2c)
{
  if ((imu == NULL) || (hi2c == NULL))
  {
    return false;
  }

  memset(imu, 0, sizeof(*imu));
  bno055_assignI2C(hi2c);

  /* BNO055 internal MCU takes up to ~650 ms POR after power-on before I2C responds */
  uint32_t t_start = HAL_GetTick();
  uint8_t detected_addr = 0U;
  do
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(BNO055_I2C_ADDR_LO << 1U), 2U, 5U) == HAL_OK)
    {
      detected_addr = BNO055_I2C_ADDR_LO;
      break;
    }
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(BNO055_I2C_ADDR_HI << 1U), 2U, 5U) == HAL_OK)
    {
      detected_addr = BNO055_I2C_ADDR_HI;
      break;
    }
    bno055_delay(25);
  } while ((uint32_t)(HAL_GetTick() - t_start) < BNO055_POR_WAIT_MS);

  if (detected_addr == 0U)
  {
    imu->connected = false;
    imu->initialized = false;
    imu->err_count++;
    return false;
  }

  bno055_assignAddress(detected_addr);
  imu->i2c_addr = detected_addr;

  bno055_setup();

  uint8_t chip_id = 0U;
  bno055_setPage(0U);
  bno055_readData(BNO055_CHIP_ID, &chip_id, 1U);
  imu->chip_id = chip_id;

  if ((_bno055_last_status != HAL_OK) || (chip_id != BNO055_ID))
  {
    imu->connected = false;
    imu->initialized = false;
    imu->err_count++;
    return false;
  }

  bno055_setOperationModeNDOF();
  bno055_delay(20);

  bno055_vector_t eul = bno055_getVectorEuler();
  if (_bno055_last_status == HAL_OK)
  {
    imu->euler = eul;
    imu->yaw_raw = (float)eul.x;
    imu->yaw_prev = imu->yaw_raw;
    imu->yaw_total = imu->yaw_raw;
    imu->yaw_offset = imu->yaw_total;
    imu->yaw = 0.0f;

    imu->roll_raw = (float)eul.y;
    imu->roll_prev = imu->roll_raw;
    imu->roll_total = imu->roll_raw;
    imu->roll_offset = imu->roll_total;
    imu->roll = 0.0f;

    imu->pitch_raw = (float)eul.z;
    imu->pitch_prev = imu->pitch_raw;
    imu->pitch_total = imu->pitch_raw;
    imu->pitch_offset = imu->pitch_total;
    imu->pitch = 0.0f;
    imu->ok_count++;
  }

  imu->calib = bno055_getCalibrationState();
  imu->connected = true;
  imu->initialized = true;
  return true;
}

bool BNO055_IMU_Update(BNO055_IMU_t *imu)
{
  if ((imu == NULL) || (!imu->connected))
  {
    return false;
  }

  bno055_vector_t eul = bno055_getVectorEuler();
  if (_bno055_last_status != HAL_OK)
  {
    imu->err_count++;
    return false;
  }

  imu->euler = eul;

  float yr = (float)eul.x;
  imu->yaw_raw = yr;
  imu->yaw_total += BNO055_WrapDeltaDeg(yr - imu->yaw_prev);
  imu->yaw_prev = yr;
  imu->yaw = imu->yaw_total - imu->yaw_offset;

  float rr = (float)eul.y;
  imu->roll_raw = rr;
  imu->roll_total += BNO055_WrapDeltaDeg(rr - imu->roll_prev);
  imu->roll_prev = rr;
  imu->roll = imu->roll_total - imu->roll_offset;

  float pr = (float)eul.z;
  imu->pitch_raw = pr;
  imu->pitch_total += BNO055_WrapDeltaDeg(pr - imu->pitch_prev);
  imu->pitch_prev = pr;
  imu->pitch = imu->pitch_total - imu->pitch_offset;

  imu->calib = bno055_getCalibrationState();
  imu->ok_count++;
  return true;
}

void BNO055_IMU_ZeroYaw(BNO055_IMU_t *imu)
{
  if (imu == NULL)
  {
    return;
  }
  imu->yaw_offset = imu->yaw_total;
  imu->yaw = 0.0f;
}

void BNO055_IMU_ZeroAll(BNO055_IMU_t *imu)
{
  if (imu == NULL)
  {
    return;
  }
  imu->yaw_offset = imu->yaw_total;
  imu->roll_offset = imu->roll_total;
  imu->pitch_offset = imu->pitch_total;
  imu->yaw = 0.0f;
  imu->roll = 0.0f;
  imu->pitch = 0.0f;
}
