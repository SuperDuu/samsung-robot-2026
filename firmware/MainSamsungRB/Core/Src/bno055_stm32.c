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
#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

static void BNO055_QuatToEuler(float qw, float qx, float qy, float qz,
                               float *yaw_deg, float *pitch_deg,
                               float *roll_deg)
{
  /* Yaw (Z-axis rotation): atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)) */
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  *yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);

  /* Pitch (Y-axis rotation): asin(2*(w*y - z*x)) */
  float sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp > 1.0f)
  {
    sinp = 1.0f;
  }
  else if (sinp < -1.0f)
  {
    sinp = -1.0f;
  }
  *pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

  /* Roll (X-axis rotation): atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2)) */
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  *roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);
}

void bno055_invalidatePageCache(void);

static void BNO055_RecoverI2C(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return;
  }
  bno055_invalidatePageCache();
#if !defined(UNIT_TEST_HOST) && (defined(STM32F405xx) || defined(STM32F4))
  if (hi2c->Instance == I2C3)
  {
    /* Clock 9 SCL pulses + STOP on PA8 (SCL) / PC9 (SDA) if BNO055 stretched/stuck SDA low */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_I2C_DeInit(hi2c);
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    for (uint8_t c = 0U; c < 9U; c++)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
      for (volatile uint32_t d = 0U; d < 120U; d++) {}
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
      for (volatile uint32_t d = 0U; d < 120U; d++) {}
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0U; d < 120U; d++) {}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile uint32_t d = 0U; d < 120U; d++) {}
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    for (volatile uint32_t d = 0U; d < 120U; d++) {}

    (void)HAL_I2C_Init(hi2c);
    return;
  }
#endif
  (void)HAL_I2C_DeInit(hi2c);
  (void)HAL_I2C_Init(hi2c);
}

void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device)
{
  _bno055_i2c_port = hi2c_device;
  bno055_invalidatePageCache();
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

bool bno055_lastIoOk(void)
{
  return (_bno055_last_status == HAL_OK);
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

  uint8_t txdata[2] = {reg, data};
  _bno055_last_status = HAL_I2C_Master_Transmit(
      _bno055_i2c_port,
      (uint16_t)(_bno055_i2c_addr << 1U),
      txdata,
      2U,
      BNO055_I2C_TIMEOUT_MS);

  if (_bno055_last_status != HAL_OK)
  {
    bno055_invalidatePageCache();
  }
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
  if ((_bno055_i2c_port == NULL) || (data == NULL) || (len == 0U))
  {
    _bno055_last_status = HAL_ERROR;
    return;
  }

  /* Two-phase discrete transfer: Transmit register address, then Receive data.
   * Avoids STM32F4 repeated-START clock stretching silicon errata (ES0182). */
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
        (uint16_t)len,
        BNO055_I2C_TIMEOUT_MS);
  }

  if (st != HAL_OK)
  {
    bno055_invalidatePageCache();
  }

  _bno055_last_status = st;
}

static float BNO055_Absf(float x)
{
  return (x < 0.0f) ? -x : x;
}

static float BNO055_ClampBiasDps(float dps)
{
  if (dps > BNO055_DRIFT_MAX_BIAS_DPS)
  {
    return BNO055_DRIFT_MAX_BIAS_DPS;
  }
  if (dps < -BNO055_DRIFT_MAX_BIAS_DPS)
  {
    return -BNO055_DRIFT_MAX_BIAS_DPS;
  }
  return dps;
}

static void BNO055_ResetDriftWindow(BNO055_IMU_t *imu)
{
  imu->still_ms = 0U;
  imu->still_win_ms = 0U;
  imu->still_raw_deg = 0.0f;
}

static void BNO055_DecodeCalib(BNO055_IMU_t *imu, uint8_t reg_val)
{
  imu->calib.sys = (uint8_t)((reg_val >> 6U) & 0x03U);
  imu->calib.gyro = (uint8_t)((reg_val >> 4U) & 0x03U);
  imu->calib.accel = (uint8_t)((reg_val >> 2U) & 0x03U);
  imu->calib.mag = (uint8_t)(reg_val & 0x03U);
}

#define BNO055_FLASH_CAL_MAGIC      0x424E3035UL /* "BN05" */
#define BNO055_FLASH_CAL_VERSION    2U           /* v2: 6-DOF IMU Accel+Gyro offsets */

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint16_t version;
  uint8_t  valid_flags;
  uint8_t  reserved;
  uint8_t  cal_data[BNO055_FLASH_CAL_DATA_LEN];
  uint16_t pad;
  uint32_t crc32;
} BNO055_FlashCalRecord_t;

#ifdef UNIT_TEST_HOST
static uint8_t s_mock_bno_flash[sizeof(BNO055_FlashCalRecord_t)];
static bool    s_mock_bno_flash_inited = false;

static const uint8_t *BNO055_FlashPtr(void)
{
  if (!s_mock_bno_flash_inited)
  {
    memset(s_mock_bno_flash, 0xFF, sizeof(s_mock_bno_flash));
    s_mock_bno_flash_inited = true;
  }
  return s_mock_bno_flash;
}

void BNO055_IMU_FlashCorruptTestWord(uint32_t byte_offset, uint8_t value)
{
  (void)BNO055_FlashPtr();
  if (byte_offset < (uint32_t)sizeof(s_mock_bno_flash))
  {
    s_mock_bno_flash[byte_offset] = value;
  }
}
#else
static const uint8_t *BNO055_FlashPtr(void)
{
  return (const uint8_t *)(uintptr_t)BNO055_FLASH_CAL_SECTOR_ADDR;
}
#endif

static uint32_t BNO055_FlashCrc32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (uint8_t b = 0U; b < 8U; b++)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

static bool BNO055_FlashValidate(const BNO055_FlashCalRecord_t *rec)
{
  if (rec == NULL)
  {
    return false;
  }
  if ((rec->magic != BNO055_FLASH_CAL_MAGIC) || (rec->version != BNO055_FLASH_CAL_VERSION))
  {
    return false;
  }
  uint32_t expected = BNO055_FlashCrc32((const uint8_t *)rec,
                                        (uint32_t)offsetof(BNO055_FlashCalRecord_t, crc32));
  return (rec->crc32 == expected);
}

static bool BNO055_FlashCalLoad(BNO055_FlashCalRecord_t *out)
{
  if (out == NULL)
  {
    return false;
  }
  BNO055_FlashCalRecord_t tmp;
  memcpy(&tmp, BNO055_FlashPtr(), sizeof(tmp));
  if (!BNO055_FlashValidate(&tmp))
  {
    return false;
  }
  *out = tmp;
  return true;
}

static bool BNO055_FlashCalErase(void)
{
#ifdef UNIT_TEST_HOST
  memset(s_mock_bno_flash, 0xFF, sizeof(s_mock_bno_flash));
  s_mock_bno_flash_inited = true;
  return true;
#else
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.Banks        = FLASH_BANK_1;
  erase.Sector       = FLASH_SECTOR_11;
  erase.NbSectors    = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  uint32_t sector_err = 0U;
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sector_err);
  (void)HAL_FLASH_Lock();
  return (st == HAL_OK) && (sector_err == 0xFFFFFFFFUL);
#endif
}

static bool BNO055_FlashCalSave(const BNO055_FlashCalRecord_t *in)
{
  if (in == NULL)
  {
    return false;
  }

  BNO055_FlashCalRecord_t staged = *in;
  staged.magic    = BNO055_FLASH_CAL_MAGIC;
  staged.version  = BNO055_FLASH_CAL_VERSION;
  staged.reserved = 0U;
  staged.pad      = 0U;
  staged.crc32    = BNO055_FlashCrc32((const uint8_t *)&staged,
                                      (uint32_t)offsetof(BNO055_FlashCalRecord_t, crc32));

#ifdef UNIT_TEST_HOST
  (void)BNO055_FlashCalErase();
  memcpy(s_mock_bno_flash, &staged, sizeof(staged));
#else
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.Banks        = FLASH_BANK_1;
  erase.Sector       = FLASH_SECTOR_11;
  erase.NbSectors    = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  uint32_t sector_err = 0U;
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sector_err);
  if ((st != HAL_OK) || (sector_err != 0xFFFFFFFFUL))
  {
    (void)HAL_FLASH_Lock();
    return false;
  }

  uint32_t words[sizeof(staged) / sizeof(uint32_t)];
  memcpy(words, &staged, sizeof(staged));
  uint32_t num_words = (uint32_t)(sizeof(staged) / sizeof(uint32_t));
  for (uint32_t w = 0U; w < num_words; w++)
  {
    uint32_t addr = BNO055_FLASH_CAL_SECTOR_ADDR + (w * 4U);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[w]) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return false;
    }
  }

  (void)HAL_FLASH_Lock();
#endif

  BNO055_FlashCalRecord_t verify;
  memcpy(&verify, BNO055_FlashPtr(), sizeof(verify));
  return (memcmp(&staged, &verify, sizeof(staged)) == 0);
}

static bool BNO055_WriteCalProfile22(const uint8_t cal22[BNO055_FLASH_CAL_DATA_LEN])
{
  bno055_setPage(0U);
  for (uint8_t i = 0U; i < BNO055_FLASH_CAL_DATA_LEN; i++)
  {
    bno055_writeData((uint8_t)(BNO055_ACC_OFFSET_X_LSB + i), cal22[i]);
    if (_bno055_last_status != HAL_OK)
    {
      return false;
    }
  }
  return true;
}

static bool BNO055_CheckAndRestoreMode(BNO055_IMU_t *imu, bool *mode_was_reset)
{
  uint8_t status_buf[9] = {0};
  *mode_was_reset = false;

  bno055_setPage(0U);
  bno055_readData(BNO055_CALIB_STAT, status_buf, (uint8_t)sizeof(status_buf));
  if (_bno055_last_status != HAL_OK)
  {
    return false;
  }

  BNO055_DecodeCalib(imu, status_buf[0]);
  imu->sys_status = status_buf[4];
  imu->sys_err = status_buf[5];
  uint8_t cur_mode = (uint8_t)(status_buf[8] & 0x0FU);
  if (cur_mode != (uint8_t)imu->op_mode)
  {
    /* Silent brown-out / soft-reset dropped BNO055 back to CONFIG (0x00):
     * reload cached 22-byte calibration profile before restoring OPR_MODE */
    if (imu->has_saved_cal)
    {
      (void)BNO055_WriteCalProfile22(imu->saved_cal_data);
    }
    bno055_setPage(0U);
    bno055_writeData(BNO055_OPR_MODE, (uint8_t)imu->op_mode);
    bno055_delay(10);
    imu->mode_resets++;
    imu->resync_yaw_prev = true;
    BNO055_ResetDriftWindow(imu);
    *mode_was_reset = true;
  }
  return true;
}

bool BNO055_IMU_InitWithMode(BNO055_IMU_t *imu, I2C_HandleTypeDef *hi2c, bno055_opmode_t op_mode)
{
  if ((imu == NULL) || (hi2c == NULL))
  {
    return false;
  }

  memset(imu, 0, sizeof(*imu));
  imu->op_mode = (op_mode == BNO055_OPERATION_MODE_CONFIG) ? BNO055_DEFAULT_FUSION_MODE : op_mode;
  imu->software_drift_comp = false; /* Smart BNO055 has internal BSX fusion; disable software ZRO fighting loop by default */
  bno055_assignI2C(hi2c);

  /* Bus recovery: 9 clocks on PA8/PC9 if previous run or slave hung SDA low */
  BNO055_RecoverI2C(hi2c);
  HAL_Delay(20);

  /* Step 0: Probe for BNO080 / BNO085 (addresses 0x4A, 0x4B) */
  uint8_t bno08x_addr = 0U;
  if (BNO08X_Detect(hi2c, &bno08x_addr))
  {
    if (BNO08X_Init(hi2c, bno08x_addr, 10000U)) /* 100 Hz = 10,000 us */
    {
      imu->i2c_addr = bno08x_addr;
      imu->chip_id = 0x85U; /* BNO085 indicator */
      imu->connected = true;
      imu->initialized = true;
      imu->healthy = true;
      imu->consec_errs = 0U;
      imu->last_ok_ms = HAL_GetTick();
      imu->prev_sample_ms = imu->last_ok_ms;
      imu->has_prev_sample_ms = true;

      /* Read initial sample to set zero offsets */
      BNO08X_Rotation_t rot = {0};
      for (uint8_t retry = 0U; retry < 15U; retry++)
      {
        HAL_Delay(10);
        if (BNO08X_Poll(hi2c, bno08x_addr, &rot))
        {
          imu->yaw_raw = rot.yaw_deg;
          imu->yaw_prev = imu->yaw_raw;
          imu->yaw_total = imu->yaw_raw;
          imu->yaw_offset = imu->yaw_total;
          imu->yaw = 0.0f;

          imu->pitch_raw = rot.pitch_deg;
          imu->pitch_prev = imu->pitch_raw;
          imu->pitch_total = imu->pitch_raw;
          imu->pitch_offset = imu->pitch_total;
          imu->pitch = 0.0f;

          imu->roll_raw = rot.roll_deg;
          imu->roll_prev = imu->roll_raw;
          imu->roll_total = imu->roll_raw;
          imu->roll_offset = imu->roll_total;
          imu->roll = 0.0f;

          imu->calib.sys = rot.accuracy;
          imu->calib.gyro = rot.accuracy;
          imu->calib.accel = rot.accuracy;
          imu->calib.mag = 3U;
          imu->ok_count++;
          break;
        }
      }
      return true;
    }
  }

  /* Probe for BNO055 by reading CHIP_ID (0x00) directly at 0x28 (ADDR=GND) or 0x29 (fallback).
   * DO NOT use HAL_I2C_IsDeviceReady(): sending 0-byte SLA+W followed by immediate STOP
   * causes BNO055 internal Cortex-M0 to stretch SCL during STOP, timing out BUSY flag in STM32 HAL! */
  uint32_t t_start = HAL_GetTick();
  uint8_t detected_addr = 0U;
  uint8_t chip_id = 0U;
  do
  {
    /* 1. Try address 0x28 (ADDR tied to GND - primary standard config) */
    bno055_assignAddress(BNO055_I2C_ADDR_LO);
    bno055_setPage(0U);
    chip_id = 0U;
    bno055_readData(BNO055_CHIP_ID, &chip_id, 1U);
    if ((bno055_getLastI2CStatus() == HAL_OK) && (chip_id == BNO055_ID))
    {
      detected_addr = BNO055_I2C_ADDR_LO;
      break;
    }

    /* 2. Try address 0x29 (fallback if ADDR pin floating or tied to 3V3) */
    bno055_assignAddress(BNO055_I2C_ADDR_HI);
    bno055_setPage(0U);
    chip_id = 0U;
    bno055_readData(BNO055_CHIP_ID, &chip_id, 1U);
    if ((bno055_getLastI2CStatus() == HAL_OK) && (chip_id == BNO055_ID))
    {
      detected_addr = BNO055_I2C_ADDR_HI;
      break;
    }
    bno055_delay(25);
  } while ((uint32_t)(HAL_GetTick() - t_start) < BNO055_POR_WAIT_MS);

  if (detected_addr == 0U)
  {
    /* Last-ditch: run bus recovery and bno055_setup at 0x28 */
    BNO055_RecoverI2C(hi2c);
    bno055_assignAddress(BNO055_I2C_ADDR_LO);
    bno055_setup();
    bno055_setPage(0U);
    chip_id = 0U;
    bno055_readData(BNO055_CHIP_ID, &chip_id, 1U);
    if ((bno055_getLastI2CStatus() == HAL_OK) && (chip_id == BNO055_ID))
    {
      detected_addr = BNO055_I2C_ADDR_LO;
    }
  }

  if (detected_addr == 0U)
  {
    imu->connected = false;
    imu->initialized = false;
    imu->chip_id = chip_id;
    imu->err_count++;
    return false;
  }

  bno055_assignAddress(detected_addr);
  imu->i2c_addr = detected_addr;

  bno055_setup();

  /* Retry reading CHIP_ID up to 10 times with 20ms delay (total 200ms window) */
  chip_id = 0U;
  for (uint8_t retry = 0U; retry < 10U; retry++)
  {
    bno055_setPage(0U);
    bno055_readData(BNO055_CHIP_ID, &chip_id, 1U);
    if ((_bno055_last_status == HAL_OK) && (chip_id == BNO055_ID))
    {
      break;
    }
    bno055_delay(20);
  }
  imu->chip_id = chip_id;

  if ((_bno055_last_status != HAL_OK) || (chip_id != BNO055_ID))
  {
    imu->connected = false;
    imu->initialized = false;
    imu->err_count++;
    return false;
  }

  imu->has_saved_cal = false;
  imu->cal_saved_to_flash = false;
  memset(imu->saved_cal_data, 0, sizeof(imu->saved_cal_data));

  BNO055_FlashCalRecord_t cal_rec;
  if (((imu->op_mode == BNO055_OPERATION_MODE_IMU) ||
       (imu->op_mode == BNO055_OPERATION_MODE_NDOF) ||
       (imu->op_mode == BNO055_OPERATION_MODE_NDOF_FMC_OFF)) &&
      BNO055_FlashCalLoad(&cal_rec))
  {
    bno055_setOperationModeConfig();
    bno055_delay(20);
    if (BNO055_WriteCalProfile22(cal_rec.cal_data))
    {
      imu->has_saved_cal = true;
      imu->cal_saved_to_flash = true;
      memcpy(imu->saved_cal_data, cal_rec.cal_data, BNO055_FLASH_CAL_DATA_LEN);
    }
  }

  bno055_setOperationMode(imu->op_mode);
  bno055_delay(30);

  bno055_vector_t quat = bno055_getVectorQuaternion();
  if (_bno055_last_status == HAL_OK)
  {
    float yr = 0.0f;
    float pr = 0.0f;
    float rr = 0.0f;
    BNO055_QuatToEuler((float)quat.w, (float)quat.x, (float)quat.y, (float)quat.z,
                       &yr, &pr, &rr);
    imu->euler.x = (double)yr;
    imu->euler.y = (double)rr;
    imu->euler.z = (double)pr;
    imu->yaw_raw = yr;
    imu->yaw_prev = imu->yaw_raw;
    imu->yaw_total = imu->yaw_raw;
    imu->yaw_offset = imu->yaw_total;
    imu->yaw = 0.0f;

    imu->roll_raw = rr;
    imu->roll_prev = imu->roll_raw;
    imu->roll_total = imu->roll_raw;
    imu->roll_offset = imu->roll_total;
    imu->roll = 0.0f;

    imu->pitch_raw = pr;
    imu->pitch_prev = imu->pitch_raw;
    imu->pitch_total = imu->pitch_raw;
    imu->pitch_offset = imu->pitch_total;
    imu->pitch = 0.0f;
    imu->ok_count++;
  }

  imu->calib = bno055_getCalibrationState();
  imu->connected = true;
  imu->initialized = true;
  imu->healthy = true;
  imu->consec_errs = 0U;
  imu->last_ok_ms = HAL_GetTick();
  imu->prev_sample_ms = imu->last_ok_ms;
  imu->has_prev_sample_ms = true;
  return true;
}

bool BNO055_IMU_Init(BNO055_IMU_t *imu, I2C_HandleTypeDef *hi2c)
{
  return BNO055_IMU_InitWithMode(imu, hi2c, BNO055_DEFAULT_FUSION_MODE);
}

void BNO055_IMU_SetStationaryHint(BNO055_IMU_t *imu, bool stationary)
{
  if (imu == NULL)
  {
    return;
  }
  if (!stationary && imu->stationary_hint)
  {
    BNO055_ResetDriftWindow(imu);
  }
  imu->stationary_hint = stationary;
}

bool BNO055_IMU_Update(BNO055_IMU_t *imu)
{
  if ((imu == NULL) || (!imu->connected) || (_bno055_i2c_port == NULL))
  {
    return false;
  }

  uint32_t now_ms = HAL_GetTick();

  /* Dedicated high-rate update for BNO080 / BNO085 (chip_id == 0x85) */
  if (imu->chip_id == 0x85U)
  {
    if (!imu->healthy)
    {
      if ((int32_t)(now_ms - imu->next_recover_ms) < 0)
      {
        return false;
      }
      BNO055_RecoverI2C(_bno055_i2c_port);
      if (BNO08X_Init(_bno055_i2c_port, imu->i2c_addr, 10000U))
      {
        imu->resync_yaw_prev = true;
        imu->healthy = true;
        imu->consec_errs = 0U;
        imu->recoveries++;
        imu->last_ok_ms = now_ms;
        imu->prev_sample_ms = now_ms;
      }
      else
      {
        imu->next_recover_ms = now_ms + BNO055_RECOVER_BACKOFF_MS;
        return false;
      }
    }

    BNO08X_Rotation_t rot = {0};
    if (BNO08X_Poll(_bno055_i2c_port, imu->i2c_addr, &rot))
    {
      if (!imu->resync_yaw_prev)
      {
        float dy = BNO055_WrapDeltaDeg(rot.yaw_deg - imu->yaw_prev);
        uint32_t dt_ms = (uint32_t)(now_ms - imu->prev_sample_ms);
        if ((dt_ms < 25U) && (fabsf(dy) > 45.0f))
        {
          /* Physically impossible >1800 deg/s spike in <25 ms -> drop corrupted I2C/SHTP sample */
          imu->err_count++;
          return false;
        }
        imu->yaw_total += dy;
      }
      else
      {
        imu->resync_yaw_prev = false;
      }
      imu->yaw_raw = rot.yaw_deg;
      imu->yaw_prev = imu->yaw_raw;
      imu->yaw = imu->yaw_total - imu->yaw_offset;

      imu->pitch_raw = rot.pitch_deg;
      float dp = BNO055_WrapDeltaDeg(imu->pitch_raw - imu->pitch_prev);
      imu->pitch_total += dp;
      imu->pitch_prev = imu->pitch_raw;
      imu->pitch = imu->pitch_total - imu->pitch_offset;

      imu->roll_raw = rot.roll_deg;
      float dr = BNO055_WrapDeltaDeg(imu->roll_raw - imu->roll_prev);
      imu->roll_total += dr;
      imu->roll_prev = imu->roll_raw;
      imu->roll = imu->roll_total - imu->roll_offset;

      imu->calib.sys = rot.accuracy;
      imu->calib.gyro = rot.accuracy;
      imu->calib.accel = rot.accuracy;
      imu->calib.mag = rot.accuracy;

      imu->ok_count++;
      imu->consec_errs = 0U;
      imu->healthy = true;
      imu->last_ok_ms = now_ms;
      imu->prev_sample_ms = now_ms;
      imu->has_prev_sample_ms = true;
      return true;
    }
    else
    {
      if ((uint32_t)(now_ms - imu->last_ok_ms) > 200U)
      {
        imu->err_count++;
        if (imu->consec_errs < 255U)
        {
          imu->consec_errs++;
        }
        if (imu->consec_errs >= BNO055_CONSEC_ERR_LIMIT)
        {
          imu->healthy = false;
          imu->next_recover_ms = now_ms + BNO055_RECOVER_BACKOFF_MS;
        }
      }
      return false;
    }
  }

  /* Non-blocking error backoff + I2C bus unstick + hot op_mode recovery */
  if (!imu->healthy)
  {
    if ((int32_t)(now_ms - imu->next_recover_ms) < 0)
    {
      return false;
    }

    BNO055_RecoverI2C(_bno055_i2c_port);
    bno055_assignAddress(imu->i2c_addr);
    uint8_t chip_id = 0U;
    bno055_setPage(0U);
    bno055_readData(BNO055_CHIP_ID, &chip_id, 1U);
    if ((_bno055_last_status == HAL_OK) && (chip_id == BNO055_ID))
    {
      uint8_t cur_mode = 0U;
      bno055_readData(BNO055_OPR_MODE, &cur_mode, 1U);
      if ((_bno055_last_status != HAL_OK) || ((uint8_t)(cur_mode & 0x0FU) != (uint8_t)imu->op_mode))
      {
        bno055_setOperationModeConfig();
        bno055_delay(10);
        if (imu->has_saved_cal)
        {
          (void)BNO055_WriteCalProfile22(imu->saved_cal_data);
        }
        bno055_setPage(0U);
        bno055_writeData(BNO055_OPR_MODE, (uint8_t)imu->op_mode);
        bno055_delay(10);
        imu->mode_resets++;
        imu->resync_yaw_prev = true;
      }
      BNO055_ResetDriftWindow(imu);
      imu->healthy = true;
      imu->consec_errs = 0U;
      imu->recoveries++;
      imu->last_ok_ms = now_ms;
      imu->prev_sample_ms = now_ms;
      imu->has_prev_sample_ms = true;
    }

    if (!imu->healthy)
    {
      imu->err_count++;
      imu->next_recover_ms = now_ms + BNO055_RECOVER_BACKOFF_MS;
      return false;
    }
  }

  bno055_vector_t quat = bno055_getVectorQuaternion();
  if (_bno055_last_status != HAL_OK)
  {
    imu->err_count++;
    if (imu->consec_errs < 255U)
    {
      imu->consec_errs++;
    }
    if (imu->consec_errs >= BNO055_CONSEC_ERR_LIMIT)
    {
      imu->healthy = false;
      imu->next_recover_ms = now_ms + BNO055_RECOVER_BACKOFF_MS;
    }
    return false;
  }

  /* Unit quaternions always have norm ~1.0; a {0,0,0,0} quat is an unequivocal
   * signature of uninitialized data or silent brownout reset. */
  float qn2 = (float)((quat.w * quat.w) + (quat.x * quat.x) +
                      (quat.y * quat.y) + (quat.z * quat.z));
  bool quat_zero_snap = !imu->resync_yaw_prev &&
                        (quat.w == 0.0) && (quat.x == 0.0) && (quat.y == 0.0) && (quat.z == 0.0);

  imu->calib_div++;
  if (quat_zero_snap || (imu->calib_div >= BNO055_CALIB_POLL_DIV))
  {
    bool mode_was_reset = false;
    imu->calib_div = 0U;
    if (BNO055_CheckAndRestoreMode(imu, &mode_was_reset) && mode_was_reset)
    {
      imu->consec_errs = 0U;
      imu->healthy = true;
      imu->last_ok_ms = now_ms;
      imu->prev_sample_ms = now_ms;
      imu->has_prev_sample_ms = true;
      imu->ok_count++;
      return true;
    }
  }

  /* Reject non-unit corrupted quaternions (I2C EMI bit flip / torn register burst) */
  if (!isfinite(qn2) || (qn2 < BNO055_QUAT_NORM2_MIN) || (qn2 > BNO055_QUAT_NORM2_MAX))
  {
    imu->err_count++;
    imu->quat_err_count++;
    imu->last_bad_qn2 = qn2;
    if (imu->consec_errs < 255U)
    {
      imu->consec_errs++;
    }
    if (imu->consec_errs >= BNO055_CONSEC_ERR_LIMIT)
    {
      imu->healthy = false;
      imu->next_recover_ms = now_ms + BNO055_RECOVER_BACKOFF_MS;
    }
    return false;
  }

  uint32_t dt_ms = imu->has_prev_sample_ms ? (uint32_t)(now_ms - imu->prev_sample_ms) : 0U;

  float yr = 0.0f;
  float pr = 0.0f;
  float rr = 0.0f;
  BNO055_QuatToEuler((float)quat.w, (float)quat.x, (float)quat.y, (float)quat.z,
                     &yr, &pr, &rr);

  if (imu->resync_yaw_prev)
  {
    imu->prev_sample_ms = now_ms;
    imu->has_prev_sample_ms = true;
    imu->consec_errs = 0U;
    imu->healthy = true;
    imu->last_ok_ms = now_ms;
    imu->euler.x = (double)yr;
    imu->euler.y = (double)rr;
    imu->euler.z = (double)pr;
    imu->yaw_raw = yr;
    imu->roll_raw = rr;
    imu->pitch_raw = pr;
    imu->yaw_prev = yr;
    imu->roll_prev = rr;
    imu->pitch_prev = pr;
    imu->resync_yaw_prev = false;
    BNO055_ResetDriftWindow(imu);
  }
  else
  {
    float raw_delta = BNO055_WrapDeltaDeg(yr - imu->yaw_prev);
    if ((dt_ms < BNO055_SPIKE_DT_MS) &&
        (BNO055_Absf(raw_delta) > BNO055_MAX_TICK_YAW_STEP_DEG))
    {
      /* Physically impossible single-tick jump (>25 deg in <25 ms): reject corrupted sample */
      imu->err_count++;
      imu->spike_count++;
      imu->last_spike_deg = raw_delta;
      if (imu->consec_errs < 255U)
      {
        imu->consec_errs++;
      }
      if (imu->consec_errs >= BNO055_CONSEC_ERR_LIMIT)
      {
        imu->healthy = false;
        imu->next_recover_ms = now_ms + BNO055_RECOVER_BACKOFF_MS;
      }
      return false;
    }

    imu->prev_sample_ms = now_ms;
    imu->has_prev_sample_ms = true;
    imu->consec_errs = 0U;
    imu->healthy = true;
    imu->last_ok_ms = now_ms;
    imu->euler.x = (double)yr;
    imu->euler.y = (double)rr;
    imu->euler.z = (double)pr;
    imu->yaw_raw = yr;
    imu->roll_raw = rr;
    imu->pitch_raw = pr;
    imu->yaw_prev = yr;

    float bias_comp = 0.0f;
    if (imu->software_drift_comp && (dt_ms > 0U) && (dt_ms <= 250U))
    {
      bias_comp = imu->yaw_bias_dps * ((float)dt_ms * 0.001f);
    }
    imu->yaw_total += (raw_delta - bias_comp);

    imu->roll_total += BNO055_WrapDeltaDeg(rr - imu->roll_prev);
    imu->roll_prev = rr;
    imu->pitch_total += BNO055_WrapDeltaDeg(pr - imu->pitch_prev);
    imu->pitch_prev = pr;

    /* Windowed adaptive ZRO drift estimator (6-DOF IMU mode only, when software_drift_comp enabled —
     * disabled by default so software bias never fights BNO055 internal BSX fusion or creates heading PID runaway) */
    if (imu->software_drift_comp &&
        (imu->op_mode == BNO055_OPERATION_MODE_IMU) &&
        imu->stationary_hint &&
        (dt_ms > 0U) && (dt_ms <= 200U) &&
        (BNO055_Absf(raw_delta) <= BNO055_DRIFT_MAX_TICK_DEG))
    {
      if (imu->still_ms < BNO055_DRIFT_SETTLE_MS)
      {
        imu->still_ms += dt_ms;
      }
      else
      {
        imu->still_win_ms += dt_ms;
        imu->still_raw_deg += raw_delta;
        if (imu->still_win_ms >= BNO055_DRIFT_WINDOW_MS)
        {
          float win_dps = imu->still_raw_deg / ((float)imu->still_win_ms * 0.001f);
          if (BNO055_Absf(win_dps) <= BNO055_DRIFT_MAX_BIAS_DPS)
          {
            if (imu->bias_updates == 0U)
            {
              imu->yaw_bias_dps = BNO055_ClampBiasDps(win_dps);
            }
            else
            {
              float next_bias = imu->yaw_bias_dps +
                                BNO055_DRIFT_EMA_ALPHA * (win_dps - imu->yaw_bias_dps);
              imu->yaw_bias_dps = BNO055_ClampBiasDps(next_bias);
            }
            imu->bias_updates++;
          }
          else
          {
            /* Smooth external rotation while wheels idle: reject & re-settle */
            imu->still_ms = 0U;
          }
          imu->still_win_ms = 0U;
          imu->still_raw_deg = 0.0f;
        }
      }
    }
    else
    {
      BNO055_ResetDriftWindow(imu);
    }
  }

  imu->yaw = imu->yaw_total - imu->yaw_offset;
  imu->roll = imu->roll_total - imu->roll_offset;
  imu->pitch = imu->pitch_total - imu->pitch_offset;

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
  imu->yaw_bias_dps = 0.0f;
  BNO055_ResetDriftWindow(imu);
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
  imu->yaw_bias_dps = 0.0f;
  BNO055_ResetDriftWindow(imu);
}

bool BNO055_IMU_HasFlashCal(void)
{
  BNO055_FlashCalRecord_t rec;
  return BNO055_FlashCalLoad(&rec);
}

bool BNO055_IMU_EnterAutoCalNDOF(BNO055_IMU_t *imu)
{
  if ((imu == NULL) || !imu->connected || (imu->chip_id == 0x85U) || (_bno055_i2c_port == NULL))
  {
    return false;
  }

  _bno055_i2c_addr = imu->i2c_addr;

  bno055_setOperationModeConfig();
  bno055_delay(20);
  imu->op_mode = BNO055_OPERATION_MODE_NDOF;
  imu->cal_saved_to_flash = false;
  bno055_setOperationMode(imu->op_mode);
  bno055_delay(30);
  imu->resync_yaw_prev = true;
  BNO055_ResetDriftWindow(imu);
  imu->prev_sample_ms = HAL_GetTick();
  imu->has_prev_sample_ms = true;
  return true;
}

bool BNO055_IMU_SaveCalToFlash(BNO055_IMU_t *imu)
{
  if ((imu == NULL) || !imu->connected || (imu->chip_id == 0x85U) || (_bno055_i2c_port == NULL))
  {
    return false;
  }

  _bno055_i2c_addr = imu->i2c_addr;

  bno055_setOperationModeConfig();
  bno055_delay(20);

  BNO055_FlashCalRecord_t rec;
  memset(&rec, 0, sizeof(rec));
  bno055_setPage(0U);
  bno055_readData(BNO055_ACC_OFFSET_X_LSB, rec.cal_data, BNO055_FLASH_CAL_DATA_LEN);
  bool read_ok = (_bno055_last_status == HAL_OK);

  bool saved_ok = false;
  if (read_ok)
  {
    rec.valid_flags = (uint8_t)(((imu->calib.mag >= 2U) ? 0x01U : 0x00U) |
                                ((imu->calib.gyro >= 3U) ? 0x02U : 0x00U));
    saved_ok = BNO055_FlashCalSave(&rec);
    if (saved_ok)
    {
      if (imu->op_mode == BNO055_OPERATION_MODE_NDOF)
      {
        imu->op_mode = BNO055_OPERATION_MODE_NDOF_FMC_OFF;
      }
      imu->has_saved_cal = true;
      imu->cal_saved_to_flash = true;
      memcpy(imu->saved_cal_data, rec.cal_data, BNO055_FLASH_CAL_DATA_LEN);
    }
  }

  /* Restore operation mode (6-DOF IMU or NDOF_FMC_OFF) without disturbing yaw_total */
  bno055_setOperationMode(imu->op_mode);
  bno055_delay(30);
  imu->resync_yaw_prev = true;
  BNO055_ResetDriftWindow(imu);
  imu->prev_sample_ms = HAL_GetTick();
  imu->has_prev_sample_ms = true;
  return saved_ok;
}

bool BNO055_IMU_EraseFlashCal(BNO055_IMU_t *imu)
{
  bool erased = BNO055_FlashCalErase();
  if (imu != NULL)
  {
    imu->has_saved_cal = false;
    imu->cal_saved_to_flash = false;
    memset(imu->saved_cal_data, 0, sizeof(imu->saved_cal_data));

    if (erased && imu->connected && (imu->chip_id != 0x85U) && (_bno055_i2c_port != NULL))
    {
      _bno055_i2c_addr = imu->i2c_addr;
      /* Soft-reset BNO055 so internal BSX filter drops old offsets cleanly */
      bno055_setPage(0U);
      bno055_writeData(BNO055_SYS_TRIGGER, 0x20U);
#ifndef UNIT_TEST_HOST
      bno055_delay(650);
#else
      bno055_delay(10);
#endif
      (void)BNO055_IMU_InitWithMode(imu, _bno055_i2c_port, BNO055_DEFAULT_FUSION_MODE);
    }
  }
  return erased;
}
