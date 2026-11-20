/**
  ******************************************************************************
  * @file    flash_cal.c
  * @brief   STM32F405 Flash Sector 11 NVS storage with IEEE CRC-32 verification.
  ******************************************************************************
  */

#include "flash_cal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert((sizeof(FlashCal_Record_t) % 4U) == 0U,
               "FlashCal_Record_t size must be a multiple of 4 bytes");
#endif

#ifdef UNIT_TEST_HOST
static uint8_t s_mock_sector[FLASH_CAL_SECTOR_SIZE];
static bool s_mock_inited = false;

static void flash_mock_ensure_init(void)
{
  if (!s_mock_inited)
  {
    memset(s_mock_sector, 0xFF, sizeof(s_mock_sector));
    s_mock_inited = true;
  }
}

uint8_t *FlashCal_MockStoragePtr(void)
{
  flash_mock_ensure_init();
  return s_mock_sector;
}

void FlashCal_MockReset(void)
{
  memset(s_mock_sector, 0xFF, sizeof(s_mock_sector));
  s_mock_inited = true;
}
#endif

static const uint8_t *flash_cal_ro_ptr(void)
{
#ifdef UNIT_TEST_HOST
  flash_mock_ensure_init();
  return s_mock_sector;
#else
  return (const uint8_t *)(uintptr_t)FLASH_CAL_SECTOR_ADDR;
#endif
}

uint32_t FlashCal_ComputeCrc32(const uint8_t *data, uint32_t len)
{
  if (data == NULL)
  {
    return 0U;
  }

  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (uint8_t b = 0U; b < 8U; b++)
    {
      uint32_t mask = (uint32_t)(-(int32_t)(crc & 1UL));
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

void FlashCal_InitDefaults(FlashCal_Record_t *rec)
{
  if (rec == NULL)
  {
    return;
  }
  memset(rec, 0, sizeof(*rec));
  rec->magic        = FLASH_CAL_MAGIC;
  rec->version      = FLASH_CAL_VERSION;
  rec->line_on_high = 1U;
  for (uint8_t i = 0U; i < FLASH_CAL_NUM_CHANNELS; i++)
  {
    rec->cal_min[i] = 0U;
    rec->cal_max[i] = 4095U;
  }
}

bool FlashCal_Validate(const FlashCal_Record_t *rec)
{
  if (rec == NULL)
  {
    return false;
  }
  if ((rec->magic != FLASH_CAL_MAGIC) || (rec->version != FLASH_CAL_VERSION))
  {
    return false;
  }

  uint32_t expected_crc = FlashCal_ComputeCrc32(
      (const uint8_t *)rec, (uint32_t)offsetof(FlashCal_Record_t, crc32));
  if (rec->crc32 != expected_crc)
  {
    return false;
  }

  if ((rec->flags & FLASH_CAL_FLAG_LINE_VALID) != 0U)
  {
    if (rec->line_on_high > 1U)
    {
      return false;
    }
    for (uint8_t i = 0U; i < FLASH_CAL_NUM_CHANNELS; i++)
    {
      if ((rec->cal_max[i] <= rec->cal_min[i]) || (rec->cal_max[i] > 4095U))
      {
        return false;
      }
    }
  }

  if ((rec->flags & FLASH_CAL_FLAG_IMU_VALID) != 0U)
  {
    if (!isfinite(rec->imu_field_offset_deg))
    {
      return false;
    }
  }

  return true;
}

bool FlashCal_Load(FlashCal_Record_t *out)
{
  if (out == NULL)
  {
    return false;
  }

  FlashCal_Record_t tmp;
  memcpy(&tmp, flash_cal_ro_ptr(), sizeof(tmp));
  if (!FlashCal_Validate(&tmp))
  {
    return false;
  }

  *out = tmp;
  return true;
}

bool FlashCal_Erase(void)
{
#ifdef UNIT_TEST_HOST
  FlashCal_MockReset();
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

bool FlashCal_Save(const FlashCal_Record_t *in)
{
  if (in == NULL)
  {
    return false;
  }

  FlashCal_Record_t staged = *in;
  staged.magic   = FLASH_CAL_MAGIC;
  staged.version = FLASH_CAL_VERSION;
  memset(staged.reserved, 0, sizeof(staged.reserved));
  staged.crc32 = FlashCal_ComputeCrc32(
      (const uint8_t *)&staged, (uint32_t)offsetof(FlashCal_Record_t, crc32));

  if (!FlashCal_Validate(&staged))
  {
    return false;
  }

#ifdef UNIT_TEST_HOST
  FlashCal_MockReset();
  memcpy(s_mock_sector, &staged, sizeof(staged));
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

  const uint32_t *words = (const uint32_t *)(const void *)&staged;
  uint32_t num_words = (uint32_t)(sizeof(staged) / sizeof(uint32_t));
  for (uint32_t w = 0U; w < num_words; w++)
  {
    uint32_t addr = FLASH_CAL_SECTOR_ADDR + (w * 4U);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[w]) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return false;
    }
  }

  (void)HAL_FLASH_Lock();
#endif

  FlashCal_Record_t verify;
  memcpy(&verify, flash_cal_ro_ptr(), sizeof(verify));
  return (memcmp(&staged, &verify, sizeof(staged)) == 0);
}

