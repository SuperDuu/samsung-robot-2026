/**
  ******************************************************************************
  * @file    flash_cal.h
  * @brief   STM32F405 Flash Sector 11 (0x080E0000, 128 KB) persistent storage
  *          for 2-point CD4067 line calibration and BNO055 arena heading offset.
  ******************************************************************************
  */

#ifndef INC_FLASH_CAL_H_
#define INC_FLASH_CAL_H_

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

#define FLASH_CAL_SECTOR_ADDR     0x080E0000UL
#define FLASH_CAL_SECTOR_SIZE     (128UL * 1024UL)
#define FLASH_CAL_MAGIC           0x49544332UL /* "ITC2" */
#define FLASH_CAL_VERSION         1U
#define FLASH_CAL_NUM_CHANNELS    16U

#define FLASH_CAL_FLAG_LINE_VALID (1U << 0)
#define FLASH_CAL_FLAG_IMU_VALID  (1U << 1)

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t flags;
  uint32_t seq;
  uint16_t cal_min[FLASH_CAL_NUM_CHANNELS];
  uint16_t cal_max[FLASH_CAL_NUM_CHANNELS];
  uint8_t  line_on_high;
  uint8_t  reserved[3];
  float    imu_field_offset_deg;
  uint32_t crc32;
} FlashCal_Record_t;

void FlashCal_InitDefaults(FlashCal_Record_t *rec);
uint32_t FlashCal_ComputeCrc32(const uint8_t *data, uint32_t len);
bool FlashCal_Validate(const FlashCal_Record_t *rec);
bool FlashCal_Load(FlashCal_Record_t *out);
bool FlashCal_Save(const FlashCal_Record_t *in);
bool FlashCal_Erase(void);

#ifdef UNIT_TEST_HOST
uint8_t *FlashCal_MockStoragePtr(void);
void FlashCal_MockReset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_CAL_H_ */

