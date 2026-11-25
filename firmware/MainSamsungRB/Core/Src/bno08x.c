/**
 ******************************************************************************
 * @file    bno08x.c
 * @brief   CEVA / Hillcrest Labs BNO080 / BNO085 / BNO086 SHTP driver for
 * STM32.
 ******************************************************************************
 */

#include "bno08x.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define BNO08X_I2C_TIMEOUT_MS 10U
#define BNO08X_POLL_SIZE 28U
#define BNO08X_BUFFER_SIZE 64U

static uint8_t s_bno08x_tx_seq = 0U;
static uint8_t s_bno08x_cmd_seq = 0U;

bool BNO08X_Detect(I2C_HandleTypeDef *hi2c, uint8_t *found_addr) {
  if (hi2c == NULL) {
    return false;
  }

  uint8_t dummy[4] = {0};

  /* Retry for up to 500 ms (25 attempts x 20 ms) to allow BNO08x internal POR
   * bootloader to finish */
  for (uint8_t attempt = 0U; attempt < 25U; attempt++) {
    /* 1. Try 0x4A (default address with SA0 = GND) */
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(BNO08X_I2C_ADDR_LO << 1U),
                               dummy, 4U, 20U) == HAL_OK) {
      uint16_t len =
          (uint16_t)dummy[0] | (((uint16_t)(dummy[1] & 0x7FU)) << 8U);
      bool empty_queue =
          (len == 0U) && (dummy[2] == 0U) && (dummy[3] == 0U);
      if (empty_queue || ((len >= 4U) && (len <= 1024U))) {
        if (found_addr != NULL) {
          *found_addr = BNO08X_I2C_ADDR_LO;
        }
        return true;
      }
    }

    /* 2. Try 0x4B (alternate address with SA0 = 3V3) */
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(BNO08X_I2C_ADDR_HI << 1U),
                               dummy, 4U, 20U) == HAL_OK) {
      uint16_t len =
          (uint16_t)dummy[0] | (((uint16_t)(dummy[1] & 0x7FU)) << 8U);
      bool empty_queue =
          (len == 0U) && (dummy[2] == 0U) && (dummy[3] == 0U);
      if (empty_queue || ((len >= 4U) && (len <= 1024U))) {
        if (found_addr != NULL) {
          *found_addr = BNO08X_I2C_ADDR_HI;
        }
        return true;
      }
    }

    HAL_Delay(20);
  }

  return false;
}

bool BNO08X_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit,
                 uint32_t report_interval_us) {
  if (hi2c == NULL) {
    return false;
  }

  uint8_t rx_buf[BNO08X_BUFFER_SIZE];

  /* Flush pending POR / SHTP advertisement packets cleanly */
  for (uint8_t i = 0U; i < 10U; i++) {
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(addr_7bit << 1U), rx_buf,
                               sizeof(rx_buf), 10U) != HAL_OK) {
      break;
    }
    if ((rx_buf[0] == 0U) && (rx_buf[1] == 0U)) {
      break;
    }
    HAL_Delay(5);
  }

  /* 1. Build Set Feature Command packet for 9-DOF Rotation Vector
   * (Report ID 0x05): Accel + Gyro + Magnetometer absolute heading anchor. */
  uint8_t cmd[21] = {0};
  cmd[0] = 21U;                    /* Packet length LSB */
  cmd[1] = 0U;                     /* Packet length MSB */
  cmd[2] = BNO08X_CHANNEL_CONTROL; /* Channel 2 = Control */
  cmd[3] = s_bno08x_tx_seq++;      /* Sequence */

  cmd[4] = BNO08X_CMD_SET_FEATURE;    /* Command 0xFD */
  cmd[5] = BNO08X_REPORT_ROT_VECTOR;  /* Feature Report ID 0x05 (9-DOF) */
  cmd[6] = 0U;                        /* Flags */
  cmd[7] = 0U;                        /* Change sensitivity LSB */
  cmd[8] = 0U;                        /* Change sensitivity MSB */
  cmd[9] = (uint8_t)(report_interval_us & 0xFFU);
  cmd[10] = (uint8_t)((report_interval_us >> 8U) & 0xFFU);
  cmd[11] = (uint8_t)((report_interval_us >> 16U) & 0xFFU);
  cmd[12] = (uint8_t)((report_interval_us >> 24U) & 0xFFU);
  cmd[13] = 0U; /* Batch interval LSB */
  cmd[14] = 0U;
  cmd[15] = 0U;
  cmd[16] = 0U; /* Batch interval MSB */
  cmd[17] = 0U; /* Sensor-specific config LSB */
  cmd[18] = 0U;
  cmd[19] = 0U;
  cmd[20] = 0U; /* Sensor-specific config MSB */

  if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr_7bit << 1U), cmd, 21U,
                              BNO08X_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  HAL_Delay(10);

  /* 2. Enable background dynamic calibration for Accel + Gyro + Mag (9-DOF) */
  uint8_t cal_cmd[16] = {0};
  cal_cmd[0] = 16U;
  cal_cmd[1] = 0U;
  cal_cmd[2] = BNO08X_CHANNEL_CONTROL;
  cal_cmd[3] = s_bno08x_tx_seq++;
  cal_cmd[4] = BNO08X_CMD_REQUEST; /* 0xF2 */
  cal_cmd[5] = s_bno08x_cmd_seq++;
  cal_cmd[6] = BNO08X_SH2_CMD_ME_CALIBRATE; /* 0x07 */
  cal_cmd[7] = 1U;                          /* Accel cal enable */
  cal_cmd[8] = 1U;                          /* Gyro cal enable */
  cal_cmd[9] = 1U;                          /* Mag cal ENABLED for 9-DOF */
  cal_cmd[10] = 0U; /* Subcommand: Configure ME Calibration */
  (void)HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr_7bit << 1U), cal_cmd, 16U,
                                BNO08X_I2C_TIMEOUT_MS);

  HAL_Delay(10);
  return true;
}

static void BNO08X_QuatToEuler(float qw, float qx, float qy, float qz,
                               float *yaw_deg, float *pitch_deg,
                               float *roll_deg) {
  /* Yaw (Z-axis rotation): atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)) */
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  *yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);

  /* Pitch (Y-axis rotation): asin(2*(w*y - z*x)) */
  float sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp > 1.0f) {
    sinp = 1.0f;
  } else if (sinp < -1.0f) {
    sinp = -1.0f;
  }
  *pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

  /* Roll (X-axis rotation): atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2)) */
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  *roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);
}

bool BNO08X_Poll(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit,
                 BNO08X_Rotation_t *rot) {
  if ((hi2c == NULL) || (rot == NULL)) {
    return false;
  }

  uint8_t buf[BNO08X_POLL_SIZE];
  if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(addr_7bit << 1U), buf,
                             sizeof(buf), 5U) != HAL_OK) {
    return false;
  }

  /* Reject continuation fragments (MSB of length = 1) which do not start with a
   * report ID */
  if ((buf[1] & 0x80U) != 0U) {
    return false;
  }

  uint16_t packet_len = (uint16_t)buf[0] | (((uint16_t)(buf[1] & 0x7FU)) << 8U);
  if ((packet_len < 4U) || (packet_len > 1024U)) {
    return false;
  }

  uint8_t channel = buf[2];
  if (channel != BNO08X_CHANNEL_REPORTS) {
    return false;
  }

  uint16_t max_scan =
      (packet_len < sizeof(buf)) ? packet_len : (uint16_t)sizeof(buf);
  uint16_t idx = 4U;
  bool got_quat = false;

  while (idx < max_scan) {
    uint8_t report_id = buf[idx];
    if (report_id == BNO08X_REPORT_BASE_TS) {
      idx += 5U; /* 1 byte report ID + 4 bytes timestamp */
    } else if ((report_id == BNO08X_REPORT_ARVR_GAME_RV) ||
               (report_id == BNO08X_REPORT_GAME_ROT_VEC) ||
               (report_id == BNO08X_REPORT_ROT_VECTOR)) {
      if ((idx + 12U) <= max_scan) {
        uint8_t status = buf[idx + 2U] & 0x03U;
        int16_t raw_i = (int16_t)((uint16_t)buf[idx + 4U] |
                                  ((uint16_t)buf[idx + 5U] << 8U));
        int16_t raw_j = (int16_t)((uint16_t)buf[idx + 6U] |
                                  ((uint16_t)buf[idx + 7U] << 8U));
        int16_t raw_k = (int16_t)((uint16_t)buf[idx + 8U] |
                                  ((uint16_t)buf[idx + 9U] << 8U));
        int16_t raw_r = (int16_t)((uint16_t)buf[idx + 10U] |
                                  ((uint16_t)buf[idx + 11U] << 8U));

        const float q_scale = 1.0f / 16384.0f;
        float qx = (float)raw_i * q_scale;
        float qy = (float)raw_j * q_scale;
        float qz = (float)raw_k * q_scale;
        float qw = (float)raw_r * q_scale;

        /* Reject corrupted / falsely-matched SHTP bytes via unit quaternion
         * norm check */
        float norm2 = (qx * qx) + (qy * qy) + (qz * qz) + (qw * qw);
        if ((norm2 >= 0.85f) && (norm2 <= 1.15f)) {
          rot->qx = qx;
          rot->qy = qy;
          rot->qz = qz;
          rot->qw = qw;
          rot->accuracy = status;

          BNO08X_QuatToEuler(rot->qw, rot->qx, rot->qy, rot->qz, &rot->yaw_deg,
                             &rot->pitch_deg, &rot->roll_deg);

          if (isfinite(rot->yaw_deg) && isfinite(rot->pitch_deg) &&
              isfinite(rot->roll_deg)) {
            got_quat = true;
          }
        }
        idx += (report_id == BNO08X_REPORT_ROT_VECTOR) ? 14U : 12U;
      } else {
        break;
      }
    } else {
      /* Unknown report ID -> stop scanning instead of byte-sliding into random
       * payload */
      break;
    }
  }

  rot->has_new_data = got_quat;
  return got_quat;
}

bool BNO08X_SaveCalibration(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit) {
  if (hi2c == NULL) {
    return false;
  }

  uint8_t cmd[16] = {0};
  cmd[0] = 16U;
  cmd[1] = 0U;
  cmd[2] = BNO08X_CHANNEL_CONTROL;
  cmd[3] = s_bno08x_tx_seq++;
  cmd[4] = BNO08X_CMD_REQUEST; /* 0xF2 */
  cmd[5] = s_bno08x_cmd_seq++;
  cmd[6] = BNO08X_SH2_CMD_SAVE_DCD; /* 0x06: Persist DCD into internal Flash */

  if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr_7bit << 1U), cmd, 16U,
                              BNO08X_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  HAL_Delay(20);
  return true;
}

bool BNO08X_ClearCalibration(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit,
                             uint32_t report_interval_us) {
  if (hi2c == NULL) {
    return false;
  }

  uint8_t cmd[16] = {0};
  cmd[0] = 16U;
  cmd[1] = 0U;
  cmd[2] = BNO08X_CHANNEL_CONTROL;
  cmd[3] = s_bno08x_tx_seq++;
  cmd[4] = BNO08X_CMD_REQUEST; /* 0xF2 */
  cmd[5] = s_bno08x_cmd_seq++;
  cmd[6] = BNO08X_SH2_CMD_CLEAR_DCD; /* 0x0B: Clear DCD & reset sensor hub */

  if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr_7bit << 1U), cmd, 16U,
                              BNO08X_I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  /* Wait for BNO080/085 internal Flash DCD erase + Cortex-M0 reboot (~450 ms),
   * confirm the chip responds again, then re-enable 6-DOF Game RV (0x08) */
  HAL_Delay(450);
  uint8_t detected_addr = addr_7bit;
  (void)BNO08X_Detect(hi2c, &detected_addr);
  return BNO08X_Init(hi2c, addr_7bit, report_interval_us);
}
