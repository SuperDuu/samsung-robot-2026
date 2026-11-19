/**
  ******************************************************************************
  * @file    tof_vl53l1x.c
  * @brief   Non-blocking multi-sensor VL53L1X driver & obstacle manager on I2C2.
  ******************************************************************************
  */

#include "tof_vl53l1x.h"
#include <string.h>

static uint16_t tof_median3(uint16_t a, uint16_t b, uint16_t c)
{
  if (a > b)
  {
    uint16_t t = a;
    a = b;
    b = t;
  }
  if (b > c)
  {
    uint16_t t = b;
    b = c;
    c = t;
  }
  if (a > b)
  {
    b = a;
  }
  return b;
}

static void tof_set_xshut(const ToF_SlotConfig_t *slot, GPIO_PinState state)
{
  if ((slot != NULL) && (slot->xshut_port != NULL))
  {
    HAL_GPIO_WritePin(slot->xshut_port, slot->xshut_pin, state);
  }
}

static void tof_recover_i2c_bus(ToF_Manager_t *mgr)
{
  if ((mgr == NULL) || (mgr->hi2c == NULL))
  {
    return;
  }

#ifndef UNIT_TEST_HOST
  if (mgr->hi2c->Instance == I2C2)
  {
    /* Clock 9 SCL pulses + STOP on PB10 (SCL) / PB11 (SDA) if slave holds SDA low */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;

    HAL_I2C_DeInit(mgr->hi2c);
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    for (uint8_t c = 0U; c < 9U; c++)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
      for (volatile uint32_t d = 0U; d < 120U; d++) {}
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
      for (volatile uint32_t d = 0U; d < 120U; d++) {}
    }
    /* STOP condition: SDA low -> SCL high -> SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0U; d < 120U; d++) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    for (volatile uint32_t d = 0U; d < 120U; d++) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    for (volatile uint32_t d = 0U; d < 120U; d++) {}

    (void)HAL_I2C_Init(mgr->hi2c);
  }
#else
  (void)HAL_I2C_DeInit(mgr->hi2c);
  (void)HAL_I2C_Init(mgr->hi2c);
#endif
  mgr->i2c_recoveries++;
}

static void tof_mark_sensor_offline(ToF_Manager_t *mgr, uint8_t idx, uint32_t now_ms)
{
  mgr->sensors[idx].online = false;
  mgr->online_mask &= (uint8_t)(~(1U << idx));
  mgr->sensors[idx].next_recover_ms = now_ms + TOF_RECOVER_BACKOFF_MS;
  tof_recover_i2c_bus(mgr);
}

static bool tof_boot_and_load_single(ToF_Manager_t *mgr, uint8_t idx)
{
  const ToF_SlotConfig_t *slot = &mgr->cfg.slots[idx];
  ToF_SensorState_t *st = &mgr->sensors[idx];

  if (!slot->enabled || (slot->xshut_port == NULL) || (slot->addr_8bit == 0U))
  {
    return false;
  }

  st->configured = true;
  st->online = false;
  st->fw_loaded = false;
  st->addr_8bit = slot->addr_8bit;
  st->filtered_mm = TOF_INVALID_DISTANCE_MM;
  st->raw_mm = TOF_INVALID_DISTANCE_MM;

  /* 1. Release this sensor's XSHUT while unconfigured/absent slots stay held in reset */
  tof_set_xshut(slot, GPIO_PIN_SET);
  (void)VL53L1_WaitMs(VL53L1_DEFAULT_ADDR_8BIT, 3);

  /* 2. Probe whether a VL53L1X actually responds on default 0x52 (if already reprogrammed, probe target too) */
  uint16_t active_addr = VL53L1_DEFAULT_ADDR_8BIT;
  bool ack_default = false;
  for (uint8_t attempt = 0U; attempt < 6U; attempt++)
  {
    if (VL53L1_Platform_IsDeviceReady(VL53L1_DEFAULT_ADDR_8BIT, 1U))
    {
      ack_default = true;
      break;
    }
    (void)VL53L1_WaitMs(VL53L1_DEFAULT_ADDR_8BIT, 2);
  }

  if (!ack_default)
  {
    if (VL53L1_Platform_IsDeviceReady(slot->addr_8bit, 1U))
    {
      active_addr = slot->addr_8bit;
    }
    else
    {
      /* Keep missing/failed slot powered down so it never blocks address 0x52 later */
      tof_set_xshut(slot, GPIO_PIN_RESET);
      return false;
    }
  }

  /* 3. Poll firmware boot bit VL53L1_FIRMWARE__SYSTEM_STATUS (0x00E5) */
  uint8_t booted = 0U;
  for (uint8_t iter = 0U; iter < 50U; iter++)
  {
    if ((VL53L1X_BootState(active_addr, &booted) == 0) && ((booted & 0x01U) != 0U))
    {
      break;
    }
    (void)VL53L1_WaitMs(active_addr, 2);
  }
  if ((booted & 0x01U) == 0U)
  {
    tof_set_xshut(slot, GPIO_PIN_RESET);
    return false;
  }

  /* 4. Validate Silicon Model ID (0xEACC) */
  uint16_t model_id = 0U;
  if ((VL53L1X_GetSensorId(active_addr, &model_id) != 0) || (model_id != VL53L1X_EXPECTED_MODEL_ID))
  {
    st->model_id = model_id;
    tof_set_xshut(slot, GPIO_PIN_RESET);
    return false;
  }
  st->model_id = model_id;

  /* 5. Reassign I2C address before uploading default firmware register configuration */
  if (active_addr != slot->addr_8bit)
  {
    if (VL53L1X_SetI2CAddress(active_addr, slot->addr_8bit) != 0)
    {
      tof_set_xshut(slot, GPIO_PIN_RESET);
      return false;
    }
    active_addr = slot->addr_8bit;
    (void)VL53L1_WaitMs(active_addr, 1);
    if (!VL53L1_Platform_IsDeviceReady(active_addr, 2U))
    {
      tof_set_xshut(slot, GPIO_PIN_RESET);
      return false;
    }
  }

  /* 6. Upload full ST official VL51L1X_DEFAULT_CONFIGURATION (0x2D..0x87) + dummy frame + VHV */
  if (VL53L1X_SensorInit(active_addr) != 0)
  {
    tof_set_xshut(slot, GPIO_PIN_RESET);
    return false;
  }
  st->fw_loaded = true;

  if (mgr->cfg.use_2v8_io)
  {
    (void)VL53L1_WrByte(active_addr, PAD_I2C_HV__EXTSUP_CONFIG, 0x01U);
    (void)VL53L1_WrByte(active_addr, 0x002FU, 0x01U);
  }

  uint16_t dm = (mgr->cfg.distance_mode == 2U) ? 2U : 1U;
  uint16_t tb = mgr->cfg.timing_budget_ms;
  if (tb == 0U)
  {
    tb = TOF_DEFAULT_TB_MS;
  }
  uint32_t im = mgr->cfg.inter_meas_ms;
  if (im < tb)
  {
    im = tb + 5U;
  }

  if ((VL53L1X_SetDistanceMode(active_addr, dm) != 0) ||
      (VL53L1X_SetTimingBudgetInMs(active_addr, tb) != 0) ||
      (VL53L1X_SetInterMeasurementInMs(active_addr, im) != 0))
  {
    return false;
  }

  uint8_t int_pol = 1U;
  (void)VL53L1X_GetInterruptPolarity(active_addr, &int_pol);
  st->int_pol = (uint8_t)(int_pol & 0x01U);

  if (VL53L1X_StartRanging(active_addr) != 0)
  {
    return false;
  }

  st->online = true;
  st->consec_errs = 0U;
  st->median_idx = 0U;
  st->median_count = 0U;
  st->clear_streak = 0U;
  st->filtered_mm = TOF_CLEAR_DISTANCE_MM;
  st->last_sample_ms = HAL_GetTick();
  return true;
}

void ToF_DefaultConfig(ToF_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }
  memset(cfg, 0, sizeof(*cfg));
  cfg->distance_mode    = TOF_DEFAULT_DIST_MODE;
  cfg->timing_budget_ms = TOF_DEFAULT_TB_MS;
  cfg->inter_meas_ms    = TOF_DEFAULT_IM_MS;
  cfg->use_2v8_io       = true; /* MainRB supplies 3V3 -> 2.8V/3.3V I2C IO (not 1.8V) */

  cfg->slots[TOF_SENSOR_FRONT].enabled    = true;
  cfg->slots[TOF_SENSOR_FRONT].xshut_port = LS1_GPIO_Port;
  cfg->slots[TOF_SENSOR_FRONT].xshut_pin  = LS1_Pin;
  cfg->slots[TOF_SENSOR_FRONT].addr_8bit  = 0x54U;

  cfg->slots[TOF_SENSOR_REAR].enabled     = true;
  cfg->slots[TOF_SENSOR_REAR].xshut_port  = LS2_GPIO_Port;
  cfg->slots[TOF_SENSOR_REAR].xshut_pin   = LS2_Pin;
  cfg->slots[TOF_SENSOR_REAR].addr_8bit   = 0x56U;

  cfg->slots[TOF_SENSOR_AUX_L].enabled    = true;
  cfg->slots[TOF_SENSOR_AUX_L].xshut_port = LS3_GPIO_Port;
  cfg->slots[TOF_SENSOR_AUX_L].xshut_pin  = LS3_Pin;
  cfg->slots[TOF_SENSOR_AUX_L].addr_8bit  = 0x58U;

  cfg->slots[TOF_SENSOR_AUX_R].enabled    = true;
  cfg->slots[TOF_SENSOR_AUX_R].xshut_port = LS4_GPIO_Port;
  cfg->slots[TOF_SENSOR_AUX_R].xshut_pin  = LS4_Pin;
  cfg->slots[TOF_SENSOR_AUX_R].addr_8bit  = 0x5AU;
}

uint8_t ToF_Init(ToF_Manager_t *mgr, I2C_HandleTypeDef *hi2c, const ToF_Config_t *cfg)
{
  if ((mgr == NULL) || (hi2c == NULL))
  {
    return 0U;
  }

  memset(mgr, 0, sizeof(*mgr));
  mgr->hi2c = hi2c;
  if (cfg != NULL)
  {
    mgr->cfg = *cfg;
  }
  else
  {
    ToF_DefaultConfig(&mgr->cfg);
  }

  VL53L1_Platform_BindI2C(hi2c);

  /* 1. Force all configured XSHUT lines LOW simultaneously to reset all sensors */
  for (uint8_t i = 0U; i < TOF_MAX_SENSORS; i++)
  {
    if (mgr->cfg.slots[i].enabled)
    {
      tof_set_xshut(&mgr->cfg.slots[i], GPIO_PIN_RESET);
    }
  }
  (void)VL53L1_WaitMs(VL53L1_DEFAULT_ADDR_8BIT, 5);

  /* Detect a stuck/always-on sensor holding 0x52 while all XSHUT are LOW */
  mgr->addr52_conflict = VL53L1_Platform_IsDeviceReady(VL53L1_DEFAULT_ADDR_8BIT, 1U);

  /* 2. Sequentially boot, assign unique I2C addresses, and upload VL53L1X default config */
  uint8_t online_cnt = 0U;
  for (uint8_t i = 0U; i < TOF_MAX_SENSORS; i++)
  {
    if (tof_boot_and_load_single(mgr, i))
    {
      mgr->online_mask |= (uint8_t)(1U << i);
      online_cnt++;
    }
  }

  mgr->last_poll_ms = HAL_GetTick();
  return online_cnt;
}

void ToF_Poll(ToF_Manager_t *mgr, uint32_t now_ms)
{
  if ((mgr == NULL) || (mgr->hi2c == NULL))
  {
    return;
  }

  if ((uint32_t)(now_ms - mgr->last_poll_ms) < TOF_POLL_SLOT_MS)
  {
    return;
  }
  mgr->last_poll_ms = now_ms;

  VL53L1_Platform_BindI2C(mgr->hi2c);

  /* Round-robin to the next enabled slot (at most 1 sensor transaction per call) */
  uint8_t chosen = 0xFFU;
  for (uint8_t step = 0U; step < TOF_MAX_SENSORS; step++)
  {
    uint8_t idx = (uint8_t)((mgr->poll_cursor + step) % TOF_MAX_SENSORS);
    if (mgr->cfg.slots[idx].enabled)
    {
      if (mgr->sensors[idx].online ||
          ((mgr->sensors[idx].consec_errs >= TOF_MAX_CONSEC_I2C_ERRS) &&
           ((int32_t)(now_ms - mgr->sensors[idx].next_recover_ms) >= 0)))
      {
        chosen = idx;
        mgr->poll_cursor = (uint8_t)((idx + 1U) % TOF_MAX_SENSORS);
        break;
      }
    }
  }

  if (chosen == 0xFFU)
  {
    return;
  }

  ToF_SensorState_t *st = &mgr->sensors[chosen];

  /* Attempt offline sensor hot-recovery on its backoff timer */
  if (!st->online)
  {
    tof_recover_i2c_bus(mgr);
    if (tof_boot_and_load_single(mgr, chosen))
    {
      mgr->online_mask |= (uint8_t)(1U << chosen);
      mgr->sensor_reinits++;
    }
    else
    {
      mgr->online_mask &= (uint8_t)(~(1U << chosen));
      st->next_recover_ms = now_ms + TOF_RECOVER_BACKOFF_MS;
    }
    return;
  }

  /* Fast 1-byte DataReady poll against cached interrupt polarity */
  uint8_t hv_status = 0U;
  if (VL53L1_RdByte(st->addr_8bit, GPIO__TIO_HV_STATUS, &hv_status) != 0)
  {
    st->consec_errs++;
    if (st->consec_errs >= TOF_MAX_CONSEC_I2C_ERRS)
    {
      tof_mark_sensor_offline(mgr, chosen, now_ms);
    }
    return;
  }

  if ((hv_status & 0x01U) != st->int_pol)
  {
    /* Measurement frame still integrating inside VL53L1X; check if overdue */
    if ((uint32_t)(now_ms - st->last_sample_ms) > TOF_SAMPLE_STALE_MS)
    {
      st->consec_errs++;
      if (st->consec_errs >= TOF_MAX_CONSEC_I2C_ERRS)
      {
        tof_mark_sensor_offline(mgr, chosen, now_ms);
      }
    }
    else
    {
      st->consec_errs = 0U;
    }
    return;
  }

  VL53L1X_Result_t res = {0};
  VL53L1X_ERROR err = VL53L1X_GetResult(st->addr_8bit, &res);
  err |= VL53L1X_ClearInterrupt(st->addr_8bit);
  if (err != 0)
  {
    st->consec_errs++;
    if (st->consec_errs >= TOF_MAX_CONSEC_I2C_ERRS)
    {
      tof_mark_sensor_offline(mgr, chosen, now_ms);
    }
    return;
  }

  st->consec_errs = 0U;
  st->last_result = res;
  st->last_range_status = res.Status;
  st->sample_count++;
  st->last_sample_ms = now_ms;

  /* Status 0 = valid ranging; Status 1/2/4/7 with no valid target clear after streak hysteresis */
  if ((res.Status == 0U) && (res.Distance >= 10U) && (res.Distance <= 4500U))
  {
    st->raw_mm = res.Distance;
    st->clear_streak = 0U;
    st->median_hist[st->median_idx] = res.Distance;
    st->median_idx = (uint8_t)((st->median_idx + 1U) % 3U);
    if (st->median_count < 3U)
    {
      st->median_count++;
    }

    if (st->median_count == 1U)
    {
      st->filtered_mm = res.Distance;
    }
    else if (st->median_count == 2U)
    {
      st->filtered_mm = (res.Distance < st->median_hist[0]) ? res.Distance : st->median_hist[0];
    }
    else
    {
      st->filtered_mm = tof_median3(st->median_hist[0], st->median_hist[1], st->median_hist[2]);
    }
  }
  else
  {
    if (st->clear_streak < 255U)
    {
      st->clear_streak++;
    }
    if (st->clear_streak >= 3U)
    {
      st->filtered_mm = TOF_CLEAR_DISTANCE_MM;
      st->median_count = 0U;
    }
  }
}

bool ToF_IsSensorFresh(const ToF_Manager_t *mgr, ToF_SensorSlot_t slot, uint32_t now_ms)
{
  if ((mgr == NULL) || ((uint8_t)slot >= TOF_MAX_SENSORS))
  {
    return false;
  }
  const ToF_SensorState_t *st = &mgr->sensors[(uint8_t)slot];
  if (!st->configured || !st->online || !st->fw_loaded)
  {
    return false;
  }
  return ((uint32_t)(now_ms - st->last_sample_ms) <= TOF_SAMPLE_STALE_MS);
}

uint16_t ToF_GetDistanceMm(const ToF_Manager_t *mgr, ToF_SensorSlot_t slot, uint32_t now_ms)
{
  if (!ToF_IsSensorFresh(mgr, slot, now_ms))
  {
    return TOF_INVALID_DISTANCE_MM;
  }
  return mgr->sensors[(uint8_t)slot].filtered_mm;
}

bool ToF_IsDirectionBlocked(const ToF_Manager_t *mgr,
                            bool forward,
                            uint16_t threshold_mm,
                            uint32_t now_ms,
                            uint16_t *out_min_mm)
{
  uint16_t min_mm = TOF_INVALID_DISTANCE_MM;
  if (mgr != NULL)
  {
    ToF_SensorSlot_t primary = forward ? TOF_SENSOR_FRONT : TOF_SENSOR_REAR;
    uint16_t d = ToF_GetDistanceMm(mgr, primary, now_ms);
    if (d < min_mm)
    {
      min_mm = d;
    }

    /* Optional Aux sensors if online: Aux L protects front-side/re-check, Aux R protects rear/side */
    ToF_SensorSlot_t aux = forward ? TOF_SENSOR_AUX_L : TOF_SENSOR_AUX_R;
    uint16_t da = ToF_GetDistanceMm(mgr, aux, now_ms);
    if (da < min_mm)
    {
      min_mm = da;
    }
  }

  if (out_min_mm != NULL)
  {
    *out_min_mm = min_mm;
  }

  return (min_mm <= threshold_mm);
}
