/**
  ******************************************************************************
  * @file    pid_controller.c
  * @brief   Fixed-point 1 kHz cascaded position + velocity PID controller,
  *          wrap-safe encoder delta, anti-windup, IR2184 bootstrap clamp,
  *          watchdog supervision, and Flash Page 15 persistence.
  ******************************************************************************
  */

#include "pid_controller.h"
#include <stddef.h>
#include <string.h>

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(PID_Config_t) == 40U, "PID_Config_t must stay 40 bytes");
_Static_assert((sizeof(PID_Config_t) % 8U) == 0U, "PID_Config_t must be 64-bit aligned for STM32G0 Flash");
#endif

#ifdef UNIT_TEST_HOST
static uint8_t g_host_flash_page[sizeof(PID_Config_t)];
#endif

static int32_t pid_clamp_i32(int64_t val, int32_t min_v, int32_t max_v)
{
  if (val > (int64_t)max_v)
  {
    return max_v;
  }
  if (val < (int64_t)min_v)
  {
    return min_v;
  }
  return (int32_t)val;
}

static int64_t pid_clamp_i64(int64_t val, int64_t min_v, int64_t max_v)
{
  if (val > max_v)
  {
    return max_v;
  }
  if (val < min_v)
  {
    return min_v;
  }
  return val;
}

static int32_t pid_div_round_i64(int64_t num, int32_t den)
{
  int64_t half = (int64_t)den / 2LL;
  if (num >= 0LL)
  {
    return (int32_t)((num + half) / (int64_t)den);
  }
  return (int32_t)((num - half) / (int64_t)den);
}

void PID_LoadDefaults(PID_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  memset(cfg, 0, sizeof(*cfg));
  cfg->magic             = PID_CONFIG_MAGIC;
  cfg->version           = PID_CONFIG_VERSION;
  cfg->node_id           = PID_DEFAULT_NODE_ID;
  cfg->invert_dir        = 0U;
  cfg->cpr               = PID_DEFAULT_CPR;
  cfg->max_duty_permille = PID_MAX_DUTY_PERMILLE;
  cfg->max_vel_rpm_x10   = PID_DEFAULT_MAX_VEL_RPM_X10;
  cfg->vel_kp_m          = PID_DEFAULT_VEL_KP_M;
  cfg->vel_ki_m          = PID_DEFAULT_VEL_KI_M;
  cfg->vel_kd_m          = PID_DEFAULT_VEL_KD_M;
  cfg->pos_kp_m          = PID_DEFAULT_POS_KP_M;
  cfg->pos_kd_m          = PID_DEFAULT_POS_KD_M;
  cfg->crc32             = PID_ConfigCrc32(cfg);
}

uint32_t PID_ConfigCrc32(const PID_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return 0UL;
  }

  const uint8_t *bytes = (const uint8_t *)cfg;
  size_t len = offsetof(PID_Config_t, crc32);
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)bytes[i];
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
  return ~crc;
}

bool PID_ValidateConfig(const PID_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return false;
  }
  if ((cfg->magic != PID_CONFIG_MAGIC) || (cfg->version != PID_CONFIG_VERSION))
  {
    return false;
  }
  if ((cfg->node_id < RS485_ADDR_SLAVE_MIN) || (cfg->node_id > RS485_ADDR_SLAVE_MAX))
  {
    return false;
  }
  if ((cfg->invert_dir > 1U) || (cfg->cpr < 16U) || (cfg->cpr > 20000U))
  {
    return false;
  }
  if ((cfg->max_duty_permille == 0U) || (cfg->max_duty_permille > PID_MAX_DUTY_PERMILLE))
  {
    return false;
  }
  if ((cfg->max_vel_rpm_x10 < 100) || (cfg->max_vel_rpm_x10 > 20000))
  {
    return false;
  }
  if ((cfg->vel_kp_m < 0) || (cfg->vel_ki_m < 0) || (cfg->vel_kd_m < 0) ||
      (cfg->pos_kp_m < 0) || (cfg->pos_kd_m < 0))
  {
    return false;
  }
  if (PID_ConfigCrc32(cfg) != cfg->crc32)
  {
    return false;
  }
  return true;
}

bool PID_LoadConfigFlash(PID_Config_t *out_cfg)
{
  PID_Config_t candidate;
  if (out_cfg == NULL)
  {
    return false;
  }

#ifdef UNIT_TEST_HOST
  memcpy(&candidate, g_host_flash_page, sizeof(candidate));
#else
  memcpy(&candidate, (const void *)PID_FLASH_CONFIG_ADDR, sizeof(candidate));
#endif

  if (!PID_ValidateConfig(&candidate))
  {
    return false;
  }

  *out_cfg = candidate;
  return true;
}

bool PID_SaveConfigFlash(PID_Controller_t *ctrl)
{
  if (ctrl == NULL)
  {
    return false;
  }

  ctrl->cfg.magic   = PID_CONFIG_MAGIC;
  ctrl->cfg.version = PID_CONFIG_VERSION;
  if ((ctrl->cfg.max_duty_permille == 0U) || (ctrl->cfg.max_duty_permille > PID_MAX_DUTY_PERMILLE))
  {
    ctrl->cfg.max_duty_permille = PID_MAX_DUTY_PERMILLE;
  }
  ctrl->cfg.crc32   = PID_ConfigCrc32(&ctrl->cfg);

#ifdef UNIT_TEST_HOST
  memcpy(g_host_flash_page, &ctrl->cfg, sizeof(ctrl->cfg));
  ctrl->flash_valid = 1U;
  return true;
#else
  uint64_t words[sizeof(PID_Config_t) / 8U];
  memcpy(words, &ctrl->cfg, sizeof(ctrl->cfg));

  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0xFFFFFFFFUL;
  HAL_StatusTypeDef st;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  /* Clear any sticky Flash status flags before erasing Page 15 */
  WRITE_REG(FLASH->SR, FLASH_SR_CLEAR);

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks     = FLASH_BANK_1;
  erase.Page      = PID_FLASH_PAGE_INDEX;
  erase.NbPages   = 1U;

  st = HAL_FLASHEx_Erase(&erase, &page_error);
  if ((st == HAL_OK) && (page_error == 0xFFFFFFFFUL))
  {
    for (uint32_t i = 0U; i < (uint32_t)(sizeof(words) / sizeof(words[0])); i++)
    {
      uint32_t addr = PID_FLASH_CONFIG_ADDR + (i * 8U);
      st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, words[i]);
      if (st != HAL_OK)
      {
        break;
      }
    }
  }

  (void)HAL_FLASH_Lock();

  if (st != HAL_OK)
  {
    ctrl->flash_valid = 0U;
    return false;
  }

  if (memcmp((const void *)PID_FLASH_CONFIG_ADDR, &ctrl->cfg, sizeof(ctrl->cfg)) != 0)
  {
    ctrl->flash_valid = 0U;
    return false;
  }

  ctrl->flash_valid = 1U;
  return true;
#endif
}

void PID_ResetControlState(PID_Controller_t *ctrl)
{
  if (ctrl == NULL)
  {
    return;
  }

  ctrl->vel_target_x10     = 0;
  ctrl->vel_int_err_ms     = 0LL;
  ctrl->duty_permille      = 0;
  ctrl->hw_duty_permille   = 0;
  ctrl->saturated          = 0U;
  ctrl->prev_vel_ema_x1000 = ctrl->vel_ema_x1000;
}

void PID_ZeroEncoder(PID_Controller_t *ctrl, uint16_t hw_cnt)
{
  if (ctrl == NULL)
  {
    return;
  }

  ctrl->enc_count   = 0;
  ctrl->prev_hw_cnt = hw_cnt;
  ctrl->enc_primed  = 1U;
  if (ctrl->mode == RS485_WHEEL_POSITION)
  {
    ctrl->target_value = 0;
  }
}

void PID_Init(PID_Controller_t *ctrl, uint16_t initial_hw_cnt, uint32_t now_ms)
{
  if (ctrl == NULL)
  {
    return;
  }

  memset(ctrl, 0, sizeof(*ctrl));
  if (PID_LoadConfigFlash(&ctrl->cfg))
  {
    ctrl->flash_valid = 1U;
  }
  else
  {
    PID_LoadDefaults(&ctrl->cfg);
    ctrl->flash_valid = 0U;
  }

  ctrl->mode        = RS485_WHEEL_DISABLED;
  ctrl->prev_hw_cnt = initial_hw_cnt;
  ctrl->enc_primed  = 1U;
  ctrl->last_cmd_ms = now_ms;
}

void PID_ApplyCommand(PID_Controller_t *ctrl,
                      RS485_WheelMode_t mode,
                      int32_t target,
                      uint8_t flags,
                      uint32_t now_ms)
{
  if (ctrl == NULL)
  {
    return;
  }

  ctrl->last_cmd_ms = now_ms;
  ctrl->has_cmd     = 1U;

  if ((flags & RS485_CMD_FLAG_CLEAR_FAULTS) != 0U)
  {
    ctrl->watchdog_tripped = 0U;
    ctrl->estop_latched    = 0U;
    ctrl->comm_err_latched = 0U;
  }

  if ((flags & RS485_CMD_FLAG_ESTOP) != 0U)
  {
    ctrl->estop_latched = 1U;
    ctrl->mode          = RS485_WHEEL_DISABLED;
    ctrl->target_value  = 0;
    PID_ResetControlState(ctrl);
    return;
  }

  /* Clear transient link-loss watchdog once a valid non-ESTOP command arrives */
  if (ctrl->watchdog_tripped != 0U)
  {
    ctrl->watchdog_tripped = 0U;
  }

  if (ctrl->estop_latched != 0U)
  {
    ctrl->mode         = RS485_WHEEL_DISABLED;
    ctrl->target_value = 0;
    PID_ResetControlState(ctrl);
    return;
  }

  RS485_WheelMode_t next_mode = mode;
  if ((uint8_t)next_mode > (uint8_t)RS485_WHEEL_POSITION)
  {
    next_mode = RS485_WHEEL_BRAKE;
  }

  if (next_mode != ctrl->mode)
  {
    PID_ResetControlState(ctrl);
    ctrl->mode = next_mode;
  }

  if (next_mode == RS485_WHEEL_DUTY)
  {
    int32_t max_d = (int32_t)ctrl->cfg.max_duty_permille;
    ctrl->target_value = pid_clamp_i32((int64_t)target, -max_d, max_d);
  }
  else if (next_mode == RS485_WHEEL_VELOCITY)
  {
    int32_t max_v = ctrl->cfg.max_vel_rpm_x10;
    ctrl->target_value = pid_clamp_i32((int64_t)target, -max_v, max_v);
  }
  else if (next_mode == RS485_WHEEL_POSITION)
  {
    ctrl->target_value = target;
  }
  else
  {
    ctrl->target_value = 0;
  }
}

uint8_t PID_HandleConfigSubcmd(PID_Controller_t *ctrl,
                               const RS485_CfgCmd_t *cmd,
                               bool *out_save_deferred)
{
  if (out_save_deferred != NULL)
  {
    *out_save_deferred = false;
  }
  if ((ctrl == NULL) || (cmd == NULL))
  {
    return 1U;
  }

  switch ((RS485_CfgSubcmd_t)cmd->subcmd)
  {
    case RS485_CFG_PING:
      return 0U;

    case RS485_CFG_SET_NODE_ID:
    {
      uint8_t new_id = (cmd->param_u8 != 0U) ? cmd->param_u8 : (uint8_t)cmd->value;
      if ((new_id < RS485_ADDR_SLAVE_MIN) || (new_id > RS485_ADDR_SLAVE_MAX))
      {
        return 1U;
      }
      ctrl->cfg.node_id = new_id;
      return 0U;
    }

    case RS485_CFG_SET_VEL_KP_M:
      if ((cmd->value < 0) || (cmd->value > 50000))
      {
        return 1U;
      }
      ctrl->cfg.vel_kp_m = cmd->value;
      return 0U;

    case RS485_CFG_SET_VEL_KI_M:
      if ((cmd->value < 0) || (cmd->value > 200000))
      {
        return 1U;
      }
      ctrl->cfg.vel_ki_m = cmd->value;
      return 0U;

    case RS485_CFG_SET_VEL_KD_M:
      if ((cmd->value < 0) || (cmd->value > 50000))
      {
        return 1U;
      }
      ctrl->cfg.vel_kd_m = cmd->value;
      return 0U;

    case RS485_CFG_SET_POS_KP_M:
      if ((cmd->value < 0) || (cmd->value > 50000))
      {
        return 1U;
      }
      ctrl->cfg.pos_kp_m = cmd->value;
      return 0U;

    case RS485_CFG_SET_POS_KD_M:
      if ((cmd->value < 0) || (cmd->value > 50000))
      {
        return 1U;
      }
      ctrl->cfg.pos_kd_m = cmd->value;
      return 0U;

    case RS485_CFG_SET_CPR:
      if ((cmd->value < 16) || (cmd->value > 20000))
      {
        return 1U;
      }
      ctrl->cfg.cpr = (uint16_t)cmd->value;
      return 0U;

    case RS485_CFG_SET_INVERT:
      ctrl->cfg.invert_dir = (cmd->param_u8 != 0U || cmd->value != 0) ? 1U : 0U;
      return 0U;

    case RS485_CFG_ZERO_ENC:
      ctrl->enc_count = 0;
      if (ctrl->mode == RS485_WHEEL_POSITION)
      {
        ctrl->target_value = 0;
      }
      PID_ResetControlState(ctrl);
      return 0U;

    case RS485_CFG_SAVE_FLASH:
      ctrl->pending_flash_save = 1U;
      if (out_save_deferred != NULL)
      {
        *out_save_deferred = true;
      }
      return 0U;

    case RS485_CFG_LOAD_DEFAULTS:
    {
      uint8_t keep_id = ctrl->cfg.node_id;
      PID_LoadDefaults(&ctrl->cfg);
      if ((cmd->param_u8 == 0U) && (keep_id >= RS485_ADDR_SLAVE_MIN) && (keep_id <= RS485_ADDR_SLAVE_MAX))
      {
        ctrl->cfg.node_id = keep_id;
      }
      PID_ResetControlState(ctrl);
      return 0U;
    }

    default:
      return 1U;
  }
}

void PID_Step1kHz(PID_Controller_t *ctrl, uint16_t hw_cnt, uint32_t now_ms)
{
  if (ctrl == NULL)
  {
    return;
  }

  /* 1. Modular wrap-safe 16-bit encoder delta (never zero HW CNT mid-flight) */
  int16_t delta_raw = 0;
  if (ctrl->enc_primed == 0U)
  {
    ctrl->prev_hw_cnt = hw_cnt;
    ctrl->enc_primed  = 1U;
  }
  else
  {
    delta_raw         = (int16_t)((uint16_t)(hw_cnt - ctrl->prev_hw_cnt));
    ctrl->prev_hw_cnt = hw_cnt;
  }

  int32_t dir_sign = (ctrl->cfg.invert_dir != 0U) ? -1 : 1;
  int32_t delta    = (int32_t)delta_raw * dir_sign;
  ctrl->enc_count += delta;

  /* 2. Instantaneous speed in milli-RPM & EMA low-pass (0.85 * prev + 0.15 * raw) */
  uint16_t cpr = (ctrl->cfg.cpr >= 16U) ? ctrl->cfg.cpr : PID_DEFAULT_CPR;
  int64_t raw_rpm_x1000 = ((int64_t)delta * 60000000LL) / (int64_t)cpr;
  raw_rpm_x1000 = pid_clamp_i64(raw_rpm_x1000, -50000000LL, 50000000LL);

  ctrl->prev_vel_ema_x1000 = ctrl->vel_ema_x1000;
  int64_t ema_num = (85LL * (int64_t)ctrl->vel_ema_x1000) + (15LL * raw_rpm_x1000);
  /* Truncate toward zero so stop decays strictly to 0 while steady error <= 0.006 RPM */
  ctrl->vel_ema_x1000  = (int32_t)(ema_num / 100LL);
  ctrl->actual_rpm_x10 = pid_div_round_i64((int64_t)ctrl->vel_ema_x1000, 100);

  /* 3. Supervision: E-STOP and 250 ms command watchdog */
  RS485_WheelMode_t eff_mode = ctrl->mode;
  if (ctrl->estop_latched != 0U)
  {
    eff_mode = RS485_WHEEL_DISABLED;
  }
  else if ((eff_mode == RS485_WHEEL_DUTY) ||
           (eff_mode == RS485_WHEEL_VELOCITY) ||
           (eff_mode == RS485_WHEEL_POSITION))
  {
    if ((uint32_t)(now_ms - ctrl->last_cmd_ms) > PID_CMD_WATCHDOG_MS)
    {
      ctrl->watchdog_tripped = 1U;
      eff_mode = RS485_WHEEL_BRAKE;
    }
  }

  /* 4. Execute selected control mode */
  int32_t max_duty = (int32_t)ctrl->cfg.max_duty_permille;
  if ((max_duty <= 0) || (max_duty > PID_MAX_DUTY_PERMILLE))
  {
    max_duty = PID_MAX_DUTY_PERMILLE;
  }

  if (eff_mode == RS485_WHEEL_DISABLED)
  {
    PID_ResetControlState(ctrl);
    ctrl->gate_enable = 0U;
    return;
  }

  if (eff_mode == RS485_WHEEL_BRAKE)
  {
    PID_ResetControlState(ctrl);
    ctrl->gate_enable = 1U;
    return;
  }

  ctrl->gate_enable = 1U;

  if (eff_mode == RS485_WHEEL_DUTY)
  {
    ctrl->vel_int_err_ms   = 0LL;
    ctrl->vel_target_x10   = 0;
    ctrl->duty_permille    = pid_clamp_i32((int64_t)ctrl->target_value, -max_duty, max_duty);
    ctrl->saturated        = (ctrl->target_value > max_duty || ctrl->target_value < -max_duty) ? 1U : 0U;
    ctrl->hw_duty_permille = ctrl->duty_permille * dir_sign;
    return;
  }

  /* 4a. Outer Position PD (0.1 deg -> 0.1 RPM setpoint), D on smooth shaft velocity */
  int32_t vel_ref_x10 = ctrl->target_value;
  if (eff_mode == RS485_WHEEL_POSITION)
  {
    int32_t enc_deg_x10 = pid_div_round_i64((int64_t)ctrl->enc_count * 3600LL, (int32_t)cpr);
    int64_t pos_err_x10 = (int64_t)ctrl->target_value - (int64_t)enc_deg_x10;
    int64_t p_vel_x10   = ((int64_t)ctrl->cfg.pos_kp_m * pos_err_x10) / 1000LL;
    /* d(deg)/dt = 6 * RPM -> d_vel_x10 = Kd_pos * 6 * RPM = pos_kd_m * 6 * vel_ema_x1000 / 100000 */
    int64_t d_vel_x10   = ((int64_t)ctrl->cfg.pos_kd_m * 6LL * (int64_t)ctrl->vel_ema_x1000) / 100000LL;
    vel_ref_x10 = pid_clamp_i32(p_vel_x10 - d_vel_x10,
                                -ctrl->cfg.max_vel_rpm_x10,
                                ctrl->cfg.max_vel_rpm_x10);
  }
  else
  {
    vel_ref_x10 = pid_clamp_i32((int64_t)ctrl->target_value,
                                -ctrl->cfg.max_vel_rpm_x10,
                                ctrl->cfg.max_vel_rpm_x10);
  }
  ctrl->vel_target_x10 = vel_ref_x10;

  /* 4b. Inner Velocity PID (fixed-point, 1000/333 scaling matching HB_RBC) */
  int64_t vel_target_x1000 = (int64_t)vel_ref_x10 * 100LL;
  int64_t err_x1000        = vel_target_x1000 - (int64_t)ctrl->vel_ema_x1000;
  int64_t d_meas_rpm_s     = (int64_t)ctrl->vel_ema_x1000 - (int64_t)ctrl->prev_vel_ema_x1000;

  /* Scale factor 1e-6: 1 duty permille == 333,000 numerator units */
  int64_t p_num = (int64_t)ctrl->cfg.vel_kp_m * err_x1000;
  int64_t d_num = (int64_t)ctrl->cfg.vel_kd_m * d_meas_rpm_s * 1000LL;

  int64_t max_int_contrib = (int64_t)max_duty * 333000LL;
  int64_t cand_int_ms     = ctrl->vel_int_err_ms + err_x1000;
  cand_int_ms = pid_clamp_i64(cand_int_ms, -1000000000LL, 1000000000LL);
  if (ctrl->cfg.vel_ki_m > 0)
  {
    int64_t max_allowed_ms = (max_int_contrib * 1000LL) / (int64_t)ctrl->cfg.vel_ki_m;
    cand_int_ms = pid_clamp_i64(cand_int_ms, -max_allowed_ms, max_allowed_ms);
  }

  int64_t i_num_cur  = ((int64_t)ctrl->cfg.vel_ki_m * ctrl->vel_int_err_ms) / 1000LL;
  int64_t pre_num    = p_num + i_num_cur - d_num;
  int32_t pre_duty   = pid_div_round_i64(pre_num, 333000);

  /* Conditional integration anti-windup */
  bool sat_hi = (pre_duty >= max_duty) && (err_x1000 > 0LL);
  bool sat_lo = (pre_duty <= -max_duty) && (err_x1000 < 0LL);
  if (!sat_hi && !sat_lo)
  {
    ctrl->vel_int_err_ms = cand_int_ms;
  }

  int64_t i_num_post = ((int64_t)ctrl->cfg.vel_ki_m * ctrl->vel_int_err_ms) / 1000LL;
  int64_t out_num    = p_num + i_num_post - d_num;
  int32_t out_duty   = pid_div_round_i64(out_num, 333000);

  if (out_duty >= max_duty)
  {
    ctrl->duty_permille = max_duty;
    ctrl->saturated     = 1U;
  }
  else if (out_duty <= -max_duty)
  {
    ctrl->duty_permille = -max_duty;
    ctrl->saturated     = 1U;
  }
  else
  {
    ctrl->duty_permille = out_duty;
    ctrl->saturated     = 0U;
  }

  ctrl->hw_duty_permille = ctrl->duty_permille * dir_sign;
}

void PID_BuildTelem(const PID_Controller_t *ctrl, RS485_Telem_t *out_telem)
{
  if ((ctrl == NULL) || (out_telem == NULL))
  {
    return;
  }

  uint8_t flags = 0U;
  if (ctrl->saturated != 0U)
  {
    flags |= RS485_STAT_SATURATED;
  }
  if (ctrl->watchdog_tripped != 0U)
  {
    flags |= RS485_STAT_WATCHDOG;
  }
  if (ctrl->flash_valid != 0U)
  {
    flags |= RS485_STAT_FLASH_VALID;
  }
  if (ctrl->gate_enable != 0U)
  {
    flags |= RS485_STAT_GATE_ENABLED;
  }
  if (ctrl->comm_err_latched != 0U)
  {
    flags |= RS485_STAT_OVER_ERR;
  }

  out_telem->node_id        = ctrl->cfg.node_id;
  out_telem->mode           = (uint8_t)ctrl->mode;
  out_telem->status_flags   = flags;
  out_telem->actual_rpm_x10 = (int16_t)pid_clamp_i32((int64_t)ctrl->actual_rpm_x10, -32768, 32767);
  out_telem->duty_permille  = (int16_t)ctrl->duty_permille;
  out_telem->enc_count      = ctrl->enc_count;
}

#ifdef UNIT_TEST_HOST
void PID_HostResetFlashBacking(void)
{
  memset(g_host_flash_page, 0xFF, sizeof(g_host_flash_page));
}

void PID_HostCorruptFlashByte(uint32_t offset, uint8_t value)
{
  if (offset < sizeof(g_host_flash_page))
  {
    g_host_flash_page[offset] = value;
  }
}
#endif
