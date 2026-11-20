/**
  ******************************************************************************
  * @file    itc2026_auto.c
  * @brief   Deterministic ITC 2026 Robot 2 autonomous AMHS track controller.
  ******************************************************************************
  */

#include "itc2026_auto.h"
#include "cd4067.h"
#include <stddef.h>
#include <string.h>

void ITC_AutoInit(ITC_Auto_t *fsm, int16_t cruise_rpm_x10, float default_exit_yaw_deg)
{
  if (fsm == NULL)
  {
    return;
  }

  memset(fsm, 0, sizeof(*fsm));
  fsm->state = ITC_STATE_IDLE;
  fsm->dir_sign = +1;
  fsm->exit_target_yaw_deg = default_exit_yaw_deg;
  fsm->has_exit_yaw = true;
  fsm->base_cruise_rpm_x10 = (cruise_rpm_x10 > 0) ? cruise_rpm_x10 : ITC_BASE_RUN_RPMX10;

  Drive_LineFollowReset(&fsm->line_state);
  fsm->line_gains.kp_rpmx10_per_err     = DRIVE_LINE_KP_DEFAULT;
  fsm->line_gains.kd_rpmx10_per_derr    = DRIVE_LINE_KD_DEFAULT;
  fsm->line_gains.loss_timeout_ms       = DRIVE_LINE_LOSS_TIMEOUT_MS;
  fsm->line_gains.max_abs_rpm_x10       = DRIVE_DEFAULT_MAX_RPMX10;
  fsm->line_gains.track_radius_mm       = DRIVE_ITC_TRACK_RADIUS_MM;
  fsm->line_gains.wheel_track_mm        = DRIVE_ITC_WHEEL_TRACK_MM;
  fsm->line_gains.center_right_sign     = +1;
  fsm->line_gains.yaw_kp_rpmx10_per_err = DRIVE_LINE_YAW_KP_DEFAULT;
  fsm->line_gains.narrow_line_max_cnt   = DRIVE_LINE_NARROW_MAX_CNT;
  fsm->line_gains.rear_polarity_sign    = +1;
}

void ITC_AutoSetCruiseSpeed(ITC_Auto_t *fsm, int16_t cruise_rpm_x10)
{
  if ((fsm != NULL) && (cruise_rpm_x10 > 0))
  {
    fsm->base_cruise_rpm_x10 = cruise_rpm_x10;
  }
}

static uint16_t itc_active_dist_mm(const ITC_Auto_t *fsm, const ITC_AutoInput_t *in)
{
  return (fsm->dir_sign >= 0) ? in->front_dist_mm : in->rear_dist_mm;
}

static bool itc_active_dir_blocked(const ITC_Auto_t *fsm, const ITC_AutoInput_t *in, uint16_t thresh_mm)
{
  if (fsm->dir_sign >= 0)
  {
    return in->front_blocked || ((in->front_dist_mm <= thresh_mm));
  }
  return in->rear_blocked || ((in->rear_dist_mm <= thresh_mm));
}

static bool itc_opposite_dir_clear(const ITC_Auto_t *fsm, const ITC_AutoInput_t *in)
{
  if (fsm->dir_sign >= 0)
  {
    /* Opposite direction is REVERSE (rear) */
    return !in->rear_blocked && (in->rear_dist_mm > ITC_TOF_REVERSE_SAFE_MM);
  }
  /* Opposite direction is FORWARD (front) */
  return !in->front_blocked && (in->front_dist_mm > ITC_TOF_REVERSE_SAFE_MM);
}

static int16_t itc_scale_speed_for_tof(int16_t cruise_rpm, uint16_t dist_mm)
{
  if (dist_mm <= ITC_TOF_STOP_MM)
  {
    return 0;
  }
  if (dist_mm >= ITC_TOF_SLOW_MM)
  {
    return cruise_rpm;
  }

  /* Smooth linear slowdown 50%..100% across [ITC_TOF_STOP_MM .. ITC_TOF_SLOW_MM] */
  int32_t span = (int32_t)ITC_TOF_SLOW_MM - (int32_t)ITC_TOF_STOP_MM;
  int32_t rem  = (int32_t)dist_mm - (int32_t)ITC_TOF_STOP_MM;
  int32_t pct  = 50 + ((50 * rem) / span);
  return (int16_t)(((int32_t)cruise_rpm * pct) / 100);
}

static void itc_enter_state(ITC_Auto_t *fsm, ITC_State_t next, uint32_t now_ms)
{
  fsm->state = next;
  fsm->state_enter_ms = now_ms;
  fsm->clear_timing = false;
  if (next == ITC_STATE_IDLE)
  {
    Drive_LineFollowReset(&fsm->line_state);
  }
}

void ITC_AutoCancelCalibration(ITC_Auto_t *fsm, uint32_t now_ms)
{
  if ((fsm != NULL) && (fsm->state == ITC_STATE_CAL_SWEEP))
  {
    itc_enter_state(fsm, ITC_STATE_IDLE, now_ms);
  }
}

static int16_t itc_moderate_speed_for_skew(const ITC_Auto_t *fsm,
                                           const ITC_AutoInput_t *in,
                                           int16_t cruise_rpm)
{
  uint8_t narrow_max = (fsm->line_gains.narrow_line_max_cnt > 0U)
                         ? fsm->line_gains.narrow_line_max_cnt
                         : DRIVE_LINE_NARROW_MAX_CNT;
  if ((fsm->line_gains.yaw_kp_rpmx10_per_err <= 0) ||
      (in->front_mask == 0U) || (in->rear_mask == 0U) ||
      (in->front_count < 1U) || (in->front_count > narrow_max) ||
      (in->rear_count  < 1U) || (in->rear_count  > narrow_max))
  {
    return cruise_rpm;
  }

  int32_t r_sign = (fsm->line_gains.rear_polarity_sign < 0) ? -1 : +1;
  int32_t yaw_diff = (int32_t)in->front_centroid_x10 - (r_sign * (int32_t)in->rear_centroid_x10);
  int32_t abs_yaw = (yaw_diff < 0) ? -yaw_diff : yaw_diff;
  if (abs_yaw <= ITC_SKEW_SLOW_START_X10)
  {
    return cruise_rpm;
  }
  if (abs_yaw >= ITC_SKEW_SLOW_MAX_X10)
  {
    return (int16_t)(((int32_t)cruise_rpm * ITC_SKEW_SLOW_MIN_PCT) / 100);
  }

  int32_t span = ITC_SKEW_SLOW_MAX_X10 - ITC_SKEW_SLOW_START_X10;
  int32_t over = abs_yaw - ITC_SKEW_SLOW_START_X10;
  int32_t pct  = 100 - (((100 - ITC_SKEW_SLOW_MIN_PCT) * over) / span);
  return (int16_t)(((int32_t)cruise_rpm * pct) / 100);
}

static void itc_run_track_step(ITC_Auto_t *fsm, const ITC_AutoInput_t *in, ITC_AutoOutput_t *out)
{
  uint8_t active_cnt = (fsm->dir_sign >= 0) ? in->front_count : in->rear_count;

  bool cooldown_elapsed = ((int32_t)(in->now_ms - fsm->port_cooldown_until_ms) >= 0);
  if ((active_cnt >= CD4067_LOADPORT_WIDE_CNT) && cooldown_elapsed)
  {
    fsm->ports_triggered++;
    out->arm_deg = ITC_SERVO_ARM_TRIGGER_DEG;
    itc_enter_state(fsm, ITC_STATE_LOAD_PORT_HOLD, in->now_ms);
    Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
    return;
  }

  if (in->manual_rf_alive && in->manual_rf_hold_req)
  {
    out->arm_deg = ITC_SERVO_ARM_TRIGGER_DEG;
    Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
    return;
  }

  int16_t mag_rpm = itc_scale_speed_for_tof(fsm->base_cruise_rpm_x10, itc_active_dist_mm(fsm, in));
  mag_rpm = itc_moderate_speed_for_skew(fsm, in, mag_rpm);
  int16_t cmd_rpm = (fsm->dir_sign >= 0) ? mag_rpm : (int16_t)(-mag_rpm);
  Drive_LineFollowDualBarStep(&fsm->line_state,
                               &fsm->line_gains,
                               in->cal_done,
                               cmd_rpm,
                               in->front_mask,
                               in->front_count,
                               in->front_centroid_x10,
                               in->rear_mask,
                               in->rear_count,
                               in->rear_centroid_x10,
                               in->now_ms,
                               false,
                               out->wheel_modes,
                               out->wheel_targets);
}

void ITC_AutoStep(ITC_Auto_t *fsm, const ITC_AutoInput_t *in, ITC_AutoOutput_t *out)
{
  if ((fsm == NULL) || (in == NULL) || (out == NULL))
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->arm_deg = ITC_SERVO_ARM_STOW_DEG;

  if (in->estop_req)
  {
    itc_enter_state(fsm, ITC_STATE_ESTOP, in->now_ms);
    Drive_LineFollowReset(&fsm->line_state);
    Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, true, out->wheel_modes, out->wheel_targets);
    out->state           = fsm->state;
    out->dir_sign        = fsm->dir_sign;
    out->ports_triggered = fsm->ports_triggered;
    out->reversals       = fsm->reversals;
    return;
  }

  if (fsm->state == ITC_STATE_ESTOP)
  {
    if (in->start_toggle_edge)
    {
      itc_enter_state(fsm, ITC_STATE_IDLE, in->now_ms);
    }
    Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
    out->state           = fsm->state;
    out->dir_sign        = fsm->dir_sign;
    out->ports_triggered = fsm->ports_triggered;
    out->reversals       = fsm->reversals;
    return;
  }

  /* 2-point White/Black calibration state takes precedence when entered from IDLE */
  if (fsm->state == ITC_STATE_CAL_SWEEP)
  {
    if (in->cal_toggle_edge)
    {
      out->req_finish_two_point_cal = true;
      itc_enter_state(fsm, ITC_STATE_IDLE, in->now_ms);
    }
    Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
    out->state           = fsm->state;
    out->dir_sign        = fsm->dir_sign;
    out->ports_triggered = fsm->ports_triggered;
    out->reversals       = fsm->reversals;
    return;
  }

  /* SW2 long-press in IDLE triggers 2-point White/Black calibration sweep */
  if ((fsm->state == ITC_STATE_IDLE) && in->cal_toggle_edge)
  {
    out->req_begin_two_point_cal = true;
    itc_enter_state(fsm, ITC_STATE_CAL_SWEEP, in->now_ms);
    Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
    out->state           = fsm->state;
    out->dir_sign        = fsm->dir_sign;
    out->ports_triggered = fsm->ports_triggered;
    out->reversals       = fsm->reversals;
    return;
  }

  /* SW1 start/stop toggle */
  if (in->start_toggle_edge)
  {
    if (fsm->state == ITC_STATE_IDLE)
    {
      if (in->cal_done)
      {
        fsm->dir_sign = +1;
        if (in->imu_ok)
        {
          fsm->exit_target_yaw_deg = in->imu_yaw_deg;
          fsm->has_exit_yaw = true;
        }
        Drive_LineFollowReset(&fsm->line_state);
        if (in->front_mask != 0U)
        {
          itc_enter_state(fsm, ITC_STATE_TRACK_RUN, in->now_ms);
        }
        else
        {
          itc_enter_state(fsm, ITC_STATE_EXIT_START_BOX, in->now_ms);
        }
      }
    }
    else
    {
      itc_enter_state(fsm, ITC_STATE_IDLE, in->now_ms);
    }
  }

  /* Optional manual-robot direction flip trigger when active */
  if ((fsm->state == ITC_STATE_TRACK_RUN) &&
      in->manual_rf_alive &&
      in->manual_rf_reverse_edge &&
      itc_opposite_dir_clear(fsm, in))
  {
    fsm->dir_sign = (int8_t)(-fsm->dir_sign);
    fsm->reversals++;
    Drive_LineFollowReset(&fsm->line_state);
  }

  switch (fsm->state)
  {
    case ITC_STATE_IDLE:
      Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      break;

    case ITC_STATE_EXIT_START_BOX:
      if (itc_active_dir_blocked(fsm, in, ITC_TOF_STOP_MM))
      {
        itc_enter_state(fsm, ITC_STATE_OBSTACLE_STOP, in->now_ms);
        Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      }
      else if (in->front_mask != 0U)
      {
        fsm->dir_sign = +1;
        Drive_LineFollowReset(&fsm->line_state);
        itc_enter_state(fsm, ITC_STATE_TRACK_RUN, in->now_ms);
        itc_run_track_step(fsm, in, out);
      }
      else if ((uint32_t)(in->now_ms - fsm->state_enter_ms) >= ITC_EXIT_BOX_TIMEOUT_MS)
      {
        itc_enter_state(fsm, ITC_STATE_IDLE, in->now_ms);
        Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      }
      else
      {
        int16_t ang = 0;
        if (in->imu_ok && fsm->has_exit_yaw)
        {
          float err = Drive_WrapAngleDeg(fsm->exit_target_yaw_deg - in->imu_yaw_deg);
          int32_t k = (int32_t)(err * (float)ITC_EXIT_HEADING_KP_RPMX10);
          if (k > 600)
          {
            k = 600;
          }
          else if (k < -600)
          {
            k = -600;
          }
          ang = (int16_t)k;
        }
        Drive_DiffCompute(ITC_EXIT_BOX_RPMX10, ang, DRIVE_DEFAULT_MAX_RPMX10,
                          false, out->wheel_modes, out->wheel_targets);
      }
      break;

    case ITC_STATE_TRACK_RUN:
      if (itc_active_dir_blocked(fsm, in, ITC_TOF_STOP_MM))
      {
        Drive_LineFollowReset(&fsm->line_state);
        itc_enter_state(fsm, ITC_STATE_OBSTACLE_STOP, in->now_ms);
        Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      }
      else
      {
        itc_run_track_step(fsm, in, out);
      }
      break;

    case ITC_STATE_LOAD_PORT_HOLD:
      out->arm_deg = ITC_SERVO_ARM_TRIGGER_DEG;
      Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      if ((uint32_t)(in->now_ms - fsm->state_enter_ms) >= ITC_LOADPORT_DWELL_MS)
      {
        if (!(in->manual_rf_alive && in->manual_rf_hold_req))
        {
          out->arm_deg = ITC_SERVO_ARM_STOW_DEG;
          fsm->port_cooldown_until_ms = in->now_ms + ITC_LOADPORT_COOLDOWN_MS;
          Drive_LineFollowReset(&fsm->line_state);
          itc_enter_state(fsm, ITC_STATE_TRACK_RUN, in->now_ms);
          itc_run_track_step(fsm, in, out);
        }
      }
      break;

    case ITC_STATE_OBSTACLE_STOP:
    {
      Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      bool still_blocked = itc_active_dir_blocked(fsm, in, ITC_TOF_RESUME_MM);

      if (!still_blocked)
      {
        if (!fsm->clear_timing)
        {
          fsm->clear_timing = true;
          fsm->clear_since_ms = in->now_ms;
        }
        else if ((uint32_t)(in->now_ms - fsm->clear_since_ms) >= ITC_OBSTACLE_CLEAR_CONFIRM_MS)
        {
          Drive_LineFollowReset(&fsm->line_state);
          itc_enter_state(fsm, ITC_STATE_TRACK_RUN, in->now_ms);
          itc_run_track_step(fsm, in, out);
        }
      }
      else
      {
        fsm->clear_timing = false;
        if (((uint32_t)(in->now_ms - fsm->state_enter_ms) >= ITC_OBSTACLE_REVERSE_WAIT_MS) &&
            itc_opposite_dir_clear(fsm, in))
        {
          fsm->dir_sign = (int8_t)(-fsm->dir_sign);
          fsm->reversals++;
          Drive_LineFollowReset(&fsm->line_state);
          itc_enter_state(fsm, ITC_STATE_TRACK_RUN, in->now_ms);
          itc_run_track_step(fsm, in, out);
        }
      }
      break;
    }

    default:
      Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, false, out->wheel_modes, out->wheel_targets);
      break;
  }

  out->state           = fsm->state;
  out->dir_sign        = fsm->dir_sign;
  out->ports_triggered = fsm->ports_triggered;
  out->reversals       = fsm->reversals;
}
