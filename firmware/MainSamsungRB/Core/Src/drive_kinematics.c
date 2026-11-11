/**
  ******************************************************************************
  * @file    drive_kinematics.c
  * @brief   Portable 4-wheel omni & 2-wheel differential kinematics + line PD.
  ******************************************************************************
  */

#include "drive_kinematics.h"
#include <math.h>
#include <stddef.h>

#define DRIVE_DEG_TO_RAD_F  0.0174532925f
#define DRIVE_PI4_F         0.7853981634f
#define DRIVE_PI34_F        2.3561944902f

static float drive_clampf(float val, float lo, float hi)
{
  if (val < lo)
  {
    return lo;
  }
  if (val > hi)
  {
    return hi;
  }
  return val;
}

static int32_t drive_roundf_i32(float val)
{
  return (val >= 0.0f) ? (int32_t)(val + 0.5f) : (int32_t)(val - 0.5f);
}

static int32_t drive_abs_i32(int32_t val)
{
  return (val < 0) ? -val : val;
}

float Drive_SlewRamp(float current, float target, float accel_step, float decel_step)
{
  if (accel_step <= 0.0f)
  {
    accel_step = 1.0f;
  }
  if (decel_step <= 0.0f)
  {
    decel_step = accel_step;
  }

  float diff = target - current;
  if (fabsf(diff) < 1e-4f)
  {
    return target;
  }

  bool decelerating = ((current * target) < 0.0f) || (fabsf(target) < fabsf(current));
  float step = decelerating ? decel_step : accel_step;

  if (diff > step)
  {
    return current + step;
  }
  if (diff < -step)
  {
    return current - step;
  }
  return target;
}

void Drive_OmniReset(Drive_OmniState_t *state)
{
  if (state == NULL)
  {
    return;
  }
  state->vx_pct = 0.0f;
  state->vy_pct = 0.0f;
  state->omega_pct = 0.0f;
}

void Drive_OmniComputeFromPs2(Drive_OmniState_t *state,
                              const Drive_Ps2Input_t *in,
                              bool estop,
                              float robot_yaw_deg,
                              bool field_centric,
                              int16_t max_abs_rpm_x10,
                              uint8_t out_modes[DRIVE_OMNI_WHEEL_COUNT],
                              int16_t out_targets[DRIVE_OMNI_WHEEL_COUNT])
{
  static const float kWheelAngles[DRIVE_OMNI_WHEEL_COUNT] =
  {
    -DRIVE_PI4_F,
    +DRIVE_PI4_F,
    +DRIVE_PI34_F,
    -DRIVE_PI34_F
  };

  if ((out_modes == NULL) || (out_targets == NULL))
  {
    return;
  }

  if (max_abs_rpm_x10 <= 0)
  {
    max_abs_rpm_x10 = DRIVE_DEFAULT_MAX_RPMX10;
  }

  if (estop)
  {
    Drive_OmniReset(state);
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++)
    {
      out_modes[i] = DRIVE_WHEEL_MODE_DISABLED;
      out_targets[i] = 0;
    }
    return;
  }

  if ((state == NULL) || (in == NULL) || !in->valid)
  {
    Drive_OmniReset(state);
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++)
    {
      out_modes[i] = DRIVE_WHEEL_MODE_BRAKE;
      out_targets[i] = 0;
    }
    return;
  }

  /* PS2_STM32.c computes joy_sum[0] = (map(LY) + map(RY)) - 4 in uint8_t:
   * pushing both sticks full forward yields (0 + 0) - 4 -> 252 underflow. */
  float raw_js0 = (in->joy_sum[0] >= 240U) ? 0.0f : (float)in->joy_sum[0];
  float raw_js1 = (float)in->joy_sum[1];
  raw_js0 = drive_clampf(raw_js0, 0.0f, 200.0f);
  raw_js1 = drive_clampf(raw_js1, 0.0f, 200.0f);

  float target_y = raw_js0 - 100.0f;
  float target_x = 100.0f - raw_js1;

  if (fabsf(target_y) <= DRIVE_PS2_DEADBAND)
  {
    target_y = 0.0f;
  }
  if (fabsf(target_x) <= DRIVE_PS2_DEADBAND)
  {
    target_x = 0.0f;
  }

  /* Digital D-pad fallback (active in both digital 0x41 and centered analog modes) */
  if (target_y == 0.0f)
  {
    bool dpad_up   = (in->ps2_btn[0] & 0x10U) != 0U;
    bool dpad_down = (in->ps2_btn[0] & 0x40U) != 0U;
    if (dpad_up && !dpad_down)
    {
      target_y = -DRIVE_OMNI_DPAD_PCT;
    }
    else if (dpad_down && !dpad_up)
    {
      target_y = +DRIVE_OMNI_DPAD_PCT;
    }
  }
  if (target_x == 0.0f)
  {
    bool dpad_left  = (in->ps2_btn[0] & 0x80U) != 0U;
    bool dpad_right = (in->ps2_btn[0] & 0x20U) != 0U;
    if (dpad_left && !dpad_right)
    {
      target_x = +DRIVE_OMNI_DPAD_PCT;
    }
    else if (dpad_right && !dpad_left)
    {
      target_x = -DRIVE_OMNI_DPAD_PCT;
    }
  }

  bool l1 = (in->ps2_btn[1] & 0x04U) != 0U;
  bool r1 = (in->ps2_btn[1] & 0x08U) != 0U;
  float target_w = 0.0f;
  if (l1 && !r1)
  {
    target_w = -DRIVE_OMNI_OMEGA_MAX;
  }
  else if (r1 && !l1)
  {
    target_w = +DRIVE_OMNI_OMEGA_MAX;
  }

  state->vx_pct    = Drive_SlewRamp(state->vx_pct,    target_x, DRIVE_OMNI_SLEW_ACCEL, DRIVE_OMNI_SLEW_DECEL);
  state->vy_pct    = Drive_SlewRamp(state->vy_pct,    target_y, DRIVE_OMNI_SLEW_ACCEL, DRIVE_OMNI_SLEW_DECEL);
  state->omega_pct = Drive_SlewRamp(state->omega_pct, target_w, DRIVE_OMNI_SLEW_ACCEL, DRIVE_OMNI_SLEW_DECEL);

  float theta_rad = 0.0f;
  if (field_centric && isfinite(robot_yaw_deg))
  {
    float wrapped_deg = fmodf(robot_yaw_deg, 360.0f);
    theta_rad = wrapped_deg * DRIVE_DEG_TO_RAD_F;
  }

  float raw_wheel[DRIVE_OMNI_WHEEL_COUNT];
  float max_abs = 0.0f;
  for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++)
  {
    float a = theta_rad + kWheelAngles[i];
    raw_wheel[i] = state->vx_pct * sinf(a) + state->vy_pct * cosf(a) + state->omega_pct;
    float mag = fabsf(raw_wheel[i]);
    if (mag > max_abs)
    {
      max_abs = mag;
    }
  }

  float norm = (max_abs > 100.0f) ? (100.0f / max_abs) : 1.0f;
  bool boost_r2 = (in->ps2_btn[1] & 0x02U) != 0U;
  float v_scale = boost_r2 ? DRIVE_OMNI_BOOST_RPMX10 : DRIVE_OMNI_NORM_RPMX10;
  if (v_scale > (float)max_abs_rpm_x10)
  {
    v_scale = (float)max_abs_rpm_x10;
  }

  bool any_nonzero = false;
  for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++)
  {
    float pct = raw_wheel[i] * norm;
    int32_t rpm_x10 = drive_roundf_i32((pct * v_scale) * 0.01f);
    if (rpm_x10 > (int32_t)max_abs_rpm_x10)
    {
      rpm_x10 = (int32_t)max_abs_rpm_x10;
    }
    else if (rpm_x10 < -(int32_t)max_abs_rpm_x10)
    {
      rpm_x10 = -(int32_t)max_abs_rpm_x10;
    }
    out_targets[i] = (int16_t)rpm_x10;
    if (out_targets[i] != 0)
    {
      any_nonzero = true;
    }
  }

  uint8_t mode = any_nonzero ? DRIVE_WHEEL_MODE_VELOCITY : DRIVE_WHEEL_MODE_BRAKE;
  for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++)
  {
    out_modes[i] = mode;
  }
}

void Drive_DiffCompute(int16_t linear_rpm_x10,
                       int16_t angular_rpm_x10,
                       int16_t max_abs_rpm_x10,
                       bool estop,
                       uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                       int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT])
{
  if ((out_modes == NULL) || (out_targets == NULL))
  {
    return;
  }

  if (max_abs_rpm_x10 <= 0)
  {
    max_abs_rpm_x10 = DRIVE_DEFAULT_MAX_RPMX10;
  }

  if (estop)
  {
    out_modes[0] = DRIVE_WHEEL_MODE_DISABLED;
    out_modes[1] = DRIVE_WHEEL_MODE_DISABLED;
    out_targets[0] = 0;
    out_targets[1] = 0;
    return;
  }

  int32_t left  = (int32_t)linear_rpm_x10 - (int32_t)angular_rpm_x10;
  int32_t right = (int32_t)linear_rpm_x10 + (int32_t)angular_rpm_x10;

  int32_t abs_l = drive_abs_i32(left);
  int32_t abs_r = drive_abs_i32(right);
  int32_t max_mag = (abs_l > abs_r) ? abs_l : abs_r;

  if (max_mag > (int32_t)max_abs_rpm_x10)
  {
    left  = (left  * (int32_t)max_abs_rpm_x10) / max_mag;
    right = (right * (int32_t)max_abs_rpm_x10) / max_mag;
  }

  out_targets[0] = (int16_t)left;
  out_targets[1] = (int16_t)right;

  if ((left == 0) && (right == 0))
  {
    out_modes[0] = DRIVE_WHEEL_MODE_BRAKE;
    out_modes[1] = DRIVE_WHEEL_MODE_BRAKE;
  }
  else
  {
    out_modes[0] = DRIVE_WHEEL_MODE_VELOCITY;
    out_modes[1] = DRIVE_WHEEL_MODE_VELOCITY;
  }
}

void Drive_LineFollowReset(Drive_LineFollowState_t *st)
{
  if (st == NULL)
  {
    return;
  }
  st->prev_error = 0;
  st->has_prev_error = false;
  st->last_seen_ms = 0U;
  st->has_seen_line = false;
}

void Drive_LineFollowStep(Drive_LineFollowState_t *st,
                          const Drive_LineGains_t *gains,
                          bool cal_done,
                          int16_t base_rpm_x10,
                          uint8_t active_mask,
                          int16_t line_error,
                          uint32_t now_ms,
                          bool estop,
                          uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                          int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT])
{
  if ((out_modes == NULL) || (out_targets == NULL))
  {
    return;
  }

  int16_t kp = (gains != NULL) ? gains->kp_rpmx10_per_err  : DRIVE_LINE_KP_DEFAULT;
  int16_t kd = (gains != NULL) ? gains->kd_rpmx10_per_derr : DRIVE_LINE_KD_DEFAULT;
  uint32_t timeout_ms = ((gains != NULL) && (gains->loss_timeout_ms > 0U))
                          ? gains->loss_timeout_ms
                          : DRIVE_LINE_LOSS_TIMEOUT_MS;
  int16_t max_rpm_x10 = ((gains != NULL) && (gains->max_abs_rpm_x10 > 0))
                          ? gains->max_abs_rpm_x10
                          : DRIVE_DEFAULT_MAX_RPMX10;

  if (estop)
  {
    Drive_LineFollowReset(st);
    Drive_DiffCompute(0, 0, max_rpm_x10, true, out_modes, out_targets);
    return;
  }

  if ((st == NULL) || !cal_done || (base_rpm_x10 == 0))
  {
    Drive_LineFollowReset(st);
    Drive_DiffCompute(0, 0, max_rpm_x10, false, out_modes, out_targets);
    return;
  }

  /* Forward uses Front bar: error > 0 -> steer Right (negative yaw).
   * Reverse uses Rear bar (same L-to-R indexing): error > 0 -> swing tail Right (positive yaw). */
  int32_t dir_sign = (base_rpm_x10 >= 0) ? -1 : +1;

  if (active_mask != 0U)
  {
    int32_t derr = st->has_prev_error ? ((int32_t)line_error - (int32_t)st->prev_error) : 0;
    if (derr > 40)
    {
      derr = 40;
    }
    else if (derr < -40)
    {
      derr = -40;
    }
    st->prev_error = line_error;
    st->has_prev_error = true;
    st->last_seen_ms = now_ms;
    st->has_seen_line = true;

    int32_t angular = dir_sign * ((int32_t)kp * (int32_t)line_error + (int32_t)kd * derr);
    if (angular > (int32_t)max_rpm_x10)
    {
      angular = (int32_t)max_rpm_x10;
    }
    else if (angular < -(int32_t)max_rpm_x10)
    {
      angular = -(int32_t)max_rpm_x10;
    }

    Drive_DiffCompute(base_rpm_x10, (int16_t)angular, max_rpm_x10, false, out_modes, out_targets);
    return;
  }

  if (st->has_seen_line && ((uint32_t)(now_ms - st->last_seen_ms) <= timeout_ms))
  {
    int32_t angular = dir_sign * ((int32_t)kp * (int32_t)line_error);
    if (angular > (int32_t)max_rpm_x10)
    {
      angular = (int32_t)max_rpm_x10;
    }
    else if (angular < -(int32_t)max_rpm_x10)
    {
      angular = -(int32_t)max_rpm_x10;
    }
    Drive_DiffCompute((int16_t)(base_rpm_x10 / 2), (int16_t)angular, max_rpm_x10, false, out_modes, out_targets);
    return;
  }

  Drive_LineFollowReset(st);
  Drive_DiffCompute(0, 0, max_rpm_x10, false, out_modes, out_targets);
}

