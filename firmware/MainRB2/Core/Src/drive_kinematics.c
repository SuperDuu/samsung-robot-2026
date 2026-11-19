/**
 ******************************************************************************
 * @file    drive_kinematics.c
 * @brief   Portable 4-wheel omni (legacy + custom modified field-centric) &
 *          2-wheel differential kinematics + curvature-FF continuous line PD.
 ******************************************************************************
 */

#include "drive_kinematics.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRIVE_DEG_TO_RAD_F 0.0174532925f
#define DRIVE_PI4_F 0.7853981634f
#define DRIVE_PI34_F 2.3561944902f

static const uint8_t kGearSpeedPct[DRIVE_OMNI_GEAR_COUNT] = {33U, 55U, 75U,
                                                             100U};

static const float kWheelAngles[DRIVE_OMNI_WHEEL_COUNT] = {
    -DRIVE_PI4_F, +DRIVE_PI4_F, +DRIVE_PI34_F, -DRIVE_PI34_F};

static float drive_clampf(float val, float lo, float hi) {
  if (val < lo) {
    return lo;
  }
  if (val > hi) {
    return hi;
  }
  return val;
}

static int32_t drive_roundf_i32(float val) {
  return (val >= 0.0f) ? (int32_t)(val + 0.5f) : (int32_t)(val - 0.5f);
}

static int32_t drive_abs_i32(int32_t val) { return (val < 0) ? -val : val; }

float Drive_WrapAngleDeg(float deg) {
  if (!isfinite(deg)) {
    return 0.0f;
  }
  float w = fmodf(deg, 360.0f);
  if (w > 180.0f) {
    w -= 360.0f;
  } else if (w <= -180.0f) {
    w += 360.0f;
  }
  return w;
}

float Drive_SlewRamp(float current, float target, float accel_step,
                     float decel_step) {
  if (accel_step <= 0.0f) {
    accel_step = 1.0f;
  }
  if (decel_step <= 0.0f) {
    decel_step = accel_step;
  }

  float diff = target - current;
  if (fabsf(diff) < 1e-4f) {
    return target;
  }

  bool decelerating =
      ((current * target) < 0.0f) || (fabsf(target) < fabsf(current));
  float step = decelerating ? decel_step : accel_step;

  if (diff > step) {
    return current + step;
  }
  if (diff < -step) {
    return current - step;
  }
  return target;
}

static void drive_extract_ps2_xy(const Drive_Ps2Input_t *in, float *out_x,
                                 float *out_y) {
  /* PS2_STM32.c computes joy_sum[0] = (map(LY) + map(RY)) - 4 in uint8_t:
   * pushing both sticks full forward yields (0 + 0) - 4 -> 252 underflow. */
  float raw_js0 = (in->joy_sum[0] >= 240U) ? 0.0f : (float)in->joy_sum[0];
  float raw_js1 = (float)in->joy_sum[1];
  raw_js0 = drive_clampf(raw_js0, 0.0f, 200.0f);
  raw_js1 = drive_clampf(raw_js1, 0.0f, 200.0f);

  float target_y = raw_js0 - 100.0f;
  float target_x = 100.0f - raw_js1;

  if (fabsf(target_y) <= DRIVE_PS2_DEADBAND) {
    target_y = 0.0f;
  }
  if (fabsf(target_x) <= DRIVE_PS2_DEADBAND) {
    target_x = 0.0f;
  }

  /* Digital D-pad fallback (active in both digital 0x41 and centered analog
   * modes; codes >= 200 are reserved for continuous 360 servos) */
  if ((target_y == 0.0f) && (in->ps2_btn[0] < 200U)) {
    bool dpad_up = (in->ps2_btn[0] & DRIVE_PS2_B0_UP) != 0U;
    bool dpad_down = (in->ps2_btn[0] & DRIVE_PS2_B0_DOWN) != 0U;
    if (dpad_up && !dpad_down) {
      target_y = -DRIVE_OMNI_DPAD_PCT;
    } else if (dpad_down && !dpad_up) {
      target_y = +DRIVE_OMNI_DPAD_PCT;
    }
  }
  if ((target_x == 0.0f) && (in->ps2_btn[0] < 200U)) {
    bool dpad_left = (in->ps2_btn[0] & DRIVE_PS2_B0_LEFT) != 0U;
    bool dpad_right = (in->ps2_btn[0] & DRIVE_PS2_B0_RIGHT) != 0U;
    if (dpad_left && !dpad_right) {
      target_x = -DRIVE_OMNI_DPAD_PCT;
    } else if (dpad_right && !dpad_left) {
      target_x = +DRIVE_OMNI_DPAD_PCT;
    }
  }

  *out_x = target_x;
  *out_y = target_y;
}

static void drive_mix_omni_wheels(float vx_pct, float vy_pct, float omega_pct,
                                  float theta_rad, float v_scale, float w_scale,
                                  int16_t max_abs_rpm_x10,
                                  uint8_t out_modes[DRIVE_OMNI_WHEEL_COUNT],
                                  int16_t out_targets[DRIVE_OMNI_WHEEL_COUNT]) {
  if (v_scale > (float)max_abs_rpm_x10) {
    v_scale = (float)max_abs_rpm_x10;
  }
  if (w_scale > (float)max_abs_rpm_x10) {
    w_scale = (float)max_abs_rpm_x10;
  }

  float raw_rpm_x10[DRIVE_OMNI_WHEEL_COUNT];
  float max_abs = 0.0f;
  for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
    float a = theta_rad + kWheelAngles[i];
    float trans_rpm_x10 = (vx_pct * sinf(a) + vy_pct * cosf(a)) * v_scale * 0.01f;
    float rot_rpm_x10 = omega_pct * w_scale * 0.01f;
    raw_rpm_x10[i] = trans_rpm_x10 + rot_rpm_x10;
    float mag = fabsf(raw_rpm_x10[i]);
    if (mag > max_abs) {
      max_abs = mag;
    }
  }

  float max_limit = (float)max_abs_rpm_x10;
  float norm = (max_abs > max_limit) ? (max_limit / max_abs) : 1.0f;

  bool any_nonzero = false;
  for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
    int32_t rpm_x10 = drive_roundf_i32(raw_rpm_x10[i] * norm);
    if (rpm_x10 > (int32_t)max_abs_rpm_x10) {
      rpm_x10 = (int32_t)max_abs_rpm_x10;
    } else if (rpm_x10 < -(int32_t)max_abs_rpm_x10) {
      rpm_x10 = -(int32_t)max_abs_rpm_x10;
    }
    out_targets[i] = (int16_t)rpm_x10;
    if (out_targets[i] != 0) {
      any_nonzero = true;
    }
  }

  uint8_t mode =
      any_nonzero ? DRIVE_WHEEL_MODE_VELOCITY : DRIVE_WHEEL_MODE_BRAKE;
  for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
    out_modes[i] = mode;
  }
}

void Drive_OmniReset(Drive_OmniState_t *state) {
  if (state == NULL) {
    return;
  }
  state->vx_pct = 0.0f;
  state->vy_pct = 0.0f;
  state->omega_pct = 0.0f;
}

void Drive_OmniComputeFromPs2(Drive_OmniState_t *state,
                              const Drive_Ps2Input_t *in, bool estop,
                              float robot_yaw_deg, bool field_centric,
                              int16_t max_abs_rpm_x10,
                              uint8_t out_modes[DRIVE_OMNI_WHEEL_COUNT],
                              int16_t out_targets[DRIVE_OMNI_WHEEL_COUNT]) {
  if ((out_modes == NULL) || (out_targets == NULL)) {
    return;
  }

  if (max_abs_rpm_x10 <= 0) {
    max_abs_rpm_x10 = DRIVE_DEFAULT_MAX_RPMX10;
  }

  if (estop) {
    Drive_OmniReset(state);
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
      out_modes[i] = DRIVE_WHEEL_MODE_DISABLED;
      out_targets[i] = 0;
    }
    return;
  }

  if ((state == NULL) || (in == NULL) || !in->valid) {
    Drive_OmniReset(state);
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
      out_modes[i] = DRIVE_WHEEL_MODE_BRAKE;
      out_targets[i] = 0;
    }
    return;
  }

  float target_x = 0.0f;
  float target_y = 0.0f;
  drive_extract_ps2_xy(in, &target_x, &target_y);

  bool boost_3x = (in->ps2_btn[1] & DRIVE_PS2_B1_L1) != 0U; /* tx_buf[3] = 4: 3x speed boost */
  bool l2 = (in->ps2_btn[1] & DRIVE_PS2_B1_L2) != 0U;
  bool r2 = (in->ps2_btn[1] & DRIVE_PS2_B1_R2) != 0U;
  /* Note: R1 is dedicated to gripper clamp toggle (180 deg servo), NOT robot rotation */
  float target_w = 0.0f;
  if (l2 && !r2) {
    target_w = +DRIVE_OMNI_TRIM_ROT_PCT; /* 20 RPM rotate left */
  } else if (r2 && !l2) {
    target_w = -DRIVE_OMNI_TRIM_ROT_PCT; /* 20 RPM rotate right */
  }

  state->vx_pct = Drive_SlewRamp(state->vx_pct, target_x, DRIVE_OMNI_SLEW_ACCEL,
                                 DRIVE_OMNI_SLEW_DECEL);
  state->vy_pct = Drive_SlewRamp(state->vy_pct, target_y, DRIVE_OMNI_SLEW_ACCEL,
                                 DRIVE_OMNI_SLEW_DECEL);
  state->omega_pct = Drive_SlewRamp(
      state->omega_pct, target_w, DRIVE_OMNI_SLEW_ACCEL, DRIVE_OMNI_SLEW_DECEL);

  float theta_rad = 0.0f;
  if (field_centric && isfinite(robot_yaw_deg)) {
    float wrapped_deg = fmodf(robot_yaw_deg, 360.0f);
    theta_rad = wrapped_deg * DRIVE_DEG_TO_RAD_F;
  }

  float v_scale = ((float)max_abs_rpm_x10 * 33.0f) * 0.01f;
  if (boost_3x) {
    v_scale = fminf(v_scale * 3.0f, (float)max_abs_rpm_x10);
  }
  drive_mix_omni_wheels(state->vx_pct, state->vy_pct, state->omega_pct,
                        theta_rad, v_scale, DRIVE_OMNI_OMEGA_SCALE_RPMX10,
                        max_abs_rpm_x10, out_modes, out_targets);
}

void Drive_OmniCtrlInit(Drive_OmniCtrl_t *ctrl, bool blue_side) {
  if (ctrl == NULL) {
    return;
  }
  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->field_centric = true;
  ctrl->heading_hold = true;
  ctrl->blue_side = blue_side;
  ctrl->gear = DRIVE_OMNI_DEFAULT_GEAR;
  ctrl->active_speed_pct = kGearSpeedPct[DRIVE_OMNI_DEFAULT_GEAR];
  ctrl->heading_kp = DRIVE_OMNI_HEADING_KP;
  ctrl->heading_kd = DRIVE_OMNI_HEADING_KD;
  ctrl->heading_deadband_deg = DRIVE_OMNI_HEADING_DEADBAND_DEG;
}

void Drive_OmniCtrlSetBlueSide(Drive_OmniCtrl_t *ctrl, bool blue_side) {
  if (ctrl == NULL) {
    return;
  }
  ctrl->blue_side = blue_side;
}

void Drive_OmniCtrlZeroHeading(Drive_OmniCtrl_t *ctrl) {
  if (ctrl == NULL) {
    return;
  }
  ctrl->target_heading_deg = 0.0f;
  ctrl->slew_heading_deg = 0.0f;
  ctrl->field_axis_offset_deg = 0.0f;
  ctrl->heading_trim_deg = 0.0f;
  ctrl->diagonal_45 = false;
  ctrl->heading_initialized = true;
  ctrl->omega_fb_pct = 0.0f;
  ctrl->prev_yaw_deg = 0.0f;
  ctrl->yaw_rate_dps = 0.0f;
  ctrl->has_prev_yaw = false;
  ctrl->ramp_omega_deg = 0.0f;
  ctrl->rot_ramp_ticks = 0U;
  ctrl->heading_settled = false;
}

void Drive_OmniCtrlStep(Drive_OmniCtrl_t *ctrl, const Drive_Ps2Input_t *in,
                        bool estop, float robot_yaw_deg, bool imu_healthy,
                        int16_t max_abs_rpm_x10,
                        uint8_t out_modes[DRIVE_OMNI_WHEEL_COUNT],
                        int16_t out_targets[DRIVE_OMNI_WHEEL_COUNT]) {
  if ((out_modes == NULL) || (out_targets == NULL)) {
    return;
  }
  if (max_abs_rpm_x10 <= 0) {
    max_abs_rpm_x10 = DRIVE_DEFAULT_MAX_RPMX10;
  }

  if (estop || (ctrl == NULL)) {
    if (ctrl != NULL) {
      Drive_OmniReset(&ctrl->slew);
      ctrl->omega_fb_pct = 0.0f;
    }
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
      out_modes[i] = DRIVE_WHEEL_MODE_DISABLED;
      out_targets[i] = 0;
    }
    return;
  }

  if ((in == NULL) || !in->valid) {
    Drive_OmniReset(&ctrl->slew);
    ctrl->omega_fb_pct = 0.0f;
    ctrl->prev_btn0 = 0U;
    ctrl->prev_btn1 = 0U;
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
      out_modes[i] = DRIVE_WHEEL_MODE_BRAKE;
      out_targets[i] = 0;
    }
    return;
  }

  uint8_t raw_b0 = in->ps2_btn[0];
  uint8_t b0 = (raw_b0 < 200U) ? raw_b0 : 0U; /* Codes >= 200 are 360-servo commands */
  uint8_t b1 = in->ps2_btn[1];
  uint8_t rise0 = (uint8_t)(b0 & (uint8_t)(~ctrl->prev_btn0));
  uint8_t rise1 = (uint8_t)(b1 & (uint8_t)(~ctrl->prev_btn1));
  uint8_t fell1 = (uint8_t)(ctrl->prev_btn1 & (uint8_t)(~b1));
  bool trim_release = ((fell1 & (DRIVE_PS2_B1_L2 | DRIVE_PS2_B1_R2)) != 0U);
  ctrl->prev_btn0 = b0;
  ctrl->prev_btn1 = b1;

  bool imu_ok = imu_healthy && isfinite(robot_yaw_deg);
  if (imu_ok && !ctrl->heading_initialized) {
    ctrl->target_heading_deg = Drive_WrapAngleDeg(robot_yaw_deg);
    ctrl->slew_heading_deg = ctrl->target_heading_deg;
    ctrl->heading_initialized = true;
  }

  /* START toggles custom modified field-centric + heading hold */
  if ((rise0 & DRIVE_PS2_B0_START) != 0U) {
    ctrl->field_centric = !ctrl->field_centric;
    ctrl->heading_hold = ctrl->field_centric;
    if (ctrl->field_centric && imu_ok) {
      ctrl->target_heading_deg =
          Drive_WrapAngleDeg(robot_yaw_deg + ctrl->heading_trim_deg);
      ctrl->slew_heading_deg = ctrl->target_heading_deg;
    }
  }

  /* L2 = fine-trim rotate LEFT, R2 = fine-trim rotate RIGHT at 20 RPM */
  bool l2 = (b1 & DRIVE_PS2_B1_L2) != 0U;
  bool r2 = (b1 & DRIVE_PS2_B1_R2) != 0U;
  bool trim_rot = (l2 != r2);

  /* D-pad micro-adjustment (16, 32, 64, 128): lock into 20% crawl speed */
  bool dpad_active = (b0 & (DRIVE_PS2_B0_UP | DRIVE_PS2_B0_RIGHT |
                            DRIVE_PS2_B0_DOWN | DRIVE_PS2_B0_LEFT)) != 0U;
  ctrl->active_speed_pct =
      dpad_active ? DRIVE_OMNI_CRAWL_PCT : kGearSpeedPct[ctrl->gear];

  /* Cardinal + diagonal field-centric snaps:
   * When pure_field_centric is true, keep field_axis_offset_deg = 0.0f so
   * joystick UP always moves toward the true field-forward direction regardless
   * of robot nose heading. */
  bool cardinal_or_diag = false;
  if ((rise1 & DRIVE_PS2_B1_TRIANGLE) != 0U) {
    ctrl->target_heading_deg = 0.0f;
    ctrl->field_axis_offset_deg = 0.0f;
    ctrl->diagonal_45 = false;
    cardinal_or_diag = true;
  } else if ((rise1 & DRIVE_PS2_B1_CIRCLE) != 0U) {
    /* Button 32 (Circle / Right): snap to -90 deg (quay phai) */
    ctrl->target_heading_deg = -90.0f;
    ctrl->field_axis_offset_deg =
        ctrl->pure_field_centric ? 0.0f : (ctrl->blue_side ? 90.0f : -90.0f);
    ctrl->diagonal_45 = false;
    cardinal_or_diag = true;
  } else if ((rise1 & DRIVE_PS2_B1_CROSS) != 0U) {
    ctrl->target_heading_deg = 180.0f;
    ctrl->field_axis_offset_deg = ctrl->pure_field_centric ? 0.0f : 180.0f;
    ctrl->diagonal_45 = false;
    cardinal_or_diag = true;
  } else if ((rise1 & DRIVE_PS2_B1_SQUARE) != 0U) {
    /* Button 128 (Square / Left): snap to +90 deg (quay trai) */
    ctrl->target_heading_deg = 90.0f;
    ctrl->field_axis_offset_deg =
        ctrl->pure_field_centric ? 0.0f : (ctrl->blue_side ? 90.0f : -90.0f);
    ctrl->diagonal_45 = false;
    cardinal_or_diag = true;
  } else if ((rise0 & DRIVE_PS2_B0_R3) != 0U) {
    /* Toggle +/-45 deg diagonal offset without losing cardinal orientation */
    ctrl->diagonal_45 = !ctrl->diagonal_45;
    ctrl->target_heading_deg = Drive_WrapAngleDeg(
        ctrl->target_heading_deg + (ctrl->diagonal_45 ? -45.0f : 45.0f));
    cardinal_or_diag = true;
  }

  float target_x = 0.0f;
  float target_y = 0.0f;
  drive_extract_ps2_xy(in, &target_x, &target_y);

  bool select_held = (b0 & DRIVE_PS2_B0_SELECT) != 0U;
  if (select_held) {
    float trim_sign = ctrl->heading_hold_invert ? -1.0f : 1.0f;
    if ((rise1 & DRIVE_PS2_B1_L1) != 0U) {
      ctrl->heading_trim_deg =
          Drive_WrapAngleDeg(ctrl->heading_trim_deg +
                             trim_sign * DRIVE_OMNI_HEADING_TRIM_STEP_DEG);
      cardinal_or_diag = true;
    }
    if ((rise1 & DRIVE_PS2_B1_R1) != 0U) {
      ctrl->heading_trim_deg =
          Drive_WrapAngleDeg(ctrl->heading_trim_deg -
                             trim_sign * DRIVE_OMNI_HEADING_TRIM_STEP_DEG);
      cardinal_or_diag = true;
    }
  }

  bool boost_3x = (b1 & DRIVE_PS2_B1_L1) != 0U; /* tx_buf[3] = 4: 3x speed boost */
  bool l1 = false; /* L1 is dedicated to 3x turbo boost, NOT left rotation */
  bool r1 = false; /* R1 is dedicated to gripper clamp toggle (180 deg servo), NOT right rotation */
  if (cardinal_or_diag || trim_rot || (!select_held && (l1 != r1))) {
    ctrl->rot_ramp_ticks = DRIVE_OMNI_HEADING_RAMP_TICKS;
    ctrl->heading_settled = false;
  }
  float yaw_meas_deg = imu_ok ? (robot_yaw_deg + ctrl->heading_trim_deg) : 0.0f;
  if (trim_release && imu_ok) {
    ctrl->target_heading_deg = yaw_meas_deg;
    ctrl->slew_heading_deg = yaw_meas_deg;
    ctrl->rot_ramp_ticks = DRIVE_OMNI_HEADING_RAMP_TICKS;
    ctrl->heading_settled = false;
  }
  float target_w = 0.0f;

  /* Update filtered yaw angular velocity (deg/s) at 100 Hz */
  float yaw_delta = 0.0f;
  if (imu_ok) {
    if (ctrl->has_prev_yaw) {
      yaw_delta = Drive_WrapAngleDeg(yaw_meas_deg - ctrl->prev_yaw_deg);
    } else {
      ctrl->has_prev_yaw = true;
    }
    ctrl->prev_yaw_deg = yaw_meas_deg;
  } else {
    ctrl->has_prev_yaw = false;
  }
  float raw_dps = yaw_delta * 100.0f;
  ctrl->yaw_rate_dps =
      ctrl->yaw_rate_dps + 0.20f * (raw_dps - ctrl->yaw_rate_dps);

  float rot_dir = ctrl->rot_btn_invert ? -1.0f : 1.0f;
  if (trim_rot) {
    /* Fine yaw trim: L2 rotates LEFT (-rot_dir * 10%), R2 rotates RIGHT (+rot_dir * 10%) at ~20 RPM */
    target_w = l2 ? (-rot_dir * DRIVE_OMNI_TRIM_ROT_PCT)
                  : (+rot_dir * DRIVE_OMNI_TRIM_ROT_PCT);
    ctrl->target_heading_deg = yaw_meas_deg;
    ctrl->slew_heading_deg = yaw_meas_deg;
    ctrl->heading_settled = false;
    ctrl->rot_ramp_ticks = DRIVE_OMNI_HEADING_RAMP_TICKS;
    ctrl->omega_fb_pct = target_w;
  } else if (ctrl->field_centric && ctrl->heading_hold && imu_ok) {
    if (!select_held && l1 && !r1) {
      ctrl->target_heading_deg = Drive_WrapAngleDeg(
          ctrl->target_heading_deg - rot_dir * DRIVE_OMNI_ROT_STEP_DEG);
    } else if (!select_held && r1 && !l1) {
      ctrl->target_heading_deg = Drive_WrapAngleDeg(
          ctrl->target_heading_deg + rot_dir * DRIVE_OMNI_ROT_STEP_DEG);
    }

    /* Anti-windup cap while operator actively turns so robot stops promptly
     * when releasing L1/R1 */
    if (l1 != r1) {
      float lead = Drive_WrapAngleDeg(ctrl->target_heading_deg - yaw_meas_deg);
      lead = drive_clampf(lead, -DRIVE_OMNI_HEADING_LEAD_CAP_DEG,
                          DRIVE_OMNI_HEADING_LEAD_CAP_DEG);
      ctrl->target_heading_deg = Drive_WrapAngleDeg(yaw_meas_deg + lead);
    }

    /* Wrapped exponential setpoint slew (fixes RBC2026_MAINBOARD_R1 wrap/unwind
     * bug) */
    float slew_delta =
        Drive_WrapAngleDeg(ctrl->target_heading_deg - ctrl->slew_heading_deg);
    ctrl->slew_heading_deg = Drive_WrapAngleDeg(
        ctrl->slew_heading_deg + DRIVE_OMNI_HEADING_SLEW * slew_delta);

    float db_enter = (ctrl->heading_deadband_deg > 0.0f)
                         ? ctrl->heading_deadband_deg
                         : DRIVE_OMNI_HEADING_DEADBAND_DEG;
    float db_exit = (db_enter + 0.50f > DRIVE_OMNI_HEADING_WAKE_DEG)
                        ? (db_enter + 0.50f)
                        : DRIVE_OMNI_HEADING_WAKE_DEG;
    float kp = (ctrl->heading_kp > 0.0f) ? ctrl->heading_kp
                                         : DRIVE_OMNI_HEADING_KP;
    float kd = (ctrl->heading_kd >= 0.0f) ? ctrl->heading_kd
                                          : DRIVE_OMNI_HEADING_KD;

    float err_deg = Drive_WrapAngleDeg(ctrl->slew_heading_deg - yaw_meas_deg);
    float abs_err = fabsf(err_deg);

    /* Schmitt-trigger settled lock during steady hold:
     * Prevents tiny sub-1 deg errors from continuously feeding <3 RPM into
     * PidBoard's velocity integrator (which otherwise winds up against static
     * friction and causes periodic stick-slip twitches).
     * Active snaps / manual turns (rot_ramp_ticks > 0) bypass settled lock with
     * fine 0.35 deg accuracy so 0.5 deg SELECT+L1/R1 trims respond immediately. */
    if (ctrl->rot_ramp_ticks > 0U) {
      ctrl->heading_settled = false;
      if (abs_err < 0.35f) {
        err_deg = 0.0f;
      }
      ctrl->rot_ramp_ticks--;
      ctrl->ramp_omega_deg = Drive_SlewRamp(ctrl->ramp_omega_deg, err_deg,
                                            DRIVE_OMNI_HEADING_RAMP_ACCEL,
                                            DRIVE_OMNI_HEADING_RAMP_DECEL);
    } else {
      if (ctrl->heading_settled) {
        if (abs_err >= db_exit) {
          ctrl->heading_settled = false;
        }
      } else {
        if ((abs_err <= db_enter) && (fabsf(ctrl->yaw_rate_dps) < 12.0f)) {
          ctrl->heading_settled = true;
        }
      }
      if (ctrl->heading_settled) {
        err_deg = 0.0f;
      }
      ctrl->ramp_omega_deg = err_deg;
    }

    float sign = ctrl->heading_hold_invert ? -1.0f : 1.0f;

    /* Gear-decoupled closed-loop PD controller (fixed 200 RPM omega scale):
     * P-term: kp = 2.0% / deg -> 4.0 RPM/deg invariant across all 4 speed gears.
     * D-term: kd = 0.12% / (deg/s) -> 0.24 RPM/(deg/s) active rate braking. */
    float pd_w = 0.0f;
    if (ctrl->heading_settled ||
        ((err_deg == 0.0f) && (fabsf(ctrl->yaw_rate_dps) < 15.0f))) {
      pd_w = 0.0f;
    } else {
      pd_w = (kp * ctrl->ramp_omega_deg) - (kd * ctrl->yaw_rate_dps);
    }

    float max_omega = ((l1 != r1) || (ctrl->rot_ramp_ticks > 0U))
                          ? DRIVE_OMNI_OMEGA_MAX
                          : DRIVE_OMNI_HEADING_HOLD_OMEGA_MAX;
    target_w = drive_clampf(sign * pd_w, -max_omega, max_omega);
    ctrl->omega_fb_pct = target_w;
  } else {
    if (trim_rot) {
      target_w = l2 ? (-rot_dir * DRIVE_OMNI_TRIM_ROT_PCT)
                    : (+rot_dir * DRIVE_OMNI_TRIM_ROT_PCT);
    } else if (!select_held && l1 && !r1) {
      target_w = -rot_dir * DRIVE_OMNI_OMEGA_MAX;
    } else if (!select_held && r1 && !l1) {
      target_w = +rot_dir * DRIVE_OMNI_OMEGA_MAX;
    }
    ctrl->omega_fb_pct = target_w;
  }

  bool translating_now = (target_x != 0.0f) || (target_y != 0.0f);
  bool spinning_now = cardinal_or_diag || trim_rot || (!select_held && (l1 != r1));

  if (translating_now || spinning_now) {
    ctrl->stop_brake_ticks = 0U;
  } else if (ctrl->was_translating) {
    /* Joystick released from translation -> trigger 100 ms hard BRAKE pulse.
     * Forces PidBoard mode VELOCITY -> BRAKE, instantly wiping vel_int_err_ms=0
     * and applying low-side dynamic braking so robot stops dead without 1s creep! */
    ctrl->slew.vx_pct = 0.0f;
    ctrl->slew.vy_pct = 0.0f;
    ctrl->stop_brake_ticks = DRIVE_OMNI_STOP_BRAKE_TICKS;
  }
  ctrl->was_translating = translating_now;

  if (ctrl->stop_brake_ticks > 0U) {
    ctrl->stop_brake_ticks--;
    ctrl->slew.vx_pct = 0.0f;
    ctrl->slew.vy_pct = 0.0f;
    ctrl->slew.omega_pct = 0.0f;
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
      out_modes[i] = DRIVE_WHEEL_MODE_BRAKE;
      out_targets[i] = 0;
    }
    return;
  }

  ctrl->slew.vx_pct =
      Drive_SlewRamp(ctrl->slew.vx_pct, target_x, DRIVE_OMNI_SLEW_ACCEL,
                     DRIVE_OMNI_SLEW_DECEL);
  ctrl->slew.vy_pct =
      Drive_SlewRamp(ctrl->slew.vy_pct, target_y, DRIVE_OMNI_SLEW_ACCEL,
                     DRIVE_OMNI_SLEW_DECEL);
  if (ctrl->field_centric && ctrl->heading_hold && imu_ok) {
    ctrl->slew.omega_pct = target_w;
  } else {
    ctrl->slew.omega_pct =
        Drive_SlewRamp(ctrl->slew.omega_pct, target_w, DRIVE_OMNI_SLEW_ACCEL,
                       DRIVE_OMNI_SLEW_DECEL);
  }

  float theta_rad = 0.0f;
  if (ctrl->field_centric && imu_ok) {
    float effective_deg =
        Drive_WrapAngleDeg(yaw_meas_deg) - ctrl->field_axis_offset_deg;
    float fc_sign = ctrl->field_centric_invert ? -1.0f : 1.0f;
    theta_rad = Drive_WrapAngleDeg(fc_sign * effective_deg) * DRIVE_DEG_TO_RAD_F;
  }

  float v_scale =
      ((float)max_abs_rpm_x10 * (float)ctrl->active_speed_pct) * 0.01f;
  if (boost_3x) {
    v_scale = fminf(v_scale * 3.0f, (float)max_abs_rpm_x10);
  }
  drive_mix_omni_wheels(ctrl->slew.vx_pct, ctrl->slew.vy_pct,
                        ctrl->slew.omega_pct, theta_rad, v_scale,
                        DRIVE_OMNI_OMEGA_SCALE_RPMX10, max_abs_rpm_x10,
                        out_modes, out_targets);
}

void Drive_DiffCompute(int16_t linear_rpm_x10, int16_t angular_rpm_x10,
                       int16_t max_abs_rpm_x10, bool estop,
                       uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                       int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]) {
  if ((out_modes == NULL) || (out_targets == NULL)) {
    return;
  }

  if (max_abs_rpm_x10 <= 0) {
    max_abs_rpm_x10 = DRIVE_DEFAULT_MAX_RPMX10;
  }

  if (estop) {
    out_modes[0] = DRIVE_WHEEL_MODE_DISABLED;
    out_modes[1] = DRIVE_WHEEL_MODE_DISABLED;
    out_targets[0] = 0;
    out_targets[1] = 0;
    return;
  }

  int32_t left = (int32_t)linear_rpm_x10 - (int32_t)angular_rpm_x10;
  int32_t right = (int32_t)linear_rpm_x10 + (int32_t)angular_rpm_x10;

  int32_t abs_l = drive_abs_i32(left);
  int32_t abs_r = drive_abs_i32(right);
  int32_t max_mag = (abs_l > abs_r) ? abs_l : abs_r;

  if (max_mag > (int32_t)max_abs_rpm_x10) {
    left = (left * (int32_t)max_abs_rpm_x10) / max_mag;
    right = (right * (int32_t)max_abs_rpm_x10) / max_mag;
  }

  out_targets[0] = (int16_t)left;
  out_targets[1] = (int16_t)right;

  if ((left == 0) && (right == 0)) {
    out_modes[0] = DRIVE_WHEEL_MODE_BRAKE;
    out_modes[1] = DRIVE_WHEEL_MODE_BRAKE;
  } else {
    out_modes[0] = DRIVE_WHEEL_MODE_VELOCITY;
    out_modes[1] = DRIVE_WHEEL_MODE_VELOCITY;
  }
}

void Drive_LineFollowReset(Drive_LineFollowState_t *st) {
  if (st == NULL) {
    return;
  }
  st->prev_error = 0;
  st->has_prev_error = false;
  st->last_seen_ms = 0U;
  st->has_seen_line = false;
}

static int32_t drive_compute_curvature_ff(const Drive_LineGains_t *gains,
                                          int16_t base_rpm_x10) {
  if ((gains == NULL) || (gains->center_right_sign == 0) ||
      (gains->track_radius_mm <= 0) || (gains->wheel_track_mm <= 0)) {
    return 0;
  }

  /* Rigid-body ICR: center on Right (+1) -> |v_left| > |v_right| for both fwd &
   * rev: angular_ff = -center_right_sign * (base_rpm_x10 * wheel_track_mm) / (2
   * * radius_mm) */
  int32_t denom = 2 * (int32_t)gains->track_radius_mm;
  int32_t num = -(int32_t)gains->center_right_sign * (int32_t)base_rpm_x10 *
                (int32_t)gains->wheel_track_mm;
  if (num >= 0) {
    return (num + (denom / 2)) / denom;
  }
  return (num - (denom / 2)) / denom;
}

static int32_t drive_clamp_i32(int32_t val, int32_t lo, int32_t hi) {
  if (val < lo) {
    return lo;
  }
  if (val > hi) {
    return hi;
  }
  return val;
}

static uint8_t drive_resolve_bar_count(uint8_t mask, uint8_t count) {
  if (mask == 0U) {
    return 0U;
  }
  if (count > 0U) {
    return count;
  }
  uint8_t c = 0U;
  for (uint8_t m = mask; m != 0U; m &= (uint8_t)(m - 1U)) {
    c++;
  }
  return c;
}

static void drive_line_follow_dual_core(
    Drive_LineFollowState_t *st, const Drive_LineGains_t *gains, bool cal_done,
    int16_t base_rpm_x10, uint8_t lead_mask, uint8_t lead_count,
    int16_t lead_err_x10, uint8_t front_count, int16_t front_err_x10,
    uint8_t rear_count, int16_t rear_err_x10, bool wide_guard, uint32_t now_ms,
    bool estop, uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
    int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]) {
  if ((out_modes == NULL) || (out_targets == NULL)) {
    return;
  }

  int16_t kp =
      (gains != NULL) ? gains->kp_rpmx10_per_err : DRIVE_LINE_KP_DEFAULT;
  int16_t kd =
      (gains != NULL) ? gains->kd_rpmx10_per_derr : DRIVE_LINE_KD_DEFAULT;
  int16_t yaw_kp = ((gains != NULL) && (gains->yaw_kp_rpmx10_per_err > 0))
                       ? gains->yaw_kp_rpmx10_per_err
                       : 0;
  uint8_t narrow_max = ((gains != NULL) && (gains->narrow_line_max_cnt > 0U))
                           ? gains->narrow_line_max_cnt
                           : DRIVE_LINE_NARROW_MAX_CNT;
  uint32_t timeout_ms = ((gains != NULL) && (gains->loss_timeout_ms > 0U))
                            ? gains->loss_timeout_ms
                            : DRIVE_LINE_LOSS_TIMEOUT_MS;
  int16_t max_rpm_x10 = ((gains != NULL) && (gains->max_abs_rpm_x10 > 0))
                            ? gains->max_abs_rpm_x10
                            : DRIVE_DEFAULT_MAX_RPMX10;

  if (estop) {
    Drive_LineFollowReset(st);
    Drive_DiffCompute(0, 0, max_rpm_x10, true, out_modes, out_targets);
    return;
  }

  if ((st == NULL) || !cal_done || (base_rpm_x10 == 0)) {
    Drive_LineFollowReset(st);
    Drive_DiffCompute(0, 0, max_rpm_x10, false, out_modes, out_targets);
    return;
  }

  /* Lateral feedback flips with motion direction:
   *   Forward (+): err > 0 (line on Right) -> steer Right (negative angular).
   *   Reverse (-): err > 0 -> swing tail Right (positive angular).
   * Dual-bar yaw feedback -(yaw_kp * (e_front - e_rear)) is body-fixed and
   * direction-invariant: Nose Left (e_front > e_rear) -> rotate CW (negative
   * angular) in BOTH directions. */
  int32_t lat_dir_sign = (base_rpm_x10 >= 0) ? -1 : +1;
  int32_t ff = drive_compute_curvature_ff(gains, base_rpm_x10);
  int32_t max_lim = (int32_t)max_rpm_x10;

  if ((lead_mask != 0U) && (!wide_guard || (lead_count <= narrow_max))) {
    bool dual_ready = wide_guard && (yaw_kp > 0) && (front_count >= 1U) &&
                      (front_count <= narrow_max) && (rear_count >= 1U) &&
                      (rear_count <= narrow_max);
    int16_t lat_err = lead_err_x10;
    int32_t u_yaw = 0;

    if (dual_ready) {
      int32_t sum = (int32_t)front_err_x10 + (int32_t)rear_err_x10;
      lat_err = (int16_t)((sum >= 0) ? ((sum + 1) / 2) : ((sum - 1) / 2));
      int32_t yaw_err = (int32_t)front_err_x10 - (int32_t)rear_err_x10;
      u_yaw = -((int32_t)yaw_kp * yaw_err) / 10;
    }

    int32_t derr =
        st->has_prev_error
            ? drive_clamp_i32((int32_t)lat_err - (int32_t)st->prev_error, -400,
                              400)
            : 0;
    st->prev_error = lat_err;
    st->has_prev_error = true;
    st->last_seen_ms = now_ms;
    st->has_seen_line = true;

    int32_t pd = lat_dir_sign *
                 (((int32_t)kp * (int32_t)lat_err + (int32_t)kd * derr) / 10);
    int32_t angular = drive_clamp_i32(pd + u_yaw + ff, -max_lim, max_lim);
    Drive_DiffCompute(base_rpm_x10, (int16_t)angular, max_rpm_x10, false,
                      out_modes, out_targets);
    return;
  }

  /* Wide-marker guard: suppress asymmetric cross/Load-Port edge centroid kicks
   * and derivative spikes */
  if (wide_guard && (lead_mask != 0U) && (lead_count > narrow_max)) {
    if (!st->has_seen_line) {
      st->prev_error = 0;
      st->has_prev_error = true;
      st->last_seen_ms = now_ms;
      st->has_seen_line = true;
    }
    if ((lead_count >= 8U) &&
        ((uint32_t)(now_ms - st->last_seen_ms) > (2U * timeout_ms))) {
      Drive_LineFollowReset(st);
      Drive_DiffCompute(0, 0, max_rpm_x10, false, out_modes, out_targets);
      return;
    }
    int32_t pd_hold =
        lat_dir_sign * (((int32_t)kp * (int32_t)st->prev_error) / 10);
    int32_t angular = drive_clamp_i32(pd_hold + ff, -max_lim, max_lim);
    Drive_DiffCompute(base_rpm_x10, (int16_t)angular, max_rpm_x10, false,
                      out_modes, out_targets);
    return;
  }

  if (st->has_seen_line &&
      ((uint32_t)(now_ms - st->last_seen_ms) <= timeout_ms)) {
    int32_t ff_half =
        drive_compute_curvature_ff(gains, (int16_t)(base_rpm_x10 / 2));
    int32_t pd_coast =
        lat_dir_sign * (((int32_t)kp * (int32_t)lead_err_x10) / 10);
    int32_t angular = drive_clamp_i32(pd_coast + ff_half, -max_lim, max_lim);
    Drive_DiffCompute((int16_t)(base_rpm_x10 / 2), (int16_t)angular,
                      max_rpm_x10, false, out_modes, out_targets);
    return;
  }

  Drive_LineFollowReset(st);
  Drive_DiffCompute(0, 0, max_rpm_x10, false, out_modes, out_targets);
}

void Drive_LineFollowStep(Drive_LineFollowState_t *st,
                          const Drive_LineGains_t *gains, bool cal_done,
                          int16_t base_rpm_x10, uint8_t active_mask,
                          int16_t line_error, uint32_t now_ms, bool estop,
                          uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                          int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]) {
  int16_t err_x10 = (int16_t)((int32_t)line_error * 10);
  uint8_t cnt = (active_mask != 0U) ? 1U : 0U;
  drive_line_follow_dual_core(st, gains, cal_done, base_rpm_x10, active_mask,
                              cnt, err_x10, cnt, err_x10, 0U, 0, false, now_ms,
                              estop, out_modes, out_targets);
}

void Drive_LineFollowCentroidStep(Drive_LineFollowState_t *st,
                                  const Drive_LineGains_t *gains, bool cal_done,
                                  int16_t base_rpm_x10, uint8_t active_mask,
                                  int16_t line_err_x10, uint32_t now_ms,
                                  bool estop,
                                  uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                                  int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]) {
  uint8_t cnt = (active_mask != 0U) ? 1U : 0U;
  drive_line_follow_dual_core(st, gains, cal_done, base_rpm_x10, active_mask,
                              cnt, line_err_x10, cnt, line_err_x10, 0U, 0,
                              false, now_ms, estop, out_modes, out_targets);
}

void Drive_LineFollowDualBarStep(Drive_LineFollowState_t *st,
                                 const Drive_LineGains_t *gains, bool cal_done,
                                 int16_t base_rpm_x10, uint8_t front_mask,
                                 uint8_t front_count, int16_t front_err_x10,
                                 uint8_t rear_mask, uint8_t rear_count,
                                 int16_t rear_err_x10, uint32_t now_ms,
                                 bool estop,
                                 uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                                 int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]) {
  uint8_t fc = drive_resolve_bar_count(front_mask, front_count);
  uint8_t rc = drive_resolve_bar_count(rear_mask, rear_count);
  int32_t r_sign =
      ((gains != NULL) && (gains->rear_polarity_sign < 0)) ? -1 : +1;
  int16_t r_aligned = (int16_t)(r_sign * (int32_t)rear_err_x10);

  uint8_t lead_mask = (base_rpm_x10 >= 0) ? front_mask : rear_mask;
  uint8_t lead_count = (base_rpm_x10 >= 0) ? fc : rc;
  int16_t lead_err = (base_rpm_x10 >= 0) ? front_err_x10 : r_aligned;

  drive_line_follow_dual_core(st, gains, cal_done, base_rpm_x10, lead_mask,
                              lead_count, lead_err, fc, front_err_x10, rc,
                              r_aligned, true, now_ms, estop, out_modes,
                              out_targets);
}
