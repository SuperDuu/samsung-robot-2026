/**
  ******************************************************************************
  * @file    drive_kinematics.h
  * @brief   Portable 4-wheel omni kinematics (with PS2 analog sum underflow
  *          guard, field-centric rotation, uniform saturation, slew ramp),
  *          2-wheel differential drive kinematics, and 2x8 PD line follower.
  ******************************************************************************
  */

#ifndef INC_DRIVE_KINEMATICS_H_
#define INC_DRIVE_KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRIVE_OMNI_WHEEL_COUNT     4U
#define DRIVE_DIFF_WHEEL_COUNT     2U

#define DRIVE_WHEEL_MODE_DISABLED  0U /* Matches RS485_WHEEL_DISABLED */
#define DRIVE_WHEEL_MODE_BRAKE     1U /* Matches RS485_WHEEL_BRAKE    */
#define DRIVE_WHEEL_MODE_VELOCITY  3U /* Matches RS485_WHEEL_VELOCITY */

#define DRIVE_DEFAULT_MAX_RPMX10   4000
#define DRIVE_OMNI_NORM_RPMX10     2500.0f
#define DRIVE_OMNI_BOOST_RPMX10    3800.0f
#define DRIVE_PS2_DEADBAND         10.0f
#define DRIVE_OMNI_DPAD_PCT        70.0f
#define DRIVE_OMNI_OMEGA_MAX       65.0f
#define DRIVE_OMNI_SLEW_ACCEL      10.0f
#define DRIVE_OMNI_SLEW_DECEL      15.0f

#define DRIVE_LINE_KP_DEFAULT      18
#define DRIVE_LINE_KD_DEFAULT      8
#define DRIVE_LINE_LOSS_TIMEOUT_MS 200U

typedef struct
{
  uint8_t joy_sum[2];
  uint8_t ps2_btn[2];
  bool    valid;
} Drive_Ps2Input_t;

typedef struct
{
  float vx_pct;
  float vy_pct;
  float omega_pct;
} Drive_OmniState_t;

typedef struct
{
  int16_t  prev_error;
  bool     has_prev_error;
  uint32_t last_seen_ms;
  bool     has_seen_line;
} Drive_LineFollowState_t;

typedef struct
{
  int16_t  kp_rpmx10_per_err;
  int16_t  kd_rpmx10_per_derr;
  uint32_t loss_timeout_ms;
  int16_t  max_abs_rpm_x10;
} Drive_LineGains_t;

float Drive_SlewRamp(float current, float target, float accel_step, float decel_step);

void Drive_OmniReset(Drive_OmniState_t *state);

void Drive_OmniComputeFromPs2(Drive_OmniState_t *state,
                              const Drive_Ps2Input_t *in,
                              bool estop,
                              float robot_yaw_deg,
                              bool field_centric,
                              int16_t max_abs_rpm_x10,
                              uint8_t out_modes[DRIVE_OMNI_WHEEL_COUNT],
                              int16_t out_targets[DRIVE_OMNI_WHEEL_COUNT]);

void Drive_DiffCompute(int16_t linear_rpm_x10,
                       int16_t angular_rpm_x10,
                       int16_t max_abs_rpm_x10,
                       bool estop,
                       uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                       int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]);

void Drive_LineFollowReset(Drive_LineFollowState_t *st);

void Drive_LineFollowStep(Drive_LineFollowState_t *st,
                          const Drive_LineGains_t *gains,
                          bool cal_done,
                          int16_t base_rpm_x10,
                          uint8_t active_mask,
                          int16_t line_error,
                          uint32_t now_ms,
                          bool estop,
                          uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                          int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* INC_DRIVE_KINEMATICS_H_ */

