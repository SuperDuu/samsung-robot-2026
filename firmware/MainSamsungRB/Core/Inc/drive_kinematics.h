/**
  ******************************************************************************
  * @file    drive_kinematics.h
  * @brief   Portable 4-wheel omni kinematics (custom modified field-centric,
  *          cardinal/diagonal snaps, speed gears, PS2 analog underflow guard,
  *          uniform saturation, slew ramp), 2-wheel differential drive kinematics,
  *          and continuous-centroid + curvature-feedforward PD line follower.
  ******************************************************************************
  */

#ifndef INC_DRIVE_KINEMATICS_H_
#define INC_DRIVE_KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DRIVE_OMNI_WHEEL_COUNT           4U
#define DRIVE_DIFF_WHEEL_COUNT           2U

#define DRIVE_WHEEL_MODE_DISABLED        0U /* Matches RS485_WHEEL_DISABLED */
#define DRIVE_WHEEL_MODE_BRAKE           1U /* Matches RS485_WHEEL_BRAKE    */
#define DRIVE_WHEEL_MODE_VELOCITY        3U /* Matches RS485_WHEEL_VELOCITY */

#define DRIVE_DEFAULT_MAX_RPMX10         4000
#define DRIVE_OMNI_NORM_RPMX10           2500.0f
#define DRIVE_OMNI_BOOST_RPMX10          3800.0f
#define DRIVE_PS2_DEADBAND               10.0f
#define DRIVE_OMNI_DPAD_PCT              100.0f  /* Directional unit vector for D-pad crawl */
#define DRIVE_OMNI_OMEGA_MAX             65.0f
#define DRIVE_OMNI_OMEGA_SCALE_RPMX10    2000.0f /* Fixed 200 RPM full-scale (2.0 RPM per 1%), decoupled from Gear! */
#define DRIVE_OMNI_SLEW_ACCEL            25.0f   /* 25% per 10ms = 250%/s punchy, responsive translation */
#define DRIVE_OMNI_SLEW_DECEL            35.0f   /* 35% per 10ms = 350%/s rapid deceleration */

#define DRIVE_OMNI_GEAR_COUNT            4U
#define DRIVE_OMNI_DEFAULT_GEAR          0U      /* 33% default base speed tier (3x boost with L1 reaches 100%) */
#define DRIVE_OMNI_CRAWL_PCT             25U     /* Precision crawl speed (25% = 100 RPM / 0.22 m/s, overcomes stiction) */
#define DRIVE_OMNI_TRIM_ROT_PCT          10.0f   /* 20 RPM fine yaw adjustment (10% of 2000 rpm_x10) */
#define DRIVE_OMNI_ROT_STEP_DEG          1.2f    /* 120 deg/s at 100 Hz while L1/R1 held */
#define DRIVE_OMNI_HEADING_TRIM_STEP_DEG 0.5f    /* SELECT + L1/R1 live IMU trim per press */
#define DRIVE_OMNI_HEADING_KP            2.0f    /* 2.0% * 2.0 RPM/% = 4.0 RPM/deg (firm, crisp, gear-invariant) */
#define DRIVE_OMNI_HEADING_KD            0.12f   /* 0.12% * 2.0 RPM/% = 0.24 RPM/(deg/s) active rate braking */
#define DRIVE_OMNI_HEADING_DEADBAND_DEG  0.80f   /* Settled hysteresis enter threshold (deg) */
#define DRIVE_OMNI_HEADING_WAKE_DEG      1.50f   /* Settled hysteresis exit threshold (kills idle stick-slip twitch) */
#define DRIVE_OMNI_STOP_BRAKE_TICKS      10U     /* 100 ms hard BRAKE pulse on stick release (wipes PidBoard I-tail) */
#define DRIVE_OMNI_HEADING_HOLD_OMEGA_MAX 45.0f  /* 45% * 2.0 = 90 RPM heading hold cap (gear-independent) */
#define DRIVE_OMNI_HEADING_OMEGA_MAX     DRIVE_OMNI_HEADING_HOLD_OMEGA_MAX
#define DRIVE_OMNI_HEADING_RAMP_ACCEL    3.0f    /* RBC2026_MAINBOARD_R1 Apply_Soft_Ramp accel (deg/step) */
#define DRIVE_OMNI_HEADING_RAMP_DECEL    6.0f    /* RBC2026_MAINBOARD_R1 Apply_Soft_Ramp decel (deg/step) */
#define DRIVE_OMNI_HEADING_RAMP_TICKS    100U    /* 100 ticks at 100 Hz = 1000 ms button rotation window */
#define DRIVE_OMNI_HEADING_SLEW          0.10f   /* rad_dh low-pass alpha (0.1f matching RBC2026_MAINBOARD_R1) */
#define DRIVE_OMNI_HEADING_LEAD_CAP_DEG  20.0f   /* Anti-windup cap while rotating */

#define DRIVE_LINE_KP_DEFAULT            18
#define DRIVE_LINE_KD_DEFAULT            8
#define DRIVE_LINE_YAW_KP_DEFAULT        14   /* Dual-bar (e_front - e_rear) yaw alignment gain */
#define DRIVE_LINE_NARROW_MAX_CNT        3U   /* >3 active sensors treated as wide Load Port / cross mark */
#define DRIVE_LINE_LOSS_TIMEOUT_MS       200U
#define DRIVE_ITC_TRACK_RADIUS_MM        2050    /* ITC 2026 4.1m diameter oval/ring */
#define DRIVE_ITC_WHEEL_TRACK_MM         320

/* PS2 button bitmasks matching tdk/Core/Src/PS2_STM32.c */
#define DRIVE_PS2_B0_SELECT              (1U << 0)
#define DRIVE_PS2_B0_L3                  (1U << 1)
#define DRIVE_PS2_B0_R3                  (1U << 2)
#define DRIVE_PS2_B0_START               (1U << 3)
#define DRIVE_PS2_B0_UP                  (1U << 4)
#define DRIVE_PS2_B0_RIGHT               (1U << 5)
#define DRIVE_PS2_B0_DOWN                (1U << 6)
#define DRIVE_PS2_B0_LEFT                (1U << 7)

#define DRIVE_PS2_B1_L2                  (1U << 0)
#define DRIVE_PS2_B1_R2                  (1U << 1)
#define DRIVE_PS2_B1_L1                  (1U << 2)
#define DRIVE_PS2_B1_R1                  (1U << 3)
#define DRIVE_PS2_B1_TRIANGLE            (1U << 4)
#define DRIVE_PS2_B1_CIRCLE              (1U << 5)
#define DRIVE_PS2_B1_CROSS               (1U << 6)
#define DRIVE_PS2_B1_SQUARE              (1U << 7)

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
  Drive_OmniState_t slew;
  bool              field_centric;
  bool              heading_hold;
  bool              blue_side;
  bool              diagonal_45;
  bool              heading_initialized;
  bool              heading_hold_invert;
  uint8_t           gear;                  /* 0..3 -> {30%, 55%, 80%, 100%} */
  uint8_t           active_speed_pct;
  uint8_t           prev_btn0;
  uint8_t           prev_btn1;
  float             target_heading_deg;    /* add_rad equivalent in deg */
  float             slew_heading_deg;      /* rad_dh equivalent in deg */
  float             field_axis_offset_deg; /* offset_rad equivalent in deg */
  float             heading_trim_deg;      /* goc_bu manual trim in deg */
  float             omega_fb_pct;
  float             prev_yaw_deg;          /* Previous yaw for angular velocity estimation */
  float             yaw_rate_dps;          /* Filtered yaw angular rate in deg/s */
  bool              has_prev_yaw;          /* Valid previous yaw sample flag */
  float             ramp_omega_deg;        /* ramp_Omega from RBC2026_MAINBOARD_R1 Apply_Soft_Ramp */
  uint16_t          rot_ramp_ticks;        /* Ticks remaining in button rotation soft-ramp window */
  float             heading_kp;            /* Live-tunable Kp (% of 200 RPM scale per deg) */
  float             heading_kd;            /* Live-tunable Kd (% of 200 RPM scale per deg/s) */
  float             heading_deadband_deg;  /* Live-tunable settled enter deadband (deg) */
  bool              heading_settled;       /* Schmitt-trigger settled lock (prevents PidBoard stick-slip windup) */
  bool              pure_field_centric;    /* true = CIRCLE/CROSS/SQUARE keep field axis fixed (0 deg offset) */
  bool              field_centric_invert;  /* true = negate theta_rad if IMU Z-axis is flipped */
  bool              rot_btn_invert;        /* true = L1 adds +deg (CCW), R1 subtracts -deg (CW) for CCW+ BNO085 */
  bool              was_translating;       /* Previous tick had non-zero joystick translation */
  uint8_t           stop_brake_ticks;      /* Countdown for hard BRAKE pulse on joystick release */
} Drive_OmniCtrl_t;

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
  /* Optional circular track curvature feedforward (0 disables FF) */
  int16_t  track_radius_mm;
  int16_t  wheel_track_mm;
  int8_t   center_right_sign; /* +1 = circle center on Right (CW forward), -1 = Left, 0 = off */
  /* Optional Dual-Bar (Front + Rear 2x8) yaw alignment & wide-marker guard (0 = safe default) */
  int16_t  yaw_kp_rpmx10_per_err; /* >0 activates (e_front - e_rear) yaw control; <=0 = single-bar only */
  uint8_t  narrow_line_max_cnt;   /* Max sensors for clean narrow centroid (0 -> DRIVE_LINE_NARROW_MAX_CNT) */
  int8_t   rear_polarity_sign;    /* +1 or 0 = Rear I8..I15 shares Front L->R sign; -1 = Rear PCB 180-deg rotated */
} Drive_LineGains_t;

float Drive_SlewRamp(float current, float target, float accel_step, float decel_step);
float Drive_WrapAngleDeg(float deg);

void Drive_OmniReset(Drive_OmniState_t *state);

void Drive_OmniComputeFromPs2(Drive_OmniState_t *state,
                              const Drive_Ps2Input_t *in,
                              bool estop,
                              float robot_yaw_deg,
                              bool field_centric,
                              int16_t max_abs_rpm_x10,
                              uint8_t out_modes[DRIVE_OMNI_WHEEL_COUNT],
                              int16_t out_targets[DRIVE_OMNI_WHEEL_COUNT]);

void Drive_OmniCtrlInit(Drive_OmniCtrl_t *ctrl, bool blue_side);
void Drive_OmniCtrlSetBlueSide(Drive_OmniCtrl_t *ctrl, bool blue_side);
void Drive_OmniCtrlZeroHeading(Drive_OmniCtrl_t *ctrl);

void Drive_OmniCtrlStep(Drive_OmniCtrl_t *ctrl,
                        const Drive_Ps2Input_t *in,
                        bool estop,
                        float robot_yaw_deg,
                        bool imu_healthy,
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

/* High-resolution sub-sensor centroid step (line_err_x10 in [-700..+700]) */
void Drive_LineFollowCentroidStep(Drive_LineFollowState_t *st,
                                  const Drive_LineGains_t *gains,
                                  bool cal_done,
                                  int16_t base_rpm_x10,
                                  uint8_t active_mask,
                                  int16_t line_err_x10,
                                  uint32_t now_ms,
                                  bool estop,
                                  uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                                  int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]);

/* Coupled Dual-Bar (Front + Rear 2x8) lateral + yaw controller with wide-marker guard */
void Drive_LineFollowDualBarStep(Drive_LineFollowState_t *st,
                                 const Drive_LineGains_t *gains,
                                 bool cal_done,
                                 int16_t base_rpm_x10,
                                 uint8_t front_mask,
                                 uint8_t front_count,
                                 int16_t front_err_x10,
                                 uint8_t rear_mask,
                                 uint8_t rear_count,
                                 int16_t rear_err_x10,
                                 uint32_t now_ms,
                                 bool estop,
                                 uint8_t out_modes[DRIVE_DIFF_WHEEL_COUNT],
                                 int16_t out_targets[DRIVE_DIFF_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* INC_DRIVE_KINEMATICS_H_ */
