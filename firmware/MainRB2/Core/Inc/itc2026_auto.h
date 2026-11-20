/**
  ******************************************************************************
  * @file    itc2026_auto.h
  * @brief   Autonomous AMHS Track Vehicle FSM for ITC 2026 Robot 2 (MainRB2).
  *          Supports standalone SW1/SW2 operation, 2-point cal save trigger,
  *          Start-Box exit, CW/CCW continuous-centroid + curvature-FF line
  *          following, Load Port metal-trigger servo hold, VL53L1X obstacle slow/
  *          stop/reversal, and optional RF_NODE_MANUAL supervisory stop/start.
  ******************************************************************************
  */

#ifndef INC_ITC2026_AUTO_H_
#define INC_ITC2026_AUTO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "drive_kinematics.h"
#include <stdbool.h>
#include <stdint.h>

#define ITC_BASE_RUN_RPMX10            1800  /* Nominal circular track cruise speed */
#define ITC_EXIT_BOX_RPMX10            1400  /* Start-box exit cruise speed */
#define ITC_EXIT_BOX_TIMEOUT_MS        2500U /* Max ms searching for track line from Start Box */
#define ITC_EXIT_HEADING_KP_RPMX10     15    /* Heading hold gain while exiting Start Box */
#define ITC_SKEW_SLOW_START_X10        200   /* Dual-bar |e_f - e_r| threshold before linear decel */
#define ITC_SKEW_SLOW_MAX_X10          600   /* Dual-bar skew at which cruise moderates to 60% */
#define ITC_SKEW_SLOW_MIN_PCT          60    /* Floor speed % while squaring large yaw skew */

#define ITC_TOF_SLOW_MM                650U  /* Begin proportional deceleration */
#define ITC_TOF_STOP_MM                350U  /* Full brake threshold */
#define ITC_TOF_RESUME_MM              420U  /* Hysteresis threshold before resuming forward */
#define ITC_TOF_REVERSE_SAFE_MM        500U  /* Minimum clear distance behind before reversing */
#define ITC_OBSTACLE_CLEAR_CONFIRM_MS  200U  /* Sustained clear dwell before resume */
#define ITC_OBSTACLE_REVERSE_WAIT_MS   1200U /* Persistent block time triggering CW<->CCW reversal */

#define ITC_LOADPORT_DWELL_MS          2500U /* Hold station READY with metal trigger extended */
#define ITC_LOADPORT_COOLDOWN_MS       1200U /* Ignore duplicate wide-bar triggers leaving port */

#define ITC_SERVO_ARM_STOW_DEG         0.0f
#define ITC_SERVO_ARM_TRIGGER_DEG      135.0f

typedef enum
{
  ITC_STATE_IDLE           = 0U,
  ITC_STATE_CAL_SWEEP      = 1U,
  ITC_STATE_EXIT_START_BOX = 2U,
  ITC_STATE_TRACK_RUN      = 3U,
  ITC_STATE_LOAD_PORT_HOLD = 4U,
  ITC_STATE_OBSTACLE_STOP  = 5U,
  ITC_STATE_ESTOP          = 6U
} ITC_State_t;

typedef struct
{
  uint32_t now_ms;
  bool     start_toggle_edge; /* SW1 press (start/stop autonomous run or clear estop) */
  bool     cal_toggle_edge;   /* SW2 long-press to enter/exit cal, or confirm finish */
  bool     estop_req;
  bool     cal_done;
  uint8_t  front_mask;
  uint8_t  rear_mask;
  uint8_t  front_count;
  uint8_t  rear_count;
  int16_t  front_centroid_x10;
  int16_t  rear_centroid_x10;
  bool     imu_ok;
  float    imu_yaw_deg;
  bool     front_blocked;     /* raw front obstacle <= ITC_TOF_STOP_MM */
  bool     rear_blocked;      /* raw rear obstacle <= ITC_TOF_STOP_MM */
  uint16_t front_dist_mm;
  uint16_t rear_dist_mm;
  bool     manual_rf_alive;
  bool     manual_rf_hold_req;
  bool     manual_rf_reverse_edge;
} ITC_AutoInput_t;

typedef struct
{
  ITC_State_t state;
  int8_t      dir_sign;               /* +1 = forward (CW), -1 = reverse (CCW) */
  uint8_t     wheel_modes[DRIVE_DIFF_WHEEL_COUNT];
  int16_t     wheel_targets[DRIVE_DIFF_WHEEL_COUNT];
  float       arm_deg;                /* SERVO_CH_1 0-270 deg metal sensor trigger */
  bool        req_begin_two_point_cal;
  bool        req_finish_two_point_cal;
  bool        req_save_flash_cal;
  uint32_t    ports_triggered;
  uint32_t    reversals;
} ITC_AutoOutput_t;

typedef struct
{
  ITC_State_t             state;
  int8_t                  dir_sign;
  uint32_t                state_enter_ms;
  uint32_t                clear_since_ms;
  bool                    clear_timing;
  uint32_t                port_cooldown_until_ms;
  float                   exit_target_yaw_deg;
  bool                    has_exit_yaw;
  uint32_t                ports_triggered;
  uint32_t                reversals;
  int16_t                 base_cruise_rpm_x10;
  Drive_LineFollowState_t line_state;
  Drive_LineGains_t       line_gains;
} ITC_Auto_t;

void ITC_AutoInit(ITC_Auto_t *fsm, int16_t cruise_rpm_x10, float default_exit_yaw_deg);
void ITC_AutoSetCruiseSpeed(ITC_Auto_t *fsm, int16_t cruise_rpm_x10);
void ITC_AutoCancelCalibration(ITC_Auto_t *fsm, uint32_t now_ms);
void ITC_AutoStep(ITC_Auto_t *fsm, const ITC_AutoInput_t *in, ITC_AutoOutput_t *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_ITC2026_AUTO_H_ */

