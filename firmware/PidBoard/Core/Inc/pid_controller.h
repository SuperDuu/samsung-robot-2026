/**
  ******************************************************************************
  * @file    pid_controller.h
  * @brief   Fixed-point 1 kHz cascaded position + velocity PID controller,
  *          16-bit wrap-safe quadrature encoder delta, anti-windup, IR2184
  *          bootstrap recharge PWM clamp (<= 975/999), 250 ms cmd watchdog,
  *          and Flash Page 15 persistent configuration for PidBoard (HB_RBC).
  ******************************************************************************
  */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UNIT_TEST_HOST
#include "mock_hal.h"
#else
#include "main.h"
#endif

#include "rs485_bus.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef PID_DEFAULT_NODE_ID
#define PID_DEFAULT_NODE_ID          1U
#endif

#define PID_FLASH_PAGE_INDEX         15U
#define PID_FLASH_CONFIG_ADDR        0x08007800UL
#define PID_CONFIG_MAGIC             0x50494431UL /* "PID1" */
#define PID_CONFIG_VERSION           1U

#define PID_PWM_ARR                  999U
#define PID_MAX_DUTY_PERMILLE        975  /* >= 25 us low-side conduction per 1 ms cycle for IR2184 bootstrap */
#define PID_DEFAULT_CPR              836U
#define PID_DEFAULT_MAX_VEL_RPM_X10  4000 /* +/- 400.0 RPM */
#define PID_CMD_WATCHDOG_MS          RS485_CMD_WATCHDOG_MS

#define PID_DEFAULT_VEL_KP_M         250  /* 0.250 */
#define PID_DEFAULT_VEL_KI_M         7000 /* 7.000 */
#define PID_DEFAULT_VEL_KD_M         0    /* 0.000 */
#define PID_DEFAULT_POS_KP_M         750  /* 0.750 */
#define PID_DEFAULT_POS_KD_M         1    /* 0.001 */

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint8_t  node_id;
  uint8_t  invert_dir;
  uint16_t cpr;
  uint16_t max_duty_permille;
  int32_t  max_vel_rpm_x10;
  int32_t  vel_kp_m;
  int32_t  vel_ki_m;
  int32_t  vel_kd_m;
  int32_t  pos_kp_m;
  int32_t  pos_kd_m;
  uint32_t crc32;
} PID_Config_t;

typedef struct
{
  PID_Config_t      cfg;
  RS485_WheelMode_t mode;
  int32_t           target_value;       /* duty permille, 0.1 RPM, or 0.1 deg */
  int32_t           vel_target_x10;     /* effective velocity setpoint (0.1 RPM) */
  int32_t           actual_rpm_x10;     /* filtered shaft speed (0.1 RPM) */
  int32_t           vel_ema_x1000;      /* internal IIR speed state (0.001 RPM) */
  int32_t           prev_vel_ema_x1000; /* previous 0.001 RPM for D-on-measurement */
  int64_t           vel_int_err_ms;     /* integral of milli-RPM error * ms */
  int32_t           duty_permille;      /* logical signed effort (-975 .. +975) */
  int32_t           hw_duty_permille;   /* polarity-adjusted H-bridge duty (-975 .. +975) */
  uint8_t           gate_enable;        /* 1 = IR2184 ~SD HIGH, 0 = Hi-Z coast */
  int32_t           enc_count;          /* cumulative signed quadrature counts */
  uint16_t          prev_hw_cnt;        /* last 16-bit TIM3->CNT sample */
  uint8_t           enc_primed;
  uint32_t          last_cmd_ms;
  uint8_t           has_cmd;
  uint8_t           watchdog_tripped;
  uint8_t           estop_latched;
  uint8_t           comm_err_latched;
  uint8_t           saturated;
  uint8_t           flash_valid;
  volatile uint8_t  pending_flash_save;
} PID_Controller_t;

void PID_LoadDefaults(PID_Config_t *cfg);
uint32_t PID_ConfigCrc32(const PID_Config_t *cfg);
bool PID_ValidateConfig(const PID_Config_t *cfg);

void PID_Init(PID_Controller_t *ctrl, uint16_t initial_hw_cnt, uint32_t now_ms);
void PID_ResetControlState(PID_Controller_t *ctrl);
void PID_ZeroEncoder(PID_Controller_t *ctrl, uint16_t hw_cnt);

void PID_ApplyCommand(PID_Controller_t *ctrl,
                      RS485_WheelMode_t mode,
                      int32_t target,
                      uint8_t flags,
                      uint32_t now_ms);

uint8_t PID_HandleConfigSubcmd(PID_Controller_t *ctrl,
                               const RS485_CfgCmd_t *cmd,
                               bool *out_save_deferred);

void PID_Step1kHz(PID_Controller_t *ctrl, uint16_t hw_cnt, uint32_t now_ms);
void PID_BuildTelem(const PID_Controller_t *ctrl, RS485_Telem_t *out_telem);

bool PID_SaveConfigFlash(PID_Controller_t *ctrl);
bool PID_LoadConfigFlash(PID_Config_t *out_cfg);

#ifdef UNIT_TEST_HOST
void PID_HostResetFlashBacking(void);
void PID_HostCorruptFlashByte(uint32_t offset, uint8_t value);
#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_CONTROLLER_H_ */
