/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "bno055_stm32.h"
#include "bno08x.h"
#include "buttons.h"
#include "cd4067.h"
#include "drive_kinematics.h"
#include "flash_cal.h"
#include "itc2026_auto.h"
#include "rf_hc12.h"
#include "rs485_bus.h"
#include "servo.h"
#include "tof_vl53l1x.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MAINRB2_DRIVE_AUTO = 0U,
  MAINRB2_DRIVE_MANUAL_VEL = 1U,
  MAINRB2_DRIVE_LINE_FOLLOW = 2U,
  MAINRB2_DRIVE_IDLE = 3U
} MainRB2_DriveMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_TX_PERIOD_MS 10U
#define RS485_WHEEL_TIMEOUT_MS 250U
#define SW2_CAL_HOLD_MS 800U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
BNO055_IMU_t g_imu = {0};
RF_HC12_t g_rf = {0};
RF_PayloadManualToAuto_t g_manual_msg = {0};
bool g_manual_link_ok = false;
uint8_t g_front_mask = 0U;
uint8_t g_rear_mask = 0U;
int16_t g_front_error = 0;
int16_t g_rear_error = 0;
Robot_Debug_t g_debug = {0};
RS485_Bus_t g_rs485 = {0};
RS485_Telem_t g_wheel_telem[DRIVE_DIFF_WHEEL_COUNT] = {0};
bool g_wheel_alive[DRIVE_DIFF_WHEEL_COUNT] = {false};
static uint32_t g_wheel_rx_ms[DRIVE_DIFF_WHEEL_COUNT] = {0};
static bool g_wheel_seen[DRIVE_DIFF_WHEEL_COUNT] = {false};
static uint8_t g_rs485_poll_idx = 0U;
static uint32_t g_last_rs485_tx_ms = 0U;
static MainRB2_DriveMode_t g_drive_mode = MAINRB2_DRIVE_AUTO;
static int16_t g_cmd_linear_rpm_x10 = 0;
static int16_t g_cmd_angular_rpm_x10 = 0;
static int16_t g_line_base_rpm_x10 = 0;
static Drive_LineFollowState_t g_line_state = {0};
static const Drive_LineGains_t g_line_gains = {DRIVE_LINE_KP_DEFAULT,
                                               DRIVE_LINE_KD_DEFAULT,
                                               DRIVE_LINE_LOSS_TIMEOUT_MS,
                                               DRIVE_DEFAULT_MAX_RPMX10,
                                               DRIVE_ITC_TRACK_RADIUS_MM,
                                               DRIVE_ITC_WHEEL_TRACK_MM,
                                               +1,
                                               DRIVE_LINE_YAW_KP_DEFAULT,
                                               DRIVE_LINE_NARROW_MAX_CNT,
                                               +1};
ToF_Manager_t g_tof = {0};
uint8_t g_tof_online_mask = 0U;
FlashCal_Record_t g_flash_cal = {0};
bool g_flash_cal_loaded = false;
ITC_Auto_t g_auto = {0};
ITC_AutoOutput_t g_auto_out = {0};
static bool g_estop_latched = false;
static bool g_clear_faults_pending = false;
static uint32_t g_last_imu_tick_ms = 0U;
static bool g_pending_sw1_edge = false;
static bool g_pending_sw2_cal_edge = false;
static bool g_sw2_long_handled = false;
static float g_prev_arm_deg = -1.0f;
volatile int32_t g_pidboard_vel_kp_m =
    1500; /* Live-tunable PidBoard inner vel Kp (milli-units), auto-synced via
             RS-485 */
volatile int32_t g_pidboard_vel_ki_m =
    2000; /* Live-tunable PidBoard inner vel Ki (milli-units), eliminates
             stick-slip windup */
volatile uint8_t g_imu_save_cal =
    0U; /* 1 = Save DCD to BNO08x internal Flash */
volatile uint8_t g_imu_clear_cal =
    0U; /* 1 = Clear DCD & reset BNO08x to recalibrate */
volatile uint32_t g_imu_cal_saved_cnt = 0U;
volatile uint32_t g_imu_cal_cleared_cnt = 0U;
static bool g_diff_wheels_stopped = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void main_on_rs485_rx(RS485_Bus_t *bus, const RS485_Frame_t *frame,
                             void *ctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void main_on_rs485_rx(RS485_Bus_t *bus, const RS485_Frame_t *frame,
                             void *ctx) {
  (void)ctx;
  if ((bus == NULL) || (frame == NULL)) {
    return;
  }

  if (frame->msg_type == (uint8_t)RS485_MSG_TELEM) {
    RS485_Telem_t telem;
    if (RS485_DecodeTelem(frame, &telem) &&
        (telem.node_id >= RS485_ADDR_SLAVE_MIN) &&
        (telem.node_id <= DRIVE_DIFF_WHEEL_COUNT)) {
      uint8_t idx = (uint8_t)(telem.node_id - 1U);
      g_wheel_telem[idx] = telem;
      g_wheel_rx_ms[idx] = bus->last_rx_tick_ms;
      g_wheel_seen[idx] = true;
    }
  }
}

void MainRB2_SetDriveCommand(int16_t linear_rpm_x10, int16_t angular_rpm_x10) {
  g_cmd_linear_rpm_x10 = linear_rpm_x10;
  g_cmd_angular_rpm_x10 = angular_rpm_x10;
  g_drive_mode = ((linear_rpm_x10 == 0) && (angular_rpm_x10 == 0))
                     ? MAINRB2_DRIVE_IDLE
                     : MAINRB2_DRIVE_MANUAL_VEL;
}

void MainRB2_EnableLineFollow(bool enable, int16_t base_rpm_x10) {
  if (enable) {
    if (g_drive_mode != MAINRB2_DRIVE_LINE_FOLLOW) {
      Drive_LineFollowReset(&g_line_state);
    }
    g_line_base_rpm_x10 = base_rpm_x10;
    g_drive_mode = MAINRB2_DRIVE_LINE_FOLLOW;
  } else {
    g_drive_mode = MAINRB2_DRIVE_IDLE;
    g_line_base_rpm_x10 = 0;
    g_cmd_linear_rpm_x10 = 0;
    g_cmd_angular_rpm_x10 = 0;
    Drive_LineFollowReset(&g_line_state);
  }
}

void MainRB2_SetEstop(bool estop) {
  g_estop_latched = estop;
  if (estop) {
    g_cmd_linear_rpm_x10 = 0;
    g_cmd_angular_rpm_x10 = 0;
    Drive_LineFollowReset(&g_line_state);
    Servo_StopAll360();
  }
}

void MainRB2_ClearWheelFaults(void) {
  g_estop_latched = false;
  g_clear_faults_pending = true;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* Half-duplex RS-485 Master on USART2 + PA1 RE/DE for 2 diff wheels (PidBoard
   * IDs 1..2) */
  RS485_Init(&g_rs485, &huart2, RE_DE_GPIO_Port, RE_DE_Pin, RS485_ADDR_MASTER);
  RS485_SetRxCallback(&g_rs485, main_on_rs485_rx, NULL);
  if (RS485_StartRx(&g_rs485) != HAL_OK) {
    Error_Handler();
  }
  Drive_LineFollowReset(&g_line_state);

  /* Autonomous 2x8 dual-end line array on CD4067 (Front = I0..I7, Rear =
   * I8..I15) with 11-LED L1..L11 encoding; preload Sector 11 Flash calibration
   * if present. */
  CD4067_Init(&hadc1, &htim7);
  g_flash_cal_loaded = FlashCal_Load(&g_flash_cal);
  if (g_flash_cal_loaded &&
      ((g_flash_cal.flags & FLASH_CAL_FLAG_LINE_VALID) != 0U)) {
    (void)CD4067_ApplyTwoPointCal(g_flash_cal.cal_min, g_flash_cal.cal_max,
                                  g_flash_cal.line_on_high);
  }
  if (CD4067_StartLine2x8(CD4067_DEFAULT_THRESHOLD) != HAL_OK) {
    Error_Handler();
  }

  /* Onboard switches SW1 (PB12) & SW2 (PA15): external +5V/+3V3 FT-safe inputs
   */
  Buttons_Init(true, true);

  /* Dual-mode servo driver across 6 PWM channels (CH1 = Load Port trigger 0-270
   * deg) */
  if (Servo_Init(&htim3, &htim4, &htim12) != HAL_OK) {
    Error_Handler();
  }
  (void)Servo_Config270(SERVO_CH_1, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config270(SERVO_CH_2, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config270(SERVO_CH_3, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config360(SERVO_CH_4, SERVO_PWM_NEUTRAL_US,
                        SERVO_360_DEFAULT_HALF_SPAN_US,
                        SERVO_DEFAULT_DEADBAND_PCT, false);
  (void)Servo_Config360(SERVO_CH_5, SERVO_PWM_NEUTRAL_US,
                        SERVO_360_DEFAULT_HALF_SPAN_US,
                        SERVO_DEFAULT_DEADBAND_PCT, false);
  (void)Servo_Config360(SERVO_CH_6, SERVO_PWM_NEUTRAL_US,
                        SERVO_360_DEFAULT_HALF_SPAN_US,
                        SERVO_DEFAULT_DEADBAND_PCT, false);
  (void)Servo_SetAngle270(SERVO_CH_1, ITC_SERVO_ARM_STOW_DEG);
  g_prev_arm_deg = ITC_SERVO_ARM_STOW_DEG;

  /* Upload official ST VL53L1X_ULD firmware configuration via XSHUT (LS1..LS4)
   * on I2C2 */
  g_tof_online_mask = ToF_Init(&g_tof, &hi2c2, NULL);

  /* HC-12 RF link (Autonomous Robot = RF_NODE_AUTO):
   * STRICT source whitelist = RF_SRC_MASK(RF_NODE_MANUAL) ONLY.
   * Frames from RF_NODE_TDK (handheld controller) are rejected and discarded.
   */
  RF_HC12_Init(&g_rf, &huart4, NULL, 0U, RF_NODE_AUTO,
               RF_SRC_MASK(RF_NODE_MANUAL));
  (void)RF_HC12_SetAddress(&g_rf, RF_HC12_DEFAULT_CHANNEL,
                           RF_HC12_DEFAULT_POWER, RF_HC12_DEFAULT_BAUD);
  if (RF_HC12_StartRx(&g_rf) != HAL_OK) {
    Error_Handler();
  }

  /* Wait for BNO055 POR (up to 750 ms) and start NDOF fusion on I2C3 (PA8/PC9)
   */
  BNO055_IMU_Init(&g_imu, &hi2c3);
  float saved_heading = (g_flash_cal_loaded &&
                         ((g_flash_cal.flags & FLASH_CAL_FLAG_IMU_VALID) != 0U))
                            ? g_flash_cal.imu_field_offset_deg
                            : 0.0f;
  ITC_AutoInit(&g_auto, ITC_BASE_RUN_RPMX10, saved_heading);
  g_last_imu_tick_ms = HAL_GetTick();
  g_last_rs485_tx_ms = g_last_imu_tick_ms;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();
    RF_Frame_t rx_frame;

    /* 1. Drain UART4 circular DMA RX; strictly accepts RF_NODE_MANUAL, blocks
     * RF_NODE_TDK */
    RF_HC12_ProcessRx(&g_rf, now_ms);
    if (RF_HC12_PopLatestFrame(&g_rf, &rx_frame)) {
      (void)RF_HC12_DecodeManualToAuto(&rx_frame, &g_manual_msg);
    }

    g_manual_link_ok =
        RF_HC12_IsLinkAlive(&g_rf, now_ms, RF_HC12_DEFAULT_TIMEOUT_MS);
    if (!g_manual_link_ok) {
      Servo_StopAll360();
    }

    /* 2. Drain USART2 RS-485 DMA RX telemetry & maintain wheel liveness */
    RS485_ProcessRx(&g_rs485, now_ms);
    RS485_TickMs(&g_rs485, now_ms);
    for (uint8_t i = 0U; i < DRIVE_DIFF_WHEEL_COUNT; i++) {
      g_wheel_alive[i] =
          g_wheel_seen[i] &&
          ((uint32_t)(now_ms - g_wheel_rx_ms[i]) <= RS485_WHEEL_TIMEOUT_MS);
    }

    /* 3. Non-blocking round-robin VL53L1X poll (1 sensor every 5 ms) */
    ToF_Poll(&g_tof, now_ms);

    /* 4. SW1 starts/stops autonomous run; SW2 hold>=800ms sweeps 2-pt cal, tap
     * cycles LED view */
    Buttons_Update(now_ms);
    if (Buttons_ConsumePressed(BTN_SW1)) {
      g_pending_sw1_edge = true;
      MainRB2_ClearWheelFaults();
    }
    if (!g_sw2_long_handled && Buttons_IsPressed(BTN_SW2) &&
        (Buttons_GetHoldMs(BTN_SW2, now_ms) >= SW2_CAL_HOLD_MS)) {
      g_sw2_long_handled = true;
      g_pending_sw2_cal_edge = true;
    }
    if (Buttons_WasReleased(BTN_SW2)) {
      if (g_sw2_long_handled) {
        g_sw2_long_handled = false;
      } else if (g_auto.state == ITC_STATE_CAL_SWEEP) {
        g_pending_sw2_cal_edge = true;
      } else {
        (void)CD4067_CycleLedViewMode();
      }
    }

    /* 5. Snapshot Front (I0..I7) and Rear (I8..I15) line state & drive L1..L11
     */
    g_front_mask = CD4067_GetFrontMask();
    g_rear_mask = CD4067_GetRearMask();
    g_front_error = CD4067_GetFrontError();
    g_rear_error = CD4067_GetRearError();
    CD4067_UpdateLeds(now_ms);

    /* 6. Execute pending BNO08x DCD Save / Clear (triggered by Live
     * Expressions) */
    if ((g_imu_save_cal != 0U) && g_imu.connected && (g_imu.chip_id == 0x85U)) {
      g_imu_save_cal = 0U;
      if (BNO08X_SaveCalibration(&hi2c3, g_imu.i2c_addr)) {
        g_imu_cal_saved_cnt++;
        BNO055_IMU_ZeroYaw(&g_imu);
      }
    }
    if ((g_imu_clear_cal != 0U) && g_imu.connected &&
        (g_imu.chip_id == 0x85U)) {
      g_imu_clear_cal = 0U;
      if (BNO08X_ClearCalibration(&hi2c3, g_imu.i2c_addr, 10000U)) {
        g_imu_cal_cleared_cnt++;
        g_imu.resync_yaw_prev = true;
        g_imu.calib.mag = 0U;
        BNO055_IMU_ZeroYaw(&g_imu);
      }
    }

    /* 7. Poll IMU at 100 Hz (10 ms) IMMEDIATELY BEFORE autonomous
     * FSM/kinematics so heading uses a fresh yaw sample (eliminates 10 ms
     * stale-sample lag) */
    if ((uint32_t)(now_ms - g_last_imu_tick_ms) >= 10U) {
      g_last_imu_tick_ms = now_ms;
      if (g_imu.connected) {
        BNO055_IMU_SetStationaryHint(&g_imu, g_diff_wheels_stopped);
        BNO055_IMU_Update(&g_imu);
      } else {
        static uint32_t s_last_reconnect_ms = 0U;
        if ((uint32_t)(now_ms - s_last_reconnect_ms) >= 500U) {
          s_last_reconnect_ms = now_ms;
          (void)BNO055_IMU_Init(&g_imu, &hi2c3);
        }
      }
    }

    /* 8. 100 Hz (10 ms) ITC 2026 autonomous FSM & RS-485 TDMA poll (IDs 1..2)
     */
    if ((uint32_t)(now_ms - g_last_rs485_tx_ms) >= RS485_TX_PERIOD_MS) {
      g_last_rs485_tx_ms = now_ms;

      uint8_t modes[DRIVE_DIFF_WHEEL_COUNT] = {0};
      int16_t targets[DRIVE_DIFF_WHEEL_COUNT] = {0};

      if (g_drive_mode == MAINRB2_DRIVE_AUTO) {
        uint16_t f_mm = TOF_INVALID_DISTANCE_MM;
        uint16_t r_mm = TOF_INVALID_DISTANCE_MM;
        bool f_blk = ToF_IsDirectionBlocked(&g_tof, true, ITC_TOF_STOP_MM,
                                            now_ms, &f_mm);
        bool r_blk = ToF_IsDirectionBlocked(&g_tof, false, ITC_TOF_STOP_MM,
                                            now_ms, &r_mm);

        ITC_AutoInput_t ain = {0};
        ain.now_ms = now_ms;
        ain.start_toggle_edge = g_pending_sw1_edge;
        ain.cal_toggle_edge = g_pending_sw2_cal_edge;
        ain.estop_req = g_estop_latched;
        ain.cal_done = CD4067_IsCalibrated();
        ain.front_mask = g_front_mask;
        ain.rear_mask = g_rear_mask;
        ain.front_count = cd4067_data.front_count;
        ain.rear_count = cd4067_data.rear_count;
        ain.front_centroid_x10 = CD4067_GetFrontCentroidX10();
        ain.rear_centroid_x10 = CD4067_GetRearCentroidX10();
        ain.imu_ok = g_imu.connected && g_imu.healthy;
        ain.imu_yaw_deg = g_imu.yaw;
        ain.front_blocked = f_blk;
        ain.rear_blocked = r_blk;
        ain.front_dist_mm = f_mm;
        ain.rear_dist_mm = r_mm;
        ain.manual_rf_alive = g_manual_link_ok;
        ain.manual_rf_hold_req =
            g_manual_link_ok &&
            ((g_manual_msg.flags & RF_MANUAL_FLAG_SW2) != 0U);
        g_pending_sw1_edge = false;
        g_pending_sw2_cal_edge = false;

        ITC_AutoStep(&g_auto, &ain, &g_auto_out);

        if (g_auto_out.req_begin_two_point_cal) {
          CD4067_BeginTwoPointCal();
        }
        if (g_auto_out.req_finish_two_point_cal) {
          if (CD4067_FinishTwoPointCal(CD4067_POLARITY_AUTO)) {
            if (!g_flash_cal_loaded) {
              FlashCal_InitDefaults(&g_flash_cal);
            }
            g_flash_cal.seq++;
            g_flash_cal.flags |= FLASH_CAL_FLAG_LINE_VALID;
            CD4067_ExportTwoPointCal(g_flash_cal.cal_min, g_flash_cal.cal_max,
                                     &g_flash_cal.line_on_high);
            if (g_imu.connected && g_imu.healthy) {
              BNO055_IMU_ZeroYaw(&g_imu);
              g_flash_cal.imu_field_offset_deg = g_imu.yaw;
              g_flash_cal.flags |= FLASH_CAL_FLAG_IMU_VALID;
              if (g_imu.chip_id == 0x85U) {
                (void)BNO08X_SaveCalibration(&hi2c3, g_imu.i2c_addr);
              }
            }
            g_flash_cal_loaded = FlashCal_Save(&g_flash_cal);
          }
        }

        if (g_auto_out.arm_deg != g_prev_arm_deg) {
          (void)Servo_SetAngle270(SERVO_CH_1, g_auto_out.arm_deg);
          g_prev_arm_deg = g_auto_out.arm_deg;
        }

        modes[0] = g_auto_out.wheel_modes[0];
        modes[1] = g_auto_out.wheel_modes[1];
        targets[0] = g_auto_out.wheel_targets[0];
        targets[1] = g_auto_out.wheel_targets[1];
      } else if (g_drive_mode == MAINRB2_DRIVE_LINE_FOLLOW) {
        Drive_LineFollowDualBarStep(
            &g_line_state, &g_line_gains, CD4067_IsCalibrated(),
            g_line_base_rpm_x10, g_front_mask, cd4067_data.front_count,
            CD4067_GetFrontCentroidX10(), g_rear_mask, cd4067_data.rear_count,
            CD4067_GetRearCentroidX10(), now_ms, g_estop_latched, modes,
            targets);
      } else if (g_drive_mode == MAINRB2_DRIVE_MANUAL_VEL) {
        Drive_DiffCompute(g_cmd_linear_rpm_x10, g_cmd_angular_rpm_x10,
                          DRIVE_DEFAULT_MAX_RPMX10, g_estop_latched, modes,
                          targets);
      } else {
        Drive_DiffCompute(0, 0, DRIVE_DEFAULT_MAX_RPMX10, g_estop_latched,
                          modes, targets);
      }

      RS485_MultiCmd_t mcmd = {0};
      mcmd.num_wheels = DRIVE_DIFF_WHEEL_COUNT;
      mcmd.poll_id = (uint8_t)(g_rs485_poll_idx + 1U);
      g_rs485_poll_idx =
          (uint8_t)((g_rs485_poll_idx + 1U) % DRIVE_DIFF_WHEEL_COUNT);
      mcmd.flags = (g_estop_latched ? RS485_CMD_FLAG_ESTOP : 0U) |
                   (g_clear_faults_pending ? RS485_CMD_FLAG_CLEAR_FAULTS : 0U);
      for (uint8_t i = 0U; i < DRIVE_DIFF_WHEEL_COUNT; i++) {
        mcmd.wheels[i].mode = modes[i];
        mcmd.wheels[i].target = targets[i];
      }

      g_diff_wheels_stopped = (targets[0] == 0) && (targets[1] == 0);

      /* Auto-broadcast anti-stick-slip inner velocity PID gains to both
       * PidBoards over RS-485:
       *  - Immediately whenever any PidBoard powers on / comes online
       *  - Immediately when g_pidboard_vel_kp_m / ki_m change in Live
       * Expressions
       *  - Periodically every 3s while stopped */
      static bool s_prev_wheel_alive[DRIVE_DIFF_WHEEL_COUNT] = {false};
      static uint32_t s_pid_last_sync_ms = 0U;
      static uint8_t s_pid_cfg_stage = 0U;
      static int32_t s_sent_kp_m = -1;
      static int32_t s_sent_ki_m = -1;
      bool wheel_came_online = false;
      for (uint8_t i = 0U; i < DRIVE_DIFF_WHEEL_COUNT; i++) {
        if (g_wheel_alive[i] && !s_prev_wheel_alive[i]) {
          wheel_came_online = true;
        }
        s_prev_wheel_alive[i] = g_wheel_alive[i];
      }
      bool idle_refresh_due =
          g_diff_wheels_stopped &&
          ((uint32_t)(now_ms - s_pid_last_sync_ms) >= 3000U);
      if ((s_pid_cfg_stage == 0U) && (wheel_came_online || idle_refresh_due ||
                                      (g_pidboard_vel_kp_m != s_sent_kp_m) ||
                                      (g_pidboard_vel_ki_m != s_sent_ki_m))) {
        s_pid_cfg_stage = 2U;
        s_pid_last_sync_ms = now_ms;
      }

      if (s_pid_cfg_stage == 2U) {
        RS485_CfgCmd_t cfg = {0};
        cfg.subcmd = (uint8_t)RS485_CFG_SET_VEL_KP_M;
        cfg.value = g_pidboard_vel_kp_m;
        if (RS485_SendCfgCmd(&g_rs485, RS485_ADDR_BCAST, &cfg, now_ms) ==
            HAL_OK) {
          s_sent_kp_m = g_pidboard_vel_kp_m;
          s_pid_cfg_stage = 1U;
        }
      } else if (s_pid_cfg_stage == 1U) {
        RS485_CfgCmd_t cfg = {0};
        cfg.subcmd = (uint8_t)RS485_CFG_SET_VEL_KI_M;
        cfg.value = g_pidboard_vel_ki_m;
        if (RS485_SendCfgCmd(&g_rs485, RS485_ADDR_BCAST, &cfg, now_ms) ==
            HAL_OK) {
          s_sent_ki_m = g_pidboard_vel_ki_m;
          s_pid_cfg_stage = 0U;
        }
      } else if (RS485_SendMultiCmd(&g_rs485, &mcmd, now_ms) == HAL_OK) {
        g_clear_faults_pending = false;
        g_last_rs485_tx_ms = now_ms;
      }
    }

    /* 9. Update unified debug struct (viewable in CubeIDE Live Expressions
     * without overhead) */
    g_debug.tdk.connected = g_manual_link_ok ? 1U : 0U;
    g_debug.rs485.tx_cnt = g_rs485.tx_seq;
    g_debug.rs485.last_tx_ms = g_last_rs485_tx_ms;

    g_debug.imu.connected = g_imu.connected ? 1U : 0U;
    g_debug.imu.yaw = g_imu.yaw;
    g_debug.imu.roll = g_imu.roll;
    g_debug.imu.pitch = g_imu.pitch;

    for (uint8_t i = 0U; i < 16U; i++) {
      g_debug.line.adc_raw[i] = cd4067_data.raw[i];
    }
    g_debug.line.active_mask = cd4067_data.active_mask;
    g_debug.line.active_count = cd4067_data.active_count;
    g_debug.line.line_error = cd4067_data.line_error;

    g_debug.board.sw1 = Buttons_IsPressed(BTN_SW1) ? 1U : 0U;
    g_debug.board.sw2 = Buttons_IsPressed(BTN_SW2) ? 1U : 0U;
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void) {

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L3_Pin | L2_Pin | L1_Pin | MH_RST_Pin | L10_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    s3_Pin | s2_Pin | s1_Pin | s0_Pin | MH_DC_Pin | LS3_Pin |
                        LS2_Pin | LS1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MH_CS_GPIO_Port, MH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L11_Pin | L8_Pin | L7_Pin | L6_Pin | L5_Pin | L4_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LS4_GPIO_Port, LS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : L3_Pin L2_Pin L1_Pin MH_RST_Pin
                           LS3_Pin LS2_Pin LS1_Pin L10_Pin */
  GPIO_InitStruct.Pin = L3_Pin | L2_Pin | L1_Pin | MH_RST_Pin | LS3_Pin |
                        LS2_Pin | LS1_Pin | L10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : s3_Pin s2_Pin s1_Pin s0_Pin */
  GPIO_InitStruct.Pin = s3_Pin | s2_Pin | s1_Pin | s0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RE_DE_Pin MH_CS_Pin */
  GPIO_InitStruct.Pin = RE_DE_Pin | MH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MH_DC_Pin */
  GPIO_InitStruct.Pin = MH_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MH_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L11_Pin LS4_Pin L8_Pin L7_Pin
                           L6_Pin L5_Pin L4_Pin */
  GPIO_InitStruct.Pin =
      L11_Pin | LS4_Pin | L8_Pin | L7_Pin | L6_Pin | L5_Pin | L4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L9_Pin */
  GPIO_InitStruct.Pin = L9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L9_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  CD4067_TIM_PeriodElapsedCallback(htim);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  CD4067_ADC_ConvCpltCallback(hadc);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  RS485_TxCpltCallback(&g_rs485, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  RF_HC12_UartErrorCallback(&g_rf, huart);
  RS485_UartErrorCallback(&g_rs485, huart);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
