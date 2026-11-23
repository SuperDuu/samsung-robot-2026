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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "bno055_stm32.h"
#include "buttons.h"
#include "cd4067.h"
#include "drive_kinematics.h"
#include "rf_hc12.h"
#include "rs485_bus.h"
#include "servo.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_TX_PERIOD_MS 10U
#define RS485_WHEEL_TIMEOUT_MS 250U
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

/* USER CODE BEGIN PV */
volatile uint8_t g_i2c3_scan_found[16] = {0};
volatile uint8_t g_i2c3_scan_count = 0;
volatile uint8_t g_i2c2_scan_found[16] = {0};
volatile uint8_t g_i2c2_scan_count = 0;

// static void main_scan_i2c(I2C_HandleTypeDef *hi2c, volatile uint8_t *found,
// volatile uint8_t *count)
//{
//   *count = 0U;
//   for (uint8_t a = 1U; a < 128U; a++)
//   {
//     uint8_t reg = 0U;
//     if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(a << 1U), &reg, 1U, 10U) ==
//     HAL_OK)
//     {
//       if (*count < 16U)
//       {
//         found[*count] = a;
//         (*count)++;
//       }
//     }
//     HAL_Delay(1);
//   }
// }

BNO055_IMU_t g_imu = {0};
RF_HC12_t g_rf = {0};
RF_PayloadTdk_t g_tdk_cmd = {0};
bool g_tdk_link_ok = false;
Robot_Debug_t g_debug = {0};
volatile uint8_t g_servo_test_cmd =
    0U; /* Live Expression: 0 = Stop, 1 = Forward, 2 = Reverse */
volatile uint16_t g_servo_test_pulse =
    0U; /* Direct pulse test in us (e.g. 500..2500, 0 = disabled) */
volatile uint8_t g_servo_anti_stall_kick =
    1U; /* 1 = Periodic 40ms neutral pulse to reset digital servo stall watchdog
         */
volatile int32_t g_pidboard_vel_kp_m =
    1500; /* Live-tunable PidBoard inner vel Kp (milli-units), auto-synced via
             RS-485 */
volatile int32_t g_pidboard_vel_ki_m =
    2000; /* Live-tunable PidBoard inner vel Ki (milli-units), eliminates
             stick-slip windup */
RS485_Bus_t g_rs485 = {0};
RS485_Telem_t g_wheel_telem[DRIVE_OMNI_WHEEL_COUNT] = {0};
bool g_wheel_alive[DRIVE_OMNI_WHEEL_COUNT] = {false};
static uint32_t g_wheel_rx_ms[DRIVE_OMNI_WHEEL_COUNT] = {0};
static bool g_wheel_seen[DRIVE_OMNI_WHEEL_COUNT] = {false};
static uint8_t g_rs485_poll_idx = 0U;
static uint32_t g_last_rs485_tx_ms = 0U;
Drive_OmniCtrl_t g_omni_ctrl = {0};
typedef struct {
  uint32_t joy_wake_cnt;
  uint32_t imu_wake_cnt;
  uint32_t rf_drop_cnt;
  uint32_t imu_err_cnt;
  uint8_t last_wake_joy0;
  uint8_t last_wake_joy1;
  uint8_t last_wake_btn0;
  uint8_t last_wake_btn1;
  float last_wake_yaw;
  float last_wake_target_hdg;
  float last_wake_omega_pct;
  int16_t last_wake_target0;
} Twitch_Diag_t;
volatile Twitch_Diag_t g_twitch_dbg = {0};
volatile uint8_t g_imu_save_cal =
    0U; /* Set 1 (or hold SELECT >=1s) -> Save BNO085 DCD to Flash */
volatile uint8_t g_imu_clear_cal =
    0U; /* Set 1 (or hold START >=2s)  -> Clear BNO085 DCD & reset */
volatile uint32_t g_imu_cal_saved_cnt =
    0U; /* Increments on successful Save DCD */
volatile uint32_t g_imu_cal_cleared_cnt =
    0U; /* Increments on successful Clear DCD & Reset */
static float g_yaw_offset_deg = 0.0f;
static uint8_t g_prev_select_btn = 0U;
static uint8_t g_prev_l3_btn = 0U;
static uint8_t g_prev_r1_btn = 0U;
static bool g_clamp_opened = false;
static bool g_estop_latched = false;
static bool g_clear_faults_pending = false;
static uint32_t g_last_imu_tick_ms = 0U;
static uint32_t g_last_auto_tx_ms = 0U;
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
        (telem.node_id <= DRIVE_OMNI_WHEEL_COUNT)) {
      uint8_t idx = (uint8_t)(telem.node_id - 1U);
      g_wheel_telem[idx] = telem;
      g_wheel_rx_ms[idx] = bus->last_rx_tick_ms;
      g_wheel_seen[idx] = true;
    }
  }
}

void MainSamsungRB_SetEstop(bool estop) {
  g_estop_latched = estop;
  if (estop) {
    Drive_OmniReset(&g_omni_ctrl.slew);
    Servo_StopAll360();
  }
}

void MainSamsungRB_ClearWheelFaults(void) {
  g_estop_latched = false;
  g_clear_faults_pending = true;
}

static volatile int16_t s_test_motor_rpm = 0;
static volatile uint32_t s_test_motor_until_ms = 0;
static volatile float s_test_servo_speed = 0.0f;
static volatile uint32_t s_test_servo_until_ms = 0;

void Main_OnUsbCdcRx(const uint8_t *buf, uint32_t len) {
  if ((buf == NULL) || (len == 0U)) {
    return;
  }
  char c = (char)buf[0];
  uint32_t now = HAL_GetTick();
  if (c == '1') {
    s_test_motor_rpm = 500;              /* +50.0 RPM */
    s_test_motor_until_ms = now + 4000U; /* Run for 4 seconds */
  } else if (c == '2') {
    s_test_motor_rpm = -500; /* -50.0 RPM */
    s_test_motor_until_ms = now + 4000U;
  } else if (c == '3') {
    s_test_motor_rpm = 1000; /* +100.0 RPM */
    s_test_motor_until_ms = now + 4000U;
  } else if (c == '0' || c == ' ') {
    s_test_motor_rpm = 0;
    s_test_motor_until_ms = 0U;
    s_test_servo_speed = 0.0f;
    s_test_servo_until_ms = 0U;
    Servo_Stop(SERVO_CH_4);
  } else if (c == 'u' || c == 'U') {
    s_test_servo_speed = 60.0f;
    s_test_servo_until_ms = now + 4000U; /* Spin forward (+60%) for 4 seconds */
    Servo_SetSpeed360(SERVO_CH_4, 60.0f);
    char msg[] = "\r\n[SERVO CH4 TIM12_CH2] FORWARD +60% (4s)\r\n";
    CDC_Transmit_FS((uint8_t *)msg, sizeof(msg) - 1);
  } else if (c == 'j' || c == 'J') {
    s_test_servo_speed = -60.0f;
    s_test_servo_until_ms = now + 4000U; /* Spin reverse (-60%) for 4 seconds */
    Servo_SetSpeed360(SERVO_CH_4, -60.0f);
    char msg[] = "\r\n[SERVO CH4 TIM12_CH2] REVERSE -60% (4s)\r\n";
    CDC_Transmit_FS((uint8_t *)msg, sizeof(msg) - 1);
  } else if (c == 'k' || c == 'K') {
    s_test_servo_speed = 0.0f;
    s_test_servo_until_ms = 0U;
    Servo_Stop(SERVO_CH_4);
    char msg[] = "\r\n[SERVO CH4 TIM12_CH2] STOPPED (0%)\r\n";
    CDC_Transmit_FS((uint8_t *)msg, sizeof(msg) - 1);
  } else if (c == 'c' || c == 'C') {
    (void)RF_HC12_SetAddress(&g_rf, RF_HC12_DEFAULT_CHANNEL,
                             RF_HC12_DEFAULT_POWER, RF_HC12_DEFAULT_BAUD);
    (void)RF_HC12_StartRx(&g_rf);
  } else if (c == 'a' || c == 'A') {
    char msg[160];
    int len =
        snprintf(msg, sizeof(msg),
                 "\r\n[ADC 0..7] %u %u %u %u %u %u %u %u\r\n[ADC 8..15] %u %u "
                 "%u %u %u %u %u %u\r\n",
                 cd4067_data.raw[0], cd4067_data.raw[1], cd4067_data.raw[2],
                 cd4067_data.raw[3], cd4067_data.raw[4], cd4067_data.raw[5],
                 cd4067_data.raw[6], cd4067_data.raw[7], cd4067_data.raw[8],
                 cd4067_data.raw[9], cd4067_data.raw[10], cd4067_data.raw[11],
                 cd4067_data.raw[12], cd4067_data.raw[13], cd4067_data.raw[14],
                 cd4067_data.raw[15]);
    if (len > 0) {
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
    }
  } else if (c == 'd' || c == 'D') {
    char msg[140];
    int len = snprintf(
        msg, sizeof(msg),
        "\r\n[DEBUG] TDK:%u YAW:%.1f M1:%d M2:%d M3:%d M4:%d RS485_CNT:%u\r\n",
        g_debug.tdk.connected, g_debug.imu.yaw, g_debug.rs485.motor_rpm[0],
        g_debug.rs485.motor_rpm[1], g_debug.rs485.motor_rpm[2],
        g_debug.rs485.motor_rpm[3], g_debug.rs485.tx_cnt);
    if (len > 0) {
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
    }
  }
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
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  if (Servo_Init(&htim3, &htim4, &htim12) != HAL_OK) {
    Error_Handler();
  }
  /* Actuator channel configuration:
   *  - SERVO_360_CH1 (CH4 / TIM12_CH2 / PB15 / J36): Continuous 360 #1 (Right Stick UP/DOWN)
   *  - SERVO_360_CH2 (CH5 / TIM4_CH3  / PB8  / J37): Continuous 360 #2 (Right Stick LEFT/RIGHT)
   *  - SERVO_CLAMP_CH (CH6 / TIM4_CH4 / PB9  / J38): Positional 180 Clamp (R1 toggle 15 <-> 0 deg)
   *  - CH1/CH2/CH3: Additional 180/270 ports available
   */
  (void)Servo_Config180(SERVO_CLAMP_CH, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config180(SERVO_CH_1, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config180(SERVO_CH_2, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config270(SERVO_CH_3, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US, false);
  (void)Servo_Config360(SERVO_360_CH1, SERVO_PWM_NEUTRAL_US,
                        SERVO_360_DEFAULT_HALF_SPAN_US,
                        SERVO_DEFAULT_DEADBAND_PCT, false);
  (void)Servo_Config360(SERVO_360_CH2, SERVO_PWM_NEUTRAL_US,
                        SERVO_360_DEFAULT_HALF_SPAN_US,
                        SERVO_DEFAULT_DEADBAND_PCT, false);

  /* Initial clamp state: 0 deg (closed per user spec) */
  (void)Servo_SetAngle180(SERVO_CLAMP_CH, SERVO_CLAMP_DEFAULT_CLOSE_DEG);
  (void)Servo_SetAngle180(SERVO_CH_1, SERVO_CLAMP_DEFAULT_CLOSE_DEG);
  (void)Servo_SetAngle180(SERVO_CH_2, SERVO_CLAMP_DEFAULT_CLOSE_DEG);
  g_debug.clamp.closed = 1U;
  g_debug.clamp.angle_deg = SERVO_CLAMP_DEFAULT_CLOSE_DEG;
  Servo_ChannelState_t clamp_init_st;
  if (Servo_GetState(SERVO_CLAMP_CH, &clamp_init_st)) {
    g_debug.clamp.pulse_us = clamp_init_st.current_us;
  }
  g_debug.servo.speed_pct = 60;

  /* Half-duplex RS-485 Master on USART2 + PA1 RE/DE for 4 omni PidBoards
   * (IDs 1..4) */
  RS485_Init(&g_rs485, &huart2, RE_DE_GPIO_Port, RE_DE_Pin, RS485_ADDR_MASTER);
  RS485_SetRxCallback(&g_rs485, main_on_rs485_rx, NULL);
  if (RS485_StartRx(&g_rs485) != HAL_OK) {
    Error_Handler();
  }
  Drive_OmniCtrlInit(&g_omni_ctrl, true);
  g_omni_ctrl.pure_field_centric =
      true; /* True Field-Centric: CIRCLE/CROSS/SQUARE keep field axis fixed */

  /* Scan all 16 limit switch channels on CD4067 (I0..I15) with Schmitt + 8 ms
   * debounce */
  CD4067_Init(&hadc1, &htim7);
  if (CD4067_StartLimitSwitches(0xFFFFU) != HAL_OK) {
    Error_Handler();
  }

  /* Onboard switches SW1 (PB12) & SW2 (PA15): external +5V/+3V3 FT-safe inputs
   */
  Buttons_Init(true, true);

  /* Dual-mode servo driver across 6 PWM channels (CH1..3 = 0-270 deg, CH4..6 =
   * 360 continuous) */

  /* HC-12 RF link (Manual Robot = RF_NODE_MANUAL, accepts ONLY RF_NODE_TDK,
   * relays to RF_NODE_AUTO) */
  RF_HC12_Init(&g_rf, &huart4, NULL, 0U, RF_NODE_MANUAL,
               RF_SRC_MASK(RF_NODE_TDK));
  (void)RF_HC12_SetAddress(&g_rf, RF_HC12_DEFAULT_CHANNEL,
                           RF_HC12_DEFAULT_POWER, RF_HC12_DEFAULT_BAUD);
  if (RF_HC12_StartRx(&g_rf) != HAL_OK) {
    Error_Handler();
  }

  /* Wait for IMU POR (BNO080/085 up to 500ms, BNO055 up to 1200ms) and
   * initialize in 6-DOF Accel+Gyro mode (BNO08x Game Rotation Vector 0x08 /
   * BNO055 IMU 0x08), immune to motor/steel magnetic interference */
  BNO055_IMU_Init(&g_imu, &hi2c3);
  if (g_imu.connected) {
    /* Verified from PS2_STM32.c (vx=+RIGHT, vy=-FWD) + BNO085 CCW+ yaw:
     * theta_rad = +effective_deg (field_centric_invert = false) matches all 4
     * quadrants; L1/R1 rotation buttons invert for CCW+ BNO085 so L1=Left,
     * R1=Right. */
    g_omni_ctrl.field_centric_invert = false;
    g_omni_ctrl.rot_btn_invert = (g_imu.chip_id == 0x85U);
  }
  g_last_imu_tick_ms = HAL_GetTick();
  g_last_auto_tx_ms = g_last_imu_tick_ms;
  g_last_rs485_tx_ms = g_last_imu_tick_ms;
  bool omni_wheels_stopped = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();
    RF_Frame_t rx_frame;
    bool forward_now = false;

    /* 1. Drain UART4 circular DMA RX & enforce strict RF_NODE_TDK source filter
     */
    RF_HC12_ProcessRx(&g_rf, now_ms);
    if (RF_HC12_PopLatestFrame(&g_rf, &rx_frame)) {
      if (RF_HC12_DecodeTdkCmd(&rx_frame, &g_tdk_cmd)) {
        forward_now = true; /* Reply inside post-tdk ~15 ms quiet slot before
                               tdk's next frame */
        (void)forward_now;
      }
    }

    g_tdk_link_ok =
        RF_HC12_IsLinkAlive(&g_rf, now_ms, RF_HC12_DEFAULT_TIMEOUT_MS);
    bool ps2_valid =
        g_tdk_link_ok && ((g_tdk_cmd.ext_sw & RF_TDK_PS2_VALID_BIT) != 0U);
    if (!g_tdk_link_ok && (g_servo_test_cmd == 0U) &&
        (g_debug.servo.cmd == 0U)) {
      if (now_ms >= s_test_servo_until_ms) {
        Servo_StopAll360();
      }
    }

    /* 2. Drain USART2 RS-485 DMA RX telemetry & run TX watchdog / wheel
     * liveness */
    RS485_ProcessRx(&g_rs485, now_ms);
    RS485_TickMs(&g_rs485, now_ms);
    for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
      g_wheel_alive[i] =
          g_wheel_seen[i] &&
          ((uint32_t)(now_ms - g_wheel_rx_ms[i]) <= RS485_WHEEL_TIMEOUT_MS);
    }

    /* 3. Read debounced SW1 / SW2 and mirror limit switch mask onto L1..L11 */
    Buttons_Update(now_ms);
    if (Buttons_ConsumePressed(BTN_SW1)) {
      BNO055_IMU_ZeroYaw(&g_imu);
      g_yaw_offset_deg = 0.0f;
      Drive_OmniCtrlZeroHeading(&g_omni_ctrl);
      MainSamsungRB_ClearWheelFaults();
    }
    if (Buttons_ConsumePressed(BTN_SW2)) {
      (void)CD4067_CycleLedViewMode();
    }

    /* Non-blocking IMU calibration visual feedback on onboard L1..L11 LED bar:
     *   0 = normal limit-switch display (CD4067_UpdateLeds)
     *   1 = CALIBRATING: blink all 11 LEDs at 400 ms period (200 ms ON / 200 ms
     * OFF) until BNO085 reaches calib.mag >= 2 (auto-saves Flash -> chasing
     * LEDs) 2 = CALIB SUCCESS / SAVED: chasing LED bar L1 -> L11 (2 passes, 60
     * ms/LED) */
    static uint8_t s_imu_cal_led_state = 0U;
    static uint32_t s_imu_cal_led_start_ms = 0U;
    static uint32_t s_imu_cal_ok_since_ms = 0U;

    /* PS2 L3 zeros field heading; Select clears PidBoard faults; ext_sw selects
     * Blue/Red field axis */
    if (ps2_valid) {
      uint8_t raw_btn0 = g_tdk_cmd.ps2_btn[0];
      uint8_t clean_btn0 = (raw_btn0 < 200U) ? raw_btn0 : 0U;
      uint8_t l3_now =
          ((clean_btn0 & DRIVE_PS2_B0_L3) != 0U) ? 1U : 0U;
      uint8_t select_now =
          ((clean_btn0 & DRIVE_PS2_B0_SELECT) != 0U) ? 1U : 0U;
      uint8_t r1_now =
          ((g_tdk_cmd.ps2_btn[1] & DRIVE_PS2_B1_R1) != 0U) ? 1U : 0U;
      Drive_OmniCtrlSetBlueSide(&g_omni_ctrl,
                                (g_tdk_cmd.ext_sw & RF_TDK_EXT_SW_BIT) == 0U);
      if ((l3_now != 0U) && (g_prev_l3_btn == 0U) && g_imu.connected) {
        BNO055_IMU_ZeroYaw(&g_imu);
        g_yaw_offset_deg = 0.0f;
        Drive_OmniCtrlZeroHeading(&g_omni_ctrl);
      }
      if ((select_now != 0U) && (g_prev_select_btn == 0U)) {
        MainSamsungRB_ClearWheelFaults();
      }
      if ((r1_now != 0U) && (g_prev_r1_btn == 0U)) {
        /* R1 toggle: initial 0 deg (closed), 1st press opens to 40 deg, 2nd press closes to 0 deg */
        g_clamp_opened = !g_clamp_opened;
        float clamp_angle = g_clamp_opened ? SERVO_CLAMP_DEFAULT_OPEN_DEG
                                           : SERVO_CLAMP_DEFAULT_CLOSE_DEG;
        (void)Servo_SetAngle180(SERVO_CLAMP_CH, clamp_angle);
        (void)Servo_SetAngle180(SERVO_CH_1, clamp_angle);
        (void)Servo_SetAngle180(SERVO_CH_2, clamp_angle);
        g_debug.clamp.closed = g_clamp_opened ? 0U : 1U;
        g_debug.clamp.angle_deg = clamp_angle;
      }
      g_prev_l3_btn = l3_now;
      g_prev_select_btn = select_now;
      g_prev_r1_btn = r1_now;

      /* BNO085 9-DOF Flash calibration buttons (tx_buf[4] = ps2_btn[0]):
       *  - Hold SELECT (1) alone >= 1.0s -> Save DCD into BNO085 internal Flash
       *  - Hold START  (8) alone >= 2.0s -> Clear DCD & reset BNO085 to
       * recalibrate */
      static uint32_t s_sel_hold_start_ms = 0U;
      static bool s_sel_hold_fired = false;
      bool sel_only = (clean_btn0 == DRIVE_PS2_B0_SELECT) &&
                      (g_tdk_cmd.ps2_btn[1] == 0U);
      if (sel_only) {
        if (s_sel_hold_start_ms == 0U) {
          s_sel_hold_start_ms = now_ms;
        } else if (!s_sel_hold_fired &&
                   ((uint32_t)(now_ms - s_sel_hold_start_ms) >= 1000U)) {
          s_sel_hold_fired = true;
          g_imu_save_cal = 1U;
        }
      } else {
        s_sel_hold_start_ms = 0U;
        s_sel_hold_fired = false;
      }

      static uint32_t s_start_hold_start_ms = 0U;
      static bool s_start_hold_fired = false;
      bool start_only = (clean_btn0 == DRIVE_PS2_B0_START) &&
                        (g_tdk_cmd.ps2_btn[1] == 0U);
      if (start_only) {
        if (s_start_hold_start_ms == 0U) {
          s_start_hold_start_ms = now_ms;
        } else if (!s_start_hold_fired &&
                   ((uint32_t)(now_ms - s_start_hold_start_ms) >= 2000U)) {
          s_start_hold_fired = true;
          g_imu_clear_cal = 1U;
        }
      } else {
        s_start_hold_start_ms = 0U;
        s_start_hold_fired = false;
      }
    } else {
      g_prev_l3_btn = 0U;
      g_prev_select_btn = 0U;
      g_prev_r1_btn = 0U;
    }

    /* Execute pending BNO08x DCD Save / Clear (triggered by SELECT/START hold
     * or Live Expressions) */
    if ((g_imu_save_cal != 0U) && g_imu.connected && (g_imu.chip_id == 0x85U)) {
      g_imu_save_cal = 0U;
      if (BNO08X_SaveCalibration(&hi2c3, g_imu.i2c_addr)) {
        g_imu_cal_saved_cnt++;
        BNO055_IMU_ZeroYaw(&g_imu);
        g_yaw_offset_deg = 0.0f;
        Drive_OmniCtrlZeroHeading(&g_omni_ctrl);
        now_ms = HAL_GetTick();   /* Refresh after blocking HAL_Delay(20) */
        s_imu_cal_led_state = 2U; /* Trigger chasing LEDs L1..L11 */
        s_imu_cal_led_start_ms = now_ms;
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
        g_yaw_offset_deg = 0.0f;
        g_omni_ctrl.field_centric = true;
        g_omni_ctrl.heading_hold = true;
        Drive_OmniCtrlZeroHeading(&g_omni_ctrl);
        now_ms = HAL_GetTick(); /* Refresh after blocking reboot HAL_Delay */
        s_imu_cal_led_state =
            1U; /* Blink all 11 LEDs at 400 ms while calibrating */
        s_imu_cal_led_start_ms = now_ms;
        s_imu_cal_ok_since_ms = 0U;
      }
    }

    /* Render L1..L11 onboard LED bar (non-blocking) */
    if (s_imu_cal_led_state == 1U) {
      int32_t dt_signed = (int32_t)(now_ms - s_imu_cal_led_start_ms);
      uint32_t elapsed_ms = (dt_signed > 0) ? (uint32_t)dt_signed : 0U;
      bool blink_on =
          (((elapsed_ms / 200U) & 1U) == 0U); /* 400 ms full blink period */
      CD4067_SetAllLineLeds(blink_on);

      /* 6-DOF Game Rotation Vector (0x08): keep robot STILL on flat floor!
       * Auto-saves Gyro+Accel DCD to Flash once calib.gyro >= 2 for 300 ms
       * (after >= 2.0s stillness) or after 4.5s stillness fallback. */
      bool cal_ready = g_imu.connected &&
                       (((elapsed_ms >= 2000U) && (g_imu.calib.gyro >= 2U)) ||
                        (elapsed_ms >= 4500U));
      if (cal_ready) {
        if (s_imu_cal_ok_since_ms == 0U) {
          s_imu_cal_ok_since_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_imu_cal_ok_since_ms) >= 300U) {
          g_imu_save_cal = 1U;
        }
      } else {
        s_imu_cal_ok_since_ms = 0U;
      }
    } else if (s_imu_cal_led_state == 2U) {
      int32_t dt_signed = (int32_t)(now_ms - s_imu_cal_led_start_ms);
      uint32_t elapsed_ms = (dt_signed > 0) ? (uint32_t)dt_signed : 0U;
      uint32_t step = elapsed_ms / 75U; /* 75 ms/LED x 2 passes = 1.65 s */
      if (step >= (CD4067_NUM_LINE_SENSORS * 2U)) {
        s_imu_cal_led_state = 0U;
        CD4067_UpdateLeds(now_ms);
      } else {
        uint8_t active_led = (uint8_t)(step % CD4067_NUM_LINE_SENSORS);
        for (uint8_t i = 0U; i < CD4067_NUM_LINE_SENSORS; i++) {
          CD4067_SetLineLed(i, (i == active_led));
        }
      }
    } else {
      CD4067_UpdateLeds(now_ms);
    }

    /* Servo CH4 (TIM12_CH2, PB15) control priority:
     * 1. Live Expression debug command (g_servo_test_cmd / g_debug.servo.cmd):
     *    1 = Forward (+speed%), 2 = Reverse (-speed%), 0 = Stop / Manual
     * 2. PS2 buttons: Triangle = Forward, Cross = Reverse
     * 3. USB CDC test timer (s_test_servo_until_ms)
     * 4. Default: Stop (1500 us)
     */
    /* Synchronize Live Expression control variables bidirectionally */
    static uint8_t s_prev_debug_cmd = 0U;
    if (g_debug.servo.cmd != s_prev_debug_cmd) {
      g_servo_test_cmd = g_debug.servo.cmd;
      s_prev_debug_cmd = g_debug.servo.cmd;
    } else if (g_servo_test_cmd != s_prev_debug_cmd) {
      g_debug.servo.cmd = g_servo_test_cmd;
      s_prev_debug_cmd = g_servo_test_cmd;
    }
    uint8_t live_cmd = g_servo_test_cmd;

    /* Digital Servo Anti-Stall Keep-Alive kick (every 1200ms, pulse 1500us for
     * 40ms): Digital servos with MCUs detect zero change in potentiometer ADC
     * under load and shut down via Stall Protection after ~2-3 seconds ("vài
     * vòng"). Temporarily pulsing neutral (1500us) resets the error and the
     * stall watchdog without overcoming mechanical rotor inertia. */
    static uint32_t s_last_stall_kick_ms = 0U;
    bool stall_kick_active = false;
    if ((g_servo_anti_stall_kick != 0U) &&
        ((live_cmd == 1U) || (live_cmd == 2U))) {
      uint32_t cycle_ms = (uint32_t)(now_ms - s_last_stall_kick_ms);
      if (cycle_ms >= 1200U) {
        if (cycle_ms < 1240U) {
          stall_kick_active = true;
        } else {
          s_last_stall_kick_ms = now_ms;
        }
      }
    } else {
      s_last_stall_kick_ms = now_ms;
    }

    float active_speed =
        (g_debug.servo.speed_pct != 0) ? (float)g_debug.servo.speed_pct : 70.0f;

    uint8_t servo_cmd = (g_tdk_link_ok && !g_estop_latched) ? g_tdk_cmd.ps2_btn[0] : 0U;

    if (g_servo_test_pulse != 0U) {
      /* Direct microsecond pulse test (e.g. 1000..2000 us) to calibrate modded
       * 360 servos */
      __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, g_servo_test_pulse);
    } else if (stall_kick_active) {
      Servo_Stop(SERVO_CH_4);
      Servo_Stop(SERVO_CH_5);
    } else if (live_cmd == 1U) {
      Servo_SetSpeed360(SERVO_CH_4, active_speed);
      Servo_Stop(SERVO_CH_5);
    } else if (live_cmd == 2U) {
      Servo_SetSpeed360(SERVO_CH_4, -active_speed);
      Servo_Stop(SERVO_CH_5);
    } else if (servo_cmd == 200U) {
      /* Right Stick UP: Servo 1 quay thuận */
      Servo_SetSpeed360(SERVO_CH_4, active_speed);
      Servo_Stop(SERVO_CH_5);
    } else if (servo_cmd == 201U) {
      /* Right Stick DOWN: Servo 1 quay ngược */
      Servo_SetSpeed360(SERVO_CH_4, -active_speed);
      Servo_Stop(SERVO_CH_5);
    } else if (servo_cmd == 202U) {
      /* Right Stick LEFT: Servo 2 quay thuận */
      Servo_Stop(SERVO_CH_4);
      Servo_SetSpeed360(SERVO_CH_5, active_speed);
    } else if (servo_cmd == 203U) {
      /* Right Stick RIGHT: Servo 2 quay ngược */
      Servo_Stop(SERVO_CH_4);
      Servo_SetSpeed360(SERVO_CH_5, -active_speed);
    } else if (now_ms < s_test_servo_until_ms) {
      Servo_SetSpeed360(SERVO_CH_4, s_test_servo_speed);
      Servo_Stop(SERVO_CH_5);
    } else {
      Servo_Stop(SERVO_CH_4);
      Servo_Stop(SERVO_CH_5);
    }

    /* Update real-time Live Expression read-back */
    Servo_ChannelState_t s_state;
    if (g_servo_test_pulse != 0U) {
      g_debug.servo.pulse_us = g_servo_test_pulse;
    } else if (Servo_GetState(SERVO_CH_4, &s_state)) {
      g_debug.servo.pulse_us = s_state.current_us;
    }

    /* Synchronize Live Expressions clamp debug override & read-back */
    static uint8_t s_prev_dbg_clamp_closed = 1U;
    if (g_debug.clamp.closed != s_prev_dbg_clamp_closed) {
      g_clamp_opened = (g_debug.clamp.closed == 0U);
      s_prev_dbg_clamp_closed = g_debug.clamp.closed;
      float clamp_angle = g_clamp_opened ? SERVO_CLAMP_DEFAULT_OPEN_DEG
                                         : SERVO_CLAMP_DEFAULT_CLOSE_DEG;
      (void)Servo_SetAngle180(SERVO_CLAMP_CH, clamp_angle);
      (void)Servo_SetAngle180(SERVO_CH_1, clamp_angle);
      (void)Servo_SetAngle180(SERVO_CH_2, clamp_angle);
      g_debug.clamp.angle_deg = clamp_angle;
    }
    Servo_ChannelState_t clamp_st;
    if (Servo_GetState(SERVO_CLAMP_CH, &clamp_st)) {
      g_debug.clamp.pulse_us = clamp_st.current_us;
    }

    /* 4. Poll IMU at 100 Hz (10 ms) IMMEDIATELY BEFORE omni kinematics so
     * heading PD uses a fresh yaw sample (eliminates 10 ms stale-sample lag),
     * or auto-reconnect every 500 ms if not connected */
    if ((uint32_t)(now_ms - g_last_imu_tick_ms) >= 10U) {
      g_last_imu_tick_ms = now_ms;
      if (g_imu.connected) {
        BNO055_IMU_SetStationaryHint(&g_imu, omni_wheels_stopped);
        BNO055_IMU_Update(&g_imu);
      } else {
        static uint32_t s_last_reconnect_ms = 0U;
        if ((uint32_t)(now_ms - s_last_reconnect_ms) >= 500U) {
          s_last_reconnect_ms = now_ms;
          if (BNO055_IMU_Init(&g_imu, &hi2c3)) {
            g_omni_ctrl.field_centric_invert = false;
            g_omni_ctrl.rot_btn_invert = (g_imu.chip_id == 0x85U);
          }
        }
      }
    }

    /* 5. 100 Hz (10 ms) 4-wheel omni kinematics & RS-485 TDMA command/poll
     * broadcast */
    if ((uint32_t)(now_ms - g_last_rs485_tx_ms) >= RS485_TX_PERIOD_MS) {
      g_last_rs485_tx_ms = now_ms;

      Drive_Ps2Input_t ps2_in = {0};
      ps2_in.joy_sum[0] = g_tdk_cmd.joy_sum[0];
      ps2_in.joy_sum[1] = g_tdk_cmd.joy_sum[1];
      ps2_in.ps2_btn[0] = g_tdk_cmd.ps2_btn[0];
      ps2_in.ps2_btn[1] = g_tdk_cmd.ps2_btn[1];
      ps2_in.valid = ps2_valid;

      uint8_t modes[DRIVE_OMNI_WHEEL_COUNT] = {0};
      int16_t targets[DRIVE_OMNI_WHEEL_COUNT] = {0};
      bool imu_ok = g_imu.connected && g_imu.healthy;
      float yaw_rel = imu_ok ? (g_imu.yaw - g_yaw_offset_deg) : 0.0f;

      Drive_OmniCtrlStep(&g_omni_ctrl, &ps2_in, g_estop_latched, yaw_rel,
                         imu_ok, DRIVE_DEFAULT_MAX_RPMX10, modes, targets);

      if (!ps2_valid && (now_ms < s_test_motor_until_ms)) {
        modes[0] = RS485_WHEEL_VELOCITY;
        targets[0] = s_test_motor_rpm;
      }

      RS485_MultiCmd_t mcmd = {0};
      mcmd.num_wheels = DRIVE_OMNI_WHEEL_COUNT;
      mcmd.poll_id = (uint8_t)(g_rs485_poll_idx + 1U);
      g_rs485_poll_idx =
          (uint8_t)((g_rs485_poll_idx + 1U) % DRIVE_OMNI_WHEEL_COUNT);
      mcmd.flags = (g_estop_latched ? RS485_CMD_FLAG_ESTOP : 0U) |
                   (g_clear_faults_pending ? RS485_CMD_FLAG_CLEAR_FAULTS : 0U);
      for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
        mcmd.wheels[i].mode = modes[i];
        mcmd.wheels[i].target = targets[i];
        g_debug.rs485.motor_rpm[i] = targets[i] / 10;
        g_debug.rs485.motor_cmd[i] = (uint8_t)((targets[i] / 10) + 100);
      }

      bool prev_stopped = omni_wheels_stopped;
      omni_wheels_stopped = (targets[0] == 0) && (targets[1] == 0) &&
                            (targets[2] == 0) && (targets[3] == 0);
      g_twitch_dbg.rf_drop_cnt = g_rf.stats.crc_err_frames;
      g_twitch_dbg.imu_err_cnt = g_imu.err_count;
      if (prev_stopped && !omni_wheels_stopped) {
        bool from_joy = (g_omni_ctrl.slew.vx_pct != 0.0f) ||
                        (g_omni_ctrl.slew.vy_pct != 0.0f);
        if (from_joy) {
          g_twitch_dbg.joy_wake_cnt++;
        } else {
          g_twitch_dbg.imu_wake_cnt++;
        }
        g_twitch_dbg.last_wake_joy0 = ps2_in.joy_sum[0];
        g_twitch_dbg.last_wake_joy1 = ps2_in.joy_sum[1];
        g_twitch_dbg.last_wake_btn0 = ps2_in.ps2_btn[0];
        g_twitch_dbg.last_wake_btn1 = ps2_in.ps2_btn[1];
        g_twitch_dbg.last_wake_yaw = yaw_rel;
        g_twitch_dbg.last_wake_target_hdg = g_omni_ctrl.target_heading_deg;
        g_twitch_dbg.last_wake_omega_pct = g_omni_ctrl.omega_fb_pct;
        g_twitch_dbg.last_wake_target0 = targets[0];
      }

      /* Auto-broadcast anti-stick-slip inner velocity PID gains to all 4
       * PidBoards over RS-485:
       *  - Immediately whenever any PidBoard powers on / comes online
       *  - Immediately when g_pidboard_vel_kp_m / ki_m change in Live
       * Expressions
       *  - Periodically every 3s while stopped (survives 12V toggle while USB
       * stays on) */
      static bool s_prev_wheel_alive[DRIVE_OMNI_WHEEL_COUNT] = {false};
      static uint32_t s_pid_last_sync_ms = 0U;
      static uint8_t s_pid_cfg_stage = 0U;
      static int32_t s_sent_kp_m = -1;
      static int32_t s_sent_ki_m = -1;
      bool wheel_came_online = false;
      for (uint8_t i = 0U; i < DRIVE_OMNI_WHEEL_COUNT; i++) {
        if (g_wheel_alive[i] && !s_prev_wheel_alive[i]) {
          wheel_came_online = true;
        }
        s_prev_wheel_alive[i] = g_wheel_alive[i];
      }
      /* Maintain full velocity Ki (2000 m-units) at all times so PidBoard integral term
       * accumulates torque to overcome static friction at crawl/trim speeds.
       * Prevents motor stalling, sluggish deadband, and driver overcurrent trips. */
      int32_t target_ki_m = g_pidboard_vel_ki_m;

      bool idle_refresh_due =
          omni_wheels_stopped &&
          ((uint32_t)(now_ms - s_pid_last_sync_ms) >= 3000U);
      if (s_pid_cfg_stage == 0U) {
        if (target_ki_m != s_sent_ki_m) {
          /* Immediate Ki switch for D-pad micro-adjustment (Ki=0 wipes integral lag) */
          s_pid_cfg_stage = 1U;
          s_pid_last_sync_ms = now_ms;
        } else if (wheel_came_online || idle_refresh_due ||
                   (g_pidboard_vel_kp_m != s_sent_kp_m)) {
          s_pid_cfg_stage = 2U;
          s_pid_last_sync_ms = now_ms;
        }
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
        cfg.value = target_ki_m;
        if (RS485_SendCfgCmd(&g_rs485, RS485_ADDR_BCAST, &cfg, now_ms) ==
            HAL_OK) {
          s_sent_ki_m = target_ki_m;
          s_pid_cfg_stage = 0U;
        }
      } else if (RS485_SendMultiCmd(&g_rs485, &mcmd, now_ms) == HAL_OK) {
        g_clear_faults_pending = false;
        g_last_rs485_tx_ms = now_ms;
      }
    }

    /* 6. Forward state to Autonomous Robot (MainRB2 = RF_NODE_AUTO) when
     * user presses START / Button on TDK */
    /* When no button is pressed or released: HC-12 remains 100% silent in RX
     * listen mode (NO TX) to avoid channel congestion and prevent spurious transmissions during servo moves */
    uint8_t clean_btn0_auto =
        (g_tdk_cmd.ps2_btn[0] < 200U) ? g_tdk_cmd.ps2_btn[0] : 0U;
    bool trigger_auto =
        g_tdk_link_ok && (((clean_btn0_auto & DRIVE_PS2_B0_START) != 0U) ||
                          ((g_tdk_cmd.ext_sw & RF_TDK_EXT_BUTTON_BIT) != 0U));

    static bool s_prev_trigger_auto = false;
    static uint8_t s_auto_burst_cnt = 0U;
    static uint8_t s_auto_release_burst_cnt = 0U;

    if (trigger_auto && !s_prev_trigger_auto) {
      /* Rising edge of trigger: burst of 5 frames with TRIGGER_R2 = 1, cmd = RF_AUTO_CMD_START */
      s_auto_burst_cnt = 5U;
      s_auto_release_burst_cnt = 0U;
    } else if (!trigger_auto && s_prev_trigger_auto) {
      /* Falling edge of trigger: burst of 5 release frames with TRIGGER_R2 = 0, cmd = RF_AUTO_CMD_NONE
       * Guarantees R2 edge detector clears so subsequent START presses trigger reliably */
      s_auto_release_burst_cnt = 5U;
      s_auto_burst_cnt = 0U;
    }
    s_prev_trigger_auto = trigger_auto;

    bool send_auto_frame = false;
    uint8_t frame_cmd = RF_AUTO_CMD_NONE;
    bool frame_trigger_bit = false;

    if (s_auto_burst_cnt > 0U) {
      if ((uint32_t)(now_ms - g_last_auto_tx_ms) >= 20U) {
        send_auto_frame = true;
        frame_cmd = RF_AUTO_CMD_START;
        frame_trigger_bit = true;
        s_auto_burst_cnt--;
      }
    } else if (s_auto_release_burst_cnt > 0U) {
      if ((uint32_t)(now_ms - g_last_auto_tx_ms) >= 20U) {
        send_auto_frame = true;
        frame_cmd = RF_AUTO_CMD_NONE;
        frame_trigger_bit = false;
        s_auto_release_burst_cnt--;
      }
    } else if (trigger_auto) {
      /* While button is held, continue transmitting every 60 ms */
      if ((uint32_t)(now_ms - g_last_auto_tx_ms) >= 60U) {
        send_auto_frame = true;
        frame_cmd = RF_AUTO_CMD_START;
        frame_trigger_bit = true;
      }
    }

    if (send_auto_frame) {
      g_last_auto_tx_ms = now_ms;
      RF_PayloadManualToAuto_t auto_msg = {0};
      auto_msg.cmd = frame_cmd;
      auto_msg.mode = (uint8_t)CD4067_GetLedViewMode();
      auto_msg.flags = (g_tdk_link_ok ? RF_MANUAL_FLAG_TDK_LINK : 0U) |
                       (frame_trigger_bit ? RF_MANUAL_FLAG_TRIGGER_R2 : 0U) |
                       (Buttons_IsPressed(BTN_SW1) ? RF_MANUAL_FLAG_SW1 : 0U) |
                       (Buttons_IsPressed(BTN_SW2) ? RF_MANUAL_FLAG_SW2 : 0U);
      auto_msg.reserved = (uint8_t)((g_omni_ctrl.gear & 0x03U) |
                                    (g_omni_ctrl.field_centric ? 0x04U : 0U) |
                                    (g_omni_ctrl.blue_side ? 0x08U : 0U));
      auto_msg.limit_mask = CD4067_GetLimitMask();
      auto_msg.arg0 = (int16_t)g_tdk_cmd.joy_left[0];
      auto_msg.arg1 = (int16_t)g_tdk_cmd.joy_left[1];
      auto_msg.arg2 = (int16_t)(((uint16_t)g_tdk_cmd.ps2_btn[1] << 8U) |
                                clean_btn0_auto);
      auto_msg.arg3 = (int16_t)(g_imu.yaw * 10.0f);
      (void)RF_HC12_SendManualToAuto(&g_rf, RF_NODE_AUTO, &auto_msg);
    }

    /* 7. Stream live IMU & system diagnostics over USB CDC at 10 Hz (100 ms) */
    static uint32_t s_last_usb_cdc_ms = 0U;
    if ((uint32_t)(now_ms - s_last_usb_cdc_ms) >= 100U) {
      s_last_usb_cdc_ms = now_ms;
      int32_t y_i = (int32_t)g_imu.yaw;
      int32_t y_f = (int32_t)(fabsf(g_imu.yaw - (float)y_i) * 100.0f);
      int32_t p_i = (int32_t)g_imu.pitch;
      int32_t p_f = (int32_t)(fabsf(g_imu.pitch - (float)p_i) * 100.0f);
      int32_t r_i = (int32_t)g_imu.roll;
      int32_t r_f = (int32_t)(fabsf(g_imu.roll - (float)r_i) * 100.0f);
      char cdc_msg[140];
      int cdc_len = snprintf(
          cdc_msg, sizeof(cdc_msg),
          "[IMU] CHIP:0x%02X ADDR:0x%02X YAW:%s%ld.%02ld PITCH:%s%ld.%02ld "
          "ROLL:%s%ld.%02ld CAL:%d OK:%lu ERR:%lu\r\n",
          g_imu.chip_id, g_imu.i2c_addr,
          (g_imu.yaw < 0.0f && y_i == 0) ? "-" : "", (long)y_i, (long)y_f,
          (g_imu.pitch < 0.0f && p_i == 0) ? "-" : "", (long)p_i, (long)p_f,
          (g_imu.roll < 0.0f && r_i == 0) ? "-" : "", (long)r_i, (long)r_f,
          g_imu.calib.sys, (unsigned long)g_imu.ok_count,
          (unsigned long)g_imu.err_count);
      if (cdc_len > 0) {
        CDC_Transmit_FS((uint8_t *)cdc_msg, (uint16_t)cdc_len);
      }
    }

    /* 8. Update unified debug struct (viewable in CubeIDE Live Expressions
     * without CPU overhead) */
    g_debug.tdk.connected = g_tdk_link_ok ? 1U : 0U;
    g_debug.tdk.joy_y = g_tdk_cmd.joy_left[1];
    g_debug.tdk.joy_x = g_tdk_cmd.joy_left[0];
    g_debug.tdk.target_y = (float)g_tdk_cmd.joy_left[1] - 100.0f;
    g_debug.tdk.target_x = 100.0f - (float)g_tdk_cmd.joy_left[0];
    g_debug.tdk.btn1 = g_tdk_cmd.ps2_btn[1];
    g_debug.tdk.btn0 = g_tdk_cmd.ps2_btn[0];
    g_debug.tdk.cnt = g_rf.tx_seq;
    g_debug.tdk.last_rx_ms = g_rf.last_rx_tick_ms;
    g_debug.tdk.rx_packet_count = g_rf.stats.rx_ok_frames;

    g_debug.rs485.tx_cnt = g_rs485.tx_seq;
    g_debug.rs485.gain_db = 17U; /* default (17 - 7 = 10x gain) */
    g_debug.rs485.last_tx_ms = g_last_rs485_tx_ms;

    g_debug.imu.connected = g_imu.connected ? 1U : 0U;
    g_debug.imu.yaw = g_imu.yaw;
    g_debug.imu.roll = g_imu.roll;
    g_debug.imu.pitch = g_imu.pitch;
    g_debug.imu.target_heading = g_omni_ctrl.target_heading_deg;

    for (uint8_t i = 0U; i < 16U; i++) {
      g_debug.line.adc_raw[i] = cd4067_data.raw[i];
    }
    g_debug.line.active_mask = cd4067_data.active_mask;
    g_debug.line.active_count = cd4067_data.active_count;
    g_debug.line.line_error = cd4067_data.line_error;

    g_debug.board.sw1 = Buttons_IsPressed(BTN_SW1) ? 1U : 0U;
    g_debug.board.sw2 = Buttons_IsPressed(BTN_SW2) ? 1U : 0U;
    g_debug.board.blue_side = g_omni_ctrl.blue_side ? 1U : 0U;
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
  /* Pre-start TIM12_CH2 with 1500 us neutral stop pulse so continuous 360
   * servos do not spin during the initial power-on boot window. */
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 1500U);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
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
