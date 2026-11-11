/**
  ******************************************************************************
  * @file    cd4067.h
  * @brief   16-channel CD74HC4067M analog/digital multiplexer driver matched to MainRB PCB
  *          (U8 + U6A LMV358 + ADC1_IN0 PA0 + TIM7 + L1..L11 active-LOW LEDs).
  *
  * Supported operating modes:
  *   - CD4067_MODE_LINE_11      : Backward-compatible 11-sensor line array (I0..I10 -> L1..L11).
  *   - CD4067_MODE_LIMIT_SWITCH : 16-channel limit switch scanner (I0..I15) with Schmitt
  *                                hysteresis + multi-sweep debounce (skips boot offset tare).
  *   - CD4067_MODE_LINE_2X8     : Dual-end 2x8 line sensor array (Front I0..I7, Rear I8..I15)
  *                                with 11-LED multi-view encoding (L1..L11).
  ******************************************************************************
  */

#ifndef INC_CD4067_H_
#define INC_CD4067_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UNIT_TEST_HOST
#include "mock_hal.h"
#else
#include "main.h"
#endif

#include <stdbool.h>
#include <stdint.h>

#define CD4067_NUM_CHANNELS          16U
#define CD4067_NUM_LINE_SENSORS      11U
#define CD4067_NUM_ROW_SENSORS       8U
#define CD4067_DEFAULT_THRESHOLD     700U
#define CD4067_CALIB_WARMUP_SWEEPS   100U
#define CD4067_CALIB_AVG_SWEEPS      256U

/* Limit-switch Schmitt trigger hysteresis thresholds on 12-bit ADC (0..4095 @ 3.3V) */
#define CD4067_LIMIT_LOW_MAX         1400U /* <= 1.13 V -> logic LOW  */
#define CD4067_LIMIT_HIGH_MIN        2600U /* >= 2.10 V -> logic HIGH */
#define CD4067_LIMIT_DEBOUNCE_SWEEPS 5U    /* 5 * 1.6 ms = 8.0 ms debounce */

/* Default alternating dwell period (ms) between Front (I0..I7) and Rear (I8..I15) on L1..L8 */
#define CD4067_LED_ALT_PERIOD_MS     600U

typedef enum
{
  CD4067_MODE_LINE_11      = 0U,
  CD4067_MODE_LIMIT_SWITCH = 1U,
  CD4067_MODE_LINE_2X8     = 2U
} CD4067_Mode_t;

typedef enum
{
  CD4067_LED_VIEW_LOW11      = 0U, /* L1..L11 = ch 0..10 (1:1 for LINE_11 / Limit lower 11) */
  CD4067_LED_VIEW_AUTO_ALT   = 1U, /* L1..L8 alt 600ms Front/Rear 1:1; L9=Front any, L10=Rear any, L11=Bank */
  CD4067_LED_VIEW_FRONT_LOCK = 2U, /* L1..L8 = Front I0..I7 1:1; L9=Front any, L10=Rear any, L11=OFF */
  CD4067_LED_VIEW_REAR_LOCK  = 3U, /* L1..L8 = Rear I8..I15 1:1; L9=Front any, L10=Rear any, L11=ON  */
  CD4067_LED_VIEW_DUAL_5_1_5 = 4U  /* L1..L5=Front(5), L6=Both active separator, L7..L11=Rear(5) */
} CD4067_LedView_t;

typedef struct
{
  volatile uint16_t         raw[CD4067_NUM_CHANNELS];
  volatile uint16_t         offset[CD4067_NUM_CHANNELS];
  volatile uint8_t          sensor_active[CD4067_NUM_CHANNELS];
  volatile uint16_t         active_mask;
  volatile uint8_t          active_count;
  volatile int16_t          line_error;
  volatile uint8_t          current_channel;
  volatile uint8_t          cal_done;
  volatile uint16_t         sweep_count;
  volatile uint16_t         threshold;
  volatile CD4067_Mode_t    mode;
  volatile CD4067_LedView_t led_view;
  volatile uint16_t         active_low_mask;
  volatile uint16_t         limit_mask;
  volatile uint8_t          front_mask;
  volatile uint8_t          rear_mask;
  volatile uint8_t          front_count;
  volatile uint8_t          rear_count;
  volatile int16_t          front_error;
  volatile int16_t          rear_error;
  volatile uint32_t         sweep_seq;
} CD4067_Data_t;

extern CD4067_Data_t cd4067_data;

void CD4067_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);
HAL_StatusTypeDef CD4067_Start(void);
HAL_StatusTypeDef CD4067_StartLimitSwitches(uint16_t active_low_mask);
HAL_StatusTypeDef CD4067_StartLine2x8(uint16_t threshold);
void CD4067_Stop(void);
void CD4067_StartCalibration(void);
void CD4067_SetThreshold(uint16_t threshold);
void CD4067_SelectChannel(uint8_t channel);
void CD4067_SetLineLed(uint8_t index, bool on);
void CD4067_SetAllLineLeds(bool on);

void CD4067_SetLedViewMode(CD4067_LedView_t view);
CD4067_LedView_t CD4067_CycleLedViewMode(void);
CD4067_LedView_t CD4067_GetLedViewMode(void);
void CD4067_UpdateLeds(uint32_t now_ms);

uint16_t CD4067_GetRaw(uint8_t channel);
int16_t CD4067_GetLineError(void);
uint16_t CD4067_GetActiveMask(void);
bool CD4067_IsCalibrated(void);

/* Mode-specific accessors */
uint16_t CD4067_GetLimitMask(void);
bool CD4067_IsLimitPressed(uint8_t channel);
uint8_t CD4067_GetFrontMask(void);
uint8_t CD4067_GetRearMask(void);
int16_t CD4067_GetFrontError(void);
int16_t CD4067_GetRearError(void);

/* Dispatch hooks to invoke from HAL_TIM_PeriodElapsedCallback / HAL_ADC_ConvCpltCallback */
void CD4067_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void CD4067_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* INC_CD4067_H_ */
