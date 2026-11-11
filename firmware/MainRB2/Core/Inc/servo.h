/**
  ******************************************************************************
  * @file    servo.h
  * @brief   2-mode Servo driver (0-270 deg positional & 360 deg continuous speed)
  *          for the 6 PWM outputs on MainRB (TIM3 CH3/4, TIM12 CH1/2, TIM4 CH3/4).
  *          Requires 50 Hz timer base: PSC = 83, ARR = 19999 -> 1 tick = 1 us.
  ******************************************************************************
  */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

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

#define SERVO_PWM_MIN_US               500U
#define SERVO_PWM_NEUTRAL_US           1500U
#define SERVO_PWM_MAX_US               2500U
#define SERVO_270_MAX_DEG              270.0f
#define SERVO_360_DEFAULT_HALF_SPAN_US 500U   /* 1000..2000 us continuous range (pass 1000 for 500..2500 us) */
#define SERVO_DEFAULT_DEADBAND_PCT     2.0f

typedef enum
{
  SERVO_CH_1 = 0U, /* PB0  - TIM3_CH3  (J2)  */
  SERVO_CH_2 = 1U, /* PB1  - TIM3_CH4  (J34) */
  SERVO_CH_3 = 2U, /* PB14 - TIM12_CH1 (J35) */
  SERVO_CH_4 = 3U, /* PB15 - TIM12_CH2 (J36) */
  SERVO_CH_5 = 4U, /* PB8  - TIM4_CH3  (J37) */
  SERVO_CH_6 = 5U, /* PB9  - TIM4_CH4  (J38) */
  SERVO_NUM_CHANNELS = 6U
} Servo_Channel_t;

typedef enum
{
  SERVO_MODE_UNCONFIGURED   = 0U,
  SERVO_MODE_ANGLE_270      = 1U,
  SERVO_MODE_CONTINUOUS_360 = 2U
} Servo_Mode_t;

typedef struct
{
  Servo_Mode_t mode;
  uint16_t     min_us;
  uint16_t     neutral_us;
  uint16_t     max_us;
  float        deadband_pct;
  bool         inverted;
  bool         started;
  uint16_t     current_us;
  float        last_cmd;
} Servo_ChannelState_t;

HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim3,
                             TIM_HandleTypeDef *htim4,
                             TIM_HandleTypeDef *htim12);

HAL_StatusTypeDef Servo_Config270(Servo_Channel_t ch,
                                  uint16_t min_us,
                                  uint16_t max_us,
                                  bool inverted);

HAL_StatusTypeDef Servo_Config360(Servo_Channel_t ch,
                                  uint16_t neutral_us,
                                  uint16_t half_span_us,
                                  float deadband_pct,
                                  bool inverted);

/* Mode 1: Positional 0.0 .. 270.0 degrees (linearly mapped to min_us..max_us) */
HAL_StatusTypeDef Servo_SetAngle270(Servo_Channel_t ch, float angle_deg);

/* Mode 2: Continuous 360 rotation (-100.0% CCW .. 0.0% stop .. +100.0% CW) */
HAL_StatusTypeDef Servo_SetSpeed360(Servo_Channel_t ch, float speed_pct);

/* Convenience helper for 360 mode: dir > 0 CW, dir < 0 CCW, dir == 0 stop */
HAL_StatusTypeDef Servo_SetDirection360(Servo_Channel_t ch, int8_t dir, float speed_abs_pct);

/* Stops 360 continuous servo at neutral_us, or cuts PWM pulse (0 us) on 270 servo */
HAL_StatusTypeDef Servo_Stop(Servo_Channel_t ch);
void Servo_StopAll360(void);

HAL_StatusTypeDef Servo_WriteRawUs(Servo_Channel_t ch, uint16_t pulse_us);
bool Servo_GetState(Servo_Channel_t ch, Servo_ChannelState_t *out_state);

#ifdef __cplusplus
}
#endif

#endif /* INC_SERVO_H_ */
