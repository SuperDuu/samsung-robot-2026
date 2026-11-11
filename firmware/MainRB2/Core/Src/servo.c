/**
  ******************************************************************************
  * @file    servo.c
  * @brief   2-mode Servo driver (0-270 deg positional & 360 deg continuous speed)
  *          for the 6 PWM outputs on MainRB (TIM3 CH3/4, TIM12 CH1/2, TIM4 CH3/4).
  ******************************************************************************
  */

#include "servo.h"
#include <math.h>
#include <string.h>

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t           tim_ch;
} Servo_HwMap_t;

static struct
{
  Servo_HwMap_t        hw[SERVO_NUM_CHANNELS];
  Servo_ChannelState_t ch[SERVO_NUM_CHANNELS];
  bool                 initialized;
} s_servo;

static uint16_t servo_round_to_u16(float val)
{
  if (val <= 0.0f)
  {
    return 0U;
  }
  return (uint16_t)(val + 0.5f);
}

static HAL_StatusTypeDef servo_apply_us(Servo_Channel_t ch, uint16_t pulse_us)
{
  if (!s_servo.initialized || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS))
  {
    return HAL_ERROR;
  }

  TIM_HandleTypeDef *htim = s_servo.hw[ch].htim;
  uint32_t tim_ch = s_servo.hw[ch].tim_ch;
  if (htim == NULL)
  {
    return HAL_ERROR;
  }

  if ((pulse_us > 0U) && (pulse_us < SERVO_PWM_MIN_US))
  {
    pulse_us = SERVO_PWM_MIN_US;
  }
  if (pulse_us > SERVO_PWM_MAX_US)
  {
    pulse_us = SERVO_PWM_MAX_US;
  }

  if (!s_servo.ch[ch].started)
  {
    __HAL_TIM_SET_COMPARE(htim, tim_ch, pulse_us);
    if (HAL_TIM_PWM_Start(htim, tim_ch) != HAL_OK)
    {
      return HAL_ERROR;
    }
    s_servo.ch[ch].started = true;
  }
  else
  {
    __HAL_TIM_SET_COMPARE(htim, tim_ch, pulse_us);
  }

  s_servo.ch[ch].current_us = pulse_us;
  return HAL_OK;
}

HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim3,
                             TIM_HandleTypeDef *htim4,
                             TIM_HandleTypeDef *htim12)
{
  memset(&s_servo, 0, sizeof(s_servo));

  s_servo.hw[SERVO_CH_1].htim   = htim3;
  s_servo.hw[SERVO_CH_1].tim_ch = TIM_CHANNEL_3; /* PB0 */
  s_servo.hw[SERVO_CH_2].htim   = htim3;
  s_servo.hw[SERVO_CH_2].tim_ch = TIM_CHANNEL_4; /* PB1 */
  s_servo.hw[SERVO_CH_3].htim   = htim12;
  s_servo.hw[SERVO_CH_3].tim_ch = TIM_CHANNEL_1; /* PB14 */
  s_servo.hw[SERVO_CH_4].htim   = htim12;
  s_servo.hw[SERVO_CH_4].tim_ch = TIM_CHANNEL_2; /* PB15 */
  s_servo.hw[SERVO_CH_5].htim   = htim4;
  s_servo.hw[SERVO_CH_5].tim_ch = TIM_CHANNEL_3; /* PB8 */
  s_servo.hw[SERVO_CH_6].htim   = htim4;
  s_servo.hw[SERVO_CH_6].tim_ch = TIM_CHANNEL_4; /* PB9 */

  for (uint8_t i = 0U; i < (uint8_t)SERVO_NUM_CHANNELS; i++)
  {
    if (s_servo.hw[i].htim != NULL)
    {
      __HAL_TIM_SET_COMPARE(s_servo.hw[i].htim, s_servo.hw[i].tim_ch, 0U);
    }
  }

  s_servo.initialized = true;
  return HAL_OK;
}

HAL_StatusTypeDef Servo_Config270(Servo_Channel_t ch,
                                  uint16_t min_us,
                                  uint16_t max_us,
                                  bool inverted)
{
  if (!s_servo.initialized || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS) ||
      (s_servo.hw[ch].htim == NULL))
  {
    return HAL_ERROR;
  }

  if (min_us == 0U)
  {
    min_us = SERVO_PWM_MIN_US;
  }
  if (max_us == 0U)
  {
    max_us = SERVO_PWM_MAX_US;
  }
  if (min_us < SERVO_PWM_MIN_US)
  {
    min_us = SERVO_PWM_MIN_US;
  }
  if (max_us > SERVO_PWM_MAX_US)
  {
    max_us = SERVO_PWM_MAX_US;
  }
  if (min_us >= max_us)
  {
    return HAL_ERROR;
  }

  s_servo.ch[ch].mode = SERVO_MODE_ANGLE_270;
  s_servo.ch[ch].min_us = min_us;
  s_servo.ch[ch].neutral_us = (uint16_t)((min_us + max_us) / 2U);
  s_servo.ch[ch].max_us = max_us;
  s_servo.ch[ch].deadband_pct = 0.0f;
  s_servo.ch[ch].inverted = inverted;
  return HAL_OK;
}

HAL_StatusTypeDef Servo_Config360(Servo_Channel_t ch,
                                  uint16_t neutral_us,
                                  uint16_t half_span_us,
                                  float deadband_pct,
                                  bool inverted)
{
  if (!s_servo.initialized || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS) ||
      (s_servo.hw[ch].htim == NULL))
  {
    return HAL_ERROR;
  }

  if (neutral_us == 0U)
  {
    neutral_us = SERVO_PWM_NEUTRAL_US;
  }
  if (neutral_us < 1000U)
  {
    neutral_us = 1000U;
  }
  if (neutral_us > 2000U)
  {
    neutral_us = 2000U;
  }

  if (half_span_us == 0U)
  {
    half_span_us = SERVO_360_DEFAULT_HALF_SPAN_US;
  }

  uint16_t min_us = (neutral_us > half_span_us) ? (uint16_t)(neutral_us - half_span_us) : SERVO_PWM_MIN_US;
  uint16_t max_us = (uint16_t)(neutral_us + half_span_us);
  if (min_us < SERVO_PWM_MIN_US)
  {
    min_us = SERVO_PWM_MIN_US;
  }
  if (max_us > SERVO_PWM_MAX_US)
  {
    max_us = SERVO_PWM_MAX_US;
  }

  if (!isfinite(deadband_pct) || (deadband_pct < 0.0f))
  {
    deadband_pct = 0.0f;
  }
  if (deadband_pct > 25.0f)
  {
    deadband_pct = 25.0f;
  }

  s_servo.ch[ch].mode = SERVO_MODE_CONTINUOUS_360;
  s_servo.ch[ch].min_us = min_us;
  s_servo.ch[ch].neutral_us = neutral_us;
  s_servo.ch[ch].max_us = max_us;
  s_servo.ch[ch].deadband_pct = deadband_pct;
  s_servo.ch[ch].inverted = inverted;
  s_servo.ch[ch].last_cmd = 0.0f;

  /* Command neutral stop pulse immediately on continuous servo setup */
  return servo_apply_us(ch, neutral_us);
}

HAL_StatusTypeDef Servo_SetAngle270(Servo_Channel_t ch, float angle_deg)
{
  if (!s_servo.initialized || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS))
  {
    return HAL_ERROR;
  }
  if (s_servo.ch[ch].mode != SERVO_MODE_ANGLE_270)
  {
    return HAL_ERROR;
  }
  if (!isfinite(angle_deg))
  {
    return HAL_ERROR;
  }

  if (angle_deg < 0.0f)
  {
    angle_deg = 0.0f;
  }
  else if (angle_deg > SERVO_270_MAX_DEG)
  {
    angle_deg = SERVO_270_MAX_DEG;
  }

  float eff = s_servo.ch[ch].inverted ? (SERVO_270_MAX_DEG - angle_deg) : angle_deg;
  float span = (float)(s_servo.ch[ch].max_us - s_servo.ch[ch].min_us);
  uint16_t pulse_us = servo_round_to_u16((float)s_servo.ch[ch].min_us + (eff / SERVO_270_MAX_DEG) * span);

  s_servo.ch[ch].last_cmd = angle_deg;
  return servo_apply_us(ch, pulse_us);
}

HAL_StatusTypeDef Servo_SetSpeed360(Servo_Channel_t ch, float speed_pct)
{
  if (!s_servo.initialized || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS))
  {
    return HAL_ERROR;
  }
  if (s_servo.ch[ch].mode != SERVO_MODE_CONTINUOUS_360)
  {
    return HAL_ERROR;
  }
  if (!isfinite(speed_pct))
  {
    s_servo.ch[ch].last_cmd = 0.0f;
    (void)servo_apply_us(ch, s_servo.ch[ch].neutral_us);
    return HAL_ERROR;
  }

  if (speed_pct < -100.0f)
  {
    speed_pct = -100.0f;
  }
  else if (speed_pct > 100.0f)
  {
    speed_pct = 100.0f;
  }

  uint16_t pulse_us;
  if (fabsf(speed_pct) <= s_servo.ch[ch].deadband_pct)
  {
    s_servo.ch[ch].last_cmd = 0.0f;
    pulse_us = s_servo.ch[ch].neutral_us;
  }
  else
  {
    float eff = s_servo.ch[ch].inverted ? -speed_pct : speed_pct;
    s_servo.ch[ch].last_cmd = speed_pct;
    if (eff >= 0.0f)
    {
      float pos_span = (float)(s_servo.ch[ch].max_us - s_servo.ch[ch].neutral_us);
      pulse_us = servo_round_to_u16((float)s_servo.ch[ch].neutral_us + (eff / 100.0f) * pos_span);
    }
    else
    {
      float neg_span = (float)(s_servo.ch[ch].neutral_us - s_servo.ch[ch].min_us);
      pulse_us = servo_round_to_u16((float)s_servo.ch[ch].neutral_us + (eff / 100.0f) * neg_span);
    }
  }

  return servo_apply_us(ch, pulse_us);
}

HAL_StatusTypeDef Servo_SetDirection360(Servo_Channel_t ch, int8_t dir, float speed_abs_pct)
{
  if (!isfinite(speed_abs_pct))
  {
    return Servo_SetSpeed360(ch, NAN);
  }
  if (speed_abs_pct < 0.0f)
  {
    speed_abs_pct = -speed_abs_pct;
  }
  if (dir > 0)
  {
    return Servo_SetSpeed360(ch, speed_abs_pct);
  }
  if (dir < 0)
  {
    return Servo_SetSpeed360(ch, -speed_abs_pct);
  }
  return Servo_SetSpeed360(ch, 0.0f);
}

HAL_StatusTypeDef Servo_Stop(Servo_Channel_t ch)
{
  if (!s_servo.initialized || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS))
  {
    return HAL_ERROR;
  }

  if (s_servo.ch[ch].mode == SERVO_MODE_CONTINUOUS_360)
  {
    s_servo.ch[ch].last_cmd = 0.0f;
    return servo_apply_us(ch, s_servo.ch[ch].neutral_us);
  }

  if (s_servo.hw[ch].htim != NULL)
  {
    __HAL_TIM_SET_COMPARE(s_servo.hw[ch].htim, s_servo.hw[ch].tim_ch, 0U);
    if (s_servo.ch[ch].started)
    {
      (void)HAL_TIM_PWM_Stop(s_servo.hw[ch].htim, s_servo.hw[ch].tim_ch);
      s_servo.ch[ch].started = false;
    }
    s_servo.ch[ch].current_us = 0U;
    return HAL_OK;
  }
  return HAL_ERROR;
}

void Servo_StopAll360(void)
{
  for (uint8_t i = 0U; i < (uint8_t)SERVO_NUM_CHANNELS; i++)
  {
    if (s_servo.ch[i].mode == SERVO_MODE_CONTINUOUS_360)
    {
      (void)Servo_Stop((Servo_Channel_t)i);
    }
  }
}

HAL_StatusTypeDef Servo_WriteRawUs(Servo_Channel_t ch, uint16_t pulse_us)
{
  return servo_apply_us(ch, pulse_us);
}

bool Servo_GetState(Servo_Channel_t ch, Servo_ChannelState_t *out_state)
{
  if ((out_state == NULL) || ((uint8_t)ch >= (uint8_t)SERVO_NUM_CHANNELS))
  {
    return false;
  }
  *out_state = s_servo.ch[ch];
  return true;
}
