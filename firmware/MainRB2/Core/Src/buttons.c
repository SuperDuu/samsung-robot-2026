/**
  ******************************************************************************
  * @file    buttons.c
  * @brief   Debounced onboard switch reader for SW1 (PB12) & SW2 (PA15) on MainRB.
  ******************************************************************************
  */

#include "buttons.h"
#include <string.h>

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  bool          active_low;
  bool          raw_last;
  bool          debounced;
  bool          pressed_edge;
  bool          released_edge;
  uint32_t      raw_change_ms;
  uint32_t      press_start_ms;
} Button_State_t;

static Button_State_t s_btns[BTN_COUNT];

static bool btn_read_raw(const Button_State_t *b)
{
  GPIO_PinState st = HAL_GPIO_ReadPin(b->port, b->pin);
  return b->active_low ? (st == GPIO_PIN_RESET) : (st == GPIO_PIN_SET);
}

void Buttons_Init(bool sw1_active_low, bool sw2_active_low)
{
  uint32_t now_ms = HAL_GetTick();
  memset(s_btns, 0, sizeof(s_btns));

  s_btns[BTN_SW1].port = SW1_GPIO_Port;
  s_btns[BTN_SW1].pin = SW1_Pin;
  s_btns[BTN_SW1].active_low = sw1_active_low;

  s_btns[BTN_SW2].port = SW2_GPIO_Port;
  s_btns[BTN_SW2].pin = SW2_Pin;
  s_btns[BTN_SW2].active_low = sw2_active_low;

  for (uint8_t i = 0U; i < (uint8_t)BTN_COUNT; i++)
  {
    bool init_raw = btn_read_raw(&s_btns[i]);
    s_btns[i].raw_last = init_raw;
    s_btns[i].debounced = init_raw;
    s_btns[i].raw_change_ms = now_ms;
    s_btns[i].press_start_ms = init_raw ? now_ms : 0U;
  }
}

void Buttons_Update(uint32_t now_ms)
{
  for (uint8_t i = 0U; i < (uint8_t)BTN_COUNT; i++)
  {
    Button_State_t *b = &s_btns[i];
    bool raw = btn_read_raw(b);

    b->pressed_edge = false;
    b->released_edge = false;

    if (raw != b->raw_last)
    {
      b->raw_last = raw;
      b->raw_change_ms = now_ms;
    }

    if ((raw != b->debounced) && ((uint32_t)(now_ms - b->raw_change_ms) >= BUTTONS_DEBOUNCE_MS))
    {
      b->debounced = raw;
      if (raw)
      {
        b->pressed_edge = true;
        b->press_start_ms = now_ms;
      }
      else
      {
        b->released_edge = true;
      }
    }
  }
}

bool Buttons_IsPressed(Button_Id_t id)
{
  if ((uint8_t)id >= (uint8_t)BTN_COUNT)
  {
    return false;
  }
  return s_btns[id].debounced;
}

bool Buttons_WasPressed(Button_Id_t id)
{
  if ((uint8_t)id >= (uint8_t)BTN_COUNT)
  {
    return false;
  }
  return s_btns[id].pressed_edge;
}

bool Buttons_WasReleased(Button_Id_t id)
{
  if ((uint8_t)id >= (uint8_t)BTN_COUNT)
  {
    return false;
  }
  return s_btns[id].released_edge;
}

bool Buttons_ConsumePressed(Button_Id_t id)
{
  if ((uint8_t)id >= (uint8_t)BTN_COUNT)
  {
    return false;
  }
  bool edge = s_btns[id].pressed_edge;
  s_btns[id].pressed_edge = false;
  return edge;
}

uint32_t Buttons_GetHoldMs(Button_Id_t id, uint32_t now_ms)
{
  if (((uint8_t)id >= (uint8_t)BTN_COUNT) || !s_btns[id].debounced)
  {
    return 0U;
  }
  return (uint32_t)(now_ms - s_btns[id].press_start_ms);
}

uint8_t Buttons_GetMask(void)
{
  uint8_t mask = 0U;
  if (s_btns[BTN_SW1].debounced)
  {
    mask |= (1U << 0);
  }
  if (s_btns[BTN_SW2].debounced)
  {
    mask |= (1U << 1);
  }
  return mask;
}
