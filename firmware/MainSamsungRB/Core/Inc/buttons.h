/**
  ******************************************************************************
  * @file    buttons.h
  * @brief   Debounced onboard switch reader for SW1 (PB12) & SW2 (PA15) on MainRB.
  *          Note: SW1/SW2 MUST stay GPIO_NOPULL on STM32F405 because J39/J40
  *          pin 1 carries +5V (preserving FT 5V-tolerance on PB12 & PA15).
  ******************************************************************************
  */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

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

#define BUTTONS_DEBOUNCE_MS 20U

typedef enum
{
  BTN_SW1 = 0U, /* PB12 (J39) */
  BTN_SW2 = 1U, /* PA15 (J40) */
  BTN_COUNT = 2U
} Button_Id_t;

void Buttons_Init(bool sw1_active_low, bool sw2_active_low);
void Buttons_Update(uint32_t now_ms);

bool Buttons_IsPressed(Button_Id_t id);
bool Buttons_WasPressed(Button_Id_t id);
bool Buttons_WasReleased(Button_Id_t id);
bool Buttons_ConsumePressed(Button_Id_t id);
uint32_t Buttons_GetHoldMs(Button_Id_t id, uint32_t now_ms);
uint8_t Buttons_GetMask(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_BUTTONS_H_ */
