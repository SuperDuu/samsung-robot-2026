/*
 * PS2_STM32.c
 *
 *  Created on: Apr 13, 2025
 *      Author: htbt2
 *  Updated for tay_dk: safe disconnect reset + digital/analog fallback.
 */

#include "PS2_F103.h"

Data *data;
TIM_HandleTypeDef *PS2_TIMER;

static void ps2_set_neutral(Data *d, uint8_t addr, uint8_t b0, uint8_t b1)
{
  if (d == NULL)
  {
    return;
  }
  d->address   = addr;
  d->data[0]   = 100U;
  d->data[1]   = 100U;
  d->data2[0]  = 50U;
  d->data2[1]  = 50U;
  d->button[0] = b0;
  d->button[1] = b1;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(PS2_TIMER, 0);
  while (__HAL_TIM_GET_COUNTER(PS2_TIMER) < us)
  {
  }
}

uint8_t PS2_SendByte(uint8_t cmd)
{
  uint8_t rx_byte = 0x00;
  uint16_t buffer = 0x01;
  for (buffer = 0x01; buffer < 0x0100; buffer <<= 1)
  {
    if (buffer & cmd)
    {
      HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
    delay_us(8);
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
    delay_us(8);
    HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin))
    {
      rx_byte |= (uint8_t)buffer;
    }
  }
  delay_us(8);
  return rx_byte;
}

void PS2_Cmd(uint8_t *transmit, uint8_t *receive, uint8_t length)
{
  HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);
  delay_us(8);
  for (uint8_t i = 0; i < length; i++)
  {
    receive[i] = PS2_SendByte(transmit[i]);
  }
  HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET);
  delay_us(8);
}

void PS2_Poll(void)
{
  uint8_t transmit[5] = { 0x01, 0x42 };
  uint8_t receive[5] = { 0x00 };
  PS2_Cmd(transmit, receive, 5);
}

void PS2_EnterConfig(void)
{
  uint8_t transmit[5] = { 0x01, 0x43, 0x00, 0x01 };
  uint8_t receive[5] = { 0x00 };
  PS2_Cmd(transmit, receive, 5);
}

void PS2_AnalogMode(void)
{
  uint8_t transmit[9] = { 0x01, 0x44, 0x00, 0x01, 0x03 };
  uint8_t receive[9] = { 0x00 };
  PS2_Cmd(transmit, receive, 9);
}

void PS2_ExitConfig(void)
{
  uint8_t transmit[9] = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
  uint8_t receive[9] = { 0x00 };
  PS2_Cmd(transmit, receive, 9);
}

void PS2_Init(TIM_HandleTypeDef *timer, Data *data_tx)
{
  PS2_TIMER = timer;
  data = data_tx;
  ps2_set_neutral(data, 0U, 0U, 0U);
  if (PS2_TIMER == NULL)
  {
    return;
  }
  HAL_TIM_Base_Start(PS2_TIMER);
  PS2_Poll();
  PS2_EnterConfig();
  PS2_AnalogMode();
  PS2_Poll();
  PS2_ExitConfig();
}

void PS2_Update(void)
{
  if ((PS2_TIMER == NULL) || (data == NULL))
  {
    return;
  }

  uint8_t transmit[9] = { 0x01, 0x42 };
  uint8_t receive[9] = { 0x00 };
  PS2_Cmd(transmit, receive, 9);

  if ((receive[1] != 0x41U) && (receive[1] != 0x73U))
  {
    ps2_set_neutral(data, 0U, 0U, 0U);
    return;
  }

  uint8_t buttonStatus[2];
  buttonStatus[0] = (uint8_t)(~receive[3]);
  buttonStatus[1] = (uint8_t)(~receive[4]);

  if (receive[1] == 0x41U)
  {
    /* Digital mode: valid buttons + neutral stick values */
    ps2_set_neutral(data, 0x41U, buttonStatus[0], buttonStatus[1]);
    return;
  }

  uint8_t Joystick[2];
  Joystick[0] = (uint8_t)((map(receive[8], 0, 255, 0, 100) + map(receive[6], 0, 255, 0, 100)) - 4);
  Joystick[1] = (uint8_t)(map(receive[7], 0, 255, 0, 100) + map(receive[5], 0, 255, 0, 100));

  data->data2[0]  = (uint8_t)map(receive[6], 0, 255, 0, 100);
  data->data2[1]  = (uint8_t)map(receive[5], 0, 255, 0, 100);
  data->address   = 255U;
  data->data[0]   = Joystick[0];
  data->data[1]   = Joystick[1];
  data->button[0] = buttonStatus[0];
  data->button[1] = buttonStatus[1];
}
