/**
  ******************************************************************************
  * @file    mock_hal.c
  * @brief   Host virtual STM32 HAL, VL53L1X register-bank simulator, BNO055 I2C3
  *          fault-injection simulator, and USART DMA ring injector.
  ******************************************************************************
  */

#include "mock_hal.h"
#include <string.h>

GPIO_TypeDef  g_mock_gpioa;
GPIO_TypeDef  g_mock_gpiob;
GPIO_TypeDef  g_mock_gpioc;
GPIO_TypeDef  g_mock_gpiod;
USART_TypeDef g_mock_usart1;
USART_TypeDef g_mock_usart2;
USART_TypeDef g_mock_uart4;
I2C_TypeDef   g_mock_i2c2;
I2C_TypeDef   g_mock_i2c3;
ADC_TypeDef   g_mock_adc1;
TIM_TypeDef   g_mock_tim3;
TIM_TypeDef   g_mock_tim4;
TIM_TypeDef   g_mock_tim7;
TIM_TypeDef   g_mock_tim12;

Mock_VL53L1X_t g_mock_vl53[MOCK_VL53_MAX_SENSORS];
Mock_BNO055_t  g_mock_bno;
uint32_t       g_mock_i2c_recover_calls = 0U;
uint32_t       g_mock_uart_abort_rx_calls = 0U;

static uint32_t s_tick_ms = 0U;

static void mock_vl53_reset_single(Mock_VL53L1X_t *s)
{
  memset(s->regs, 0, sizeof(s->regs));
  s->addr_8bit      = 0x52U;
  s->regs[0x0001]   = 0x29U;
  s->regs[0x00E5]   = s->hang_boot ? 0x00U : 0x03U; /* BootState */
  s->regs[0x010F]   = 0xEAU;                        /* Model ID MSB */
  s->regs[0x0110]   = 0xCCU;                        /* Model ID LSB */
  s->regs[0x0030]   = 0x02U;
  s->ranging_active    = false;
  s->init_cfg_captured = false;
  memset(s->init_cfg_snapshot, 0, sizeof(s->init_cfg_snapshot));
  s->data_ready     = false;
  s->range_status   = 9U; /* Valid */
  s->cfg_writes     = 0U;
  s->fw_cfg_writes  = 0U;
}

static bool mock_vl53_is_awake(const Mock_VL53L1X_t *s)
{
  if (!s->present)
  {
    return false;
  }
  if (s->stuck_addr52)
  {
    return true;
  }
  if (s->xshut_port == NULL)
  {
    return true;
  }
  return MockHal_GetGpioOut(s->xshut_port, s->xshut_pin);
}

static void mock_vl53_sync_xshut(void)
{
  for (uint8_t i = 0U; i < MOCK_VL53_MAX_SENSORS; i++)
  {
    Mock_VL53L1X_t *s = &g_mock_vl53[i];
    if (!s->present || s->stuck_addr52)
    {
      continue;
    }
    bool xshut_hi = MockHal_GetGpioOut(s->xshut_port, s->xshut_pin);
    if (!xshut_hi && s->prev_xshut)
    {
      mock_vl53_reset_single(s);
    }
    else if (xshut_hi && !s->prev_xshut)
    {
      mock_vl53_reset_single(s);
    }
    s->prev_xshut = xshut_hi;
  }
}

static Mock_VL53L1X_t *mock_vl53_find(uint16_t dev)
{
  mock_vl53_sync_xshut();
  uint8_t addr = (uint8_t)(dev & 0xFEU);
  for (uint8_t i = 0U; i < MOCK_VL53_MAX_SENSORS; i++)
  {
    Mock_VL53L1X_t *s = &g_mock_vl53[i];
    if (mock_vl53_is_awake(s) && (s->addr_8bit == addr) && !s->force_nack)
    {
      return s;
    }
  }
  return NULL;
}

static uint8_t mock_vl53_read_reg(Mock_VL53L1X_t *s, uint16_t reg)
{
  if (reg >= sizeof(s->regs))
  {
    return 0U;
  }
  if (reg == 0x00E5U)
  {
    return s->hang_boot ? 0x00U : 0x03U;
  }
  if (reg == 0x0031U) /* GPIO__TIO_HV_STATUS */
  {
    if (s->hang_ranging)
    {
      uint8_t int_pol = (uint8_t)(!(s->regs[0x0030] & 0x10U) ? 1U : 0U);
      return (uint8_t)(int_pol ^ 1U);
    }
    uint8_t active_pol = (uint8_t)(((s->regs[0x0030] & 0x10U) == 0U) ? 1U : 0U);
    return s->data_ready ? active_pol : (uint8_t)(active_pol ^ 1U);
  }
  if (reg == 0x0089U) /* RESULT__RANGE_STATUS */
  {
    return s->range_status;
  }
  if (reg == 0x0096U) /* RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 MSB */
  {
    return (uint8_t)((s->dist_mm >> 8U) & 0xFFU);
  }
  if (reg == 0x0097U)
  {
    return (uint8_t)(s->dist_mm & 0xFFU);
  }
  if (reg == 0x008EU)
  {
    return 0x01U; /* Stream count */
  }
  return s->regs[reg];
}

static void mock_vl53_write_reg(Mock_VL53L1X_t *s, uint16_t reg, uint8_t val)
{
  if ((reg == 0x0087U) && (val == 0x40U) && !s->init_cfg_captured)
  {
    memcpy(s->init_cfg_snapshot, &s->regs[0x2DU], 91U);
    s->init_cfg_captured = true;
  }

  if (reg < sizeof(s->regs))
  {
    s->regs[reg] = val;
  }
  s->cfg_writes++;

  if ((reg >= 0x002DU) && (reg <= 0x0087U))
  {
    s->fw_cfg_writes++;
  }

  if (reg == 0x0001U) /* I2C_SLAVE__DEVICE_ADDRESS */
  {
    s->addr_8bit = (uint8_t)((val & 0x7FU) << 1U);
  }
  else if (reg == 0x0087U) /* SYSTEM__MODE_START */
  {
    if (val == 0x40U)
    {
      s->ranging_active = true;
      s->data_ready     = true;
    }
    else if (val == 0x00U)
    {
      s->ranging_active = false;
    }
  }
  else if (reg == 0x0086U) /* SYSTEM__INTERRUPT_CLEAR */
  {
    if (val == 0x01U)
    {
      /* Keep next frame ready automatically if continuous ranging */
      s->data_ready = s->ranging_active;
    }
  }
}

void MockHal_ResetAll(void)
{
  memset(&g_mock_gpioa, 0, sizeof(g_mock_gpioa));
  memset(&g_mock_gpiob, 0, sizeof(g_mock_gpiob));
  memset(&g_mock_gpioc, 0, sizeof(g_mock_gpioc));
  memset(&g_mock_gpiod, 0, sizeof(g_mock_gpiod));
  memset(&g_mock_usart1, 0, sizeof(g_mock_usart1));
  memset(&g_mock_usart2, 0, sizeof(g_mock_usart2));
  memset(&g_mock_uart4, 0, sizeof(g_mock_uart4));
  memset(&g_mock_i2c2, 0, sizeof(g_mock_i2c2));
  memset(&g_mock_i2c3, 0, sizeof(g_mock_i2c3));
  memset(&g_mock_adc1, 0, sizeof(g_mock_adc1));
  memset(&g_mock_tim3, 0, sizeof(g_mock_tim3));
  memset(&g_mock_tim4, 0, sizeof(g_mock_tim4));
  memset(&g_mock_tim7, 0, sizeof(g_mock_tim7));
  memset(&g_mock_tim12, 0, sizeof(g_mock_tim12));
  memset(g_mock_vl53, 0, sizeof(g_mock_vl53));
  memset(&g_mock_bno, 0, sizeof(g_mock_bno));

  g_mock_bno.addr_8bit  = 0x50U; /* 0x28 << 1 */
  g_mock_bno.calib_stat = 0xFFU;
  g_mock_i2c_recover_calls = 0U;
  g_mock_uart_abort_rx_calls = 0U;
  s_tick_ms = 0U;
}

void MockHal_SetTick(uint32_t ms)
{
  s_tick_ms = ms;
}

void MockHal_AdvanceTick(uint32_t ms)
{
  s_tick_ms += ms;
}

uint32_t HAL_GetTick(void)
{
  return s_tick_ms;
}

void HAL_Delay(uint32_t ms)
{
  s_tick_ms += ms;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  if (GPIOx == NULL)
  {
    return;
  }
  if (PinState == GPIO_PIN_SET)
  {
    GPIOx->ODR |= GPIO_Pin;
  }
  else
  {
    GPIOx->ODR &= (uint32_t)(~GPIO_Pin);
  }
  mock_vl53_sync_xshut();
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  if (GPIOx == NULL)
  {
    return GPIO_PIN_RESET;
  }
  return ((GPIOx->IDR & GPIO_Pin) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

bool MockHal_GetGpioOut(GPIO_TypeDef *port, uint16_t pin)
{
  if (port == NULL)
  {
    return false;
  }
  uint32_t bsrr_set   = port->BSRR & 0xFFFFU;
  uint32_t bsrr_reset = (port->BSRR >> 16U) & 0xFFFFU;
  if ((bsrr_set & pin) != 0U)
  {
    return true;
  }
  if ((bsrr_reset & pin) != 0U)
  {
    return false;
  }
  return ((port->ODR & pin) != 0U);
}

void MockHal_VL53SetupDefault(uint8_t idx, GPIO_TypeDef *port, uint16_t pin, uint16_t dist_mm)
{
  if (idx >= MOCK_VL53_MAX_SENSORS)
  {
    return;
  }
  Mock_VL53L1X_t *s = &g_mock_vl53[idx];
  memset(s, 0, sizeof(*s));
  s->present    = true;
  s->xshut_port = port;
  s->xshut_pin  = pin;
  s->dist_mm    = dist_mm;
  mock_vl53_reset_single(s);
}

void MockHal_VL53SetDistance(uint8_t idx, uint16_t dist_mm, uint8_t range_status)
{
  if (idx < MOCK_VL53_MAX_SENSORS)
  {
    g_mock_vl53[idx].dist_mm      = dist_mm;
    g_mock_vl53[idx].range_status = range_status;
    g_mock_vl53[idx].data_ready   = true;
  }
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
  return (htim != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim)
{
  return (htim != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  (void)Channel;
  return (htim != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  (void)Channel;
  return (htim != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
{
  (void)Length;
  if ((hadc == NULL) || (pData == NULL))
  {
    return HAL_ERROR;
  }
  hadc->dma_dest = pData;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)
{
  return (hadc != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }
  huart->gState  = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)Timeout;
  if ((huart == NULL) || (pData == NULL))
  {
    return HAL_ERROR;
  }
  uint16_t cpy = (Size <= sizeof(huart->last_tx_buf)) ? Size : (uint16_t)sizeof(huart->last_tx_buf);
  memcpy(huart->last_tx_buf, pData, cpy);
  huart->last_tx_len = cpy;
  huart->tx_calls++;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)huart;
  (void)pData;
  (void)Size;
  (void)Timeout;
  return HAL_TIMEOUT;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  if ((huart == NULL) || (pData == NULL))
  {
    return HAL_ERROR;
  }
  uint16_t cpy = (Size <= sizeof(huart->last_tx_buf)) ? Size : (uint16_t)sizeof(huart->last_tx_buf);
  memcpy(huart->last_tx_buf, pData, cpy);
  huart->last_tx_len = cpy;
  huart->tx_calls++;
  huart->gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if ((huart == NULL) || (huart->hdmarx == NULL))
  {
    return HAL_ERROR;
  }
  huart->pRxBuffPtr     = pData;
  huart->RxXferSize     = Size;
  huart->hdmarx->cndtr  = Size;
  huart->hdmarx->State  = HAL_DMA_STATE_READY;
  huart->RxState        = HAL_UART_STATE_BUSY_RX;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }
  huart->gState  = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }
  huart->gState = HAL_UART_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }
  huart->RxState = HAL_UART_STATE_READY;
  g_mock_uart_abort_rx_calls++;
  return HAL_OK;
}

uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart)
{
  return (huart != NULL) ? huart->ErrorCode : 0U;
}

void MockHal_UartInjectDmaBytes(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
  if ((huart == NULL) || (huart->hdmarx == NULL) || (huart->pRxBuffPtr == NULL) || (huart->RxXferSize == 0U))
  {
    return;
  }
  uint32_t remaining = huart->hdmarx->cndtr;
  uint16_t write_pos = (uint16_t)(huart->RxXferSize - remaining);
  if (write_pos >= huart->RxXferSize)
  {
    write_pos = 0U;
  }
  for (uint16_t i = 0U; i < len; i++)
  {
    huart->pRxBuffPtr[write_pos] = data[i];
    write_pos = (uint16_t)((write_pos + 1U) % huart->RxXferSize);
  }
  huart->hdmarx->cndtr = (uint32_t)(huart->RxXferSize - write_pos);
}

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }
  g_mock_i2c_recover_calls++;
  if (hi2c->Instance == I2C3)
  {
    g_mock_bno.bus_stuck = false;
  }
  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State     = HAL_I2C_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }
  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State     = HAL_I2C_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  (void)Trials;
  (void)Timeout;
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  if (hi2c->Instance == I2C2)
  {
    if (mock_vl53_find(DevAddress) != NULL)
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
      return HAL_OK;
    }
    hi2c->ErrorCode = HAL_I2C_ERROR_AF;
    return HAL_ERROR;
  }

  if (hi2c->Instance == I2C3)
  {
    if (g_mock_bno.bus_stuck)
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_TIMEOUT;
      return HAL_TIMEOUT;
    }
    if (g_mock_bno.nack_remaining > 0U)
    {
      g_mock_bno.nack_remaining--;
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }
    if (g_mock_bno.present && ((uint8_t)(DevAddress & 0xFEU) == g_mock_bno.addr_8bit))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
      return HAL_OK;
    }
    hi2c->ErrorCode = HAL_I2C_ERROR_AF;
    return HAL_ERROR;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  if ((Size == 0U) || (pData == NULL))
  {
    return HAL_ERROR;
  }
  return HAL_I2C_Mem_Write(hi2c, DevAddress, pData[0], I2C_MEMADD_SIZE_8BIT,
                           (Size > 1U) ? &pData[1] : pData,
                           (Size > 1U) ? (uint16_t)(Size - 1U) : 0U,
                           Timeout);
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  return HAL_I2C_Mem_Read(hi2c, DevAddress, g_mock_bno.last_reg, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
}

static void bno_encode_i16_le(uint8_t *dst, float deg)
{
  int16_t v = (int16_t)(deg * 16.0f);
  dst[0] = (uint8_t)((uint16_t)v & 0xFFU);
  dst[1] = (uint8_t)(((uint16_t)v >> 8U) & 0xFFU);
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                    uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)Timeout;
  if ((hi2c == NULL) || (pData == NULL))
  {
    return HAL_ERROR;
  }

  if ((hi2c->Instance == I2C2) && (MemAddSize == I2C_MEMADD_SIZE_16BIT))
  {
    Mock_VL53L1X_t *s = mock_vl53_find(DevAddress);
    if (s == NULL)
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }
    for (uint16_t i = 0U; i < Size; i++)
    {
      mock_vl53_write_reg(s, (uint16_t)(MemAddress + i), pData[i]);
    }
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    return HAL_OK;
  }

  if ((hi2c->Instance == I2C3) && (MemAddSize == I2C_MEMADD_SIZE_8BIT))
  {
    g_mock_bno.last_reg = (uint8_t)MemAddress;
    if (Size > 0U)
    {
      g_mock_bno.i2c_tx_count++;
    }
    if (g_mock_bno.bus_stuck)
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_TIMEOUT;
      return HAL_TIMEOUT;
    }
    if (g_mock_bno.nack_remaining > 0U)
    {
      g_mock_bno.nack_remaining--;
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }
    if (!g_mock_bno.present || ((uint8_t)(DevAddress & 0xFEU) != g_mock_bno.addr_8bit))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }
    if (Size >= 1U)
    {
      if (MemAddress == 0x07U)
      {
        g_mock_bno.page = pData[0];
      }
      else if (MemAddress == 0x3DU)
      {
        g_mock_bno.opr_mode = pData[0];
      }
    }
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)Timeout;
  if ((hi2c == NULL) || (pData == NULL) || (Size == 0U))
  {
    return HAL_ERROR;
  }

  if ((hi2c->Instance == I2C2) && (MemAddSize == I2C_MEMADD_SIZE_16BIT))
  {
    Mock_VL53L1X_t *s = mock_vl53_find(DevAddress);
    if (s == NULL)
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }
    for (uint16_t i = 0U; i < Size; i++)
    {
      pData[i] = mock_vl53_read_reg(s, (uint16_t)(MemAddress + i));
    }
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    return HAL_OK;
  }

  if ((hi2c->Instance == I2C3) && (MemAddSize == I2C_MEMADD_SIZE_8BIT))
  {
    g_mock_bno.i2c_rx_count++;
    if (g_mock_bno.bus_stuck)
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_TIMEOUT;
      return HAL_TIMEOUT;
    }
    if (g_mock_bno.nack_remaining > 0U)
    {
      g_mock_bno.nack_remaining--;
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }
    if (!g_mock_bno.present || ((uint8_t)(DevAddress & 0xFEU) != g_mock_bno.addr_8bit))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      return HAL_ERROR;
    }

    uint8_t page0[64];
    memset(page0, 0, sizeof(page0));
    page0[0x00] = 0xA0U; /* BNO055 CHIP_ID */
    page0[0x07] = g_mock_bno.page;
    bno_encode_i16_le(&page0[0x1A], g_mock_bno.yaw_deg);
    bno_encode_i16_le(&page0[0x1C], g_mock_bno.roll_deg);
    bno_encode_i16_le(&page0[0x1E], g_mock_bno.pitch_deg);
    page0[0x35] = g_mock_bno.calib_stat;
    page0[0x3D] = g_mock_bno.opr_mode;

    for (uint16_t i = 0U; i < Size; i++)
    {
      uint16_t r = (uint16_t)(MemAddress + i);
      pData[i] = (r < sizeof(page0)) ? page0[r] : 0U;
    }
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    return HAL_OK;
  }

  return HAL_ERROR;
}

uint32_t HAL_I2C_GetError(const I2C_HandleTypeDef *hi2c)
{
  return (hi2c != NULL) ? hi2c->ErrorCode : 0U;
}
