/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PidBoard (STM32G030F6Px / HB_RBC) 1 kHz Motor PID + RS-485 Slave
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
#include "pid_controller.h"
#include "rs485_bus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
PID_Controller_t g_pid = {0};
RS485_Bus_t g_rs485 = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void pid_apply_motor_outputs(uint8_t gate_enable, int32_t hw_duty_permille);
static void pid_handle_rx_frame(const RS485_Frame_t *frame, uint32_t now_ms);
static void pid_on_rs485_rx(RS485_Bus_t *bus, const RS485_Frame_t *frame, void *ctx);
static void pid_update_status_leds(uint32_t now_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void pid_apply_motor_outputs(uint8_t gate_enable, int32_t hw_duty_permille)
{
  if (gate_enable == 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    HAL_GPIO_WritePin(S_GPIO_Port, S_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
    return;
  }

  /* Enable both IR2184 ~SD inputs; PWM duty is clamped <= 975 to guarantee bootstrap recharge */
  HAL_GPIO_WritePin(S_GPIO_Port, S_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_SET);

  if (hw_duty_permille > 0)
  {
    uint32_t ccr = (uint32_t)hw_duty_permille;
    if (ccr > (uint32_t)PID_MAX_DUTY_PERMILLE)
    {
      ccr = (uint32_t)PID_MAX_DUTY_PERMILLE;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
  }
  else if (hw_duty_permille < 0)
  {
    uint32_t ccr = (uint32_t)(-hw_duty_permille);
    if (ccr > (uint32_t)PID_MAX_DUTY_PERMILLE)
    {
      ccr = (uint32_t)PID_MAX_DUTY_PERMILLE;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
  }
  else
  {
    /* Active low-side brake: both IN pins low while ~SD remains HIGH */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  }
}

static void pid_handle_rx_frame(const RS485_Frame_t *frame, uint32_t now_ms)
{
  if (frame == NULL)
  {
    return;
  }

  if (frame->msg_type == (uint8_t)RS485_MSG_MULTI_CMD)
  {
    RS485_MultiCmd_t multi;
    if (!RS485_DecodeMultiCmd(frame, &multi))
    {
      return;
    }

    uint8_t node_id = g_pid.cfg.node_id;
    uint8_t slot_idx = (node_id >= RS485_ADDR_SLAVE_MIN) ? (uint8_t)(node_id - 1U) : 0xFFU;
    bool need_telem = false;
    RS485_Telem_t telem = {0};

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((multi.flags & RS485_CMD_FLAG_ESTOP) != 0U)
    {
      PID_ApplyCommand(&g_pid, RS485_WHEEL_DISABLED, 0, multi.flags, now_ms);
    }
    else if (slot_idx < multi.num_wheels)
    {
      PID_ApplyCommand(&g_pid,
                       (RS485_WheelMode_t)multi.wheels[slot_idx].mode,
                       (int32_t)multi.wheels[slot_idx].target,
                       multi.flags,
                       now_ms);
    }

    if (multi.poll_id == node_id)
    {
      PID_BuildTelem(&g_pid, &telem);
      need_telem = true;
    }
    if (primask == 0U)
    {
      __enable_irq();
    }

    if (need_telem)
    {
      (void)RS485_ScheduleTelemReply(&g_rs485, frame->src_id, &telem, now_ms);
    }
  }
  else if (frame->msg_type == (uint8_t)RS485_MSG_WHEEL_CMD)
  {
    RS485_WheelCmd_t wcmd;
    if (!RS485_DecodeWheelCmd(frame, &wcmd))
    {
      return;
    }

    RS485_Telem_t telem = {0};
    bool unicast = (frame->dst_id == g_pid.cfg.node_id);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_ApplyCommand(&g_pid, (RS485_WheelMode_t)wcmd.mode, wcmd.target, wcmd.flags, now_ms);
    if (unicast)
    {
      PID_BuildTelem(&g_pid, &telem);
    }
    if (primask == 0U)
    {
      __enable_irq();
    }

    if (unicast)
    {
      (void)RS485_ScheduleTelemReply(&g_rs485, frame->src_id, &telem, now_ms);
    }
  }
  else if (frame->msg_type == (uint8_t)RS485_MSG_CFG_CMD)
  {
    RS485_CfgCmd_t ccmd;
    if (!RS485_DecodeCfgCmd(frame, &ccmd))
    {
      return;
    }

    bool save_deferred = false;
    uint8_t status;
    uint8_t updated_id;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    status = PID_HandleConfigSubcmd(&g_pid, &ccmd, &save_deferred);
    updated_id = g_pid.cfg.node_id;
    if (primask == 0U)
    {
      __enable_irq();
    }
    (void)save_deferred;

    RS485_SetLocalId(&g_rs485, updated_id);

    if (frame->dst_id != RS485_ADDR_BCAST)
    {
      RS485_CfgAck_t ack = {0};
      ack.node_id  = updated_id;
      ack.subcmd   = ccmd.subcmd;
      ack.status   = status;
      ack.reserved = 0U;
      (void)RS485_ScheduleCfgAckReply(&g_rs485, frame->src_id, &ack, now_ms);
    }
  }
}

static void pid_on_rs485_rx(RS485_Bus_t *bus, const RS485_Frame_t *frame, void *ctx)
{
  (void)ctx;
  if ((bus != NULL) && (frame != NULL))
  {
    pid_handle_rx_frame(frame, bus->last_rx_tick_ms);
  }
}

static void pid_update_status_leds(uint32_t now_ms)
{
  /* LED1 (PC15, active-LOW): solid ON when RS-485 bus active, 1 Hz blink when idle */
  bool link_alive = RS485_IsLinkAlive(&g_rs485, now_ms, PID_CMD_WATCHDOG_MS);
  GPIO_PinState led1_state = (link_alive || (((now_ms / 500U) & 1U) == 0U))
                               ? GPIO_PIN_RESET
                               : GPIO_PIN_SET;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led1_state);

  /* LED2 (PC14, active-LOW): ON on E-STOP / watchdog / comm error / saturation, OFF when healthy */
  bool fault_or_sat = (g_pid.watchdog_tripped != 0U) ||
                      (g_pid.estop_latched != 0U) ||
                      (g_pid.comm_err_latched != 0U) ||
                      (g_pid.saturated != 0U);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, fault_or_sat ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  HAL_GPIO_WritePin(S_GPIO_Port, S_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  PID_Init(&g_pid, (uint16_t)__HAL_TIM_GET_COUNTER(&htim3), HAL_GetTick());

  RS485_Init(&g_rs485, &huart2, RE_DE_GPIO_Port, RE_DE_Pin, g_pid.cfg.node_id);
  RS485_SetRxCallback(&g_rs485, pid_on_rs485_rx, NULL);
  if (RS485_StartRx(&g_rs485) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  uint32_t seen_uart_errs = g_rs485.stats.uart_err_recoveries;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    RS485_ProcessRx(&g_rs485, now_ms);

    uint32_t err_cnt = g_rs485.stats.uart_err_recoveries;
    if (err_cnt != seen_uart_errs)
    {
      seen_uart_errs = err_cnt;
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      g_pid.comm_err_latched = 1U;
      if (primask == 0U)
      {
        __enable_irq();
      }
    }

    RS485_TickMs(&g_rs485, now_ms);

    if ((g_pid.pending_flash_save != 0U) &&
        (g_rs485.pending_reply_valid == 0U) &&
        (g_rs485.tx_active == 0U) &&
        (huart2.gState == HAL_UART_STATE_READY))
    {
      (void)HAL_TIM_Base_Stop_IT(&htim14);
      pid_apply_motor_outputs(0U, 0);
      PID_ResetControlState(&g_pid);
      g_pid.mode = RS485_WHEEL_DISABLED;
      (void)PID_SaveConfigFlash(&g_pid);
      g_pid.pending_flash_save = 0U;
      RS485_SetLocalId(&g_rs485, g_pid.cfg.node_id);
      (void)HAL_TIM_Base_Start_IT(&htim14);
    }

    pid_update_status_leds(now_ms);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 63;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RE_DE_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S_GPIO_Port, S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RE_DE_Pin */
  GPIO_InitStruct.Pin = RE_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RE_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S2_Pin */
  GPIO_InitStruct.Pin = S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S_Pin */
  GPIO_InitStruct.Pin = S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14)
  {
    uint16_t hw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    PID_Step1kHz(&g_pid, hw_cnt, HAL_GetTick());
    pid_apply_motor_outputs(g_pid.gate_enable, g_pid.hw_duty_permille);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  RS485_TxCpltCallback(&g_rs485, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  RS485_UartErrorCallback(&g_rs485, huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  HAL_GPIO_WritePin(S_GPIO_Port, S_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
