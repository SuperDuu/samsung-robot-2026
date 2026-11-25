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
#include "PS2_F103.h"
#include "rf_hc12.h"
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
Data du;
RF_HC12_t g_rf;
volatile uint8_t g_tim3_10ms_flag = 0U;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t raw_last;
  uint8_t debounced;
  uint32_t change_ms;
} TdkSwitch_t;

static TdkSwitch_t g_sw_pa8 = {SW_GPIO_Port, SW_Pin, 0U, 0U, 0U};
static TdkSwitch_t g_btn_pb3 = {BUTTON_GPIO_Port, BUTTON_Pin, 0U, 0U, 0U};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void tdk_switch_init(TdkSwitch_t *sw, uint32_t now_ms) {
  /* tay_dk SW1 (PA8) & SW2 (PB3) have 10k pull-ups to +3V3 and pull to GND when
   * pressed (active-LOW) */
  uint8_t raw =
      (HAL_GPIO_ReadPin(sw->port, sw->pin) == GPIO_PIN_RESET) ? 1U : 0U;
  sw->raw_last = raw;
  sw->debounced = raw;
  sw->change_ms = now_ms;
}

static uint8_t tdk_switch_update(TdkSwitch_t *sw, uint32_t now_ms) {
  uint8_t raw =
      (HAL_GPIO_ReadPin(sw->port, sw->pin) == GPIO_PIN_RESET) ? 1U : 0U;
  if (raw != sw->raw_last) {
    sw->raw_last = raw;
    sw->change_ms = now_ms;
  }
  if ((raw != sw->debounced) && ((uint32_t)(now_ms - sw->change_ms) >= 20U)) {
    sw->debounced = raw;
  }
  return sw->debounced;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    g_tim3_10ms_flag = 1U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  RF_HC12_UartErrorCallback(&g_rf, huart);
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
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  RF_HC12_Init(&g_rf, &huart2, NULL, 0U, RF_NODE_TDK, 0U);
  (void)RF_HC12_SetAddress(&g_rf, RF_HC12_DEFAULT_CHANNEL,
                           RF_HC12_DEFAULT_POWER, RF_HC12_DEFAULT_BAUD);

  PS2_Init(&htim14, &du);
  HAL_Delay(150);

  uint32_t boot_ms = HAL_GetTick();
  tdk_switch_init(&g_sw_pa8, boot_ms);
  tdk_switch_init(&g_btn_pb3, boot_ms);

  uint8_t tx_div = 0U;
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_tim3_10ms_flag != 0U) {
      g_tim3_10ms_flag = 0U;
      uint32_t now_ms = HAL_GetTick();

      /* Bit-bang PS2 in thread context so interrupts / SysTick / DMA TX are
       * never blocked */
      PS2_Update();

      uint8_t sw_pressed = tdk_switch_update(&g_sw_pa8, now_ms);
      uint8_t btn_pressed = tdk_switch_update(&g_btn_pb3, now_ms);

      /* Active-HIGH LED_1: solid ON when PS2 responds, blinking when
       * disconnected */
      bool ps2_valid = (du.address == 255U) || (du.address == 0x41U);
      bool ps2_analog = (du.address == 255U);
      if (ps2_valid) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin,
                          (((now_ms / 250U) & 1U) != 0U) ? GPIO_PIN_SET
                                                         : GPIO_PIN_RESET);
      }

      /* Active-HIGH LED_3: ON when AT configured OR user presses SW/BUTTON */
      bool led3_on = (sw_pressed != 0U) || (btn_pressed != 0U) ||
                     (g_rf.at_configured != 0U);
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin,
                        led3_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

      /* Encode mutually-exclusive uint8_t command code via if/else-if
       * (RB2026_TX_PS2_R1 style): eliminates bitmask overlaps and bit flips.
       * Debounce R1 across 2 consecutive 10 ms SPI polls to suppress contact/SPI glitches. */
      static uint8_t s_r1_poll_cnt = 0U;
      if ((du.button[0] == 0U) && (du.button[1] == RF_TDK_CMD_R1_CLAMP)) {
        if (s_r1_poll_cnt < 255U) {
          s_r1_poll_cnt++;
        }
      } else {
        s_r1_poll_cnt = 0U;
      }

      /* 50 Hz RF transmit (20 ms period -> leaves clean ~15 ms TDMA window for
       * manual->auto relay) */
      tx_div++;
      if (tx_div >= 2U) {
        tx_div = 0U;
        RF_PayloadTdk_t cmd;
        cmd.ps2_addr = du.address;
        cmd.joy_sum[0] = du.data[0];
        cmd.joy_sum[1] = du.data[1];
        cmd.joy_left[0] = du.data2[0];
        cmd.joy_left[1] = du.data2[1];

        uint8_t cmd_code = RF_TDK_CMD_NONE;
        if (du.button[1] == RF_TDK_CMD_L2) {
          cmd_code = RF_TDK_CMD_L2;
        } else if (du.button[1] == RF_TDK_CMD_R2) {
          cmd_code = RF_TDK_CMD_R2;
        } else if (du.button[1] == RF_TDK_CMD_L1_BOOST) {
          cmd_code = RF_TDK_CMD_L1_BOOST;
        } else if (du.button[1] == RF_TDK_CMD_L1_L2) {
          cmd_code = RF_TDK_CMD_L1_L2;
        } else if (du.button[1] == RF_TDK_CMD_L1_R2) {
          cmd_code = RF_TDK_CMD_L1_R2;
        } else if (du.button[1] == RF_TDK_CMD_R1_CLAMP) {
          if (s_r1_poll_cnt >= 2U) {
            cmd_code = RF_TDK_CMD_R1_CLAMP;
          }
        } else if (du.button[1] == RF_TDK_CMD_TRIANGLE) {
          cmd_code = RF_TDK_CMD_TRIANGLE;
        } else if (du.button[1] == RF_TDK_CMD_L1_TRIANGLE) {
          cmd_code = RF_TDK_CMD_L1_TRIANGLE;
        } else if (du.button[1] == RF_TDK_CMD_CIRCLE) {
          cmd_code = RF_TDK_CMD_CIRCLE;
        } else if (du.button[1] == RF_TDK_CMD_L1_CIRCLE) {
          cmd_code = RF_TDK_CMD_L1_CIRCLE;
        } else if (du.button[1] == RF_TDK_CMD_CROSS) {
          cmd_code = RF_TDK_CMD_CROSS;
        } else if (du.button[1] == RF_TDK_CMD_L1_CROSS) {
          cmd_code = RF_TDK_CMD_L1_CROSS;
        } else if (du.button[1] == RF_TDK_CMD_SQUARE) {
          cmd_code = RF_TDK_CMD_SQUARE;
        } else if (du.button[1] == RF_TDK_CMD_L1_SQUARE) {
          cmd_code = RF_TDK_CMD_L1_SQUARE;
        }

        /* Right Stick (du.data2) servo commands */
        if (du.data2[0] < 20U) {
          cmd_code = RF_TDK_CMD_SERVO1_FWD;
        } else if (du.data2[0] > 80U) {
          cmd_code = RF_TDK_CMD_SERVO1_REV;
        } else if (du.data2[1] < 20U) {
          cmd_code = RF_TDK_CMD_SERVO2_FWD;
        } else if (du.data2[1] > 80U) {
          cmd_code = RF_TDK_CMD_SERVO2_REV;
        }

        /* Center/D-pad buttons (du.button[0]) & PB3 button override into disjoint [140..213] codes */
        if (btn_pressed != 0U) {
          cmd_code = RF_TDK_CMD_START_AUTO;
        } else if (du.button[0] == 1U) {
          if (du.button[1] == 0U) {
            cmd_code = RF_TDK_CMD_SELECT;
          } else if (du.button[1] == 4U) {
            cmd_code = RF_TDK_CMD_SELECT_L1;
          } else if (du.button[1] == 8U) {
            cmd_code = RF_TDK_CMD_SELECT_R1;
          }
        } else if (du.button[0] == 9U) {
          cmd_code = RF_TDK_CMD_SELECT_START;
        } else if (du.button[0] == 2U) {
          cmd_code = RF_TDK_CMD_L3;
        } else if (du.button[0] == 4U) {
          cmd_code = RF_TDK_CMD_R3;
        } else if (du.button[0] == 8U) {
          cmd_code = RF_TDK_CMD_START_AUTO;
        } else if (du.button[0] == 16U) {
          cmd_code = RF_TDK_CMD_DPAD_UP;
        } else if (du.button[0] == 32U) {
          cmd_code = RF_TDK_CMD_DPAD_RIGHT;
        } else if (du.button[0] == 64U) {
          cmd_code = RF_TDK_CMD_DPAD_DOWN;
        } else if (du.button[0] == 128U) {
          cmd_code = RF_TDK_CMD_DPAD_LEFT;
        }

        cmd.ps2_btn[1] = cmd_code;
        cmd.ps2_btn[0] = (uint8_t)(cmd_code ^ RF_TDK_CMD_TAG_XOR);
        cmd.ext_sw = (sw_pressed ? RF_TDK_EXT_SW_BIT : 0U) |
                     (btn_pressed ? RF_TDK_EXT_BUTTON_BIT : 0U) |
                     (ps2_valid ? RF_TDK_PS2_VALID_BIT : 0U) |
                     (ps2_analog ? RF_TDK_PS2_ANALOG_BIT : 0U);
        cmd.aux = (g_rf.at_configured != 0U) ? 1U : 0U;

        if (RF_HC12_SendTdkCmd(&g_rf, RF_NODE_MANUAL, &cmd) == HAL_OK) {
          HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
        }
      }
    }
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
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
static void MX_TIM14_Init(void) {

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 127;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_PS2_GPIO_Port, DO_PS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CLK_PS2_Pin | CS_PS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_3_Pin | BUZZER_Pin | LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DO_PS2_Pin */
  GPIO_InitStruct.Pin = DO_PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DO_PS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DI_PS2_Pin */
  GPIO_InitStruct.Pin = DI_PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DI_PS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_PS2_Pin CS_PS2_Pin */
  GPIO_InitStruct.Pin = CLK_PS2_Pin | CS_PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin BUZZER_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin | BUZZER_Pin | LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
