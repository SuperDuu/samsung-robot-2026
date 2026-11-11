/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DO_PS2_Pin GPIO_PIN_7
#define DO_PS2_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_9
#define LED_2_GPIO_Port GPIOB
#define DI_PS2_Pin GPIO_PIN_15
#define DI_PS2_GPIO_Port GPIOC
#define CLK_PS2_Pin GPIO_PIN_1
#define CLK_PS2_GPIO_Port GPIOA
#define CS_PS2_Pin GPIO_PIN_4
#define CS_PS2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_8
#define SW_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_3
#define BUTTON_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
