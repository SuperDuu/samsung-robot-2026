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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MainRB2_SetDriveCommand(int16_t linear_rpm_x10, int16_t angular_rpm_x10);
void MainRB2_EnableLineFollow(bool enable, int16_t base_rpm_x10);
void MainRB2_SetEstop(bool estop);
void MainRB2_ClearWheelFaults(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L3_Pin GPIO_PIN_13
#define L3_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_14
#define L2_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_15
#define L1_GPIO_Port GPIOC
#define s3_Pin GPIO_PIN_0
#define s3_GPIO_Port GPIOC
#define s2_Pin GPIO_PIN_1
#define s2_GPIO_Port GPIOC
#define s1_Pin GPIO_PIN_2
#define s1_GPIO_Port GPIOC
#define s0_Pin GPIO_PIN_3
#define s0_GPIO_Port GPIOC
#define RE_DE_Pin GPIO_PIN_1
#define RE_DE_GPIO_Port GPIOA
#define MH_CS_Pin GPIO_PIN_4
#define MH_CS_GPIO_Port GPIOA
#define MH_RST_Pin GPIO_PIN_4
#define MH_RST_GPIO_Port GPIOC
#define MH_DC_Pin GPIO_PIN_5
#define MH_DC_GPIO_Port GPIOC
#define L11_Pin GPIO_PIN_2
#define L11_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_12
#define SW1_GPIO_Port GPIOB
#define LS4_Pin GPIO_PIN_13
#define LS4_GPIO_Port GPIOB
#define LS3_Pin GPIO_PIN_6
#define LS3_GPIO_Port GPIOC
#define LS2_Pin GPIO_PIN_7
#define LS2_GPIO_Port GPIOC
#define LS1_Pin GPIO_PIN_8
#define LS1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOA
#define L10_Pin GPIO_PIN_12
#define L10_GPIO_Port GPIOC
#define L9_Pin GPIO_PIN_2
#define L9_GPIO_Port GPIOD
#define L8_Pin GPIO_PIN_3
#define L8_GPIO_Port GPIOB
#define L7_Pin GPIO_PIN_4
#define L7_GPIO_Port GPIOB
#define L6_Pin GPIO_PIN_5
#define L6_GPIO_Port GPIOB
#define L5_Pin GPIO_PIN_6
#define L5_GPIO_Port GPIOB
#define L4_Pin GPIO_PIN_7
#define L4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
