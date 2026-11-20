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
typedef struct
{
  uint16_t adc_raw;            /* Raw 12-bit ADC value (0..4095) */
  uint8_t active;              /* 1 = seeing line */
} LineEye_Debug_t;

typedef struct
{
  /* 1. TDK (HC-12 Remote Control) */
  struct {
    uint8_t  connected;        /* 1 = live link, 0 = lost */
    uint8_t  joy_y;            /* raw joy Y (0..200, center 100) */
    uint8_t  joy_x;            /* raw joy X (0..200, center 100) */
    float    target_y;         /* Joy Y: -100.0f .. +100.0f */
    float    target_x;         /* Joy X: -100.0f .. +100.0f */
    uint8_t  btn1;             /* du.button[1]: 8 action buttons / combos (120, 121...) */
    uint8_t  btn0;             /* du.button[0]: D-pad buttons */
    uint8_t  cnt;              /* rolling counter (0..249) */
    uint32_t last_rx_ms;       /* timestamp of last received frame */
    uint32_t rx_packet_count;  /* total valid frames received */
  } tdk;

  /* 2. RS-485 Motor Bus */
  struct {
    uint8_t  tx_cnt;           /* rolling watchdog counter sent to PID boards (0..255..0) */
    uint8_t  gain_db;          /* speed multiplier (db - 7) */
    uint8_t  motor_cmd[4];     /* raw bytes sent to 4 wheel PID boards (0..200, center 100) */
    int16_t  motor_rpm[4];     /* target RPM for 4 wheels */
    uint8_t  lift_cmd;         /* lift motor command (0..200, center 100) */
    uint32_t last_tx_ms;       /* timestamp of last RS-485 broadcast */
  } rs485;

  /* 3. IMU (BNO055 / BNO08x) */
  struct {
    uint8_t connected;
    float   yaw;               /* current yaw in degrees (-180..+180) */
    float   roll;
    float   pitch;
    float   target_heading;    /* field-centric reference */
  } imu;

  /* 4. Line Sensors (CD4067 16-channel ADC) */
  struct {
    uint16_t adc_raw[16];      /* Raw 12-bit ADC values (0..4095) for all 16 channels */
    uint16_t active_mask;      /* Bitmask of channels seeing line */
    uint8_t  active_count;     /* Number of active sensors */
    int16_t  line_error;       /* Centroid position / error */
    LineEye_Debug_t front[8];   /* IN9..IN16, right to left */
    LineEye_Debug_t rear[8];    /* IN1..IN8 */
  } line;

  /* 5. Board Inputs / States */
  struct {
    uint8_t sw1;               /* SW1 button state */
    uint8_t sw2;               /* SW2 button state */
    uint8_t blue_side;         /* 1 = blue side, 0 = red side */
  } board;

} Robot_Debug_t;

extern Robot_Debug_t g_debug;
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
#define BAUMER_INPUT_Pin GPIO_PIN_14
#define BAUMER_INPUT_GPIO_Port GPIOB
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
