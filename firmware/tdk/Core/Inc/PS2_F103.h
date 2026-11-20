/*
 * PS2_F103.h
 *
 *  Created on: Apr 13, 2025
 *      Author: htbt2
 *  Remapped for tay_dk / STM32G030F6Px (PB7=DO_PS2/MOSI, PA1=CLK_PS2/SCK, PC15=DI_PS2/MISO, PA4=CS_PS2/SS)
 */

#ifndef INC_PS2_F103_H_
#define INC_PS2_F103_H_

#include "main.h"
#define MOSI_GPIO_Port DO_PS2_GPIO_Port
#define MOSI_Pin       DO_PS2_Pin
#define SCK_GPIO_Port  CLK_PS2_GPIO_Port
#define SCK_Pin        CLK_PS2_Pin
#define MISO_GPIO_Port DI_PS2_GPIO_Port
#define MISO_Pin       DI_PS2_Pin
#define SS_GPIO_Port   CS_PS2_GPIO_Port
#define SS_Pin         CS_PS2_Pin
typedef struct{
	uint8_t address;
	uint8_t data[2];
	uint8_t button[2];
	uint8_t data2[2];
}Data;

void delay_us(uint16_t);
uint8_t PS2_SendByte(uint8_t);
void PS2_Cmd(uint8_t *, uint8_t *, uint8_t );
void PS2_Poll(void);
void PS2_EnterConfig(void);
void PS2_AnalogMode(void);
void PS2_ExitConfig(void);
void PS2_Init(TIM_HandleTypeDef*, Data*);
void PS2_Update(void);

#endif /* INC_PS2_F103_H_ */

