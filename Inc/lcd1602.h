
#ifndef __LCD1602_H
#define __LCD1602_H	 
#include <stdint.h>
#include "stm32f4xx_hal.h"

//1602液晶指令/数据选择引脚
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define	LCD_RS_Set()	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET)
#define	LCD_RS_Clr()	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET)

//1602液晶读写引脚
#define	LCD_RW_Set()	HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET)
#define	LCD_RW_Clr()	HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET)

//1602液晶使能引脚
#define	LCD_EN_Set()	HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET)
#define	LCD_EN_Clr()	HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET)

//1602液晶数据端口	PD0~7
//#define	DATAOUT(x)	 GPIOD->ODR |= (x << 8)

void DATAOUT(u8 x);
void LCD1602_Wait_Ready(void);
void LCD1602_Write_Cmd(u8 cmd);
void LCD1602_Write_Dat(u8 dat);
void LCD1602_ClearScreen(void);
void LCD1602_Set_Cursor(u8 x, u8 y);
void LCD1602_Show_Str(u8 x, u8 y, u8 *str);
void LCD1602_Init(void);

#endif

