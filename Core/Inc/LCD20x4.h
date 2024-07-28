#ifndef LCD20X4_H_
#define LCD20X4_H_

#include "stm32f1xx_hal.h"

#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_12
#define LCD_EN_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_8
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_9
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_10
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_11
#define LCD_D7_GPIO_Port GPIOA

void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Clear(void);
void LCD_PrintInt(int num);

#endif /* LCD20X4_H_ */
