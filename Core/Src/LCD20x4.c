#include "lcd20x4.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
static void LCD_Enable(void) {
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

static void LCD_SendNibble(uint8_t nibble) {
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (nibble >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (nibble >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (nibble >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (nibble >> 3) & 0x01);
    LCD_Enable();
}

void LCD_Init(void) {
    // Configure GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_EN_Pin | LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_RS_GPIO_Port, &GPIO_InitStruct);

    // Initialization sequence
    HAL_Delay(50); // Wait for LCD to power up
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_SendNibble(0x03);
    HAL_Delay(5);
    LCD_SendNibble(0x03);
    HAL_Delay(1);
    LCD_SendNibble(0x03);
    LCD_SendNibble(0x02);

    // Configure LCD
    LCD_SendCommand(0x28); // Function set: 4-bit mode, 2 lines, 5x8 dots
    LCD_SendCommand(0x0C); // Display on, cursor off, no blink
    LCD_SendCommand(0x06); // Entry mode: increment automatically, no display shift
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(2);
}

void LCD_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_SendNibble(cmd >> 4);
    LCD_SendNibble(cmd & 0x0F);
}

void LCD_SendData(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_SendNibble(data >> 4);
    LCD_SendNibble(data & 0x0F);
}

void LCD_SendString(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    switch (row) {
        case 0: address = 0x00 + col; break;
        case 1: address = 0x40 + col; break;
        case 2: address = 0x14 + col; break;
        case 3: address = 0x54 + col; break;
        default: address = 0x00 + col; break;
    }
    LCD_SendCommand(0x80 | address);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(2);
}
void LCD_PrintInt(int num) {
    char buffer[16]; // Đủ lớn để chứa số nguyên và ký tự null
    sprintf(buffer, "%d", num);
    LCD_SendString(buffer);
}
