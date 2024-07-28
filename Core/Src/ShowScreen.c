/*
 * hienthi.c
 *
 *  Created on: Jun 14, 2024
 *      Author: ASUS
 */

#include <ShowScreen.h>
#include <LCD20x4.h>


void hienthi (uint16_t NhietDo , uint16_t DoAm)
{
	LCD_SetCursor(0,0);
	LCD_SendString("Nhiet do:");
	LCD_SetCursor(0,11);
	LCD_SendString("        ");
	LCD_SetCursor(0,11);
	LCD_PrintInt(NhietDo);
	LCD_SetCursor(1,0);
	LCD_SendString("Do am:");
	LCD_SetCursor(1,11);
	LCD_SendString("        ");
	LCD_SetCursor(1,11);
	LCD_PrintInt(DoAm);

}


