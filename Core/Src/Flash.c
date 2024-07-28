/*
 * Flash.c
 *
 *  Created on: Jun 20, 2024
 *      Author: ASUS
 */
#include <Flash.h>

void FlashErase (uint32_t address)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.Banks = 1;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.PageAddress = address;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t pageeerr;
	HAL_FLASHEx_Erase(&EraseInitStruct , &pageeerr);
	HAL_FLASH_Lock();
}
void FlashWrite (int value , uint32_t address)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD , address , value);
	HAL_FLASH_Lock();
}
int FlashRead (uint32_t address)
{
	return  *(__IO uint16_t*) (address);
}
