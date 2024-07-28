/*
 * Flash.h
 *
 *  Created on: Jun 20, 2024
 *      Author: ASUS
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_
#include <main.h>

void FlashErase (uint32_t address);
void FlashWrite (int value , uint32_t address);
int FlashRead (uint32_t address);


#endif /* INC_FLASH_H_ */
