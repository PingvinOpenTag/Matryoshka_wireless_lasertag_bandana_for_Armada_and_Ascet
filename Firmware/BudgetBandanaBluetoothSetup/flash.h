#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx.h"

#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xCDEF89AB


#define APP_START        0x08002800 // 8k for bootloader
#define APP_END          0x0800FC00
#define VTOR_SIZE              0xC0
#define PAGE_SIZE              1024


void FLASH_waitBusy(void);
void FLASH_unlock(void);
void FLASH_lock(void);
void FLASH_erasePage(uint32_t addr);
void FLASH_write(uint32_t addr, uint32_t const * data, size_t dataLen);
uint32_t FLASH_Read(uint32_t address);

#endif
