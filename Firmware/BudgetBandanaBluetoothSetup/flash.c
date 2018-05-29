#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx.h"
#include "flash.h"

void FLASH_waitBusy(void) {
    while(FLASH->SR & FLASH_SR_BSY) {}
}

void FLASH_unlock(void) {
    FLASH_waitBusy();

    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    FLASH_waitBusy();
}


void FLASH_lock(void) {
    FLASH_waitBusy();
    FLASH->CR |= FLASH_CR_LOCK;
    FLASH_waitBusy();
}

// addr may be any one for the page to erase
void FLASH_erasePage(uint32_t addr) {
    FLASH_waitBusy();

    FLASH->CR |= FLASH_CR_PER; // one page
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT; // start
    asm volatile(       // Wait for busy-flag. See STM32F100x4/6/8/B Errata sheet, 2.9
        " nop " "\n\t"
        " nop " "\n\t"
    );

    FLASH_waitBusy();
    FLASH->CR &= ~FLASH_CR_PER;
}


void FLASH_write(uint32_t addr, uint32_t const * data, size_t dataLen) {
    FLASH_waitBusy();
    FLASH->CR |= FLASH_CR_PG; // programming
    FLASH_waitBusy();

    for(; dataLen > 0; ++data, --dataLen) {
        uint32_t v = *data;
        *(volatile uint16_t*)addr = (uint16_t)v;
        FLASH_waitBusy();

        addr += 2;
        v >>= 16;
        *(volatile uint16_t*)addr = (uint16_t)v;
        FLASH_waitBusy();

        addr += 2;
    }

    FLASH->CR &= ~FLASH_CR_PG;
}

uint32_t FLASH_Read(uint32_t address)
{
    return (*(__IO uint32_t*)address);
}
