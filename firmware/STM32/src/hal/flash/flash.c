#include "flash.h"
#include "mcu.h"
#include "stm32f0xx.h"
#include <string.h>

/* CMSIS defines FLASH_KEY1/2 as register-bit masks; use distinct names for the
 * unlock-sequence values. */
#define FLASH_UNLOCK_KEY_A   0x45670123u
#define FLASH_UNLOCK_KEY_B   0xCDEF89ABu

static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_UNLOCK_KEY_A;
        FLASH->KEYR = FLASH_UNLOCK_KEY_B;
    }
}

static void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

static bool flash_wait_done(void) {
    while (FLASH->SR & FLASH_SR_BSY) { }
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) {
        FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPERR | FLASH_SR_EOP;   /* clear */
        return false;
    }
    FLASH->SR = FLASH_SR_EOP;
    return true;
}

bool flash_erase_page(void) {
    mcu_iwdg_pet();
    flash_unlock();

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = FLASH_USER_PAGE_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    bool ok = flash_wait_done();
    FLASH->CR &= ~FLASH_CR_PER;

    flash_lock();
    mcu_iwdg_pet();
    return ok;
}

bool flash_write_page(const void *src, uint32_t bytes) {
    if (bytes > FLASH_USER_PAGE_SIZE) bytes = FLASH_USER_PAGE_SIZE;
    if (!flash_erase_page()) return false;

    flash_unlock();
    FLASH->CR |= FLASH_CR_PG;

    const uint8_t *p = (const uint8_t *)src;
    uint32_t addr = FLASH_USER_PAGE_ADDR;
    bool ok = true;
    for (uint32_t i = 0; i < bytes; i += 2) {
        uint16_t hw = p[i];
        if (i + 1 < bytes) hw |= (uint16_t)((uint16_t)p[i + 1] << 8);
        else               hw |= 0xFF00u;   /* pad odd tail with erased state */
        *(volatile uint16_t *)addr = hw;
        if (!flash_wait_done()) { ok = false; break; }
        addr += 2;
    }

    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
    mcu_iwdg_pet();

    if (!ok) return false;
    /* Verify by readback. */
    return memcmp((const void *)FLASH_USER_PAGE_ADDR, src, bytes) == 0;
}

void flash_read_page(void *dst, uint32_t bytes) {
    if (bytes > FLASH_USER_PAGE_SIZE) bytes = FLASH_USER_PAGE_SIZE;
    memcpy(dst, (const void *)FLASH_USER_PAGE_ADDR, bytes);
}
