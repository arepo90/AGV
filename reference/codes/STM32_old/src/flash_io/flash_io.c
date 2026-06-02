#include "flash_io.h"
#include "log.h"
#include "system.h"
#include "types.h"
#include "stm32f0xx.h"
#include <string.h>

/* CMSIS already defines FLASH_KEY1/2 as register-bit masks (different value
 * meaning), so we use distinct names for the unlock-sequence values. */
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
    /* Spin until BSY clears. SR.EOP is set on completion if EOPIE wired; we
     * just check the error flags. With ~30 ms erase, SysTick keeps firing
     * (interrupts continue) but the main loop is paused — we pet the IWDG
     * before calling and again after. */
    while (FLASH->SR & FLASH_SR_BSY) { }
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) {
        FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPERR | FLASH_SR_EOP;  /* clear */
        return false;
    }
    FLASH->SR = FLASH_SR_EOP;
    return true;
}

bool flash_erase_user_page(void) {
    system_iwdg_pet();
    flash_unlock();

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = FLASH_USER_PAGE_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    bool ok = flash_wait_done();
    FLASH->CR &= ~FLASH_CR_PER;

    flash_lock();
    system_iwdg_pet();

    if (!ok) log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR, LOG_CODE_FLASH_WRITE_FAIL, 1);
    return ok;
}

bool flash_write_user_page(const void *src, uint32_t bytes) {
    if (bytes > FLASH_USER_PAGE_SIZE) bytes = FLASH_USER_PAGE_SIZE;

    if (!flash_erase_user_page()) return false;

    flash_unlock();
    FLASH->CR |= FLASH_CR_PG;

    /* Halfword writes. If `bytes` is odd, pad the trailing byte with 0xFF
     * (flash erased state). */
    const uint8_t *p = (const uint8_t *)src;
    uint32_t addr   = FLASH_USER_PAGE_ADDR;

    bool ok = true;
    for (uint32_t i = 0; i < bytes; i += 2) {
        uint16_t hw = p[i];
        if (i + 1 < bytes) hw |= ((uint16_t)p[i+1] << 8);
        else               hw |= 0xFF00u;
        *(volatile uint16_t *)addr = hw;
        if (!flash_wait_done()) { ok = false; break; }
        addr += 2;
    }

    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
    system_iwdg_pet();

    if (!ok) {
        log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR, LOG_CODE_FLASH_WRITE_FAIL, 2);
        return false;
    }

    /* Verify by readback. */
    if (memcmp((const void *)FLASH_USER_PAGE_ADDR, src, bytes) != 0) {
        log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR, LOG_CODE_FLASH_WRITE_FAIL, 3);
        return false;
    }
    return true;
}

void flash_read_user_page(void *dst, uint32_t bytes) {
    if (bytes > FLASH_USER_PAGE_SIZE) bytes = FLASH_USER_PAGE_SIZE;
    memcpy(dst, (const void *)FLASH_USER_PAGE_ADDR, bytes);
}
