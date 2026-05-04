#ifndef FLASH_IO_H
#define FLASH_IO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* =============================================================================
 *  Raw flash access for persistent calibration storage on STM32F051.
 *
 *  F0 flash is organised in 1 KB pages. Programming is halfword-aligned.
 *  Erase takes ~30 ms; halfword writes ~30 µs each. Both stall the bus, so
 *  any flash-resident code freezes during the operation — irrelevant since
 *  we only call from main loop and the IWDG window (500 ms) easily covers it.
 *
 *  We reserve the LAST 1 KB page (0x0800FC00..0x0800FFFF) for calibration.
 *  Schema is owned by the caller (e.g. nav_line.c writes a versioned struct
 *  with its own CRC).
 *
 *  All functions return false on failure and log via LOG_CODE_FLASH_*.
 * =============================================================================
 */

#define FLASH_USER_PAGE_SIZE   1024u
#define FLASH_USER_PAGE_ADDR   0x0800FC00u   /* last 1 KB of 64 KB flash */

bool flash_erase_user_page(void);
bool flash_write_user_page(const void *src, uint32_t bytes);   /* erases first */

/* Convenience: copy raw flash bytes into RAM (just memcpy from constant addr). */
void flash_read_user_page(void *dst, uint32_t bytes);

#endif /* FLASH_IO_H */
