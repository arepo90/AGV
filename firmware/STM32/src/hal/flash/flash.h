#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* =============================================================================
 *  Raw flash access for persistent calibration storage on STM32F051. (hal tier)
 *
 *  F0 flash is 1 KB pages, halfword-aligned programming. We reserve the LAST
 *  page (0x0800FC00..0x0800FFFF) for calibration. The schema (magic/version/CRC)
 *  is the caller's concern — this layer only erases, writes, and reads bytes.
 *
 *  Erase ~30 ms and writes ~30 µs each stall the bus, but we only call from the
 *  main loop and pet the IWDG around the operation. Returns false on failure;
 *  the caller logs.
 * =============================================================================
 */

#define FLASH_USER_PAGE_SIZE   1024u
#define FLASH_USER_PAGE_ADDR   0x0800FC00u   /* last 1 KB of 64 KB flash */

bool flash_erase_page(void);
bool flash_write_page(const void *src, uint32_t bytes);   /* erases first */
void flash_read_page(void *dst, uint32_t bytes);

#endif /* FLASH_H */
