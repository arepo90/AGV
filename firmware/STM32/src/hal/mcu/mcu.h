#ifndef MCU_H
#define MCU_H

#include <stdint.h>
#include "types.h"

/* =============================================================================
 *  MCU core: clock tree, SysTick, watchdog, reset cause.  (hal tier)
 *
 *  PLL brings SYSCLK to 48 MHz; SysTick fires at SYSTICK_HZ (1 kHz) and drives
 *  the millisecond counter the cooperative scheduler in main.c reads. The IWDG
 *  is petted from the main loop only — if the loop stalls, the chip resets.
 * =============================================================================
 */

void          mcu_init(void);            /* PLL → 48 MHz, SysTick, capture reset cause */
uint32_t      mcu_now_ms(void);          /* monotonic ms since boot (wraps ~49 days) */
reset_cause_t mcu_reset_cause(void);

void          mcu_iwdg_init(void);       /* configure + start IWDG */
void          mcu_iwdg_pet(void);        /* call from main loop only */

void          mcu_soft_reset(void);      /* NVIC system reset request */
void          mcu_delay_ms(uint32_t ms); /* busy-wait on SysTick; init sequencing only */

#endif /* MCU_H */
