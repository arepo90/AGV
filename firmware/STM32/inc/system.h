#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 * System core: clock tree, SysTick, watchdog, reset cause.
 *
 * SysTick fires at SYSTICK_HZ (1 kHz) and increments the millisecond counter.
 * The main loop reads system_now_ms() to drive the cooperative scheduler.
 * IWDG is petted from main loop only — if it expires the chip resets.
 * =============================================================================
 */

typedef enum {
    RESET_CAUSE_UNKNOWN  = 0,
    RESET_CAUSE_POWER_ON = 1,
    RESET_CAUSE_PIN      = 2,
    RESET_CAUSE_SOFTWARE = 3,
    RESET_CAUSE_WATCHDOG = 4,
    RESET_CAUSE_LOW_POWER = 5,
} reset_cause_t;

void          system_init(void);                 /* PLL → 48 MHz, SysTick, capture reset cause */
void          system_iwdg_init(void);            /* configures IWDG and starts it */
void          system_iwdg_pet(void);             /* call from main loop only */
void          system_soft_reset(void);           /* triggers NVIC system reset */
uint32_t      system_now_ms(void);               /* monotonic ms since boot (rolls every ~49 days) */
reset_cause_t system_reset_cause(void);

#endif /* SYSTEM_H */
