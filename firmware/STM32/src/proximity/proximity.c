#include "proximity.h"
#include "config.h"
#include "estop.h"
#include "log.h"
#include "stm32f0xx.h"
#include "types.h"

#define PROX_PINS_MASK   ((1u<<6) | (1u<<7) | (1u<<8) | (1u<<9))
#define PROX_EXTI_MASK   ((1u<<6) | (1u<<7) | (1u<<8) | (1u<<9))

static volatile uint8_t s_obstructed = 0;   /* bits 6..9 */

static uint8_t read_obstructed_now(void) {
    uint16_t idr = (uint16_t)GPIOC->IDR;
    uint16_t inactive = idr & PROX_PINS_MASK;
#if PROX_ACTIVE_LOW
    /* obstructed = pin LOW; mask the inverse, restricted to our pins */
    uint16_t obstructed = (~inactive) & PROX_PINS_MASK;
#else
    uint16_t obstructed = inactive;
#endif
    return (uint8_t)obstructed;
}

uint8_t proximity_obstructed(void) {
    return s_obstructed;
}

void proximity_init(void) {
#if DISABLE_PROXIMITY
    return;
#else
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    /* PC6..PC9 → input mode (00 — reset default), with internal pull-ups so
     * a disconnected sensor reads HIGH (no obstacle). */
    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 |
                      GPIO_MODER_MODER8 | GPIO_MODER_MODER9);

    /* SYSCFG EXTICR: route EXTI lines 6,7 to PC; lines 8,9 to PC.
     * Each EXTICR field is 4 bits; port C = 0b0010. */
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~((0xFu << 8) | (0xFu << 12)))
                      | (0x2u << 8)  | (0x2u << 12);   /* lines 6,7 */
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~((0xFu << 0) | (0xFu << 4)))
                      | (0x2u << 0)  | (0x2u << 4);    /* lines 8,9 */

    /* Both edges, unmasked. */
    EXTI->IMR  |= PROX_EXTI_MASK;
    EXTI->RTSR |= PROX_EXTI_MASK;
    EXTI->FTSR |= PROX_EXTI_MASK;

    /* Capture initial state so we don't enable motors with an obstacle in front. */
    s_obstructed = read_obstructed_now();
    if (s_obstructed) {
        estop_assert(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_WARN, LOG_CODE_PROX_TRIGGERED, s_obstructed);
    }

    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
#endif
}

/* Shared with comms/system at NVIC level — runs on EXTI line 4..15 events. */
void EXTI4_15_IRQHandler(void) {
    uint32_t pr = EXTI->PR & PROX_EXTI_MASK;
    if (!pr) return;

    EXTI->PR = pr;   /* clear pending bits (write 1 to clear) */

    uint8_t prev = s_obstructed;
    uint8_t now  = read_obstructed_now();
    s_obstructed = now;

    if (now && !prev) {
        estop_assert(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_WARN, LOG_CODE_PROX_TRIGGERED, now);
    } else if (!now && prev) {
        estop_clear_autoclearing(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_INFO, LOG_CODE_PROX_CLEARED, prev);
    } else if (now != prev) {
        /* Same E-STOP state but different sensor mask — log for diagnostics. */
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_INFO, LOG_CODE_PROX_TRIGGERED,
                   ((uint32_t)prev << 8) | now);
    }
}
