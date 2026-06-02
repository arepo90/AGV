#include "proximity.h"
#include "config.h"
#include "estop.h"
#include "log.h"
#include "stm32f0xx.h"
#include "types.h"

#define PROX_PINS_MASK   ((1u<<6) | (1u<<7) | (1u<<8) | (1u<<9))
#define PROX_EXTI_MASK   ((1u<<6) | (1u<<7) | (1u<<8) | (1u<<9))

/* uint16_t so bits 8 and 9 fit alongside 6 and 7. */
static volatile uint16_t s_obstructed = 0;   /* bits 6..9 */

static uint16_t read_obstructed_now(void) {
    uint16_t idr = (uint16_t)GPIOC->IDR;
    uint16_t inactive = idr & PROX_PINS_MASK;
#if PROX_ACTIVE_LOW
    /* obstructed = pin LOW; flip and re-mask to our pins */
    return (uint16_t)((~inactive) & PROX_PINS_MASK);
#else
    return inactive;
#endif
}

uint16_t proximity_obstructed(void) {
    return s_obstructed;
}

void proximity_init(void) {
#if DISABLE_PROXIMITY
    return;
#else
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    /* PC6..PC9 → input mode (00 — reset default). */
    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 |
                      GPIO_MODER_MODER8 | GPIO_MODER_MODER9);

    /* Internal pull-ups enabled. The E18-D80NK is NPN open-collector; with no
     * external pull-up the pin floats when no obstacle is present. */
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7 |
                      GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    GPIOC->PUPDR |=  (1u << (6 * 2)) | (1u << (7 * 2)) |
                     (1u << (8 * 2)) | (1u << (9 * 2));

    /* SYSCFG EXTICR: route EXTI lines 6,7,8,9 to port C (0b0010 in each field). */
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

void EXTI4_15_IRQHandler(void) {
    uint32_t pr = EXTI->PR & PROX_EXTI_MASK;
    if (!pr) return;   /* shared with other EXTI lines 10..15 if ever enabled */

    EXTI->PR = pr;     /* write-1-to-clear only our bits */

    uint16_t prev = s_obstructed;
    uint16_t now  = read_obstructed_now();
    s_obstructed = now;

    if (now && !prev) {
        estop_assert(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_WARN, LOG_CODE_PROX_TRIGGERED, now);
    } else if (!now && prev) {
        estop_clear_autoclearing(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_INFO, LOG_CODE_PROX_CLEARED, prev);
    } else if (now != prev) {
        /* Same E-STOP state, different sensor mask. Pack prev in the high 16 of
         * the log data so both masks fit without overlap. */
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_INFO, LOG_CODE_PROX_TRIGGERED,
                   ((uint32_t)prev << 16) | now);
    }
}
