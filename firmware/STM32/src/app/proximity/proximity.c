#include "proximity.h"
#include "config.h"
#include "log.h"
#include "safety.h"
#include "stm32f0xx.h"

#define PROX_MASK ((1u<<6) | (1u<<7) | (1u<<8) | (1u<<9))   /* PC6..PC9 / EXTI6..9 */

static volatile uint16_t s_obstructed = 0;   /* bits 6..9 */

static uint16_t read_now(void) {
    uint16_t pins = (uint16_t)(GPIOC->IDR) & PROX_MASK;
#if PROX_ACTIVE_LOW
    return (uint16_t)((~pins) & PROX_MASK);   /* obstructed = pin LOW */
#else
    return pins;
#endif
}

uint16_t proximity_obstructed(void) { return s_obstructed; }

void proximity_init(void) {
#if DISABLE_PROXIMITY
    return;
#else
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 |
                      GPIO_MODER_MODER8 | GPIO_MODER_MODER9);   /* inputs */
    /* Internal pull-ups: the NPN open-collector sensor floats when clear. */
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7 |
                      GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    GPIOC->PUPDR |=  (1u << (6*2)) | (1u << (7*2)) | (1u << (8*2)) | (1u << (9*2));

    /* Route EXTI 6,7,8,9 → port C (0b0010 per nibble). */
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~((0xFu << 8) | (0xFu << 12)))
                      | (0x2u << 8) | (0x2u << 12);
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~((0xFu << 0) | (0xFu << 4)))
                      | (0x2u << 0) | (0x2u << 4);

    EXTI->IMR  |= PROX_MASK;
    EXTI->RTSR |= PROX_MASK;
    EXTI->FTSR |= PROX_MASK;

    /* Capture initial state so we don't enable motors facing an obstacle. */
    s_obstructed = read_now();
    if (s_obstructed) {
        safety_estop_assert(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_WARN, LOG_CODE_PROX_TRIGGERED, s_obstructed);
    }

    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
#endif
}

void EXTI4_15_IRQHandler(void) {
    uint32_t pr = EXTI->PR & PROX_MASK;
    if (!pr) return;
    EXTI->PR = pr;   /* write-1-to-clear our bits */

    uint16_t prev = s_obstructed;
    uint16_t now  = read_now();
    s_obstructed = now;

    if (now && !prev) {
        safety_estop_assert(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_WARN, LOG_CODE_PROX_TRIGGERED, now);
    } else if (!now && prev) {
        safety_estop_clear_autoclearing(ESTOP_SRC_PROXIMITY);
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_INFO, LOG_CODE_PROX_CLEARED, prev);
    } else if (now != prev) {
        log_record(LOG_MOD_PROXIMITY, LOG_SEV_INFO, LOG_CODE_PROX_TRIGGERED,
                   ((uint32_t)prev << 16) | now);
    }
}
