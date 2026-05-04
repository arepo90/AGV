#include "motors.h"
#include "config.h"
#include "stm32f0xx.h"

static bool s_enabled = false;

void motors_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    /* CRITICAL: drive SLEEP pins LOW *before* switching to output mode, so the
     * pins never glitch HIGH during configuration and accidentally wake the
     * driver while we're still booting. */
    GPIOB->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_3;

    /* PB1, PB3 → general-purpose output, push-pull (default), low speed. */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER3))
                 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER3_0;

    s_enabled = false;
}

void motors_set_enabled(bool enabled) {
    if (enabled) {
        GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_3;
    } else {
        GPIOB->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_3;
    }
    s_enabled = enabled;
}

bool motors_enabled(void) {
    return s_enabled;
}
