#include "pwm.h"
#include "config.h"
#include "stm32f0xx.h"

/* DIR bits on GPIOB: ch0 → PB0, ch1 → PB2.  SLEEP bits: PB1, PB3. */

static void gpio_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* SLEEP LOW *before* switching the pins to outputs, so the driver can't
     * glitch awake during configuration. */
    GPIOB->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_3;

    /* PB0/PB1/PB2/PB3 → general-purpose push-pull outputs. */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                                     GPIO_MODER_MODER2 | GPIO_MODER_MODER3))
                 | GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0
                 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0;
    GPIOB->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_2;   /* DIR forward */

    /* PA8 → AF2 (TIM1_CH1), PA11 → AF2 (TIM1_CH4). */
    GPIOA->MODER  = (GPIOA->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11))
                  | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER11_1;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFu << 0) | (0xFu << 12)))
                  | (2u << 0) | (2u << 12);
}

static void tim1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->CR1  = 0;
    TIM1->PSC  = 0;
    TIM1->ARR  = (uint32_t)(PWM_PERIOD - 1u);
    TIM1->CCR1 = 0;
    TIM1->CCR4 = 0;

    /* PWM mode 1, output preload on — CCR updates apply at the next UEV. */
    TIM1->CCMR1 = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 = (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2) | TIM_CCMR2_OC4PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR  = TIM_BDTR_MOE;     /* advanced-timer outputs require MOE */
    TIM1->EGR   = TIM_EGR_UG;       /* force preload load */
    TIM1->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void pwm_init(void) {
    gpio_init();
    tim1_init();
}

void pwm_set_duty(uint8_t ch, float mag01) {
    if (mag01 < 0.0f) mag01 = 0.0f;
    if (mag01 > 1.0f) mag01 = 1.0f;
    uint32_t ccr = (uint32_t)(mag01 * (float)PWM_PERIOD + 0.5f);
    if (ccr > PWM_PERIOD) ccr = PWM_PERIOD;
    if (ch == 0) TIM1->CCR1 = ccr;
    else         TIM1->CCR4 = ccr;
}

void pwm_set_dir(uint8_t ch, bool reverse) {
    uint32_t pin = (ch == 0) ? 0u : 2u;           /* PB0 / PB2 */
    GPIOB->BSRR = reverse ? (1u << pin) : (1u << (pin + 16u));
}

void pwm_set_enabled(bool enabled) {
    if (enabled) GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_3;
    else         GPIOB->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_3;
}
