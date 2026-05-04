#include "motors.h"
#include "config.h"
#include "stm32f0xx.h"

/* TIM1 PWM period: SYSCLK / PWM_FREQ → ARR = period - 1, CCR ∈ [0, period]. */
#define PWM_PERIOD   (SYSCLK_HZ / PWM_FREQ_HZ)

static bool  s_enabled = false;
static float s_last_duty[2];

/* ---- GPIO + TIM1 bring-up ----------------------------------------------- */

static void gpio_init_pwm_dir_sleep(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* SLEEP first: drive PB1, PB3 LOW *before* changing mode, so they don't
     * glitch HIGH during configuration and accidentally wake the driver. */
    GPIOB->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_3;

    /* PB1, PB3 → output (SLEEP).
     * PB0, PB2 → output (DIR).
     * Both are general-purpose push-pull, low speed. */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                                     GPIO_MODER_MODER2 | GPIO_MODER_MODER3))
                 | GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0
                 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0;

    /* DIR pins start LOW. */
    GPIOB->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_2;

    /* PA8 → AF2 (TIM1_CH1). */
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODER8)  | GPIO_MODER_MODER8_1;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFu << 0))       | (2u << 0);

    /* PA11 → AF2 (TIM1_CH4). */
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODER11) | GPIO_MODER_MODER11_1;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFu << 12))      | (2u << 12);
}

static void tim1_init_pwm(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->CR1   = 0;
    TIM1->PSC   = 0;
    TIM1->ARR   = (uint32_t)(PWM_PERIOD - 1u);
    TIM1->CCR1  = 0;
    TIM1->CCR4  = 0;

    /* PWM mode 1 (OC1M=110), preload enabled — CCR updates take effect at next
     * UEV, never mid-cycle. */
    TIM1->CCMR1 = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 = (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2) | TIM_CCMR2_OC4PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR  = TIM_BDTR_MOE;          /* required for advanced timer outputs */
    TIM1->EGR   = TIM_EGR_UG;            /* force preload load */
    TIM1->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void motors_init(void) {
    gpio_init_pwm_dir_sleep();
    tim1_init_pwm();
    s_enabled = false;
    s_last_duty[MOTOR_LEFT]  = 0.0f;
    s_last_duty[MOTOR_RIGHT] = 0.0f;
}

/* ---- SLEEP control ------------------------------------------------------ */

void motors_set_enabled(bool enabled) {
    if (enabled) GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_3;
    else         GPIOB->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_3;
    s_enabled = enabled;
}

bool motors_enabled(void) { return s_enabled; }

/* ---- PWM + DIR ---------------------------------------------------------- */

static void apply_dir(motor_side_t side, bool reverse) {
    /* DIR HIGH = reverse, LOW = forward — convention is arbitrary, fixed by the
     * MOTOR_INVERT_LEFT/RIGHT flags in config.h. */
    uint32_t set_l = reverse ? GPIO_BSRR_BS_0 : GPIO_BSRR_BR_0;
    uint32_t set_r = reverse ? GPIO_BSRR_BS_2 : GPIO_BSRR_BR_2;
    GPIOB->BSRR = (side == MOTOR_LEFT) ? set_l : set_r;
}

void motors_set_pwm_signed(motor_side_t side, float duty) {
    /* Clamp */
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    s_last_duty[side] = duty;

    /* Per-side polarity flip from config. */
    bool inverted = (side == MOTOR_LEFT)  ? (MOTOR_INVERT_LEFT  != 0)
                                          : (MOTOR_INVERT_RIGHT != 0);
    if (inverted) duty = -duty;

    bool reverse = (duty < 0.0f);
    if (reverse) duty = -duty;

    apply_dir(side, reverse);

    uint32_t ccr = (uint32_t)(duty * (float)PWM_PERIOD + 0.5f);
    if (ccr > PWM_PERIOD) ccr = PWM_PERIOD;

    if (side == MOTOR_LEFT) TIM1->CCR1 = ccr;
    else                    TIM1->CCR4 = ccr;
}

float motors_last_duty(motor_side_t side) { return s_last_duty[side]; }
