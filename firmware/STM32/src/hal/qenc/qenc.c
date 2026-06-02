#include "qenc.h"
#include "stm32f0xx.h"

static void config_encoder_mode(TIM_TypeDef *tim) {
    tim->CR1   = 0;
    tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;   /* CC1S=CC2S=01 (TI mapping) */
    tim->CCMR1 |= (3u << 4) | (3u << 12);               /* input filter N=8 */
    tim->CCER  = 0;                                      /* both inputs rising-edge active */
    tim->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;        /* SMS=011 → encoder mode 3 (×4) */
    tim->ARR   = 0xFFFFu;                                /* wrap as 16-bit even on 32-bit TIM2 */
    tim->CNT   = 0;
    tim->CR1  |= TIM_CR1_CEN;
}

void qenc_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* TIM2: PA0/PA1 (AF2). */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1))
                 | GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 0) | (0xFu << 4)))
                  | (2u << 0) | (2u << 4);

    /* TIM3: PA6/PA7 (AF1). */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
                 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 24) | (0xFu << 28)))
                  | (1u << 24) | (1u << 28);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    config_encoder_mode(TIM2);
    config_encoder_mode(TIM3);
}

uint16_t qenc_raw(uint8_t ch) {
    return (uint16_t)((ch == 0) ? TIM2->CNT : TIM3->CNT);
}
