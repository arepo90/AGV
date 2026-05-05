#include "encoders.h"
#include "config.h"
#include "stm32f0xx.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static int32_t  s_count[2];           /* accumulated signed counts */
static uint16_t s_last_cnt[2];        /* previous CNT snapshot (low 16 bits) */
static float    s_velocity_mps[2];    /* linear m/s at wheel rim */
static float    s_velocity_rpm[2];

/* Counts → metres at the wheel rim. PPR is electrical; we use 4× decoding so
 * one revolution = ENCODER_PPR × 4 counts. */
#define COUNTS_PER_REV   ((float)(ENCODER_COUNTS_PER_REV))
#define M_PER_COUNT      ((2.0f * M_PI * (float)WHEEL_RADIUS_M) / COUNTS_PER_REV)

static void config_tim_quadrature(TIM_TypeDef *tim) {
    tim->CR1   = 0;
    tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;   /* CC1S=01, CC2S=01 */
    tim->CCMR1 |= (3u << 4) | (3u << 12);               /* IC1F=IC2F=N=8 noise filter */
    tim->CCER  = 0;                                     /* both channels rising-edge active (default) */
    tim->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;       /* SMS=011 → encoder mode 3 (x4) */
    tim->ARR   = 0xFFFFu;                               /* let TIM2's upper 16 bits do nothing useful */
    tim->CNT   = 0;
    tim->CR1  |= TIM_CR1_CEN;
}

void encoders_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* TIM2 on PA0 (AF2), PA1 (AF2). */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1))
                 | GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 0) | (0xFu << 4)))
                  | (2u << 0) | (2u << 4);
    /* Pull-ups so a disconnected encoder reads HIGH instead of floating. */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1))
                 | GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;

    /* TIM3 on PA6 (AF1), PA7 (AF1). */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
                 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 24) | (0xFu << 28)))
                  | (1u << 24) | (1u << 28);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7))
                 | GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    config_tim_quadrature(TIM2);
    config_tim_quadrature(TIM3);

    encoders_reset();
}

void encoders_reset(void) {
    s_count[ENC_LEFT]        = 0;
    s_count[ENC_RIGHT]       = 0;
    s_last_cnt[ENC_LEFT]     = (uint16_t)TIM2->CNT;
    s_last_cnt[ENC_RIGHT]    = (uint16_t)TIM3->CNT;
    s_velocity_mps[ENC_LEFT] = 0.0f;
    s_velocity_mps[ENC_RIGHT]= 0.0f;
    s_velocity_rpm[ENC_LEFT] = 0.0f;
    s_velocity_rpm[ENC_RIGHT]= 0.0f;
}

void encoders_tick(float dt_s) {
    if (dt_s <= 0.0f) return;

    uint16_t cnt_l = (uint16_t)TIM2->CNT;
    uint16_t cnt_r = (uint16_t)TIM3->CNT;

    /* Cast difference through int16_t so wrap is handled correctly regardless
     * of timer width. (Trick: int16_t(0x0001 - 0xFFFE) == +3, not -65533.) */
    int32_t d_l = (int16_t)(cnt_l - s_last_cnt[ENC_LEFT]);
    int32_t d_r = (int16_t)(cnt_r - s_last_cnt[ENC_RIGHT]);
    s_last_cnt[ENC_LEFT]  = cnt_l;
    s_last_cnt[ENC_RIGHT] = cnt_r;

#if ENCODER_INVERT_LEFT
    d_l = -d_l;
#endif
#if ENCODER_INVERT_RIGHT
    d_r = -d_r;
#endif

    s_count[ENC_LEFT]  += d_l;
    s_count[ENC_RIGHT] += d_r;

    s_velocity_mps[ENC_LEFT]  = ((float)d_l * M_PER_COUNT) / dt_s;
    s_velocity_mps[ENC_RIGHT] = ((float)d_r * M_PER_COUNT) / dt_s;

    /* RPM: revs/sec × 60. revs = counts / counts_per_rev. */
    float rps_l = (float)d_l / (COUNTS_PER_REV * dt_s);
    float rps_r = (float)d_r / (COUNTS_PER_REV * dt_s);
    s_velocity_rpm[ENC_LEFT]  = rps_l * 60.0f;
    s_velocity_rpm[ENC_RIGHT] = rps_r * 60.0f;
}

int32_t encoders_count(encoder_side_t s)         { return s_count[s]; }
float   encoders_velocity_mps(encoder_side_t s)  { return s_velocity_mps[s]; }
float   encoders_velocity_rpm(encoder_side_t s)  { return s_velocity_rpm[s]; }
