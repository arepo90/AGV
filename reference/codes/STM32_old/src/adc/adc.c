#include "adc.h"
#include "config.h"
#include "stm32f0xx.h"
#include <stdbool.h>

/* DMA buffer — written by hardware, read by main loop. Volatile so the
 * compiler doesn't cache reads across iterations. */
static volatile uint16_t s_buf[ADC_NUM_CHANNELS];

/* Latest stable snapshot (copied out of DMA buffer once each scan completes).
 * This avoids the small window where DMA might be partway through writing. */
static uint16_t s_latest[ADC_NUM_CHANNELS];

static bool     s_have_data = false;
static bool     s_scan_in_flight = false;
static uint32_t s_last_scan_ms = 0;

#define CHSELR_BITS    ( ADC_CHSELR_CHSEL2  | ADC_CHSELR_CHSEL3  \
                       | ADC_CHSELR_CHSEL4  | ADC_CHSELR_CHSEL5  \
                       | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL11 \
                       | ADC_CHSELR_CHSEL12 | ADC_CHSELR_CHSEL13 \
                       | ADC_CHSELR_CHSEL14 | ADC_CHSELR_CHSEL15 )

static void gpio_init_analog_pins(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

    /* MODER 11 = analog. */
    GPIOA->MODER |= GPIO_MODER_MODER2 | GPIO_MODER_MODER3
                  | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
    GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1
                  | GPIO_MODER_MODER2 | GPIO_MODER_MODER3
                  | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
}

static void dma_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel1->CCR  = 0;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)s_buf;
    /* Length set per scan in adc_kick(). */
    /* MEMORY half-word increment, peripheral half-word fixed, peripheral→memory. */
    DMA1_Channel1->CCR  = DMA_CCR_MINC
                        | DMA_CCR_MSIZE_0   /* 16-bit memory */
                        | DMA_CCR_PSIZE_0;  /* 16-bit peripheral */
}

static void adc_hw_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    /* Calibration must run in async clock mode (CKMODE=00, HSI14 managed by
     * hardware). Setting CKMODE before calibration on STM32F051 prevents ADRDY
     * from ever being set after ADEN. */
    ADC1->CR = ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }

    /* Now safe to select synchronous clock: CKMODE=10 → PCLK/4 = 12 MHz. */
    ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;

    /* Clear any stale ADRDY (can be set by a debugger attach), then enable. */
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR  = ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }
    ADC1->ISR = ADC_ISR_ADRDY;

    /* Sample time = 7.5 ADC clocks → ~625 ns @ 12 MHz; with 12.5-cycle
     * conversion, ≈ 1.67 µs per channel. 10 channels ≈ 17 µs per scan. */
    ADC1->SMPR  = 2u;   /* SMP = 010 → 7.5 cycles */
    ADC1->CHSELR = CHSELR_BITS;

    /* DMA enabled, one-shot per scan (DMACFG=0). SCANDIR=0 → forward order. */
    ADC1->CFGR1 = ADC_CFGR1_DMAEN;
}

void adc_init(void) {
    gpio_init_analog_pins();
    dma_init();
    adc_hw_init();
    s_have_data = false;
    s_scan_in_flight = false;
    for (uint32_t i = 0; i < ADC_NUM_CHANNELS; i++) s_latest[i] = 0;
}

static void adc_kick(void) {
    /* Reload DMA for a fresh scan. CCR.EN must be 0 during configuration. */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMA1_Channel1->CNDTR = ADC_NUM_CHANNELS;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->ISR = ADC_ISR_EOSEQ | ADC_ISR_EOC | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    s_scan_in_flight = true;
}

static void adc_harvest_if_done(void) {
    if (!s_scan_in_flight) return;
    if (!(DMA1->ISR & DMA_ISR_TCIF1)) return;

    DMA1->IFCR = DMA_IFCR_CTCIF1;
    /* Snapshot the volatile buffer into the stable read array. */
    for (uint32_t i = 0; i < ADC_NUM_CHANNELS; i++) s_latest[i] = s_buf[i];

    s_scan_in_flight = false;
    s_have_data = true;
}

void adc_tick(uint32_t now_ms) {
    /* Always check for completion of any in-flight scan first. */
    adc_harvest_if_done();

    if (!s_scan_in_flight && (now_ms - s_last_scan_ms) >= (1000u / ADC_SCAN_HZ)) {
        s_last_scan_ms = now_ms;
        adc_kick();
    }
}

uint16_t adc_raw(uint32_t idx) {
    return (idx < ADC_NUM_CHANNELS) ? s_latest[idx] : 0;
}

uint16_t adc_motor_current_ma(uint32_t side) {
    uint16_t raw = adc_raw(side ? ADC_IDX_M2_CURRENT : ADC_IDX_M1_CURRENT);
    /* Pololu G2 24v14 current sense is unipolar (V_sense proportional to |I|),
     * so a plain raw × NUM/DEN scale is correct. If a bidirectional driver is
     * ever swapped in this will need an offset and a sign. */
    uint32_t ma = ((uint32_t)raw * MOTOR_CURRENT_MA_PER_COUNT_NUM)
                  / MOTOR_CURRENT_MA_PER_COUNT_DEN;
    return (ma > 0xFFFFu) ? 0xFFFFu : (uint16_t)ma;
}

uint16_t adc_qtr(uint32_t idx) {
    if (idx >= ADC_IDX_QTR_COUNT) return 0;
    return adc_raw(ADC_IDX_QTR_FIRST + idx);
}

bool adc_has_data(void) { return s_have_data; }
