#include "analog.h"
#include "config.h"
#include "stm32f0xx.h"

static volatile uint16_t s_buf[ANALOG_NUM_CHANNELS];      /* DMA-written */
static uint16_t          s_latest[ANALOG_NUM_CHANNELS];   /* stable snapshot */
static bool              s_have_data = false;
static bool              s_scan_in_flight = false;
static uint32_t          s_last_scan_ms = 0;

static const uint8_t     s_qtr_map[ANALOG_QTR_COUNT] = QTR_PIN_MAP;

#define CHSELR_BITS ( ADC_CHSELR_CHSEL2  | ADC_CHSELR_CHSEL3  \
                    | ADC_CHSELR_CHSEL4  | ADC_CHSELR_CHSEL5  \
                    | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL11 \
                    | ADC_CHSELR_CHSEL12 | ADC_CHSELR_CHSEL13 \
                    | ADC_CHSELR_CHSEL14 | ADC_CHSELR_CHSEL15 )

static void gpio_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    GPIOA->MODER |= GPIO_MODER_MODER2 | GPIO_MODER_MODER3
                  | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;     /* analog */
    GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1
                  | GPIO_MODER_MODER2 | GPIO_MODER_MODER3
                  | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
}

static void dma_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel1->CCR  = 0;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)s_buf;
    DMA1_Channel1->CCR  = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;  /* 16-bit */
}

static void adc_hw_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    /* Calibrate in async clock mode (CKMODE=00) — required for ADRDY on F051. */
    ADC1->CR = ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }

    ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;   /* CKMODE=10 → PCLK/4 = 12 MHz */

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR  = ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }
    ADC1->ISR = ADC_ISR_ADRDY;

    ADC1->SMPR   = 7u;          /* 239.5 cycles/sample — the QTR-8A's high source
                                 * impedance needs the settle time; a full 10-ch
                                 * scan is still ~210 µs at 12 MHz (100 Hz rate) */
    ADC1->CHSELR = CHSELR_BITS;
    ADC1->CFGR1  = ADC_CFGR1_DMAEN;   /* one-shot per scan */
}

void analog_init(void) {
    gpio_init();
    dma_init();
    adc_hw_init();
    s_have_data = false;
    s_scan_in_flight = false;
    for (uint32_t i = 0; i < ANALOG_NUM_CHANNELS; i++) s_latest[i] = 0;
}

static void scan_kick(void) {
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMA1_Channel1->CNDTR = ANALOG_NUM_CHANNELS;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->ISR = ADC_ISR_EOSEQ | ADC_ISR_EOC | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    s_scan_in_flight = true;
}

static void harvest_if_done(void) {
    if (!s_scan_in_flight) return;
    if (!(DMA1->ISR & DMA_ISR_TCIF1)) return;
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    for (uint32_t i = 0; i < ANALOG_NUM_CHANNELS; i++) s_latest[i] = s_buf[i];
    s_scan_in_flight = false;
    s_have_data = true;
}

void analog_tick(uint32_t now_ms) {
    harvest_if_done();
    if (!s_scan_in_flight && (now_ms - s_last_scan_ms) >= (1000u / ADC_SCAN_HZ)) {
        s_last_scan_ms = now_ms;
        scan_kick();
    }
}

bool analog_has_data(void) { return s_have_data; }

static uint16_t raw(uint32_t idx) {
    return (idx < ANALOG_NUM_CHANNELS) ? s_latest[idx] : 0;
}

uint16_t analog_qtr(uint32_t idx) {
    if (idx >= ANALOG_QTR_COUNT) return 0;
    return raw(ANALOG_IDX_QTR_FIRST + s_qtr_map[idx]);
}

uint16_t analog_current_ma(uint32_t side) {
    uint16_t r = raw(side ? ANALOG_IDX_M2_CURRENT : ANALOG_IDX_M1_CURRENT);
    uint32_t ma = ((uint32_t)r * MOTOR_CURRENT_MA_PER_COUNT_NUM)
                  / MOTOR_CURRENT_MA_PER_COUNT_DEN;
    return (ma > 0xFFFFu) ? 0xFFFFu : (uint16_t)ma;
}
