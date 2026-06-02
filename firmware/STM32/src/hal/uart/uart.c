#include "uart.h"
#include "config.h"
#include "stm32f0xx.h"
#include <string.h>

/* USART1: PB6=TX (AF0), PB7=RX (AF0). DMA1 Ch2=TX, Ch3=RX. */

/* ---- TX: fixed-slot frame ring -------------------------------------------
 * Each slot reserves PROTO_MAX_FRAME bytes, so this dominates RAM. The ring
 * absorbs a worst-case burst (telemetry + ACK + a few log frames). */
typedef struct {
    uint8_t buf[PROTO_MAX_FRAME];
    uint8_t len;
} tx_slot_t;

static tx_slot_t        s_tx[UART_TX_SLOTS];
static volatile uint8_t s_tx_head = 0;   /* producer (main loop) */
static volatile uint8_t s_tx_tail = 0;   /* consumer (TX ISR)    */
static volatile bool    s_tx_busy = false;

/* ---- RX: DMA circular ring ------------------------------------------------ */
static uint8_t  s_rx_ring[UART_RX_RING_SIZE];
static uint32_t s_rx_tail = 0;           /* next byte to consume */

/* ---- Line-error counters (set in ISR, read by proto) ---------------------- */
static volatile uint16_t s_err_overrun = 0;
static volatile uint16_t s_err_framing = 0;
static volatile uint16_t s_err_noise   = 0;

/* ===========================================================================
 *  Hardware bring-up
 * =========================================================================== */

static void gpio_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    /* PB6/PB7 AF mode (AF0 is the reset default — no AFR write needed). */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
                 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7;   /* clean edges at 921600 */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR7) | GPIO_PUPDR_PUPDR7_0;  /* RX idle high */
}

static void usart_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->CR1 = 0;
    USART1->BRR = (SYSCLK_HZ + (UART_BAUD / 2u)) / UART_BAUD;   /* nearest-int divisor */
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void dma_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* RX: peripheral→memory, circular, byte, memory-increment. */
    DMA1_Channel3->CCR   = 0;
    DMA1_Channel3->CPAR  = (uint32_t)&USART1->RDR;
    DMA1_Channel3->CMAR  = (uint32_t)s_rx_ring;
    DMA1_Channel3->CNDTR = UART_RX_RING_SIZE;
    DMA1_Channel3->CCR   = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;

    /* TX: kept disabled until kicked. */
    DMA1_Channel2->CCR  = 0;
    DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;

    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

void uart_init(void) {
    gpio_init();
    usart_init();
    dma_init();
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
}

/* ===========================================================================
 *  TX
 * =========================================================================== */

static void tx_kick(void) {
    /* Caller holds IRQ-off. Start DMA on the tail slot if idle and non-empty. */
    if (s_tx_busy)              return;
    if (s_tx_tail == s_tx_head) return;

    tx_slot_t *slot = &s_tx[s_tx_tail];
    DMA1_Channel2->CCR   = 0;
    DMA1_Channel2->CMAR  = (uint32_t)slot->buf;
    DMA1_Channel2->CNDTR = slot->len;
    DMA1_Channel2->CCR   = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN;
    s_tx_busy = true;
}

bool uart_send(const uint8_t *data, uint16_t len) {
    if (len > PROTO_MAX_FRAME) return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t next = (uint8_t)((s_tx_head + 1u) % UART_TX_SLOTS);
    if (next == s_tx_tail) {
        if (!primask) __enable_irq();
        return false;   /* queue full */
    }
    memcpy(s_tx[s_tx_head].buf, data, len);
    s_tx[s_tx_head].len = (uint8_t)len;
    s_tx_head = next;
    tx_kick();

    if (!primask) __enable_irq();
    return true;
}

void DMA1_Channel2_3_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        DMA1_Channel2->CCR = 0;
        s_tx_busy = false;
        s_tx_tail = (uint8_t)((s_tx_tail + 1u) % UART_TX_SLOTS);
        tx_kick();
    }
    /* Channel 3 (RX) shares this vector but enables no interrupts. */
}

/* ===========================================================================
 *  RX
 * =========================================================================== */

bool uart_rx_pop(uint8_t *out) {
    uint32_t cndtr = DMA1_Channel3->CNDTR;
    uint32_t head  = (UART_RX_RING_SIZE - cndtr) % UART_RX_RING_SIZE;
    if (s_rx_tail == head) return false;

    *out = s_rx_ring[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1u) % UART_RX_RING_SIZE;
    return true;
}

/* ===========================================================================
 *  Error ISR — clear flags, count. proto.c logs the deltas.
 * =========================================================================== */

void USART1_IRQHandler(void) {
    uint32_t isr = USART1->ISR;
    if (isr & USART_ISR_ORE) { USART1->ICR = USART_ICR_ORECF; s_err_overrun++; }
    if (isr & USART_ISR_FE)  { USART1->ICR = USART_ICR_FECF;  s_err_framing++; }
    if (isr & USART_ISR_NE)  { USART1->ICR = USART_ICR_NCF;   s_err_noise++;   }
}

uint16_t uart_err_overrun(void) { return s_err_overrun; }
uint16_t uart_err_framing(void) { return s_err_framing; }
uint16_t uart_err_noise(void)   { return s_err_noise;   }
