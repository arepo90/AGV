#include "comms.h"
#include "config.h"
#include "crc.h"
#include "log.h"
#include "stm32f0xx.h"
#include <string.h>

/* USART1: PB6=TX (AF0), PB7=RX (AF0). DMA1 Ch2=TX, Ch3=RX. */

/* ---- TX queue: fixed-slot frame ring ------------------------------------- */
#define TX_SLOTS  16u

typedef struct {
    uint8_t buf[PROTO_MAX_FRAME];
    uint8_t len;
} tx_slot_t;

static tx_slot_t s_tx[TX_SLOTS];
static volatile uint8_t s_tx_head = 0;   /* write index (producer = main loop) */
static volatile uint8_t s_tx_tail = 0;   /* read index  (consumer = TX-ISR / kick) */
static volatile bool    s_tx_busy = false;

/* ---- RX DMA ring --------------------------------------------------------- */
static uint8_t s_rx_ring[UART_RX_RING_SIZE];
static uint32_t s_rx_tail = 0;           /* next byte to consume from ring */

/* ---- Frame parser state -------------------------------------------------- */
typedef enum {
    PARSE_MAGIC0,
    PARSE_MAGIC1,
    PARSE_VER,
    PARSE_SEQ,
    PARSE_TYPE,
    PARSE_LEN,
    PARSE_PAYLOAD,
    PARSE_CRC_LO,
    PARSE_CRC_HI,
} parse_state_t;

static parse_state_t s_pstate = PARSE_MAGIC0;
static uint8_t       s_p_ver, s_p_seq, s_p_type, s_p_len;
static uint8_t       s_p_payload[PROTO_MAX_PAYLOAD];
static uint16_t      s_p_payload_idx = 0;
static uint16_t      s_p_crc_received = 0;

/* ---- Decoded-packet handoff to main loop -------------------------------- */
/* Single-slot mailbox: parser writes when ready, main loop drains via comms_recv().
 * If main loop is slow and another frame arrives, the new one overwrites the old
 * and we log a dropped-packet event. Telemetry-rate work won't realistically
 * generate this contention. */
static volatile bool     s_rx_pkt_ready = false;
static packet_t          s_rx_pkt;
static volatile uint32_t s_rx_pkt_dropped = 0;

/* ---- Outgoing SEQ counter ----------------------------------------------- */
static uint8_t s_tx_seq = 0;

/* ---- Per-frame received SEQ tracking for gap detection ------------------ */
static bool    s_rx_seq_seen = false;
static uint8_t s_rx_last_seq = 0;

/* ===========================================================================
 *  Hardware bring-up
 * =========================================================================== */

static void gpio_usart1_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    /* PB6, PB7: AF mode. AF0 is reset default — no AFR write needed. */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
                 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    /* Push-pull, high speed — at 921600 we want clean edges. */
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7;
    /* Pull-up on RX so a disconnected line idles high. */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR7) | GPIO_PUPDR_PUPDR7_0;
}

static void usart1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR1 = 0;                                   /* disable while configuring */
    USART1->BRR = (SYSCLK_HZ + (UART_BAUD / 2u)) / UART_BAUD;   /* nearest-int divisor */
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE;  /* DMA + error IRQ */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void dma_rx_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel3->CCR    = 0;   /* disable while configuring */
    DMA1_Channel3->CPAR   = (uint32_t)&USART1->RDR;
    DMA1_Channel3->CMAR   = (uint32_t)s_rx_ring;
    DMA1_Channel3->CNDTR  = UART_RX_RING_SIZE;
    /* circular, memory increment, peripheral→memory, byte transfers, enable */
    DMA1_Channel3->CCR    = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
}

static void dma_tx_init(void) {
    /* TX channel kept disabled until kicked off in tx_kick(). */
    DMA1_Channel2->CCR  = 0;
    DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;

    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);  /* lower than SysTick */
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/* ===========================================================================
 *  Outbound path
 * =========================================================================== */

static void tx_kick(void) {
    /* Caller must hold IRQ-off lock. Starts DMA on s_tx_tail's slot if any. */
    if (s_tx_busy)            return;
    if (s_tx_tail == s_tx_head) return;

    tx_slot_t *slot = &s_tx[s_tx_tail];
    DMA1_Channel2->CCR   = 0;
    DMA1_Channel2->CMAR  = (uint32_t)slot->buf;
    DMA1_Channel2->CNDTR = slot->len;
    /* memory→peripheral, byte, MINC, TCIE, enable */
    DMA1_Channel2->CCR   = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN;

    s_tx_busy = true;
}

static bool tx_enqueue(const uint8_t *frame, uint8_t total_len) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t next = (s_tx_head + 1u) % TX_SLOTS;
    if (next == s_tx_tail) {
        if (!primask) __enable_irq();
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_TX_QUEUE_FULL, total_len);
        return false;
    }

    memcpy(s_tx[s_tx_head].buf, frame, total_len);
    s_tx[s_tx_head].len = total_len;
    s_tx_head = next;

    tx_kick();

    if (!primask) __enable_irq();
    return true;
}

static uint8_t encode_frame(uint8_t out[PROTO_MAX_FRAME], uint8_t type,
                            uint8_t seq, const uint8_t *payload, uint8_t len) {
    out[0] = PROTO_MAGIC0;
    out[1] = PROTO_MAGIC1;
    out[2] = PROTO_VERSION;
    out[3] = seq;
    out[4] = type;
    out[5] = len;
    if (len && payload) memcpy(&out[6], payload, len);

    uint16_t crc = crc16_compute(&out[2], 4u + (uint32_t)len);  /* VER..LAST_PAYLOAD */
    out[6 + len]     = (uint8_t)(crc & 0xFFu);
    out[6 + len + 1] = (uint8_t)(crc >> 8);
    return 8u + len;
}

bool comms_send(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t total = encode_frame(frame, type, s_tx_seq++, payload, len);
    return tx_enqueue(frame, total);
}

bool comms_send_ack(uint8_t seq, uint8_t status) {
    uint8_t p[2] = { seq, status };
    return comms_send(PKT_ACK, p, sizeof p);
}

bool comms_send_nack(uint8_t seq, uint8_t error_code) {
    uint8_t p[2] = { seq, error_code };
    return comms_send(PKT_NACK, p, sizeof p);
}

/* ===========================================================================
 *  Inbound path: DMA ring → frame parser → mailbox
 * =========================================================================== */

static void parser_reset(void) {
    s_pstate = PARSE_MAGIC0;
    s_p_payload_idx = 0;
}

static void parser_publish(void) {
    if (s_rx_pkt_ready) {
        /* Main loop hasn't consumed the previous one — overwrite and count. */
        s_rx_pkt_dropped++;
    }
    s_rx_pkt.type = s_p_type;
    s_rx_pkt.seq  = s_p_seq;
    s_rx_pkt.len  = s_p_len;
    if (s_p_len) memcpy(s_rx_pkt.payload, s_p_payload, s_p_len);
    s_rx_pkt_ready = true;

    /* SEQ gap detection */
    if (s_rx_seq_seen) {
        uint8_t expected = (uint8_t)(s_rx_last_seq + 1u);
        if (s_p_seq != expected) {
            uint32_t gap = (uint32_t)((uint8_t)(s_p_seq - expected));
            log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_FRAGMENT_OUT_OF_ORDER, gap);
        }
    }
    s_rx_last_seq = s_p_seq;
    s_rx_seq_seen = true;
}

static void parser_feed(uint8_t b) {
    switch (s_pstate) {
    case PARSE_MAGIC0:
        if (b == PROTO_MAGIC0) s_pstate = PARSE_MAGIC1;
        break;

    case PARSE_MAGIC1:
        if (b == PROTO_MAGIC1) {
            s_pstate = PARSE_VER;
        } else {
            log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_BAD_MAGIC, b);
            s_pstate = (b == PROTO_MAGIC0) ? PARSE_MAGIC1 : PARSE_MAGIC0;
        }
        break;

    case PARSE_VER:
        s_p_ver = b;
        if (b != PROTO_VERSION) {
            log_record(LOG_MOD_COMMS, LOG_SEV_ERROR, LOG_CODE_BAD_VERSION, b);
            parser_reset();
            break;
        }
        s_pstate = PARSE_SEQ;
        break;

    case PARSE_SEQ:
        s_p_seq = b;
        s_pstate = PARSE_TYPE;
        break;

    case PARSE_TYPE:
        s_p_type = b;
        s_pstate = PARSE_LEN;
        break;

    case PARSE_LEN:
        s_p_len = b;
        if (s_p_len == 0) {
            s_pstate = PARSE_CRC_LO;
        } else {
            s_p_payload_idx = 0;
            s_pstate = PARSE_PAYLOAD;
        }
        break;

    case PARSE_PAYLOAD:
        s_p_payload[s_p_payload_idx++] = b;
        if (s_p_payload_idx >= s_p_len) s_pstate = PARSE_CRC_LO;
        break;

    case PARSE_CRC_LO:
        s_p_crc_received = b;
        s_pstate = PARSE_CRC_HI;
        break;

    case PARSE_CRC_HI: {
        s_p_crc_received |= ((uint16_t)b << 8);
        /* Recompute over [VER, SEQ, TYPE, LEN, PAYLOAD...] */
        uint8_t hdr[4] = { s_p_ver, s_p_seq, s_p_type, s_p_len };
        /* Build a temp contiguous buffer for CRC: header then payload. */
        uint8_t tmp[4 + PROTO_MAX_PAYLOAD];
        memcpy(tmp, hdr, 4);
        if (s_p_len) memcpy(&tmp[4], s_p_payload, s_p_len);
        uint16_t crc = crc16_compute(tmp, 4u + s_p_len);

        if (crc == s_p_crc_received) {
            parser_publish();
        } else {
            log_record(LOG_MOD_COMMS, LOG_SEV_ERROR, LOG_CODE_BAD_CRC,
                       ((uint32_t)s_p_crc_received << 16) | crc);
            comms_send_nack(s_p_seq, NACK_BAD_CRC);
        }
        parser_reset();
        break;
    }
    }
}

void comms_poll(void) {
    /* Drain DMA RX ring up to current write position. */
    uint32_t cndtr = DMA1_Channel3->CNDTR;
    uint32_t head  = (UART_RX_RING_SIZE - cndtr) % UART_RX_RING_SIZE;

    while (s_rx_tail != head) {
        parser_feed(s_rx_ring[s_rx_tail]);
        s_rx_tail = (s_rx_tail + 1u) % UART_RX_RING_SIZE;
    }
}

bool comms_recv(packet_t *out) {
    if (!s_rx_pkt_ready) return false;
    /* No need to disable IRQs — the parser runs in main loop context (comms_poll),
     * not from an ISR, so producer and consumer are the same task. */
    *out = s_rx_pkt;
    s_rx_pkt_ready = false;
    return true;
}

/* ===========================================================================
 *  ISR: DMA TX complete → advance queue and kick the next frame
 * =========================================================================== */

void DMA1_Channel2_3_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        DMA1_Channel2->CCR = 0;     /* disable so we can reload */
        s_tx_busy = false;
        s_tx_tail = (s_tx_tail + 1u) % TX_SLOTS;
        tx_kick();
    }
    /* Channel 3 (RX) shares this vector but we don't enable any of its IRQs. */
}

/* ===========================================================================
 *  ISR: USART error flags (overrun, framing, noise) — log and clear
 * =========================================================================== */

void USART1_IRQHandler(void) {
    uint32_t isr = USART1->ISR;
    if (isr & USART_ISR_ORE) {
        USART1->ICR = USART_ICR_ORECF;
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UART_OVERRUN, 0);
    }
    if (isr & USART_ISR_FE) {
        USART1->ICR = USART_ICR_FECF;
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UART_FRAMING_ERR, 0);
    }
    if (isr & USART_ISR_NE) {
        USART1->ICR = USART_ICR_NCF;
        log_record(LOG_MOD_COMMS, LOG_SEV_WARN, LOG_CODE_UART_NOISE, 0);
    }
}

/* ===========================================================================
 *  Init
 * =========================================================================== */

void comms_init(void) {
    crc_init();
    gpio_usart1_init();
    usart1_init();
    dma_rx_init();
    dma_tx_init();

    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);

    parser_reset();
}
