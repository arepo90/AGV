#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  USART1 byte transport to the ESP32 (PB6 TX / PB7 RX, AF0).  (hal tier)
 *
 *  RX : DMA circular ring, drained byte-by-byte by the frame parser in proto.c.
 *       No RX interrupt — the DMA can't be late, we just chase its write head.
 *  TX : fixed-slot frame ring; each uart_send() copies a complete frame and a
 *       per-frame DMA kick drains them. Queue-full returns false (caller logs).
 *
 *  This layer is framing-agnostic: it moves opaque bytes. All protocol meaning
 *  (magic, CRC, packet types) lives in proto.c.
 * =============================================================================
 */

void     uart_init(void);

/* Enqueue one complete frame for DMA transmission. False if the queue is full. */
bool     uart_send(const uint8_t *data, uint16_t len);

/* Pop the next received byte. False if the RX ring is empty. */
bool     uart_rx_pop(uint8_t *out);

/* USART line-error counters, monotonic since boot (proto.c logs the deltas). */
uint16_t uart_err_overrun(void);
uint16_t uart_err_framing(void);
uint16_t uart_err_noise(void);

#endif /* UART_H */
