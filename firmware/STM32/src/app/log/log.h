#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/* =============================================================================
 * Fault log: OBD2-style RAM ring buffer for non-fatal events.
 *
 * Producers call log_record() from any context (main loop or ISR) — writes use
 * a short critical section so ISRs can record safely. main.c drains entries and
 * forwards them as PKT_LOG frames (fire-and-forget). RAM only; the reboot cause
 * comes from mcu_reset_cause() recorded at boot.
 *
 * This module has no upward dependencies (no proto/uart) — it is a pure buffer.
 * =============================================================================
 */

typedef struct {
    uint32_t       timestamp_ms;
    log_code_t     code;
    log_severity_t severity;
    log_module_t   module;
    uint32_t       data;       /* free-form context */
} log_entry_t;

void     log_init(void);
void     log_record(log_module_t module, log_severity_t sev, log_code_t code, uint32_t data);
bool     log_pop(log_entry_t *out);     /* false if empty */
uint32_t log_pending_count(void);
uint32_t log_dropped_count(void);       /* lost to overflow since last clear */
void     log_clear(void);

#endif /* LOG_H */
