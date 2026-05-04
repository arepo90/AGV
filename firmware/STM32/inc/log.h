#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/* =============================================================================
 * Fault log: OBD2-style ring buffer for non-fatal events.
 *
 * Producers call log_record() from any context (main loop or ISR). The buffer
 * is a single-producer-friendly ring: writes use a critical section, so ISRs
 * can record too without corruption.
 *
 * The comms module drains pending entries periodically (one LOG packet per
 * entry, fire-and-forget) and clears them when the workstation either dumps
 * everything or asks for a clear.
 *
 * Storage is RAM only — entries do not survive a reboot. The "did we just
 * reboot from a watchdog/brownout?" indicator comes from system_reset_cause()
 * and is recorded as LOG_CODE_*_RESET_DETECTED on first boot.
 *
 * Severity policy:
 *   INFO/WARN/ERROR — log only, do not influence robot behavior
 *   CRITICAL        — also raises the firmware-fault E-STOP source
 * =============================================================================
 */

typedef struct {
    uint32_t       timestamp_ms;    /* monotonic ms since boot */
    log_code_t     code;            /* 16-bit narrowed via cast */
    log_severity_t severity;        /* 8-bit on the wire */
    log_module_t   module;          /* 8-bit on the wire */
    uint32_t       data;            /* free-form context: counter, ADC value, source bits, ... */
} log_entry_t;

void          log_init(void);
void          log_record(log_module_t module, log_severity_t sev, log_code_t code, uint32_t data);
bool          log_pop(log_entry_t *out);          /* false if empty */
uint32_t      log_pending_count(void);
uint32_t      log_dropped_count(void);            /* entries lost to overflow since last clear */
void          log_clear(void);

#endif /* LOG_H */
