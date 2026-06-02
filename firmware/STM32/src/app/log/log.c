#include "log.h"
#include "config.h"
#include "mcu.h"
#include "stm32f0xx.h"

static log_entry_t       s_buf[FAULT_LOG_DEPTH];
static volatile uint32_t s_head = 0;     /* write index */
static volatile uint32_t s_tail = 0;     /* read index  */
static volatile uint32_t s_dropped = 0;

void log_init(void) {
    s_head = 0;
    s_tail = 0;
    s_dropped = 0;
}

void log_record(log_module_t module, log_severity_t sev, log_code_t code, uint32_t data) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t next = (s_head + 1u) % FAULT_LOG_DEPTH;
    if (next == s_tail) {
        /* Full — drop the oldest. */
        s_tail = (s_tail + 1u) % FAULT_LOG_DEPTH;
        s_dropped++;
    }
    log_entry_t *e = &s_buf[s_head];
    e->timestamp_ms = mcu_now_ms();
    e->code         = code;
    e->severity     = sev;
    e->module       = module;
    e->data         = data;
    s_head = next;

    if (!primask) __enable_irq();
}

bool log_pop(log_entry_t *out) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool ok = false;
    if (s_tail != s_head) {
        *out = s_buf[s_tail];
        s_tail = (s_tail + 1u) % FAULT_LOG_DEPTH;
        ok = true;
    }
    if (!primask) __enable_irq();
    return ok;
}

uint32_t log_pending_count(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t h = s_head, t = s_tail;
    uint32_t n = (h >= t) ? (h - t) : (FAULT_LOG_DEPTH - t + h);
    if (!primask) __enable_irq();
    return n;
}

uint32_t log_dropped_count(void) { return s_dropped; }

void log_clear(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_head = 0;
    s_tail = 0;
    s_dropped = 0;
    if (!primask) __enable_irq();
}
