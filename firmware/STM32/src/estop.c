#include "estop.h"
#include "config.h"
#include "log.h"
#include "motors.h"
#include "stm32f0xx.h"

static volatile uint8_t s_sources = 0;

void estop_init(void) {
    s_sources = 0;
}

static void log_transition(estop_source_t src, bool now_set) {
    log_record(LOG_MOD_ESTOP,
               now_set ? LOG_SEV_CRITICAL : LOG_SEV_INFO,
               now_set ? LOG_CODE_ESTOP_ASSERTED : LOG_CODE_ESTOP_CLEARED,
               (uint32_t)src);
}

void estop_assert(estop_source_t src) {
    if (src == 0) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool was_set = (s_sources & (uint8_t)src) != 0;
    s_sources |= (uint8_t)src;
    if (!primask) __enable_irq();

    if (!was_set) log_transition(src, true);
}

void estop_clear_autoclearing(estop_source_t src) {
    /* Only allow clearing of bits in the auto-clear set. Sticky sources stay set
     * regardless of how many times their condition appears to "go away". */
    uint8_t allowed = (uint8_t)src & (uint8_t)ESTOP_AUTOCLEAR_MASK;
    if (!allowed) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool was_set = (s_sources & allowed) != 0;
    s_sources &= (uint8_t)~allowed;
    if (!primask) __enable_irq();

    if (was_set) log_transition(src, false);
}

void estop_force_clear(uint8_t source_mask) {
    if (!source_mask) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t actually_cleared = s_sources & source_mask;
    s_sources &= (uint8_t)~source_mask;
    if (!primask) __enable_irq();

    if (actually_cleared) {
        log_record(LOG_MOD_ESTOP, LOG_SEV_WARN, LOG_CODE_ESTOP_OVERRIDE,
                   (uint32_t)actually_cleared);
    }
}

void estop_clear_all(void) {
    estop_force_clear(0xFFu);
}

bool estop_active(void) {
    return s_sources != 0;
}

uint8_t estop_sources(void) {
    return s_sources;
}

bool estop_apply(void) {
    bool enable = (s_sources == 0);
    if (motors_enabled() != enable) {
        motors_set_enabled(enable);
    }
    return enable;
}
