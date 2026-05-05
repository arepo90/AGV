#include "heartbeat.h"
#include "config.h"
#include "estop.h"
#include "log.h"
#include "state.h"
#include "system.h"
#include "types.h"

static uint32_t s_last_ms = 0;
static uint8_t  s_stage = 0;
static bool     s_unsupervised_due_to_timeout = false;

void heartbeat_init(void) {
    s_last_ms = system_now_ms();
    s_stage = 0;
    s_unsupervised_due_to_timeout = false;
}

uint32_t heartbeat_ms_since_last(void) {
    return system_now_ms() - s_last_ms;
}

uint8_t heartbeat_stage(void) { return s_stage; }

void heartbeat_received(void) {
    s_last_ms = system_now_ms();

    if (s_stage > 0) {
        log_record(LOG_MOD_HEARTBEAT, LOG_SEV_INFO,
                   LOG_CODE_HEARTBEAT_RESTORED, (uint32_t)s_stage);
        s_stage = 0;

        /* Restore SUPERVISED if we degraded ourselves into UNSUPERVISED. We do
         * NOT auto-clear ESTOP_SRC_HEARTBEAT_TIMEOUT — workstation must do that
         * explicitly via OVERRIDE_ESTOP_SOURCE or RESET. */
        if (s_unsupervised_due_to_timeout && state_mode() == MODE_UNSUPERVISED) {
            state_set_mode(MODE_SUPERVISED);
        }
        s_unsupervised_due_to_timeout = false;
    }
}

void heartbeat_tick(void) {
#if DISABLE_HEARTBEAT_WATCH
    return;
#else
    /* Watcher only meaningful in SUPERVISED mode (or when we ourselves
     * degraded the mode and are waiting on the grace timer). */
    if (state_mode() == MODE_UNSUPERVISED && !s_unsupervised_due_to_timeout) {
        s_last_ms = system_now_ms();   /* keep timer fresh; watcher dormant */
        return;
    }

    uint32_t elapsed = system_now_ms() - s_last_ms;

    if (s_stage == 0 && elapsed > HEARTBEAT_TIMEOUT_MS) {
        s_stage = 1;
        log_record(LOG_MOD_HEARTBEAT, LOG_SEV_WARN, LOG_CODE_HEARTBEAT_LOST, elapsed);
        if (state_mode() == MODE_SUPERVISED) {
            s_unsupervised_due_to_timeout = true;
            state_set_mode(MODE_UNSUPERVISED);
        }
    }

    if (s_stage == 1 && elapsed > (HEARTBEAT_TIMEOUT_MS + HEARTBEAT_GRACE_MS)) {
        s_stage = 2;
        log_record(LOG_MOD_HEARTBEAT, LOG_SEV_CRITICAL,
                   LOG_CODE_HEARTBEAT_GRACE_EXPIRED, elapsed);
        estop_assert(ESTOP_SRC_HEARTBEAT_TIMEOUT);
    }
#endif
}
