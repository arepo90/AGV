#include "state.h"
#include "caution.h"
#include "log.h"
#include "config.h"

static agv_mode_t     s_mode = MODE_SUPERVISED;
static agv_function_t s_func = FUNC_STANDBY;

bool function_is_supervised_only(agv_function_t f) {
    return f == FUNC_REMOTE_CONTROL;
}

bool function_is_navigating(agv_function_t f) {
    return f != FUNC_STANDBY;
}

static void update_unsupervised_baseline(void) {
    /* Architecture: UNSUPERVISED + navigating → CAUTION baseline. */
    if (s_mode == MODE_UNSUPERVISED && function_is_navigating(s_func)) {
        caution_set(CAUTION_SRC_UNSUPERVISED_NAV, CAUTION_LEVEL_CAUTION);
    } else {
        caution_clear(CAUTION_SRC_UNSUPERVISED_NAV);
    }
}

void state_init(void) {
    s_mode = MODE_SUPERVISED;
    s_func = FUNC_STANDBY;
    update_unsupervised_baseline();
}

agv_mode_t     state_mode(void)     { return s_mode; }
agv_function_t state_function(void) { return s_func; }

bool state_set_mode(agv_mode_t new_mode) {
    if (new_mode == s_mode) return true;

    if (new_mode != MODE_SUPERVISED && new_mode != MODE_UNSUPERVISED) {
        log_record(LOG_MOD_STATE, LOG_SEV_ERROR, LOG_CODE_ILLEGAL_TRANSITION,
                   ((uint32_t)s_mode << 8) | (uint32_t)new_mode);
        return false;
    }

    agv_mode_t old_mode = s_mode;
    s_mode = new_mode;
    log_record(LOG_MOD_STATE, LOG_SEV_INFO, LOG_CODE_MODE_TRANSITION,
               ((uint32_t)old_mode << 8) | (uint32_t)new_mode);

    /* On SUPERVISED→UNSUPERVISED, downgrade SUPERVISED-only functions. */
    if (s_mode == MODE_UNSUPERVISED && function_is_supervised_only(s_func)) {
        agv_function_t old_func = s_func;
        s_func = FUNC_STANDBY;
        log_record(LOG_MOD_STATE, LOG_SEV_WARN, LOG_CODE_FUNCTION_TRANSITION,
                   ((uint32_t)old_func << 8) | (uint32_t)s_func);
    }

    update_unsupervised_baseline();
    return true;
}

bool state_set_function(agv_function_t new_func) {
    if (new_func == s_func) return true;

    /* Range check */
    if (new_func != FUNC_STANDBY        && new_func != FUNC_REMOTE_CONTROL &&
        new_func != FUNC_LINE_FOLLOW    && new_func != FUNC_TRAJECTORY_FOLLOW) {
        log_record(LOG_MOD_STATE, LOG_SEV_ERROR, LOG_CODE_ILLEGAL_TRANSITION,
                   ((uint32_t)s_func << 8) | (uint32_t)new_func);
        return false;
    }

    /* Mode legality */
    if (function_is_supervised_only(new_func) && s_mode != MODE_SUPERVISED) {
        log_record(LOG_MOD_STATE, LOG_SEV_ERROR, LOG_CODE_ILLEGAL_TRANSITION,
                   ((uint32_t)s_func << 8) | (uint32_t)new_func);
        return false;
    }

    agv_function_t old_func = s_func;
    s_func = new_func;
    log_record(LOG_MOD_STATE, LOG_SEV_INFO, LOG_CODE_FUNCTION_TRANSITION,
               ((uint32_t)old_func << 8) | (uint32_t)s_func);

    update_unsupervised_baseline();
    return true;
}
