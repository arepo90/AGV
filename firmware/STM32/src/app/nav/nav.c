#include "nav.h"
#include "nav_line.h"
#include "safety.h"
#include "stm32f0xx.h"
#include "types.h"

static volatile float s_remote_v = 0.0f;
static volatile float s_remote_w = 0.0f;

void nav_init(void) {
    s_remote_v = 0.0f;
    s_remote_w = 0.0f;
    nav_line_init();
}

void nav_reset(void) {
    nav_line_reset();
}

void nav_remote_set(float linear, float angular) {
    /* Keep the (v, ω) pair consistent against a future ISR reader. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_remote_v = linear;
    s_remote_w = angular;
    if (!primask) __enable_irq();
}

void nav_remote_get(float *linear, float *angular) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *linear  = s_remote_v;
    *angular = s_remote_w;
    if (!primask) __enable_irq();
}

void nav_get_target(float dt_s, float *v_target, float *omega_target) {
    switch (safety_function()) {
    case FUNC_REMOTE_CONTROL: nav_remote_get(v_target, omega_target);      return;
    case FUNC_LINE_FOLLOW:    nav_line_get(dt_s, v_target, omega_target);  return;
    case FUNC_STANDBY:
    default:                     *v_target = 0.0f; *omega_target = 0.0f;      return;
    }
}
