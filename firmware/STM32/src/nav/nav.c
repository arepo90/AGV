#include "nav.h"
#include "nav_line.h"
#include "nav_traj.h"
#include "state.h"
#include "stm32f0xx.h"
#include "types.h"

static volatile float s_remote_linear  = 0.0f;
static volatile float s_remote_angular = 0.0f;

void nav_init(void) {
    s_remote_linear  = 0.0f;
    s_remote_angular = 0.0f;
    nav_line_init();
    nav_traj_init();
}

void nav_reset(void) {
    nav_line_reset();
    nav_traj_reset();
}

void nav_remote_set(float linear, float angular) {
    /* Disable IRQs so the float pair is updated atomically — proximity ISR
     * can't preempt mid-write. cmd.c is the only writer otherwise. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_remote_linear  = linear;
    s_remote_angular = angular;
    if (!primask) __enable_irq();
}

void nav_remote_get(float *linear, float *angular) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *linear  = s_remote_linear;
    *angular = s_remote_angular;
    if (!primask) __enable_irq();
}

void nav_get_target(float dt_s, float *v_target, float *omega_target) {
    switch (state_function()) {
    case FUNC_REMOTE_CONTROL:
        nav_remote_get(v_target, omega_target);
        return;

    case FUNC_LINE_FOLLOW:
        nav_line_get(dt_s, v_target, omega_target);
        return;

    case FUNC_TRAJECTORY_FOLLOW:
        nav_traj_get(dt_s, v_target, omega_target);
        return;

    case FUNC_STANDBY:
    default:
        *v_target = 0.0f; *omega_target = 0.0f;
        return;
    }
}
