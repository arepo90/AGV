#ifndef NAV_H
#define NAV_H

/* =============================================================================
 *  Navigator dispatcher.  (app tier)
 *
 *  control_tick() calls nav_get_target() once per tick; it switches on the
 *  active function and forwards to the matching control law:
 *    STANDBY           → (0, 0)
 *    REMOTE_CONTROL    → latest workstation (v, ω)
 *    LINE_FOLLOW       → nav_line (QTR centroid + PID)
 *
 *  Navigators are pure control laws — they never change function; the safety
 *  state machine does. nav_reset() clears navigator-internal state on E-STOP.
 * =============================================================================
 */

void nav_init(void);
void nav_reset(void);
void nav_get_target(float dt_s, float *v_target, float *omega_target);

/* REMOTE_CONTROL setpoint (set by the CMD_VEL_CMD handler in proto.c). */
void nav_remote_set(float linear, float angular);
void nav_remote_get(float *linear, float *angular);

#endif /* NAV_H */
