#include "nav_traj.h"
#include "config.h"
#include "log.h"
#include "odometry.h"
#include "types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct { float x, y; } waypoint_t;

static waypoint_t s_wp[MAX_WAYPOINTS];
static uint8_t    s_count = 0;
static uint8_t    s_active = 0;
static bool       s_completed = false;

void nav_traj_init(void) {
    s_count = 0;
    s_active = 0;
    s_completed = false;
}

void nav_traj_reset(void) {
    /* Preserve waypoints — only the PID-equivalent state needs reset, and
     * we have no PID here (point-and-steer is purely proportional). Active
     * index is also preserved so an E-STOP/clear cycle resumes mid-route. */
}

void nav_traj_clear(void) {
    s_count = 0;
    s_active = 0;
    s_completed = false;
}

bool nav_traj_add(float x, float y) {
    if (s_count >= MAX_WAYPOINTS) return false;
    s_wp[s_count].x = x;
    s_wp[s_count].y = y;
    s_count++;
    /* Adding waypoints reactivates the trajectory; if we'd previously hit the
     * end, the new waypoints are the new tail to follow. */
    if (s_completed) {
        s_active = (uint8_t)(s_count - 1u);
        s_completed = false;
    }
    return true;
}

uint8_t nav_traj_count(void)         { return s_count; }
uint8_t nav_traj_active_index(void)  { return s_active; }
bool    nav_traj_complete(void)      { return s_completed; }

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void nav_traj_get(float dt_s, float *v_target, float *omega_target) {
    (void)dt_s;   /* unused: this is a pure proportional law on heading error */

    if (s_count == 0 || s_active >= s_count || s_completed) {
        *v_target = 0.0f;
        *omega_target = 0.0f;
        return;
    }

    float dx = s_wp[s_active].x - odometry_x();
    float dy = s_wp[s_active].y - odometry_y();
    float dist = sqrtf(dx*dx + dy*dy);

    if (dist < WAYPOINT_REACH_RADIUS_M) {
        log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_WAYPOINT_REACHED, s_active);
        s_active++;
        if (s_active >= s_count) {
            s_completed = true;
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_TRAJECTORY_COMPLETE, s_count);
        }
        /* Don't recurse into the next waypoint this tick — it could put a
         * sharp transient on the cascade. Output zero for one tick; next call
         * naturally starts steering toward the new active waypoint. */
        *v_target = 0.0f;
        *omega_target = 0.0f;
        return;
    }

    float heading_to_wp = atan2f(dy, dx);
    float err = wrap_pi(heading_to_wp - odometry_theta());

    float omega = TRAJECTORY_HEADING_KP * err;
    /* Clamp to chassis maxima; the cascade will further apply caution. */
    if (omega >  MAX_ANGULAR_SPEED_RADPS) omega =  MAX_ANGULAR_SPEED_RADPS;
    if (omega < -MAX_ANGULAR_SPEED_RADPS) omega = -MAX_ANGULAR_SPEED_RADPS;

    float v = TRAJECTORY_CRUISE_MPS;
    if (err > TRAJECTORY_SLOWDOWN_RAD || err < -TRAJECTORY_SLOWDOWN_RAD) {
        v *= 0.5f;
    }

    *v_target = v;
    *omega_target = omega;
}
