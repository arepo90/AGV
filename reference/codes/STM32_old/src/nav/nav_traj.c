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
static waypoint_t s_start;            /* implicit waypoint 0: pose at trajectory start */
static bool       s_start_captured = false;
static uint8_t    s_count = 0;
static uint8_t    s_active = 0;       /* index of the waypoint we're heading TO */
static bool       s_completed = false;

static float      s_cruise_mps    = TRAJECTORY_CRUISE_MPS;
static float      s_lookahead_m   = PURE_PURSUIT_LOOKAHEAD_M;
static float      s_curv_slowdown = TRAJECTORY_CURV_SLOWDOWN;

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void nav_traj_init(void) {
    s_count = 0;
    s_active = 0;
    s_completed = false;
    s_start_captured = false;
}

void nav_traj_reset(void) {
    /* Re-anchor the first segment to the AGV's current pose on next tick.
     * Waypoints and active index are preserved so an E-STOP/clear cycle
     * resumes mid-route with sensible cross-track reference. */
    s_start_captured = false;
}

void nav_traj_clear(void) {
    s_count = 0;
    s_active = 0;
    s_completed = false;
    s_start_captured = false;
}

bool nav_traj_add(float x, float y) {
    if (s_count >= MAX_WAYPOINTS) return false;
    s_wp[s_count].x = x;
    s_wp[s_count].y = y;
    s_count++;
    if (s_completed) {
        /* Appending after completion: treat the new waypoints as the next
         * leg starting from where we are now. */
        s_active = (uint8_t)(s_count - 1u);
        s_completed = false;
        s_start_captured = false;
    }
    return true;
}

uint8_t nav_traj_count(void)         { return s_count; }
uint8_t nav_traj_active_index(void)  { return s_active; }
bool    nav_traj_complete(void)      { return s_completed; }

/* Project pose P onto the line segment A→B. Returns the projection parameter
 * t in [-∞, +∞] (clamped only at the call site if needed). */
static float project_t(waypoint_t a, waypoint_t b, float px, float py) {
    float sdx = b.x - a.x;
    float sdy = b.y - a.y;
    float slen2 = sdx*sdx + sdy*sdy;
    if (slen2 < 1e-6f) return 1.0f;   /* zero-length segment → already past it */
    return ((px - a.x)*sdx + (py - a.y)*sdy) / slen2;
}

void nav_traj_get(float dt_s, float *v_target, float *omega_target) {
    (void)dt_s;

    if (s_count == 0 || s_completed) {
        *v_target = 0.0f;
        *omega_target = 0.0f;
        return;
    }

    float px = odometry_x();
    float py = odometry_y();
    float pt = odometry_theta();

    /* Capture the first-segment anchor on entry to the trajectory. */
    if (!s_start_captured) {
        s_start.x = px;
        s_start.y = py;
        s_start_captured = true;
    }

    /* ---- Terminal check: within reach radius of the final waypoint --------- */
    {
        float ddx = s_wp[s_count - 1].x - px;
        float ddy = s_wp[s_count - 1].y - py;
        if (ddx*ddx + ddy*ddy < WAYPOINT_REACH_RADIUS_M * WAYPOINT_REACH_RADIUS_M) {
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_WAYPOINT_REACHED, s_count - 1);
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_TRAJECTORY_COMPLETE, s_count);
            s_active = s_count;
            s_completed = true;
            *v_target = 0.0f;
            *omega_target = 0.0f;
            return;
        }
    }

    /* ---- Step 1: advance s_active until the projection lies inside the
     * active segment (t < 1). Skip degenerate segments and log each WP we
     * pass. */
    while (s_active < s_count) {
        waypoint_t a = (s_active == 0) ? s_start : s_wp[s_active - 1];
        waypoint_t b = s_wp[s_active];
        float t = project_t(a, b, px, py);
        if (t >= 1.0f) {
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_WAYPOINT_REACHED, s_active);
            s_active++;
            continue;
        }
        break;
    }
    if (s_active >= s_count) {
        /* Walked past the last waypoint without crossing the reach radius —
         * still mark complete so we don't drive off into the distance. */
        log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_TRAJECTORY_COMPLETE, s_count);
        s_completed = true;
        *v_target = 0.0f;
        *omega_target = 0.0f;
        return;
    }

    /* ---- Step 2: lookahead walk -------------------------------------------- */
    waypoint_t a = (s_active == 0) ? s_start : s_wp[s_active - 1];
    waypoint_t b = s_wp[s_active];
    float sdx = b.x - a.x;
    float sdy = b.y - a.y;
    float seg_len = sqrtf(sdx*sdx + sdy*sdy);

    float t = project_t(a, b, px, py);
    if (t < 0.0f) t = 0.0f;   /* AGV is behind the segment start — walk from start */

    float lx, ly;
    float remaining = s_lookahead_m;

    /* Distance from the projection point to the current segment endpoint. */
    float seg_remaining = (1.0f - t) * seg_len;

    if (remaining <= seg_remaining || seg_len < 1e-6f) {
        /* Lookahead lands within the current segment. */
        float walk_t = t + (seg_len > 1e-6f ? remaining / seg_len : 0.0f);
        lx = a.x + walk_t * sdx;
        ly = a.y + walk_t * sdy;
    } else {
        /* Cross the endpoint of the active segment, continue onto successors. */
        lx = b.x;
        ly = b.y;
        remaining -= seg_remaining;
        uint8_t i = (uint8_t)(s_active + 1u);
        while (i < s_count && remaining > 0.0f) {
            waypoint_t aa = s_wp[i - 1];
            waypoint_t bb = s_wp[i];
            float dx2 = bb.x - aa.x;
            float dy2 = bb.y - aa.y;
            float len2 = sqrtf(dx2*dx2 + dy2*dy2);
            if (len2 < 1e-6f) { i++; continue; }
            if (remaining <= len2) {
                float frac = remaining / len2;
                lx = aa.x + frac * dx2;
                ly = aa.y + frac * dy2;
                remaining = 0.0f;
            } else {
                lx = bb.x;
                ly = bb.y;
                remaining -= len2;
                i++;
            }
        }
        /* If we ran out of segments, lx/ly is at the final waypoint — fine. */
    }

    /* ---- Step 3: curvature law --------------------------------------------- */
    float dx_la = lx - px;
    float dy_la = ly - py;
    float la_dist2 = dx_la*dx_la + dy_la*dy_la;

    if (la_dist2 < 1e-6f) {
        /* Lookahead coincides with AGV position — nothing sensible to do. */
        *v_target = 0.0f;
        *omega_target = 0.0f;
        return;
    }

    float la_dist = sqrtf(la_dist2);
    float alpha   = wrap_pi(atan2f(dy_la, dx_la) - pt);

    /* Lookahead is behind us — pure pursuit can't drive in reverse. Spin
     * toward it at half ω_max until the front catches up. */
    if (alpha > (float)M_PI * 0.5f || alpha < -(float)M_PI * 0.5f) {
        *v_target = 0.0f;
        *omega_target = (alpha > 0.0f ? 1.0f : -1.0f) * 0.5f * MAX_ANGULAR_SPEED_RADPS;
        return;
    }

    /* Curvature of the arc from current pose tangent to the lookahead. */
    float kappa = 2.0f * sinf(alpha) / la_dist;
    float abs_k = kappa < 0.0f ? -kappa : kappa;

    /* Curvature-aware speed: v = cruise / (1 + g·|κ|). g has units of metres
     * (since κ is 1/m). g = 0.5 m halves v at a 0.5 m turn radius. */
    float v     = s_cruise_mps / (1.0f + s_curv_slowdown * abs_k);
    float omega = v * kappa;

    if (omega >  MAX_ANGULAR_SPEED_RADPS) omega =  MAX_ANGULAR_SPEED_RADPS;
    if (omega < -MAX_ANGULAR_SPEED_RADPS) omega = -MAX_ANGULAR_SPEED_RADPS;

    *v_target = v;
    *omega_target = omega;
}

void nav_traj_set_cruise_mps(float v) {
    if (v >= 0.0f && v <= 5.0f) s_cruise_mps = v;
}
void nav_traj_set_lookahead_m(float m) {
    if (m > 0.0f && m <= 5.0f) s_lookahead_m = m;
}
void nav_traj_set_curv_slowdown(float g) {
    if (g >= 0.0f && g <= 5.0f) s_curv_slowdown = g;
}
