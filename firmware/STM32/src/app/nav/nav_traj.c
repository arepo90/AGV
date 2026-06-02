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
static waypoint_t s_start;              /* implicit waypoint 0 */
static bool       s_start_captured = false;
static uint8_t    s_count = 0;
static uint8_t    s_active = 0;         /* index we're heading TO */
static bool       s_completed = false;

static float s_cruise_mps    = TRAJECTORY_CRUISE_MPS;
static float s_lookahead_m   = PURE_PURSUIT_LOOKAHEAD_M;
static float s_curv_slowdown = TRAJECTORY_CURV_SLOWDOWN;

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void nav_traj_init(void) {
    s_count = 0; s_active = 0; s_completed = false; s_start_captured = false;
}

/* Re-anchor segment 0 on the next tick; waypoints/active index preserved so an
 * E-STOP/clear cycle resumes mid-route with a sensible cross-track reference. */
void nav_traj_reset(void) { s_start_captured = false; }

void nav_traj_clear(void) {
    s_count = 0; s_active = 0; s_completed = false; s_start_captured = false;
}

bool nav_traj_add(float x, float y) {
    if (s_count >= MAX_WAYPOINTS) return false;
    s_wp[s_count].x = x;
    s_wp[s_count].y = y;
    s_count++;
    if (s_completed) {     /* append after completion → new leg from here */
        s_active = (uint8_t)(s_count - 1u);
        s_completed = false;
        s_start_captured = false;
    }
    return true;
}

uint8_t nav_traj_count(void)    { return s_count; }
bool    nav_traj_complete(void) { return s_completed; }

/* Projection parameter t of pose P onto segment A→B (unclamped). */
static float project_t(waypoint_t a, waypoint_t b, float px, float py) {
    float dx = b.x - a.x, dy = b.y - a.y;
    float len2 = dx*dx + dy*dy;
    if (len2 < 1e-6f) return 1.0f;     /* zero-length → already past */
    return ((px - a.x)*dx + (py - a.y)*dy) / len2;
}

void nav_traj_get(float dt_s, float *v_target, float *omega_target) {
    (void)dt_s;
    if (s_count == 0 || s_completed) { *v_target = 0.0f; *omega_target = 0.0f; return; }

    float px = odometry_x(), py = odometry_y(), pt = odometry_theta();

    if (!s_start_captured) { s_start.x = px; s_start.y = py; s_start_captured = true; }

    /* Terminal check: within reach radius of the final waypoint. */
    {
        float dx = s_wp[s_count-1].x - px, dy = s_wp[s_count-1].y - py;
        if (dx*dx + dy*dy < WAYPOINT_REACH_RADIUS_M * WAYPOINT_REACH_RADIUS_M) {
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_WAYPOINT_REACHED, s_count - 1);
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_TRAJECTORY_COMPLETE, s_count);
            s_active = s_count; s_completed = true;
            *v_target = 0.0f; *omega_target = 0.0f;
            return;
        }
    }

    /* Advance the active segment while the projection lies past its end. */
    while (s_active < s_count) {
        waypoint_t a = (s_active == 0) ? s_start : s_wp[s_active - 1];
        waypoint_t b = s_wp[s_active];
        if (project_t(a, b, px, py) >= 1.0f) {
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_WAYPOINT_REACHED, s_active);
            s_active++;
            continue;
        }
        break;
    }
    if (s_active >= s_count) {
        log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_TRAJECTORY_COMPLETE, s_count);
        s_completed = true;
        *v_target = 0.0f; *omega_target = 0.0f;
        return;
    }

    /* Lookahead walk from the projection point along the path. */
    waypoint_t a = (s_active == 0) ? s_start : s_wp[s_active - 1];
    waypoint_t b = s_wp[s_active];
    float sdx = b.x - a.x, sdy = b.y - a.y;
    float seg_len = sqrtf(sdx*sdx + sdy*sdy);

    float t = project_t(a, b, px, py);
    if (t < 0.0f) t = 0.0f;

    float lx, ly, remaining = s_lookahead_m;
    float seg_remaining = (1.0f - t) * seg_len;

    if (remaining <= seg_remaining || seg_len < 1e-6f) {
        float walk_t = t + (seg_len > 1e-6f ? remaining / seg_len : 0.0f);
        lx = a.x + walk_t * sdx;
        ly = a.y + walk_t * sdy;
    } else {
        lx = b.x; ly = b.y;
        remaining -= seg_remaining;
        uint8_t i = (uint8_t)(s_active + 1u);
        while (i < s_count && remaining > 0.0f) {
            waypoint_t aa = s_wp[i-1], bb = s_wp[i];
            float dx2 = bb.x - aa.x, dy2 = bb.y - aa.y;
            float len2 = sqrtf(dx2*dx2 + dy2*dy2);
            if (len2 < 1e-6f) { i++; continue; }
            if (remaining <= len2) {
                float frac = remaining / len2;
                lx = aa.x + frac * dx2; ly = aa.y + frac * dy2;
                remaining = 0.0f;
            } else {
                lx = bb.x; ly = bb.y; remaining -= len2; i++;
            }
        }
    }

    /* Curvature law. */
    float dx_la = lx - px, dy_la = ly - py;
    float la_dist2 = dx_la*dx_la + dy_la*dy_la;
    if (la_dist2 < 1e-6f) { *v_target = 0.0f; *omega_target = 0.0f; return; }

    float la_dist = sqrtf(la_dist2);
    float alpha   = wrap_pi(atan2f(dy_la, dx_la) - pt);

    /* Lookahead behind us — rotate toward it (pure pursuit can't reverse). */
    if (alpha > (float)M_PI * 0.5f || alpha < -(float)M_PI * 0.5f) {
        *v_target = 0.0f;
        *omega_target = (alpha > 0.0f ? 1.0f : -1.0f) * 0.5f * MAX_ANGULAR_SPEED_RADPS;
        return;
    }

    float kappa = 2.0f * sinf(alpha) / la_dist;
    float abs_k = kappa < 0.0f ? -kappa : kappa;
    float v     = s_cruise_mps / (1.0f + s_curv_slowdown * abs_k);
    float omega = v * kappa;
    if (omega >  MAX_ANGULAR_SPEED_RADPS) omega =  MAX_ANGULAR_SPEED_RADPS;
    if (omega < -MAX_ANGULAR_SPEED_RADPS) omega = -MAX_ANGULAR_SPEED_RADPS;

    *v_target = v;
    *omega_target = omega;
}

void nav_traj_set_cruise_mps(float v)    { if (v >= 0.0f && v <= 5.0f) s_cruise_mps = v; }
void nav_traj_set_lookahead_m(float m)   { if (m > 0.0f && m <= 5.0f) s_lookahead_m = m; }
void nav_traj_set_curv_slowdown(float g) { if (g >= 0.0f && g <= 5.0f) s_curv_slowdown = g; }
