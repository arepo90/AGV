#include "ramp.h"
#include "caution.h"
#include "config.h"
#include <math.h>

/* =============================================================================
 *  Implementation notes
 *
 *  All shape paths read max_accel_lin / max_accel_ang and scale by the live
 *  caution modifier inside ramp_step. The user-facing tunables in config.h and
 *  on the wire are the "configured" values; the actual instantaneous limit is
 *  configured × caution_modifier().
 *
 *  S-curve (jerk-limited) decel logic:
 *    v_remaining = v_cmd - v_now
 *    "While we ramp current accel back to 0 over t_stop = |a_now|/jerk, the
 *     velocity will drift by sign(a_now) · a_now² / (2·jerk). Subtract that
 *     from v_remaining to get what's left after we've committed to stopping."
 *    target_a = sign(rem) · min(max_a, √(2·jerk·|rem|))
 *    slew a_now toward target_a at jerk·dt.
 *
 *  Custom curve:
 *    On v_cmd change, open a new segment from v_now → v_cmd. Duration
 *    T = |Δv| · peak_slope_of_f / max_accel — that way the steepest portion
 *    of the operator's curve hits max_accel exactly. Progress s advances at
 *    dt/T. f(s) is piecewise-linear over up to RAMP_CURVE_POINTS_MAX points.
 * =============================================================================
 */

#define RAMP_SNAP_EPS_V        1e-4f
#define RAMP_SNAP_EPS_A        1e-3f
#define RAMP_MIN_SEGMENT_T_S   0.001f

typedef struct {
    float  v_now;
    float  a_now;
    /* Custom-shape segment state */
    float  seg_v0;
    float  seg_dv;
    float  seg_T;
    float  seg_s;
    float  seg_cmd_lock;   /* the v_cmd that opened this segment */
    bool   seg_active;
} axis_state_t;

typedef struct {
    float s[RAMP_CURVE_POINTS_MAX];
    float f[RAMP_CURVE_POINTS_MAX];
    uint8_t n;
    float peak_slope;       /* max(|f(s_{i+1})-f(s_i)| / (s_{i+1}-s_i)) */
} ramp_curve_t;

static ramp_shape_t s_shape       = RAMP_SHAPE_LINEAR;
static float        s_max_a_lin   = MAX_LINEAR_ACCEL_MPSS;
static float        s_max_a_ang   = MAX_ANGULAR_ACCEL_RADPSS;
static float        s_max_j_lin   = 4.0f;      /* m/s³  default — tweak in config if too snappy */
static float        s_max_j_ang   = 10.0f;     /* rad/s³ default */
static float        s_tau_lin     = 0.30f;
static float        s_tau_ang     = 0.20f;

static axis_state_t s_lin;
static axis_state_t s_ang;

static ramp_curve_t s_curve_active;
static ramp_curve_t s_curve_build;
static bool         s_curve_building = false;

/* ---- helpers ------------------------------------------------------------- */

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
static float signf(float x) { return (x > 0.0f) - (x < 0.0f); }
static float absf(float x)  { return x < 0.0f ? -x : x; }

static void axis_reset(axis_state_t *a) {
    a->v_now = 0.0f;
    a->a_now = 0.0f;
    a->seg_v0 = 0.0f;
    a->seg_dv = 0.0f;
    a->seg_T  = 0.0f;
    a->seg_s  = 0.0f;
    a->seg_cmd_lock = 0.0f;
    a->seg_active = false;
}

/* Default curve = identity (LINEAR equivalent). Always valid so CUSTOM never
 * dereferences an empty curve. */
static void curve_install_default(ramp_curve_t *c) {
    c->n = 2;
    c->s[0] = 0.0f; c->f[0] = 0.0f;
    c->s[1] = 1.0f; c->f[1] = 1.0f;
    c->peak_slope = 1.0f;
}

static float curve_eval(const ramp_curve_t *c, float s) {
    if (s <= 0.0f) return 0.0f;
    if (s >= 1.0f) return 1.0f;
    for (uint8_t i = 1; i < c->n; i++) {
        if (s <= c->s[i]) {
            float ds = c->s[i] - c->s[i-1];
            if (ds <= 0.0f) return c->f[i];
            float t = (s - c->s[i-1]) / ds;
            return c->f[i-1] + t * (c->f[i] - c->f[i-1]);
        }
    }
    return 1.0f;
}

/* ---- API: init / reset --------------------------------------------------- */

void ramp_init(void) {
    axis_reset(&s_lin);
    axis_reset(&s_ang);
    curve_install_default(&s_curve_active);
    s_curve_building = false;
}

void ramp_reset(void) {
    /* Don't zero v_now blindly — control.c calls reset on E-STOP, and the
     * cascade is also zeroed there. But the navigator just produced 0 in
     * STANDBY, so starting from 0 is correct. We zero accel/segment state
     * to ensure clean S-curve / custom restart. */
    s_lin.v_now = 0.0f; s_lin.a_now = 0.0f; s_lin.seg_active = false;
    s_ang.v_now = 0.0f; s_ang.a_now = 0.0f; s_ang.seg_active = false;
}

/* ---- shape implementations (one axis at a time) -------------------------- */

static void step_linear(axis_state_t *a, float v_cmd, float max_a, float dt) {
    float max_step = max_a * dt;
    float diff = v_cmd - a->v_now;
    if (absf(diff) <= max_step) a->v_now = v_cmd;
    else                        a->v_now += signf(diff) * max_step;
}

static void step_scurve(axis_state_t *a, float v_cmd,
                        float max_a, float max_j, float dt) {
    float rem = v_cmd - a->v_now;
    /* Residual velocity change if we slewed accel back to 0 at max_j. */
    float v_drift = (a->a_now * a->a_now) / (2.0f * (max_j > 1e-6f ? max_j : 1e-6f));
    if (a->a_now < 0.0f) v_drift = -v_drift;
    float rem_after = rem - v_drift;

    float target_a;
    if (absf(rem_after) < RAMP_SNAP_EPS_V) {
        target_a = 0.0f;
    } else {
        float a_cap = sqrtf(2.0f * max_j * absf(rem_after));
        if (a_cap > max_a) a_cap = max_a;
        target_a = signf(rem_after) * a_cap;
    }

    float a_diff = target_a - a->a_now;
    float max_a_step = max_j * dt;
    if (absf(a_diff) <= max_a_step) a->a_now = target_a;
    else                            a->a_now += signf(a_diff) * max_a_step;

    a->v_now += a->a_now * dt;

    /* Overshoot guard: if we crossed v_cmd this tick, snap. */
    if (signf(rem) != signf(v_cmd - a->v_now) && signf(rem) != 0.0f) {
        a->v_now = v_cmd;
        a->a_now = 0.0f;
    }
}

static void step_exponential(axis_state_t *a, float v_cmd, float tau, float dt) {
    if (tau < 1e-4f) { a->v_now = v_cmd; return; }
    float alpha = 1.0f - expf(-dt / tau);
    a->v_now += alpha * (v_cmd - a->v_now);
}

static void step_custom(axis_state_t *a, float v_cmd, float max_a, float dt) {
    const ramp_curve_t *c = &s_curve_active;
    /* New segment if the command moved meaningfully — use a small dead-band
     * relative to the current step size so noise on v_cmd doesn't constantly
     * restart segments. */
    float cmd_delta = v_cmd - a->seg_cmd_lock;
    bool need_new_segment = !a->seg_active ||
                            absf(cmd_delta) > 1e-3f * (absf(v_cmd) + 1.0f);
    if (need_new_segment) {
        a->seg_v0 = a->v_now;
        a->seg_dv = v_cmd - a->v_now;
        a->seg_s  = 0.0f;
        a->seg_cmd_lock = v_cmd;
        /* T derived so the steepest segment of f matches max_a. */
        float T = absf(a->seg_dv) * c->peak_slope / (max_a > 1e-6f ? max_a : 1e-6f);
        if (T < RAMP_MIN_SEGMENT_T_S) T = RAMP_MIN_SEGMENT_T_S;
        a->seg_T = T;
        a->seg_active = true;
    }
    a->seg_s += dt / a->seg_T;
    if (a->seg_s >= 1.0f) {
        a->seg_s = 1.0f;
        a->v_now = v_cmd;
        a->seg_active = false;
    } else {
        a->v_now = a->seg_v0 + a->seg_dv * curve_eval(c, a->seg_s);
    }
}

/* ---- public step --------------------------------------------------------- */

void ramp_step(float dt_s,
               float v_cmd_raw, float w_cmd_raw,
               float *v_out,    float *w_out) {
    float k = caution_modifier();
    float max_a_lin = s_max_a_lin * k;
    float max_a_ang = s_max_a_ang * k;
    float max_j_lin = s_max_j_lin * k;
    float max_j_ang = s_max_j_ang * k;

    switch (s_shape) {
    case RAMP_SHAPE_LINEAR:
        step_linear(&s_lin, v_cmd_raw, max_a_lin, dt_s);
        step_linear(&s_ang, w_cmd_raw, max_a_ang, dt_s);
        break;
    case RAMP_SHAPE_SCURVE:
        step_scurve(&s_lin, v_cmd_raw, max_a_lin, max_j_lin, dt_s);
        step_scurve(&s_ang, w_cmd_raw, max_a_ang, max_j_ang, dt_s);
        break;
    case RAMP_SHAPE_EXPONENTIAL:
        step_exponential(&s_lin, v_cmd_raw, s_tau_lin, dt_s);
        step_exponential(&s_ang, w_cmd_raw, s_tau_ang, dt_s);
        break;
    case RAMP_SHAPE_CUSTOM:
        step_custom(&s_lin, v_cmd_raw, max_a_lin, dt_s);
        step_custom(&s_ang, w_cmd_raw, max_a_ang, dt_s);
        break;
    default:
        s_lin.v_now = v_cmd_raw;
        s_ang.v_now = w_cmd_raw;
        break;
    }
    *v_out = s_lin.v_now;
    *w_out = s_ang.v_now;
}

/* ---- setters ------------------------------------------------------------- */

void ramp_set_shape(ramp_shape_t s) {
    if (s == s_shape) return;
    s_shape = s;
    /* Clear segment / accel state so the new shape starts cleanly. v_now is
     * preserved so the chassis doesn't snap. */
    s_lin.a_now = 0.0f; s_lin.seg_active = false;
    s_ang.a_now = 0.0f; s_ang.seg_active = false;
}

void ramp_set_max_accel_lin(float a) { if (a >= 0.0f) s_max_a_lin = a; }
void ramp_set_max_accel_ang(float a) { if (a >= 0.0f) s_max_a_ang = a; }
void ramp_set_max_jerk_lin(float j)  { if (j >= 0.0f) s_max_j_lin = j; }
void ramp_set_max_jerk_ang(float j)  { if (j >= 0.0f) s_max_j_ang = j; }
void ramp_set_tau_lin(float t)       { if (t >= 0.0f) s_tau_lin = t; }
void ramp_set_tau_ang(float t)       { if (t >= 0.0f) s_tau_ang = t; }

ramp_shape_t ramp_shape(void) { return s_shape; }

/* ---- custom-curve upload ------------------------------------------------- */

void ramp_curve_begin(void) {
    s_curve_build.n = 0;
    s_curve_build.peak_slope = 0.0f;
    s_curve_building = true;
}

bool ramp_curve_add_point(float s, float f) {
    if (!s_curve_building)                       return false;
    if (s_curve_build.n >= RAMP_CURVE_POINTS_MAX) return false;
    /* Monotonic in s, clamp f to [0,1] for safety. */
    if (s_curve_build.n > 0 && s <= s_curve_build.s[s_curve_build.n - 1]) return false;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    s_curve_build.s[s_curve_build.n] = clampf(s, 0.0f, 1.0f);
    s_curve_build.f[s_curve_build.n] = f;
    s_curve_build.n++;
    return true;
}

bool ramp_curve_commit(void) {
    if (!s_curve_building)            return false;
    if (s_curve_build.n < 2)          return false;
    if (s_curve_build.s[0] > 1e-3f)   return false;
    if (s_curve_build.s[s_curve_build.n - 1] < 1.0f - 1e-3f) return false;
    /* Enforce endpoints exactly. */
    s_curve_build.s[0] = 0.0f; s_curve_build.f[0] = 0.0f;
    s_curve_build.s[s_curve_build.n - 1] = 1.0f;
    s_curve_build.f[s_curve_build.n - 1] = 1.0f;
    /* Compute peak slope and reject non-monotonic-in-f curves. */
    float peak = 0.0f;
    for (uint8_t i = 1; i < s_curve_build.n; i++) {
        float df = s_curve_build.f[i] - s_curve_build.f[i-1];
        float ds = s_curve_build.s[i] - s_curve_build.s[i-1];
        if (df < 0.0f || ds <= 0.0f) return false;
        float slope = df / ds;
        if (slope > peak) peak = slope;
    }
    if (peak < 1e-3f) return false;
    s_curve_build.peak_slope = peak;
    /* Atomic swap — single-threaded main loop, just sequence the copy. */
    s_curve_active = s_curve_build;
    s_curve_building = false;
    /* Force restart of any active CUSTOM segments. */
    s_lin.seg_active = false;
    s_ang.seg_active = false;
    return true;
}

void ramp_curve_cancel(void) {
    s_curve_building = false;
}
