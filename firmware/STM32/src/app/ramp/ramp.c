#include "ramp.h"
#include "config.h"
#include "safety.h"
#include <math.h>

#define SNAP_EPS_V        1e-4f
#define MIN_SEGMENT_T_S   0.001f

typedef struct {
    float v_now, a_now;
    /* CUSTOM-shape segment state */
    float seg_v0, seg_dv, seg_T, seg_s, seg_cmd_lock;
    bool  seg_active;
} axis_t;

typedef struct {
    float   s[RAMP_CURVE_POINTS_MAX];
    float   f[RAMP_CURVE_POINTS_MAX];
    uint8_t n;
    float   peak_slope;
} curve_t;

static ramp_shape_t s_shape   = RAMP_SHAPE_LINEAR;
static float        s_a_lin   = MAX_LINEAR_ACCEL_MPSS;
static float        s_a_ang   = MAX_ANGULAR_ACCEL_RADPSS;
static float        s_j_lin   = 4.0f;
static float        s_j_ang   = 10.0f;
static float        s_tau_lin = 0.30f;
static float        s_tau_ang = 0.20f;

static axis_t  s_lin, s_ang;
static curve_t s_curve_active, s_curve_build;
static bool    s_curve_building = false;

/* ---- helpers ------------------------------------------------------------- */

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float signf(float x) { return (float)((x > 0.0f) - (x < 0.0f)); }
static float absf(float x)  { return x < 0.0f ? -x : x; }

static void axis_reset(axis_t *a) {
    a->v_now = a->a_now = 0.0f;
    a->seg_v0 = a->seg_dv = a->seg_T = a->seg_s = a->seg_cmd_lock = 0.0f;
    a->seg_active = false;
}

static void curve_default(curve_t *c) {     /* identity = LINEAR equivalent */
    c->n = 2;
    c->s[0] = 0.0f; c->f[0] = 0.0f;
    c->s[1] = 1.0f; c->f[1] = 1.0f;
    c->peak_slope = 1.0f;
}

static float curve_eval(const curve_t *c, float s) {
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

/* ---- init / reset -------------------------------------------------------- */

void ramp_init(void) {
    axis_reset(&s_lin);
    axis_reset(&s_ang);
    curve_default(&s_curve_active);
    s_curve_building = false;
}

void ramp_reset(void) {
    s_lin.v_now = s_lin.a_now = 0.0f; s_lin.seg_active = false;
    s_ang.v_now = s_ang.a_now = 0.0f; s_ang.seg_active = false;
}

/* ---- per-shape steppers (one axis) --------------------------------------- */

static void step_linear(axis_t *a, float v_cmd, float max_a, float dt) {
    float max_step = max_a * dt;
    float diff = v_cmd - a->v_now;
    if (absf(diff) <= max_step) a->v_now = v_cmd;
    else                        a->v_now += signf(diff) * max_step;
}

static void step_scurve(axis_t *a, float v_cmd, float max_a, float max_j, float dt) {
    float rem = v_cmd - a->v_now;
    /* Velocity drift while slewing accel back to 0 at max_j. */
    float v_drift = (a->a_now * a->a_now) / (2.0f * (max_j > 1e-6f ? max_j : 1e-6f));
    if (a->a_now < 0.0f) v_drift = -v_drift;
    float rem_after = rem - v_drift;

    float target_a;
    if (absf(rem_after) < SNAP_EPS_V) {
        target_a = 0.0f;
    } else {
        float cap = sqrtf(2.0f * max_j * absf(rem_after));
        if (cap > max_a) cap = max_a;
        target_a = signf(rem_after) * cap;
    }
    float a_diff = target_a - a->a_now;
    float a_step = max_j * dt;
    if (absf(a_diff) <= a_step) a->a_now = target_a;
    else                        a->a_now += signf(a_diff) * a_step;

    a->v_now += a->a_now * dt;

    if (signf(rem) != signf(v_cmd - a->v_now) && signf(rem) != 0.0f) {
        a->v_now = v_cmd; a->a_now = 0.0f;   /* overshoot guard: snap */
    }
}

static void step_exponential(axis_t *a, float v_cmd, float tau, float dt) {
    if (tau < 1e-4f) { a->v_now = v_cmd; return; }
    a->v_now += (1.0f - expf(-dt / tau)) * (v_cmd - a->v_now);
}

static void step_custom(axis_t *a, float v_cmd, float max_a, float dt) {
    const curve_t *c = &s_curve_active;
    bool need_new = !a->seg_active ||
                    absf(v_cmd - a->seg_cmd_lock) > 1e-3f * (absf(v_cmd) + 1.0f);
    if (need_new) {
        a->seg_v0 = a->v_now;
        a->seg_dv = v_cmd - a->v_now;
        a->seg_s  = 0.0f;
        a->seg_cmd_lock = v_cmd;
        float T = absf(a->seg_dv) * c->peak_slope / (max_a > 1e-6f ? max_a : 1e-6f);
        a->seg_T = (T < MIN_SEGMENT_T_S) ? MIN_SEGMENT_T_S : T;
        a->seg_active = true;
    }
    a->seg_s += dt / a->seg_T;
    if (a->seg_s >= 1.0f) {
        a->seg_s = 1.0f; a->v_now = v_cmd; a->seg_active = false;
    } else {
        a->v_now = a->seg_v0 + a->seg_dv * curve_eval(c, a->seg_s);
    }
}

/* ---- public step --------------------------------------------------------- */

void ramp_step(float dt_s, float v_cmd, float w_cmd, float *v_out, float *w_out) {
    float k = safety_caution_modifier();
    switch (s_shape) {
    case RAMP_SHAPE_LINEAR:
        step_linear(&s_lin, v_cmd, s_a_lin * k, dt_s);
        step_linear(&s_ang, w_cmd, s_a_ang * k, dt_s);
        break;
    case RAMP_SHAPE_SCURVE:
        step_scurve(&s_lin, v_cmd, s_a_lin * k, s_j_lin * k, dt_s);
        step_scurve(&s_ang, w_cmd, s_a_ang * k, s_j_ang * k, dt_s);
        break;
    case RAMP_SHAPE_EXPONENTIAL:
        step_exponential(&s_lin, v_cmd, s_tau_lin, dt_s);
        step_exponential(&s_ang, w_cmd, s_tau_ang, dt_s);
        break;
    case RAMP_SHAPE_CUSTOM:
        step_custom(&s_lin, v_cmd, s_a_lin * k, dt_s);
        step_custom(&s_ang, w_cmd, s_a_ang * k, dt_s);
        break;
    default:
        s_lin.v_now = v_cmd; s_ang.v_now = w_cmd;
        break;
    }
    *v_out = s_lin.v_now;
    *w_out = s_ang.v_now;
}

/* ---- setters ------------------------------------------------------------- */

void ramp_set_shape(ramp_shape_t s) {
    if (s == s_shape) return;
    s_shape = s;
    s_lin.a_now = 0.0f; s_lin.seg_active = false;   /* v_now preserved: no snap */
    s_ang.a_now = 0.0f; s_ang.seg_active = false;
}

void ramp_set_max_accel_lin(float a) { if (a >= 0.0f) s_a_lin = a; }
void ramp_set_max_accel_ang(float a) { if (a >= 0.0f) s_a_ang = a; }
void ramp_set_max_jerk_lin(float j)  { if (j >= 0.0f) s_j_lin = j; }
void ramp_set_max_jerk_ang(float j)  { if (j >= 0.0f) s_j_ang = j; }
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
    if (!s_curve_building)                        return false;
    if (s_curve_build.n >= RAMP_CURVE_POINTS_MAX) return false;
    if (s_curve_build.n > 0 && s <= s_curve_build.s[s_curve_build.n - 1]) return false;
    s_curve_build.s[s_curve_build.n] = clampf(s, 0.0f, 1.0f);
    s_curve_build.f[s_curve_build.n] = clampf(f, 0.0f, 1.0f);
    s_curve_build.n++;
    return true;
}

bool ramp_curve_commit(void) {
    if (!s_curve_building)          return false;
    if (s_curve_build.n < 2)        return false;
    if (s_curve_build.s[0] > 1e-3f) return false;
    if (s_curve_build.s[s_curve_build.n - 1] < 1.0f - 1e-3f) return false;

    s_curve_build.s[0] = 0.0f; s_curve_build.f[0] = 0.0f;
    s_curve_build.s[s_curve_build.n - 1] = 1.0f;
    s_curve_build.f[s_curve_build.n - 1] = 1.0f;

    float peak = 0.0f;
    for (uint8_t i = 1; i < s_curve_build.n; i++) {
        float df = s_curve_build.f[i] - s_curve_build.f[i-1];
        float ds = s_curve_build.s[i] - s_curve_build.s[i-1];
        if (df < 0.0f || ds <= 0.0f) return false;   /* must be non-decreasing in f */
        float slope = df / ds;
        if (slope > peak) peak = slope;
    }
    if (peak < 1e-3f) return false;
    s_curve_build.peak_slope = peak;

    s_curve_active = s_curve_build;     /* single-threaded swap */
    s_curve_building = false;
    s_lin.seg_active = false;
    s_ang.seg_active = false;
    return true;
}

void ramp_curve_cancel(void) { s_curve_building = false; }
