#include "nav_line.h"
#include "analog.h"
#include "config.h"
#include "log.h"
#include "odometry.h"
#include "pid.h"
#include "types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* T-turn phases. FOLLOW is the normal centroid/PID law; the turn rotates on
 * the spot — blind until the array is guaranteed clear of the T bar, then
 * searching until the line is back in view. */
typedef enum { LF_FOLLOW, LF_TURN_BLIND, LF_TURN_SEARCH } lf_state_t;

static pid_t    s_pid;
static bool     s_lost = false;
static float    s_position = 0.0f;

static float    s_cruise_mps      = LINE_FOLLOW_CRUISE_MPS;
static float    s_min_contrast    = QTR_MIN_CONTRAST_COUNTS;   /* ADC counts (max-min) */
static float    s_t_black         = LINE_T_BLACK_COUNTS;       /* ADC counts, absolute */

static lf_state_t s_state = LF_FOLLOW;
static uint32_t s_t_streak = 0;        /* consecutive frames matching the T bar */
static float    s_turn_swept = 0.0f;   /* |Δθ| integrated over the turn (rad) */
static float    s_turn_elapsed_s = 0.0f;
static float    s_theta_prev = 0.0f;

#define LINE_TURN_OMEGA  (LINE_TURN_CCW ? LINE_TURN_OMEGA_RADPS : -LINE_TURN_OMEGA_RADPS)

static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void nav_line_init(void) {
    pid_init(&s_pid, LINE_FOLLOW_KP, LINE_FOLLOW_KI, LINE_FOLLOW_KD,
             -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS);
    s_lost = false;
    s_position = 0.0f;
    s_state = LF_FOLLOW;
    s_t_streak = 0;
}

void nav_line_reset(void) {
    pid_reset(&s_pid);
    s_lost = false;
    s_state = LF_FOLLOW;
    s_t_streak = 0;
}

/* ---- Control law --------------------------------------------------------- */

bool  nav_line_is_lost(void)  { return s_lost; }
float nav_line_position(void) { return s_position; }

void nav_line_get(float dt_s, float *v_target, float *omega_target) {
    if (!analog_has_data()) { *v_target = 0.0f; *omega_target = 0.0f; return; }

    /* One raw scan feeds everything: min/max for the auto-ranged centroid and
     * an absolute black count for the T bar (auto-ranging alone cannot tell
     * all-black from all-white — both are just low contrast). */
    uint16_t raw[ANALOG_QTR_COUNT];
    uint16_t mn = 0xFFFFu, mx = 0u;
    uint32_t nblack = 0;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        raw[i] = analog_qtr(i);
        if (raw[i] < mn) mn = raw[i];
        if (raw[i] > mx) mx = raw[i];
        if ((float)raw[i] >= s_t_black) nblack++;   /* darker reads higher */
    }

    /* ---- 180° turn at the T (odometry-gated, on-axis) -------------------- */
    if (s_state != LF_FOLLOW) {
        s_turn_elapsed_s += dt_s;
        float th = odometry_theta();
        s_turn_swept += fabsf(wrap_pi(th - s_theta_prev));   /* θ is wrapped ±π */
        s_theta_prev = th;

        if (s_state == LF_TURN_BLIND && s_turn_swept >= LINE_TURN_BLIND_RAD)
            s_state = LF_TURN_SEARCH;

        bool line_visible = (float)(mx - mn) >= s_min_contrast &&
                            nblack < LINE_T_MIN_SENSORS;
        if (s_state == LF_TURN_SEARCH && line_visible) {
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_LINE_REACQUIRED,
                       (uint32_t)(s_turn_swept * 1000.0f));
            s_state = LF_FOLLOW;
            s_t_streak = 0;
            /* fall through — resume following on this same frame */
        } else if (s_turn_swept >= LINE_TURN_MAX_RAD ||
                   s_turn_elapsed_s * 1000.0f >= (float)LINE_TURN_TIMEOUT_MS) {
            log_record(LOG_MOD_NAV, LOG_SEV_WARN, LOG_CODE_LINE_TURN_FAILED,
                       (uint32_t)(s_turn_swept * 1000.0f));
            s_state = LF_FOLLOW;
            s_lost = true;
            *v_target = 0.0f; *omega_target = 0.0f;
            return;
        } else {
            *v_target = 0.0f;
            *omega_target = LINE_TURN_OMEGA;
            return;
        }
    } else if (nblack >= LINE_T_MIN_SENSORS) {
        if (++s_t_streak >= LINE_T_DEBOUNCE_TICKS) {
            log_record(LOG_MOD_NAV, LOG_SEV_INFO, LOG_CODE_LINE_T_DETECTED, nblack);
            s_state = LF_TURN_BLIND;
            s_turn_swept = 0.0f;
            s_turn_elapsed_s = 0.0f;
            s_theta_prev = odometry_theta();
            s_position = 0.0f;
            s_lost = false;
            pid_reset(&s_pid);
            *v_target = 0.0f;
            *omega_target = LINE_TURN_OMEGA;
            return;
        }
        /* Debouncing: hold course over the bar (it reads near-centre anyway)
         * rather than letting the low-contrast guard call it lost. */
        *v_target = s_cruise_mps; *omega_target = 0.0f;
        return;
    } else {
        s_t_streak = 0;
    }

    /* Lost is decided on ABSOLUTE contrast, not the normalised sum: auto-ranging
     * always stretches the frame to full scale, so without this guard a uniform
     * surface's sensor noise would masquerade as a line. */
    if ((float)((int32_t)mx - (int32_t)mn) < s_min_contrast) {
        if (!s_lost) {
            log_record(LOG_MOD_NAV, LOG_SEV_WARN, LOG_CODE_LINE_LOST, 0);
            s_lost = true;
            pid_reset(&s_pid);
        }
        *v_target = 0.0f; *omega_target = 0.0f;
        return;
    }
    s_lost = false;

    float span = (float)((int32_t)mx - (int32_t)mn);   /* > 0: guarded above */
    float weighted = 0.0f, total = 0.0f;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) {
        float n = (float)((int32_t)raw[i] - (int32_t)mn) / span;   /* line→1, floor→0 */
#if QTR_INVERT_ARRAY
        float idx = (float)((ANALOG_QTR_COUNT - 1u) - i);
#else
        float idx = (float)i;
#endif
        weighted += n * idx;
        total    += n;
    }

    float centre   = ((float)ANALOG_QTR_COUNT - 1.0f) * 0.5f;
    float centroid = weighted / total;             /* total >= 1 (max sensor → 1) */
    s_position = (centroid - centre) / centre;     /* [-1, +1] */

    /* Setpoint 0 (line centred). error = 0 - position; Kp>0 turns toward it. */
    float omega = pid_step(&s_pid, 0.0f, s_position, 0.0f, dt_s);

    float v = s_cruise_mps;
    if (s_position > 0.6f || s_position < -0.6f) v *= 0.5f;   /* slow on sharp corrections */

    *v_target = v;
    *omega_target = omega;
}

void nav_line_set_cruise_mps(float v)    { if (v >= 0.0f && v <= 5.0f) s_cruise_mps = v; }
void nav_line_set_lost_threshold(float t){ if (t >= 0.0f && t <= 4095.0f) s_min_contrast = t; }
void nav_line_set_t_black_counts(float c){ if (c >= 0.0f && c <= 4095.0f) s_t_black = c; }
void nav_line_set_gains(float kp, float ki, float kd) { pid_set_gains(&s_pid, kp, ki, kd); }
