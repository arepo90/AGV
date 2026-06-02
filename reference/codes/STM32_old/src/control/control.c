#include "control.h"
#include "caution.h"
#include "config.h"
#include "encoders.h"
#include "estop.h"
#include "motors.h"
#include "nav.h"
#include "odometry.h"
#include "ramp.h"
#include "sta.h"

static sta_t s_sta_inner_left, s_sta_inner_right;
static sta_t s_sta_outer_lin,  s_sta_outer_ang;

static float s_v_target_last         = 0.0f;
static float s_omega_target_last     = 0.0f;
static float s_duty_left_last        = 0.0f;
static float s_duty_right_last       = 0.0f;
static float s_v_left_target_last    = 0.0f;
static float s_v_right_target_last   = 0.0f;

static void reset_all_controllers(void) {
    sta_reset(&s_sta_inner_left);
    sta_reset(&s_sta_inner_right);
    sta_reset(&s_sta_outer_lin);
    sta_reset(&s_sta_outer_ang);
    nav_reset();   /* clear navigator-internal state too */
    ramp_reset();  /* zero accel state + close any open custom segment */
}

void control_init(void) {
    /* Inner: per-wheel velocity loop. Output is normalised duty in [-1, +1]. */
    sta_init(&s_sta_inner_left,
             STA_INNER_K1_LEFT, STA_INNER_K2_LEFT,
             STA_INNER_OUTPUT_MIN, STA_INNER_OUTPUT_MAX, STA_INNER_U1_LIMIT);
    sta_init(&s_sta_inner_right,
             STA_INNER_K1_RIGHT, STA_INNER_K2_RIGHT,
             STA_INNER_OUTPUT_MIN, STA_INNER_OUTPUT_MAX, STA_INNER_U1_LIMIT);

    /* Outer: chassis (linear, angular) loop. Output is in m/s and rad/s,
     * clamped at the configured chassis maxima so the kinematic split sees an
     * achievable target. u1 uses the same magnitude bounds. */
    sta_init(&s_sta_outer_lin,
             STA_OUTER_LIN_K1, STA_OUTER_LIN_K2,
             -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS,
             MAX_LINEAR_SPEED_MPS);
    sta_init(&s_sta_outer_ang,
             STA_OUTER_ANG_K1, STA_OUTER_ANG_K2,
             -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS,
             MAX_ANGULAR_SPEED_RADPS);
}

void control_set_inner_gains_left(float k1, float k2) {
    sta_set_gains(&s_sta_inner_left, k1, k2);
}
void control_set_inner_gains_right(float k1, float k2) {
    sta_set_gains(&s_sta_inner_right, k1, k2);
}
void control_set_outer_lin_gains(float k1, float k2) {
    sta_set_gains(&s_sta_outer_lin, k1, k2);
}
void control_set_outer_ang_gains(float k1, float k2) {
    sta_set_gains(&s_sta_outer_ang, k1, k2);
}

static float clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void control_tick(float dt_s) {
    /* E-STOP path: zero PWM, reset controllers. SLEEP is handled by main.c;
     * this keeps controller state clean so motion resumes without windup on
     * clear. */
    if (estop_active()) {
        reset_all_controllers();
        motors_set_pwm_signed(MOTOR_LEFT,  0.0f);
        motors_set_pwm_signed(MOTOR_RIGHT, 0.0f);
        s_duty_left_last  = 0.0f;
        s_duty_right_last = 0.0f;
        return;
    }

    /* 1. Navigator target */
    float v_cmd, w_cmd;
    nav_get_target(dt_s, &v_cmd, &w_cmd);

    /* 1b. Motion profile (slew limiter). Reshapes (v_cmd, w_cmd) into the
     *     ramped setpoint that downstream sees. Applies uniformly across
     *     REMOTE_CONTROL, LINE_FOLLOW, and TRAJECTORY_FOLLOW. Caution-scaling
     *     of max_accel happens inside ramp_step. */
    ramp_step(dt_s, v_cmd, w_cmd, &v_cmd, &w_cmd);

    /* 2. Caution-modified setpoint clamp (single point where modifier scales
     *    the chassis limits — see architecture §Caution Modifier). */
    float modifier = caution_modifier();
    float v_max = MAX_LINEAR_SPEED_MPS  * modifier;
    float w_max = MAX_ANGULAR_SPEED_RADPS * modifier;
    v_cmd = clamp(v_cmd, -v_max,  v_max);
    w_cmd = clamp(w_cmd, -w_max,  w_max);

    s_v_target_last     = v_cmd;
    s_omega_target_last = w_cmd;

    /* 3. Outer cascade (optional) */
    float v_corr = v_cmd, w_corr = w_cmd;
#if USE_OUTER_CASCADE
#  if !DISABLE_OUTER_LOOP
    float v_meas = odometry_v();
    float w_meas = odometry_omega();
    v_corr = sta_step(&s_sta_outer_lin, v_cmd, v_meas, dt_s);
    w_corr = sta_step(&s_sta_outer_ang, w_cmd, w_meas, dt_s);
#  endif
#endif

    /* 4. Kinematic split — differential drive */
    float half_base = 0.5f * WHEEL_BASE_M;
    float v_left_target  = v_corr - half_base * w_corr;
    float v_right_target = v_corr + half_base * w_corr;
    s_v_left_target_last  = v_left_target;
    s_v_right_target_last = v_right_target;

    /* 5. Inner per-wheel STAs */
    float v_left_meas  = encoders_velocity_mps(ENC_LEFT);
    float v_right_meas = encoders_velocity_mps(ENC_RIGHT);
    float duty_left  = sta_step(&s_sta_inner_left,  v_left_target,  v_left_meas,  dt_s);
    float duty_right = sta_step(&s_sta_inner_right, v_right_target, v_right_meas, dt_s);

    /* 6. Output write (caution modifier already factored into v_max above; no
     *    second multiplication on duty — that would double-scale.) */
    motors_set_pwm_signed(MOTOR_LEFT,  duty_left);
    motors_set_pwm_signed(MOTOR_RIGHT, duty_right);

    s_duty_left_last  = duty_left;
    s_duty_right_last = duty_right;
}

float control_v_target(void)        { return s_v_target_last; }
float control_omega_target(void)    { return s_omega_target_last; }
float control_duty_left(void)       { return s_duty_left_last; }
float control_duty_right(void)      { return s_duty_right_last; }
float control_v_left_target(void)   { return s_v_left_target_last; }
float control_v_right_target(void)  { return s_v_right_target_last; }
