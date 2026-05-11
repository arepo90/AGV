#include "control.h"
#include "caution.h"
#include "config.h"
#include "encoders.h"
#include "estop.h"
#include "motors.h"
#include "nav.h"
#include "odometry.h"
#include "pid.h"

static pid_t s_pid_inner_left, s_pid_inner_right;
static pid_t s_pid_outer_lin,  s_pid_outer_ang;

static float s_v_target_last     = 0.0f;
static float s_omega_target_last = 0.0f;
static float s_duty_left_last    = 0.0f;
static float s_duty_right_last   = 0.0f;

static void reset_all_pids(void) {
    pid_reset(&s_pid_inner_left);
    pid_reset(&s_pid_inner_right);
    pid_reset(&s_pid_outer_lin);
    pid_reset(&s_pid_outer_ang);
    nav_reset();   /* clear navigator-internal PID state too */
}

void control_init(void) {
    /* Inner: per-wheel velocity loop. Setpoint and feedback are wheel m/s.
     * Output is normalised duty in [-1, +1]. */
    pid_init(&s_pid_inner_left,
             PID_INNER_KP_LEFT, PID_INNER_KI_LEFT, PID_INNER_KD_LEFT,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_LIMIT);
    pid_init(&s_pid_inner_right,
             PID_INNER_KP_RIGHT, PID_INNER_KI_RIGHT, PID_INNER_KD_RIGHT,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_LIMIT);

    /* Outer: chassis (linear, angular) loop. Setpoint and feedback are m/s and
     * rad/s. Output is the corrected setpoint, *also* in m/s and rad/s. We
     * clamp at the configured chassis maxima so the inner kinematic split
     * receives a physically achievable target. */
    pid_init(&s_pid_outer_lin,
             PID_OUTER_LIN_KP, PID_OUTER_LIN_KI, PID_OUTER_LIN_KD,
             -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS);
    pid_init(&s_pid_outer_ang,
             PID_OUTER_ANG_KP, PID_OUTER_ANG_KI, PID_OUTER_ANG_KD,
             -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS,
             MAX_ANGULAR_SPEED_RADPS);
}

void control_set_inner_gains_left(float kp, float ki, float kd) {
    pid_set_gains(&s_pid_inner_left, kp, ki, kd);
}
void control_set_inner_gains_right(float kp, float ki, float kd) {
    pid_set_gains(&s_pid_inner_right, kp, ki, kd);
}
void control_set_outer_lin_gains(float kp, float ki, float kd) {
    pid_set_gains(&s_pid_outer_lin, kp, ki, kd);
}
void control_set_outer_ang_gains(float kp, float ki, float kd) {
    pid_set_gains(&s_pid_outer_ang, kp, ki, kd);
}

static float clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void control_tick(float dt_s) {
    /* E-STOP path: zero PWM, reset PIDs. SLEEP is handled by main.c; this
     * keeps controller state clean so motion resumes without windup on clear. */
    if (estop_active()) {
        reset_all_pids();
        motors_set_pwm_signed(MOTOR_LEFT,  0.0f);
        motors_set_pwm_signed(MOTOR_RIGHT, 0.0f);
        s_duty_left_last  = 0.0f;
        s_duty_right_last = 0.0f;
        return;
    }

    /* 1. Navigator target */
    float v_cmd, w_cmd;
    nav_get_target(dt_s, &v_cmd, &w_cmd);

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
    v_corr = pid_step(&s_pid_outer_lin, v_cmd, v_meas, dt_s);
    w_corr = pid_step(&s_pid_outer_ang, w_cmd, w_meas, dt_s);
#  endif
#endif

    /* 4. Kinematic split — differential drive */
    float half_base = 0.5f * WHEEL_BASE_M;
    float v_left_target  = v_corr - half_base * w_corr;
    float v_right_target = v_corr + half_base * w_corr;

    /* 5. Inner per-wheel PIDs */
    float v_left_meas  = encoders_velocity_mps(ENC_LEFT);
    float v_right_meas = encoders_velocity_mps(ENC_RIGHT);
    float duty_left  = pid_step(&s_pid_inner_left,  v_left_target,  v_left_meas,  dt_s);
    float duty_right = pid_step(&s_pid_inner_right, v_right_target, v_right_meas, dt_s);

    /* 6. Output write (caution modifier already factored into v_max above; no
     *    second multiplication on duty — that would double-scale.) */
    motors_set_pwm_signed(MOTOR_LEFT,  duty_left);
    motors_set_pwm_signed(MOTOR_RIGHT, duty_right);

    s_duty_left_last  = duty_left;
    s_duty_right_last = duty_right;
}

float control_v_target(void)     { return s_v_target_last; }
float control_omega_target(void) { return s_omega_target_last; }
float control_duty_left(void)    { return s_duty_left_last; }
float control_duty_right(void)   { return s_duty_right_last; }
