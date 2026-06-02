#include "control.h"
#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "nav.h"
#include "pid.h"
#include "ramp.h"
#include "safety.h"
#include "types.h"

static pid_t s_pi_left, s_pi_right;
static float s_kff_left  = WHEEL_KFF_LEFT;
static float s_kff_right = WHEEL_KFF_RIGHT;

/* Telemetry snapshots. */
static float s_v_target = 0.0f, s_w_target = 0.0f;
static float s_vl_target = 0.0f, s_vr_target = 0.0f;
static float s_duty_left = 0.0f, s_duty_right = 0.0f;

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void control_init(void) {
    pid_init(&s_pi_left,  WHEEL_KP_LEFT,  WHEEL_KI_LEFT,  0.0f, -1.0f, 1.0f, WHEEL_I_LIMIT);
    pid_init(&s_pi_right, WHEEL_KP_RIGHT, WHEEL_KI_RIGHT, 0.0f, -1.0f, 1.0f, WHEEL_I_LIMIT);
    s_kff_left  = WHEEL_KFF_LEFT;
    s_kff_right = WHEEL_KFF_RIGHT;
}

void control_set_pi_left(float kp, float ki)  { pid_set_gains(&s_pi_left,  kp, ki, 0.0f); }
void control_set_pi_right(float kp, float ki) { pid_set_gains(&s_pi_right, kp, ki, 0.0f); }
void control_set_kff_left(float kff)  { s_kff_left  = kff; }
void control_set_kff_right(float kff) { s_kff_right = kff; }

void control_tick(float dt_s) {
    /* E-STOP: zero PWM and clear controller/ramp state so motion resumes
     * without windup on clear. SLEEP is handled by main.c. */
    if (safety_estop_active()) {
        pid_reset(&s_pi_left);
        pid_reset(&s_pi_right);
        nav_reset();
        ramp_reset();
        motors_set_signed(SIDE_LEFT,  0.0f);
        motors_set_signed(SIDE_RIGHT, 0.0f);
        s_v_target = s_w_target = 0.0f;
        s_vl_target = s_vr_target = 0.0f;
        s_duty_left = s_duty_right = 0.0f;
        return;
    }

    /* 1. Navigator control law. */
    float v, w;
    nav_get_target(dt_s, &v, &w);

    /* 2. Motion profile (caution-scaled accel applied inside). */
    ramp_step(dt_s, v, w, &v, &w);

    /* 3. Caution-scaled chassis clamp — the single point where the modifier
     *    bounds speed (no second multiply on duty). */
    float k = safety_caution_modifier();
    float v_max = MAX_LINEAR_SPEED_MPS  * k;
    float w_max = MAX_ANGULAR_SPEED_RADPS * k;
    v = clampf(v, -v_max, v_max);
    w = clampf(w, -w_max, w_max);
    s_v_target = v;
    s_w_target = w;

    /* 4. Algebraic differential-drive split. */
    float half = 0.5f * (float)WHEEL_BASE_M;
    float vl = v - half * w;
    float vr = v + half * w;
    s_vl_target = vl;
    s_vr_target = vr;

    /* 5. Per-wheel PI + velocity feedforward → signed duty. */
    float vl_meas = encoders_velocity_mps(SIDE_LEFT);
    float vr_meas = encoders_velocity_mps(SIDE_RIGHT);
    float dl = pid_step(&s_pi_left,  vl, vl_meas, s_kff_left  * vl, dt_s);
    float dr = pid_step(&s_pi_right, vr, vr_meas, s_kff_right * vr, dt_s);

    motors_set_signed(SIDE_LEFT,  dl);
    motors_set_signed(SIDE_RIGHT, dr);
    s_duty_left  = dl;
    s_duty_right = dr;
}

float control_v_target(void)       { return s_v_target; }
float control_omega_target(void)   { return s_w_target; }
float control_v_left_target(void)  { return s_vl_target; }
float control_v_right_target(void) { return s_vr_target; }
float control_duty_left(void)      { return s_duty_left; }
float control_duty_right(void)     { return s_duty_right; }
