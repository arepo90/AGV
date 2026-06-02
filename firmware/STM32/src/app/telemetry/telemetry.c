#include "telemetry.h"
#include "analog.h"
#include "battery.h"
#include "config.h"
#include "control.h"
#include "encoders.h"
#include "imu.h"
#include "loadcells.h"
#include "nav_line.h"
#include "odometry.h"
#include "proto.h"
#include "proximity.h"
#include "safety.h"
#include "tof.h"
#include "types.h"
#include <string.h>

/* ---- little-endian packing helpers -------------------------------------- */
#define PUT_U8(x)  do { buf[off++] = (uint8_t)(x); } while (0)
#define PUT_U16(x) do { uint16_t _v = (uint16_t)(x); \
    buf[off++] = (uint8_t)_v; buf[off++] = (uint8_t)(_v >> 8); } while (0)
#define PUT_U32(x) do { uint32_t _v = (uint32_t)(x); \
    buf[off++] = (uint8_t)_v;        buf[off++] = (uint8_t)(_v >> 8); \
    buf[off++] = (uint8_t)(_v >> 16); buf[off++] = (uint8_t)(_v >> 24); } while (0)
#define PUT_F32(x) do { float _f = (x); uint32_t _u; memcpy(&_u, &_f, 4); PUT_U32(_u); } while (0)

static uint32_t s_last_core = 0, s_last_drive = 0, s_last_sensors = 0, s_last_qtr = 0;
static uint8_t  s_led_mode = LED_MODE_DEFAULT;

void telemetry_init(void) {
    s_last_core = s_last_drive = s_last_sensors = s_last_qtr = 0;
    s_led_mode = LED_MODE_DEFAULT;
}

void telemetry_set_led_mode(uint8_t mode) { s_led_mode = mode; }

static uint8_t status_flags(void) {
    uint8_t f = 0;
    if (analog_has_data())            f |= 0x01u;
    if (loadcells_has_data())         f |= 0x02u;
    if (imu_has_data())               f |= 0x04u;
    if (loadcells_tare_in_progress()) f |= 0x08u;
    return f;
}

static void send_core(uint32_t now_ms) {
    uint8_t buf[44];
    uint32_t off = 0;
    PUT_U32(now_ms);
    PUT_U8(safety_mode());
    PUT_U8(safety_function());
    PUT_U16(safety_estop_sources());    /* 16-bit: 9 sources incl. TOF + battery */
    PUT_U16(safety_caution_sources());
    PUT_F32(safety_caution_modifier());
    PUT_U8(status_flags());
    PUT_F32(odometry_v());
    PUT_F32(odometry_omega());
    PUT_F32(odometry_x());
    PUT_F32(odometry_y());
    PUT_F32(odometry_theta());
    PUT_U16(analog_current_ma(0));
    PUT_U16(analog_current_ma(1));
    PUT_U16(proximity_obstructed());
    PUT_U8(s_led_mode);                 /* offset 41 — drives the ESP32 ring animation */
    proto_send(PKT_TLM_CORE, buf, (uint8_t)off);
}

static void send_drive(void) {
    uint8_t buf[32];
    uint32_t off = 0;
    PUT_F32(control_v_left_target());
    PUT_F32(control_v_right_target());
    PUT_F32(encoders_velocity_mps(SIDE_LEFT));
    PUT_F32(encoders_velocity_mps(SIDE_RIGHT));
    PUT_F32(control_duty_left());
    PUT_F32(control_duty_right());
    PUT_U32((uint32_t)encoders_count(SIDE_LEFT));
    PUT_U32((uint32_t)encoders_count(SIDE_RIGHT));
    proto_send(PKT_TLM_DRIVE, buf, (uint8_t)off);
}

static void send_sensors(void) {
    uint8_t buf[44];
    uint32_t off = 0;
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) PUT_F32(loadcells_kg(c));
    PUT_F32(imu_yaw_deg());
    PUT_F32(imu_pitch_deg());
    PUT_F32(imu_roll_deg());
    uint8_t calib = (uint8_t)((imu_calib_sys()   << 6) |
                              (imu_calib_gyro()  << 4) |
                              (imu_calib_accel() << 2) |
                               imu_calib_mag());
    PUT_U8(calib);
    for (uint32_t i = 0; i < TOF_NUM_SENSORS; i++) PUT_U16(tof_distance_mm(i));  /* mm, FRLR */
    PUT_U16(battery_mv(BATTERY_3S));
    PUT_U16(battery_mv(BATTERY_6S));
    proto_send(PKT_TLM_SENSORS, buf, (uint8_t)off);
}

static void send_qtr(void) {
    uint8_t buf[20];
    uint32_t off = 0;
    for (uint32_t i = 0; i < ANALOG_QTR_COUNT; i++) PUT_U16(analog_qtr(i));
    PUT_F32(nav_line_position());
    proto_send(PKT_TLM_QTR, buf, (uint8_t)off);
}

static bool due(uint32_t *last, uint32_t now, uint32_t hz) {
    if ((now - *last) >= (1000u / hz)) { *last = now; return true; }
    return false;
}

void telemetry_tick(uint32_t now_ms) {
#if DISABLE_TELEMETRY
    (void)now_ms;
    return;
#else
    bool moving = safety_function_is_navigating(safety_function());

    if (due(&s_last_core, now_ms, moving ? TLM_CORE_HZ_MOVING : TLM_CORE_HZ_IDLE))
        send_core(now_ms);
    if (due(&s_last_drive, now_ms, TLM_DRIVE_HZ))
        send_drive();
    if (due(&s_last_sensors, now_ms, TLM_SENSORS_HZ))
        send_sensors();

    /* QTR only matters while following a line or calibrating. */
    if ((safety_function() == FUNC_LINE_FOLLOW || nav_line_cal_active()) &&
        due(&s_last_qtr, now_ms, CONTROL_LOOP_HZ))
        send_qtr();
#endif
}

#undef PUT_U8
#undef PUT_U16
#undef PUT_U32
#undef PUT_F32
