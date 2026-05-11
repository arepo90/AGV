#include "config.h"
#include "types.h"
#include "system.h"
#include "comms.h"
#include "log.h"
#include "motors.h"
#include "caution.h"
#include "estop.h"
#include "state.h"
#include "heartbeat.h"
#include "proximity.h"
#include "encoders.h"
#include "odometry.h"
#include "control.h"
#include "nav.h"
#include "nav_line.h"
#include "cmd.h"
#include "adc.h"
#include "current_monitor.h"
#include "hx711.h"
#include "cargo_monitor.h"
#include "imu.h"

#include <string.h>

/* =============================================================================
 *  AGV main entry — sensors + safety phase complete.
 *
 *  Init order (sequence matters):
 *    1.  system_init       — PLL → 48 MHz, SysTick, capture reset cause
 *    2.  log_init          — fault log buffer ready before anything that may log
 *    3.  motors_init       — SLEEP pins LOW *first*, before anything else
 *    4.  encoders_init     — TIM2/TIM3 quadrature mode
 *    5.  comms_init        — UART/DMA up so we can talk to ESP32
 *    6.  adc_init          — analog pins, DMA, ADC1
 *    7.  hx711_init        — load cell GPIO
 *    8.  imu_init          — I2C1 + BNO055 fusion mode (may fail; we continue)
 *    9.  caution_init      — all sources at NORMAL
 *   10.  estop_init        — bitmask cleared
 *   11.  state_init        — mode = SUPERVISED, function = STANDBY
 *   12.  heartbeat_init    — last-seen = now (one timeout window of grace)
 *   13.  nav_init          — navigators ready (REMOTE_CONTROL target = 0,0)
 *   14.  odometry_init     — pose at origin
 *   15.  control_init      — PIDs constructed at config-defined gains
 *   16.  current_monitor_init / cargo_monitor_init — streak counters reset
 *   17.  proximity_init    — EXTI live; ISR may immediately assert E-STOP
 *   18.  system_iwdg_init  — only after all init completes within timeout
 * =============================================================================
 */

static void boot_record(void) {
    switch (system_reset_cause()) {
    case RESET_CAUSE_WATCHDOG:
        log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR,
                   LOG_CODE_WATCHDOG_RESET_DETECTED, 0);
        break;
    case RESET_CAUSE_LOW_POWER:
        log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR,
                   LOG_CODE_BROWNOUT_RESET_DETECTED, 0);
        break;
    case RESET_CAUSE_SOFTWARE:
        log_record(LOG_MOD_SYSTEM, LOG_SEV_INFO, LOG_CODE_SOFT_RESET, 0);
        break;
    default:
        break;
    }
    log_record(LOG_MOD_SYSTEM, LOG_SEV_INFO, LOG_CODE_BOOT,
               (uint32_t)system_reset_cause());
}

static bool every_ms(uint32_t *last, uint32_t now, uint32_t period) {
    if ((now - *last) >= period) {
        *last = now;
        return true;
    }
    return false;
}

/* ---- Telemetry packing -------------------------------------------------- */

#if !DISABLE_TELEMETRY
/* Layout (111 bytes — see architecture.md / wire format) */
static void send_telemetry(uint32_t now_ms) {
    uint8_t buf[120];
    uint32_t off = 0;

    #define PUT_U32(x) do { uint32_t _v = (uint32_t)(x); \
        buf[off++]=(uint8_t)_v; buf[off++]=(uint8_t)(_v>>8); \
        buf[off++]=(uint8_t)(_v>>16); buf[off++]=(uint8_t)(_v>>24); } while (0)
    #define PUT_U16(x) do { uint16_t _v = (uint16_t)(x); \
        buf[off++]=(uint8_t)_v; buf[off++]=(uint8_t)(_v>>8); } while (0)
    #define PUT_U8(x)  do { buf[off++] = (uint8_t)(x); } while (0)
    #define PUT_F32(x) do { float _f = (x); uint32_t _u; \
        memcpy(&_u, &_f, 4); PUT_U32(_u); } while (0)

    PUT_U32(now_ms);
    PUT_U8(state_mode());
    PUT_U8(state_function());
    PUT_U8(estop_sources());
    PUT_U8(caution_active_sources());
    PUT_F32(caution_modifier());

    uint32_t pending = log_pending_count();
    uint32_t dropped = log_dropped_count();
    PUT_U16((pending > 0xFFFFu) ? 0xFFFFu : pending);
    PUT_U16((dropped > 0xFFFFu) ? 0xFFFFu : dropped);

    PUT_U32(encoders_count(ENC_LEFT));
    PUT_U32(encoders_count(ENC_RIGHT));
    PUT_F32(encoders_velocity_mps(ENC_LEFT));
    PUT_F32(encoders_velocity_mps(ENC_RIGHT));

    PUT_F32(odometry_x());
    PUT_F32(odometry_y());
    PUT_F32(odometry_theta());
    PUT_F32(odometry_v());
    PUT_F32(odometry_omega());

    PUT_F32(control_v_target());
    PUT_F32(control_omega_target());

    /* ---- Sensors (Phase 5) ---- */
    PUT_U16(adc_motor_current_ma(0));
    PUT_U16(adc_motor_current_ma(1));
    for (uint32_t i = 0; i < ADC_IDX_QTR_COUNT; i++) PUT_U16(adc_qtr(i));
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) PUT_F32(hx711_kg(c));
    PUT_F32(imu_yaw_deg());
    PUT_F32(imu_pitch_deg());
    PUT_F32(imu_roll_deg());

    /* IMU calib packed: [sys|gyro|accel|mag] each 2 bits. */
    uint8_t calib = (uint8_t)((imu_calib_sys()   << 6) |
                              (imu_calib_gyro()  << 4) |
                              (imu_calib_accel() << 2) |
                               imu_calib_mag());
    PUT_U8(calib);

    PUT_U16(proximity_obstructed());   /* u16: bits 6-9 = PC6-PC9 sensors */

    uint8_t flags = 0;
    if (adc_has_data())             flags |= 0x01;
    if (hx711_has_data())           flags |= 0x02;
    if (imu_has_data())             flags |= 0x04;
    if (hx711_tare_in_progress())   flags |= 0x08;
    PUT_U8(flags);                                   /* offset 111 */

    /* Motor output: signed duty [-1,+1] and direction derived from sign. */
    PUT_F32(control_duty_left());                    /* offset 112 */
    PUT_F32(control_duty_right());                   /* offset 116 */

    #undef PUT_U32
    #undef PUT_U16
    #undef PUT_U8
    #undef PUT_F32

    comms_send(PKT_TELEMETRY, buf, (uint8_t)off);
}
#endif

/* ---- Log forwarding ---------------------------------------------------- */

#if !DISABLE_LOG_FORWARDING
static void drain_logs(void) {
    log_entry_t e;
    for (uint32_t i = 0; i < 2u && log_pop(&e); i++) {
        uint8_t buf[12];
        buf[0]  = (uint8_t)(e.timestamp_ms);
        buf[1]  = (uint8_t)(e.timestamp_ms >> 8);
        buf[2]  = (uint8_t)(e.timestamp_ms >> 16);
        buf[3]  = (uint8_t)(e.timestamp_ms >> 24);
        buf[4]  = (uint8_t)(e.code);
        buf[5]  = (uint8_t)((uint16_t)e.code >> 8);
        buf[6]  = (uint8_t)e.severity;
        buf[7]  = (uint8_t)e.module;
        buf[8]  = (uint8_t)(e.data);
        buf[9]  = (uint8_t)(e.data >> 8);
        buf[10] = (uint8_t)(e.data >> 16);
        buf[11] = (uint8_t)(e.data >> 24);
        if (!comms_send(PKT_LOG, buf, sizeof buf)) break;
    }
}
#endif

/* ---- main --------------------------------------------------------------- */

int main(void) {
    system_init();
    log_init();
    motors_init();           /* SLEEP LOW at init: motors stay safe until first run */
    encoders_init();
    comms_init();
    adc_init();
#if !DISABLE_LOAD_CELLS
    hx711_init();
#endif
#if !DISABLE_IMU
    imu_init();
#endif
    caution_init();
    estop_init();
    state_init();
    heartbeat_init();
    nav_init();
    /* Load QTR calibration from flash (if a valid record exists) AFTER
     * nav_init has set defaults, so a missing/corrupt record falls back
     * gracefully to the compile-time defaults. */
    nav_line_load_calibration_from_flash();
    odometry_init();
    control_init();
    current_monitor_init();
    cargo_monitor_init();
    proximity_init();        /* may immediately assert E-STOP */

    boot_record();

    system_iwdg_init();

    uint32_t last_telem    = 0;
    uint32_t last_control  = system_now_ms();

    const uint32_t control_period_ms = 1000u / CONTROL_LOOP_HZ;

    while (1) {
        system_iwdg_pet();

        uint32_t now = system_now_ms();

        /* 1. Drain UART RX, parse frames */
        comms_poll();

        /* 2. Dispatch any decoded packets */
        packet_t pkt;
        while (comms_recv(&pkt)) {
            cmd_dispatch(&pkt);
        }

        /* 3. Heartbeat watcher (may transition mode and assert E-STOP) */
        heartbeat_tick();

        /* 4. Sensors (each rate-limits internally) */
        adc_tick(now);
#if !DISABLE_LOAD_CELLS
        hx711_tick(now);
#endif
#if !DISABLE_IMU
        imu_tick(now);
#endif

        /* 4a. QTR calibration sweep (only when active). Tracking happens at
         *     ADC scan rate; cheap when not active. */
        if (nav_line_cal_active()) nav_line_cal_track();

        /* 5. Sensor-derived monitors. These may assert/clear E-STOP and
         *    caution sources, which are then applied in step 6. */
        current_monitor_tick();
        cargo_monitor_tick();

        /* 6. Apply SLEEP pin. Sleep (Lo) when E-STOP is active OR in STANDBY
         *    (PWM braking handles deceleration; SLEEP for true idle/fault state).
         *    Runs BEFORE control so SLEEP is correct when control writes PWM. */
#if !DISABLE_ESTOP
        {
            bool should_run = !estop_active() && (state_function() != FUNC_STANDBY);
            if (motors_enabled() != should_run) motors_set_enabled(should_run);
        }
#endif

        /* 7. Cascade control loop at CONTROL_LOOP_HZ. */
        if ((now - last_control) >= control_period_ms) {
            float dt = (float)(now - last_control) * 0.001f;
            last_control = now;
            if (dt > 0.05f) dt = 0.05f;

            encoders_tick(dt);
            odometry_tick(dt);
            control_tick(dt);
        }

        /* 8. Forward queued log entries */
#if !DISABLE_LOG_FORWARDING
        drain_logs();
#endif

        /* 9. Periodic telemetry */
#if !DISABLE_TELEMETRY
        bool moving = state_function() != FUNC_STANDBY;
        uint32_t period_ms = 1000u / (moving ? TELEMETRY_RATE_MOVING_HZ
                                             : TELEMETRY_RATE_STANDBY_HZ);
        if (every_ms(&last_telem, now, period_ms)) {
            send_telemetry(now);
        }
#endif
    }
}
