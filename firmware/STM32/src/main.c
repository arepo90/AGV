#include "config.h"
#include "types.h"

#include "mcu.h"

#include "analog.h"
#include "battery.h"
#include "control.h"
#include "encoders.h"
#include "loadcells.h"
#include "log.h"
#include "motors.h"
#include "nav.h"
#include "odometry.h"
#include "lidar.h"
#include "proto.h"
#include "proximity.h"
#include "ramp.h"
#include "safety.h"
#include "telemetry.h"

#if I2C_SCAN
#include "i2cscan.h"
#endif

/* =============================================================================
 *  AGV main entry.
 *
 *  Init order matters:
 *    mcu      → clock/SysTick/reset-cause first (everything reads mcu_now_ms)
 *    log      → buffer ready before anything that may log
 *    motors   → SLEEP LOW first, so the driver stays safe until commanded
 *    encoders → quadrature timers
 *    proto    → UART/DMA up to talk to the ESP32
 *    analog   → ADC + DMA (current + QTR)
 *    loadcells → sensors (each may be DISABLE_*'d)
 *    safety   → mode/estop/caution/heartbeat/monitors
 *    nav      → navigators
 *    odometry/ramp/control → control chain (control reset calls ramp_reset)
 *    proximity → EXTI live; may immediately assert E-STOP
 *    telemetry → stream scheduler
 *    boot log, then IWDG last (all init must finish inside the window)
 * =============================================================================
 */

static void boot_record(void) {
    switch (mcu_reset_cause()) {
    case RESET_CAUSE_WATCHDOG:
        log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR, LOG_CODE_WATCHDOG_RESET_DETECTED, 0);
        break;
    case RESET_CAUSE_LOW_POWER:
        log_record(LOG_MOD_SYSTEM, LOG_SEV_ERROR, LOG_CODE_BROWNOUT_RESET_DETECTED, 0);
        break;
    case RESET_CAUSE_SOFTWARE:
        log_record(LOG_MOD_SYSTEM, LOG_SEV_INFO, LOG_CODE_SOFT_RESET, 0);
        break;
    default:
        break;
    }
    log_record(LOG_MOD_SYSTEM, LOG_SEV_INFO, LOG_CODE_BOOT, (uint32_t)mcu_reset_cause());
}

#if !DISABLE_LOG_FORWARDING
/* Forward a couple of queued log entries per loop as PKT_LOG (fire-and-forget). */
static void forward_logs(void) {
    log_entry_t e;
    for (uint32_t i = 0; i < 2u && log_pop(&e); i++) {
        uint8_t buf[12];
        buf[0]  = (uint8_t)(e.timestamp_ms);
        buf[1]  = (uint8_t)(e.timestamp_ms >> 8);
        buf[2]  = (uint8_t)(e.timestamp_ms >> 16);
        buf[3]  = (uint8_t)(e.timestamp_ms >> 24);
        buf[4]  = (uint8_t)((uint16_t)e.code);
        buf[5]  = (uint8_t)((uint16_t)e.code >> 8);
        buf[6]  = (uint8_t)e.severity;
        buf[7]  = (uint8_t)e.module;
        buf[8]  = (uint8_t)(e.data);
        buf[9]  = (uint8_t)(e.data >> 8);
        buf[10] = (uint8_t)(e.data >> 16);
        buf[11] = (uint8_t)(e.data >> 24);
        if (!proto_send(PKT_LOG, buf, sizeof buf)) break;   /* queue full; retry next loop */
    }
}
#endif

int main(void) {
    mcu_init();
#if I2C_SCAN
    i2cscan_run();     /* bench diagnostic: scans the bus over UART, never returns */
#endif
    log_init();
    motors_init();
    encoders_init();
    proto_init();
    analog_init();
#if !DISABLE_LOAD_CELLS
    loadcells_init();
#endif
#if !DISABLE_BATTERY
    battery_init();    /* INA219 (3S bus voltage) */
#endif
#if !DISABLE_LIDAR
    lidar_init();      /* receive-side buffer for Jetson-pushed LaserScan segments */
#endif
    safety_init();
    nav_init();
    odometry_init();
    ramp_init();
    control_init();
    proximity_init();
    telemetry_init();

    boot_record();
    mcu_iwdg_init();

    const uint32_t control_period_ms = 1000u / CONTROL_LOOP_HZ;
    uint32_t last_control = mcu_now_ms();

    while (1) {
        mcu_iwdg_pet();
        uint32_t now = mcu_now_ms();

        /* 1. Inbound packets (drained + dispatched inline). */
        proto_poll();

        /* 2. Heartbeat watcher (may degrade mode / assert E-STOP). */
        safety_heartbeat_tick();

        /* 3. Sensors (each rate-limits internally). */
        analog_tick(now);
#if !DISABLE_LOAD_CELLS
        loadcells_tick(now, safety_function_is_navigating(safety_function()));
#endif
#if !DISABLE_BATTERY
        battery_tick(now);
#endif

        /* 4. SLEEP pin: run only when not E-STOPped and not idle. Applied before
         *    control so SLEEP is correct when control writes PWM. */
        {
#if !DISABLE_ESTOP
            bool should_run = !safety_estop_active() && (safety_function() != FUNC_STANDBY);
#else
            bool should_run = (safety_function() != FUNC_STANDBY);
#endif
            if (motors_enabled() != should_run) motors_set_enabled(should_run);
        }

        /* 5. Control cadence: encoders → odometry → monitors → control. */
        if ((now - last_control) >= control_period_ms) {
            float dt = (float)(now - last_control) * 0.001f;
            last_control = now;
            if (dt > 0.05f) dt = 0.05f;

            encoders_tick(dt);
            odometry_tick(dt);
            safety_monitors_tick();
            control_tick(dt);
        }

        /* 6. Outbound: forward logs, then due telemetry streams. */
#if !DISABLE_LOG_FORWARDING
        forward_logs();
#endif
        telemetry_tick(now);
    }
}
