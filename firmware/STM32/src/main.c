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
#include "cmd.h"

#include <string.h>

/* =============================================================================
 *  AGV main entry — safety / state phase.
 *
 *  Init order (the order matters):
 *    1. system_init       — PLL → 48 MHz, SysTick, capture reset cause
 *    2. log_init          — fault log buffer ready before anything that may log
 *    3. motors_init       — SLEEP pins LOW *first*, so driver is asleep before
 *                           any other I/O is configured. Bench safety.
 *    4. comms_init        — UART/DMA up so we can talk to ESP32
 *    5. caution_init      — all sources at NORMAL
 *    6. estop_init        — bitmask cleared
 *    7. state_init        — mode = SUPERVISED, function = STANDBY
 *    8. heartbeat_init    — last-seen = now (one timeout window of grace at boot)
 *    9. proximity_init    — EXTI live; ISR may immediately assert E-STOP if
 *                           an obstacle is in front (this is the desired
 *                           behaviour, hence why motors_init came first)
 *   10. system_iwdg_init  — only after init can finish before timeout
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
static void send_telemetry(uint32_t now_ms) {
    uint8_t buf[16];
    /* Layout (little-endian, packed):
     *  [0..3]  uint32 timestamp_ms
     *  [4]     uint8  mode
     *  [5]     uint8  function
     *  [6]     uint8  estop_sources bitmask
     *  [7]     uint8  caution_sources bitmask
     *  [8..11] float  caution_modifier
     *  [12,13] uint16 log_pending
     *  [14,15] uint16 log_dropped (clamped) */
    float cm = caution_modifier();
    uint16_t pending = (uint16_t)(log_pending_count() & 0xFFFFu);
    uint32_t dropped32 = log_dropped_count();
    uint16_t dropped = (dropped32 > 0xFFFFu) ? 0xFFFFu : (uint16_t)dropped32;

    buf[0]  = (uint8_t)(now_ms);
    buf[1]  = (uint8_t)(now_ms >> 8);
    buf[2]  = (uint8_t)(now_ms >> 16);
    buf[3]  = (uint8_t)(now_ms >> 24);
    buf[4]  = (uint8_t)state_mode();
    buf[5]  = (uint8_t)state_function();
    buf[6]  = estop_sources();
    buf[7]  = caution_active_sources();
    memcpy(&buf[8], &cm, sizeof cm);
    buf[12] = (uint8_t)(pending);
    buf[13] = (uint8_t)(pending >> 8);
    buf[14] = (uint8_t)(dropped);
    buf[15] = (uint8_t)(dropped >> 8);

    comms_send(PKT_TELEMETRY, buf, sizeof buf);
}
#endif

/* ---- Log forwarding ---------------------------------------------------- */

#if !DISABLE_LOG_FORWARDING
static void drain_logs(void) {
    log_entry_t e;
    /* Cap drain per iteration so a flood of log entries can't starve the
     * 200 Hz control work coming in the next phase. */
    for (uint32_t i = 0; i < 8u && log_pop(&e); i++) {
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
        if (!comms_send(PKT_LOG, buf, sizeof buf)) break;  /* TX queue full */
    }
}
#endif

/* ---- main --------------------------------------------------------------- */

int main(void) {
    system_init();
    log_init();
    motors_init();           /* SLEEP LOW: motors safe before anything else */
    comms_init();
    caution_init();
    estop_init();
    state_init();
    heartbeat_init();
    proximity_init();        /* may assert E-STOP if an obstacle is present */

    boot_record();

    system_iwdg_init();

    uint32_t last_telem = 0;

    while (1) {
        system_iwdg_pet();

        /* 1. Drain UART RX, parse frames */
        comms_poll();

        /* 2. Dispatch any decoded packets */
        packet_t pkt;
        while (comms_recv(&pkt)) {
            cmd_dispatch(&pkt);
        }

        /* 3. Heartbeat watcher (may transition mode and assert E-STOP) */
        heartbeat_tick();

        /* 4. Apply E-STOP arbitration to motor SLEEP pins. This runs BEFORE
         *    any motor command would be written in the drivetrain phase. */
#if !DISABLE_ESTOP
        estop_apply();
#endif

        /* 5. Forward queued log entries */
#if !DISABLE_LOG_FORWARDING
        drain_logs();
#endif

        /* 6. Periodic telemetry */
#if !DISABLE_TELEMETRY
        uint32_t now = system_now_ms();
        bool moving = state_function() != FUNC_STANDBY;
        uint32_t period_ms = 1000u / (moving ? TELEMETRY_RATE_MOVING_HZ
                                             : TELEMETRY_RATE_STANDBY_HZ);
        if (every_ms(&last_telem, now, period_ms)) {
            send_telemetry(now);
        }
#endif
    }
}
