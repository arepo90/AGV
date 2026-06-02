#include "current_monitor.h"
#include "adc.h"
#include "config.h"
#include "estop.h"
#include "log.h"
#include "types.h"

#define TRIP_CONSECUTIVE   10u   /* must exceed for N ticks (≈100 ms at 100 Hz) */

static uint8_t s_streak[2];

void current_monitor_init(void) {
    s_streak[0] = 0;
    s_streak[1] = 0;
}

void current_monitor_tick(void) {
#if DISABLE_CURRENT_SENSE
    return;
#else
    if (!adc_has_data()) return;

    for (uint32_t side = 0; side < 2; side++) {
        uint16_t ma = adc_motor_current_ma(side);
        if (ma > MOTOR_OVERCURRENT_MA) {
            s_streak[side]++;
            if (s_streak[side] == TRIP_CONSECUTIVE) {
                log_record(LOG_MOD_MOTORS, LOG_SEV_CRITICAL,
                           side ? LOG_CODE_OVERCURRENT_M2 : LOG_CODE_OVERCURRENT_M1,
                           ma);
                estop_assert(ESTOP_SRC_OVERCURRENT);
            }
        } else {
            s_streak[side] = 0;
        }
    }
#endif
}
