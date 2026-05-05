#include "cargo_monitor.h"
#include "caution.h"
#include "config.h"
#include "estop.h"
#include "hx711.h"
#include "log.h"
#include "types.h"
#include <math.h>

/* Tiny floor below which imbalance is ignored — at <2 kg total the cells are
 * just reading noise. Imbalance fraction would be huge but meaningless. */
#define IMBALANCE_FLOOR_KG    2.0f

void cargo_monitor_init(void) {}

void cargo_monitor_tick(void) {
#if DISABLE_LOAD_CELLS
    return;
#else
    if (!hx711_has_data()) return;

    float corner[4];
    float total = 0.0f;
    for (uint32_t c = 0; c < 4; c++) {
        corner[c] = hx711_kg(c);
        total += corner[c];
    }

    /* ---- Total-weight thresholds ---------------------------------------- */
    if (total >= WEIGHT_TOTAL_ESTOP_KG) {
        estop_assert(ESTOP_SRC_CARGO_OVERLOAD);
        caution_set(CAUTION_SRC_LOAD_OVERWEIGHT, CAUTION_LEVEL_CRITICAL);
    } else if (total >= WEIGHT_TOTAL_CAUTION_KG) {
        estop_clear_autoclearing(ESTOP_SRC_CARGO_OVERLOAD);
        caution_set(CAUTION_SRC_LOAD_OVERWEIGHT, CAUTION_LEVEL_CAUTION);
    } else {
        estop_clear_autoclearing(ESTOP_SRC_CARGO_OVERLOAD);
        caution_clear(CAUTION_SRC_LOAD_OVERWEIGHT);
    }

    /* ---- Imbalance ------------------------------------------------------ */
    if (total < IMBALANCE_FLOOR_KG) {
        estop_clear_autoclearing(ESTOP_SRC_CARGO_IMBALANCE);
        caution_clear(CAUTION_SRC_LOAD_IMBALANCE);
        return;
    }

    float mean = total * 0.25f;
    float max_dev = 0.0f;
    for (uint32_t c = 0; c < 4; c++) {
        float d = fabsf(corner[c] - mean);
        if (d > max_dev) max_dev = d;
    }
    /* Normalize by the mean: an imbalance of 25 kg on a 100 kg load is
     * relatively less serious than the same 25 kg on a 30 kg load. */
    float frac = max_dev / mean;

    if (frac >= WEIGHT_IMBALANCE_ESTOP) {
        estop_assert(ESTOP_SRC_CARGO_IMBALANCE);
        caution_set(CAUTION_SRC_LOAD_IMBALANCE, CAUTION_LEVEL_CRITICAL);
    } else if (frac >= WEIGHT_IMBALANCE_CAUTION) {
        estop_clear_autoclearing(ESTOP_SRC_CARGO_IMBALANCE);
        caution_set(CAUTION_SRC_LOAD_IMBALANCE, CAUTION_LEVEL_CAUTION);
    } else {
        estop_clear_autoclearing(ESTOP_SRC_CARGO_IMBALANCE);
        caution_clear(CAUTION_SRC_LOAD_IMBALANCE);
    }
#endif
}
