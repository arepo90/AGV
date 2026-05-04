#ifndef CARGO_MONITOR_H
#define CARGO_MONITOR_H

/* =============================================================================
 *  Cargo monitor — aggregates the four corner load cells and feeds the caution
 *  + E-STOP layer.
 *
 *  Behaviour (thresholds in config.h):
 *    Total weight:
 *      ≥ WEIGHT_TOTAL_ESTOP_KG       → ESTOP_SRC_CARGO_OVERLOAD (auto-clear)
 *      ≥ WEIGHT_TOTAL_CAUTION_KG     → CAUTION_SRC_LOAD_OVERWEIGHT (CAUTION)
 *      below                         → both clear
 *
 *    Imbalance (max-corner deviation from mean, normalized):
 *      ≥ WEIGHT_IMBALANCE_ESTOP      → ESTOP_SRC_CARGO_IMBALANCE (auto-clear)
 *      ≥ WEIGHT_IMBALANCE_CAUTION    → CAUTION_SRC_LOAD_IMBALANCE (CRITICAL level)
 *      below                         → both clear
 *
 *  All transitions are logged. Imbalance is only meaningful when there's
 *  actual load on the cells, so we skip the check below a small total weight
 *  to avoid noise-induced trips.
 * =============================================================================
 */

void cargo_monitor_init(void);
void cargo_monitor_tick(void);

#endif /* CARGO_MONITOR_H */
