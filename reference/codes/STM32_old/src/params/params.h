#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 *  PARAM_UPDATE dispatcher.
 *
 *  Each PKT_PARAM_UPDATE payload is one or more 5-byte tuples:
 *      [u8 param_id][f32 value]    (little-endian f32)
 *
 *  params_apply_payload() walks the payload and applies each pair via
 *  params_apply_one(), which is a switch over the PARAM_* IDs from comms.h.
 *
 *  Returns the number of UNKNOWN/UNAPPLIED entries — caller may NACK if any.
 *
 *  All updates are RAM-only and lost on power cycle. The workstation is the
 *  source of truth for tunables it cares about; it should re-send PARAM_UPDATE
 *  on connect for anything it wants enforced.
 *
 *  (QTR calibration is the exception — it persists to flash via its own
 *  dedicated CMD_QTR_CALIBRATE flow.)
 * =============================================================================
 */

bool     params_apply_one(uint8_t param_id, float value);
uint32_t params_apply_payload(const uint8_t *payload, uint32_t len);

#endif /* PARAMS_H */
