#include "caution.h"
#include "config.h"
#include "stm32f0xx.h"

/* Per-source level table. Index = ffs(source bit). We support up to 8 sources;
 * caution_source_t bits live in the low 8 bits of an enum. */

#define MAX_SRC 8u

static float s_level[MAX_SRC];   /* default 1.0 (no caution from this source) */

static int src_to_index(caution_source_t src) {
    /* exactly one bit must be set */
    if (src == 0) return -1;
    int i = 0;
    while (((uint32_t)src & 1u) == 0u) { src = (caution_source_t)((uint32_t)src >> 1); i++; }
    return (i < (int)MAX_SRC) ? i : -1;
}

void caution_init(void) {
    for (uint32_t i = 0; i < MAX_SRC; i++) s_level[i] = CAUTION_NORMAL;
}

void caution_set(caution_source_t src, float level) {
    int i = src_to_index(src);
    if (i < 0) return;
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;

    /* Single-word float store is atomic on Cortex-M0 (32-bit aligned). */
    s_level[i] = level;
}

void caution_clear(caution_source_t src) {
    caution_set(src, CAUTION_NORMAL);
}

float caution_modifier(void) {
    float m = CAUTION_NORMAL;
    for (uint32_t i = 0; i < MAX_SRC; i++) {
        if (s_level[i] < m) m = s_level[i];
    }
    return m;
}

uint8_t caution_active_sources(void) {
    uint8_t bits = 0;
    for (uint32_t i = 0; i < MAX_SRC; i++) {
        if (s_level[i] < CAUTION_NORMAL) bits |= (uint8_t)(1u << i);
    }
    return bits;
}
