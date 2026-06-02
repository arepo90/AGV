#include "hx711.h"
#include "config.h"
#include "log.h"
#include "state.h"
#include "system.h"
#include "types.h"
#include "stm32f0xx.h"

/* Pins */
#define SCK_PIN_BIT     10u
#define DOUT_PIN_BIT(c) ((c) + 12u)         /* corners 0..3 → PB12..PB15 */

#define DOUT_MASK       ( (1u << 12) | (1u << 13) | (1u << 14) | (1u << 15) )

#define TARE_SAMPLE_COUNT  20u

static int32_t  s_raw[HX711_NUM_CORNERS];
static int32_t  s_offset[HX711_NUM_CORNERS];
static float    s_scale[HX711_NUM_CORNERS];
static bool     s_have_data = false;
static uint32_t s_last_read_ms = 0;

/* Tare state machine */
static bool     s_tare_active = false;
static uint8_t  s_tare_samples_left = 0;
static int64_t  s_tare_accum[HX711_NUM_CORNERS];

/* ---- Bit-bang helpers --------------------------------------------------- */

/* SCK high pulse must stay above 0.2 µs (datasheet min) and below 60 µs (else
 * the HX711 enters powerdown). A few volatile loop iterations at 48 MHz lands
 * around 100 ns — comfortably inside both bounds. */
static inline void short_delay(void) {
    for (volatile int i = 0; i < 4; i++) { }
}

static inline void sck_high(void) { GPIOB->BSRR = 1u << SCK_PIN_BIT;       }
static inline void sck_low(void)  { GPIOB->BSRR = 1u << (SCK_PIN_BIT + 16); }
static inline uint16_t dout_all(void) { return (uint16_t)(GPIOB->IDR & DOUT_MASK); }

static bool dout_all_ready(void) {
    /* Each DOUT is LOW when its chip has data ready. We require ALL four. */
    return (dout_all() == 0);
}

/* Read one 24-bit word from each corner in parallel. Caller has verified DOUT
 * is low for all corners. Sign-extends 24-bit result into int32_t. */
static void hx711_read_all(int32_t out[HX711_NUM_CORNERS]) {
    int32_t v[HX711_NUM_CORNERS] = {0, 0, 0, 0};

    /* 24 data pulses. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();   /* must keep SCK high <60 µs — block ISR jitter */

    for (int bit = 23; bit >= 0; bit--) {
        sck_high();
        short_delay();
        uint16_t lvl = dout_all();
        sck_low();
        short_delay();
        for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
            if (lvl & (1u << DOUT_PIN_BIT(c))) {
                v[c] |= (1L << bit);
            }
        }
    }

    /* 25th pulse: selects ch A gain 128 for next conversion. */
    sck_high();
    short_delay();
    sck_low();
    short_delay();

    if (!primask) __enable_irq();

    /* Sign-extend the 24-bit two's-complement into 32-bit signed. */
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
        if (v[c] & 0x800000) v[c] |= ~0xFFFFFF;
        out[c] = v[c];
    }
}

/* ---- Init -------------------------------------------------------------- */

void hx711_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    /* SCK low first, then output. */
    sck_low();
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER10) | GPIO_MODER_MODER10_0;

    /* DOUT lines (PB12..PB15) stay at reset-default input mode with no pulls;
     * the HX711 actively drives them. No GPIO config needed here. */

    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
        s_raw[c]    = 0;
        s_offset[c] = HX711_DEFAULT_OFFSET;
        s_scale[c]  = HX711_DEFAULT_SCALE;
    }
    s_have_data = false;
    s_tare_active = false;
}

/* ---- Tick scheduling --------------------------------------------------- */

static uint32_t period_ms_for_state(void) {
    bool moving = state_function() != FUNC_STANDBY;
    return moving ? (1000u / HX711_RATE_MOVING_HZ) : (1000u / HX711_RATE_STANDBY_HZ);
}

void hx711_tick(uint32_t now_ms) {
#if DISABLE_LOAD_CELLS
    (void)now_ms;
    return;
#else
    if ((now_ms - s_last_read_ms) < period_ms_for_state()) return;

    /* Bail if not all corners are ready — try again next tick. Don't burn the
     * main loop spinning on DOUT. With period >= 33 ms (30 Hz) and HX711 80 SPS
     * mode, data is essentially always ready when we look. */
    if (!dout_all_ready()) {
        /* Log once per cycle if DOUT has been high for longer than the timeout
         * window past the scheduled read — guards against a hung chip. */
        uint32_t since = now_ms - s_last_read_ms;
        if (since > (period_ms_for_state() + HX711_TIMEOUT_MS) &&
            since < (period_ms_for_state() + HX711_TIMEOUT_MS + 50u)) {
            log_record(LOG_MOD_HX711, LOG_SEV_WARN,
                       LOG_CODE_HX711_TIMEOUT, dout_all());
        }
        return;
    }

    int32_t v[HX711_NUM_CORNERS];
    hx711_read_all(v);
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) s_raw[c] = v[c];

    s_have_data = true;
    s_last_read_ms = now_ms;

    /* Tare averaging if active. */
    if (s_tare_active) {
        for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
            s_tare_accum[c] += v[c];
        }
        s_tare_samples_left--;
        if (s_tare_samples_left == 0) {
            for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
                s_offset[c] = (int32_t)(s_tare_accum[c] / TARE_SAMPLE_COUNT);
            }
            s_tare_active = false;
            log_record(LOG_MOD_HX711, LOG_SEV_INFO, LOG_CODE_HX711_TARE_COMPLETE, 0);
        }
    }
#endif
}

/* ---- Read API ---------------------------------------------------------- */

int32_t hx711_raw(uint32_t corner) {
    return (corner < HX711_NUM_CORNERS) ? s_raw[corner] : 0;
}

float hx711_kg(uint32_t corner) {
    if (corner >= HX711_NUM_CORNERS) return 0.0f;
    return (float)(s_raw[corner] - s_offset[corner]) * s_scale[corner];
}

float hx711_total_kg(void) {
    float t = 0.0f;
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) t += hx711_kg(c);
    return t;
}

void  hx711_set_offset(uint32_t c, int32_t v)  { if (c < HX711_NUM_CORNERS) s_offset[c] = v; }
void  hx711_set_scale (uint32_t c, float v)    { if (c < HX711_NUM_CORNERS) s_scale[c]  = v; }
int32_t hx711_offset(uint32_t c)               { return (c < HX711_NUM_CORNERS) ? s_offset[c] : 0; }
float   hx711_scale (uint32_t c)               { return (c < HX711_NUM_CORNERS) ? s_scale[c]  : 0.0f; }

bool hx711_has_data(void) { return s_have_data; }

void hx711_start_tare(void) {
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) s_tare_accum[c] = 0;
    s_tare_samples_left = TARE_SAMPLE_COUNT;
    s_tare_active = true;
}

bool hx711_tare_in_progress(void) { return s_tare_active; }
