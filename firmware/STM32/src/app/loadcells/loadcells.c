#include "loadcells.h"
#include "config.h"
#include "log.h"
#include "stm32f0xx.h"

#define SCK_BIT         10u
#define DOUT_BIT(c)     ((c) + 12u)                  /* physical DOUT index 0..3 → PB12..PB15 */
#define DOUT_MASK       ((1u<<12)|(1u<<13)|(1u<<14)|(1u<<15))
#define TARE_SAMPLES    20u

/* Logical corner (0=FL, 1=FR, 2=RL, 3=RR — telemetry/GUI order) → physical
 * DOUT index, from the wiring constants in config.h. */
static const uint32_t s_corner_map[HX711_NUM_CORNERS] = {
    HX711_CORNER_FRONT_LEFT, HX711_CORNER_FRONT_RIGHT,
    HX711_CORNER_REAR_LEFT,  HX711_CORNER_REAR_RIGHT,
};

static int32_t  s_raw[HX711_NUM_CORNERS];      /* indexed by physical DOUT pin */
static int32_t  s_offset[HX711_NUM_CORNERS];   /* indexed by physical DOUT pin */
static float    s_scale;                       /* counts → kg, shared by all cells */
static bool     s_have_data = false;
#if !DISABLE_LOAD_CELLS
static uint32_t s_last_read_ms = 0;
#endif

static bool     s_tare_active = false;
static uint8_t  s_tare_left = 0;
static int64_t  s_tare_accum[HX711_NUM_CORNERS];

static inline void short_delay(void) { for (volatile int i = 0; i < 4; i++) { } }
static inline void sck_high(void)    { GPIOB->BSRR = 1u << SCK_BIT; }
static inline void sck_low(void)     { GPIOB->BSRR = 1u << (SCK_BIT + 16u); }
static inline uint16_t dout_all(void){ return (uint16_t)(GPIOB->IDR & DOUT_MASK); }
static inline bool all_ready(void)   { return dout_all() == 0; }   /* DOUT low = ready */

#if !DISABLE_LOAD_CELLS
/* Read one 24-bit word from each corner in parallel; sign-extend to int32. */
static void read_all(int32_t out[HX711_NUM_CORNERS]) {
    int32_t v[HX711_NUM_CORNERS] = { 0, 0, 0, 0 };

    uint32_t primask = __get_PRIMASK();
    __disable_irq();    /* keep SCK-high pulses < 60 µs: no ISR jitter */

    for (int bit = 23; bit >= 0; bit--) {
        sck_high();
        short_delay();
        uint16_t lvl = dout_all();
        sck_low();
        short_delay();
        for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
            if (lvl & (1u << DOUT_BIT(c))) v[c] |= (1L << bit);
        }
    }
    /* 25th pulse selects channel A gain 128 for the next conversion. */
    sck_high(); short_delay();
    sck_low();  short_delay();

    if (!primask) __enable_irq();

    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
        if (v[c] & 0x800000) v[c] |= ~0xFFFFFF;
        out[c] = v[c];
    }
}
#endif /* !DISABLE_LOAD_CELLS */

void loadcells_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    sck_low();
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER10) | GPIO_MODER_MODER10_0;
    /* DOUT lines stay input (reset default); the HX711s drive them. */

    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) {
        s_raw[c]    = 0;
        s_offset[c] = HX711_DEFAULT_OFFSET;
    }
    s_scale = HX711_DEFAULT_SCALE;
    s_have_data = false;
    s_tare_active = false;
}

void loadcells_tick(uint32_t now_ms, bool moving) {
#if DISABLE_LOAD_CELLS
    (void)now_ms; (void)moving;
    return;
#else
    uint32_t period = moving ? (1000u / HX711_RATE_MOVING_HZ)
                             : (1000u / HX711_RATE_STANDBY_HZ);
    if ((now_ms - s_last_read_ms) < period) return;

    if (!all_ready()) {
        /* Not ready well past the scheduled window → log a hung-chip warning once. */
        uint32_t since = now_ms - s_last_read_ms;
        if (since > (period + HX711_TIMEOUT_MS) && since < (period + HX711_TIMEOUT_MS + 50u)) {
            log_record(LOG_MOD_HX711, LOG_SEV_WARN, LOG_CODE_HX711_TIMEOUT, dout_all());
        }
        return;
    }

    int32_t v[HX711_NUM_CORNERS];
    read_all(v);
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) s_raw[c] = v[c];
    s_have_data = true;
    s_last_read_ms = now_ms;

    if (s_tare_active) {
        for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) s_tare_accum[c] += v[c];
        if (--s_tare_left == 0) {
            for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++)
                s_offset[c] = (int32_t)(s_tare_accum[c] / (int64_t)TARE_SAMPLES);
            s_tare_active = false;
            log_record(LOG_MOD_HX711, LOG_SEV_INFO, LOG_CODE_HX711_TARE_COMPLETE, 0);
        }
    }
#endif
}

bool loadcells_has_data(void) { return s_have_data; }

float loadcells_kg(uint32_t corner) {
    if (corner >= HX711_NUM_CORNERS) return 0.0f;
    uint32_t phys = s_corner_map[corner];
    return (float)(s_raw[phys] - s_offset[phys]) * s_scale;
}

float loadcells_total_kg(void) {
    float t = 0.0f;
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) t += loadcells_kg(c);
    return t;
}

void loadcells_set_offset(uint32_t corner, int32_t v) {
    if (corner < HX711_NUM_CORNERS) s_offset[s_corner_map[corner]] = v;
}

/* Single scale shared by all cells — corner id accepted (any of PARAM 0x38-0x3B)
 * but ignored; all cells get the same factor. */
void loadcells_set_scale(uint32_t corner, float v) {
    (void)corner;
    s_scale = v;
}

void loadcells_start_tare(void) {
    for (uint32_t c = 0; c < HX711_NUM_CORNERS; c++) s_tare_accum[c] = 0;
    s_tare_left = TARE_SAMPLES;
    s_tare_active = true;
}

bool loadcells_tare_in_progress(void) { return s_tare_active; }
