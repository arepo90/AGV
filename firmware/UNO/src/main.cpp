/* =============================================================================
 *  AGV status panel — Arduino UNO + 3.5" TFT LCD shield (parallel, no touch).
 *
 *  Display-only. Reads a compact, checksummed ASCII status line from the Jetson
 *  over USB serial (driven by software/bridge panel_node) and renders mode,
 *  function, E-STOP / caution state, and 3S/6S battery charge. TOF ranges are
 *  shown as a compact footer.
 *
 *  Line format (newline-terminated):
 *    AGV,<mode>,<func>,<estop>,<caution>,<cm>,<b3>,<p3>,<b6>,<p6>,<t0>,<t1>,<t2>,<t3>*<csum>
 *  csum = 8-bit XOR of every char between 'A' and '*', two hex digits.
 *  Fields: mode 0/1; func 0..3; estop/caution u16 bitmasks; cm = caution
 *  modifier ×100; bN = pack mV; pN = % (-1 absent); tN = TOF mm (F,R,L,R).
 *
 *  Libraries (install via Arduino IDE / arduino-cli):
 *    MCUFRIEND_kbv, Adafruit_GFX. The shield's controller is auto-detected.
 * =============================================================================
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>

MCUFRIEND_kbv tft;

/* ---- 565 colours -------------------------------------------------------- */
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_GREY    0x7BEF
#define C_DGREY   0x39E7
#define C_GREEN   0x07E0
#define C_AMBER   0xFD20
#define C_RED     0xF800
#define C_CYAN    0x07FF

/* Landscape 480×320. */
static const int16_t W = 480, H = 320;

static const char *MODE_NAMES[] = { "SUPERVISED", "UNSUPERVISED" };
static const char *FUNC_NAMES[] = { "STANDBY", "REMOTE CTRL", "LINE FOLLOW", "TRAJECTORY" };

struct Status {
    int mode, func;
    unsigned estop, caution;
    int cm;                  /* caution modifier ×100 */
    long b3, b6;             /* mV */
    int  p3, p6;             /* % or -1 */
    int  tof[4];             /* mm */
    bool valid;
};

static Status s_cur, s_prev;
static char    s_line[160];
static uint8_t s_len = 0;
static uint32_t s_last_rx_ms = 0;
static bool    s_linked = false;     /* tracks NO-LINK banner state */
static bool    s_first_draw = true;

/* ---- line parsing ------------------------------------------------------- */
static int8_t hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Validate checksum and split the 14 fields into `out`. Returns true on success. */
static bool parse_line(char *line, Status *out) {
    char *star = strchr(line, '*');
    if (!star || line[0] != 'A') return false;

    uint8_t csum = 0;
    for (char *p = line; p < star; p++) csum ^= (uint8_t)*p;
    int8_t hi = hexval(star[1]), lo = hexval(star[2]);
    if (hi < 0 || lo < 0) return false;
    if (((hi << 4) | lo) != csum) return false;

    *star = '\0';                       /* terminate body before checksum */
    long f[13];                         /* 13 values follow the "AGV" header */
    int n = 0;
    char *tok = strtok(line, ",");      /* "AGV" header */
    if (!tok || strcmp(tok, "AGV") != 0) return false;
    while ((tok = strtok(NULL, ",")) != NULL && n < 13) f[n++] = atol(tok);
    if (n != 13) return false;

    out->mode = f[0]; out->func = f[1];
    out->estop = (unsigned)f[2]; out->caution = (unsigned)f[3];
    out->cm = f[4];
    out->b3 = f[5]; out->p3 = f[6];
    out->b6 = f[7]; out->p6 = f[8];
    out->tof[0] = f[9]; out->tof[1] = f[10]; out->tof[2] = f[11]; out->tof[3] = f[12];
    out->valid = true;
    return true;
}

/* ---- drawing helpers ---------------------------------------------------- */
static void label(int16_t x, int16_t y, const char *txt, uint16_t col, uint8_t size) {
    tft.setTextColor(col, C_BLACK);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    tft.print(txt);
}

/* Erase a fixed-width box then print right-padded text (avoids ghosting). */
static void value_box(int16_t x, int16_t y, int16_t w, int16_t h,
                      const char *txt, uint16_t col, uint8_t size) {
    tft.fillRect(x, y, w, h, C_BLACK);
    label(x, y, txt, col, size);
}

static void draw_bar(int16_t x, int16_t y, int16_t w, int16_t h, int pct, uint16_t col) {
    tft.drawRect(x, y, w, h, C_GREY);
    tft.fillRect(x + 1, y + 1, w - 2, h - 2, C_BLACK);
    if (pct > 0) {
        int16_t fillw = (int32_t)(w - 2) * (pct > 100 ? 100 : pct) / 100;
        tft.fillRect(x + 1, y + 1, fillw, h - 2, col);
    }
}

static uint16_t pct_color(int pct) {
    if (pct < 0)  return C_GREY;
    if (pct <= 15) return C_RED;
    if (pct <= 35) return C_AMBER;
    return C_GREEN;
}

static uint16_t tof_color(int mm) {
    if (mm <= 200) return C_RED;
    if (mm <= 800) return C_AMBER;
    return C_WHITE;
}

/* Static labels / chrome drawn once. */
static void draw_chrome() {
    tft.fillScreen(C_BLACK);
    tft.drawFastHLine(0, 34, W, C_DGREY);
    label(8, 8, "AGV STATUS", C_CYAN, 3);
    label(8, 70,  "MODE", C_GREY, 2);
    label(8, 120, "FUNCTION", C_GREY, 2);
    label(8, 196, "3S", C_GREY, 2);
    label(8, 244, "6S", C_GREY, 2);
    label(8, 292, "TOF", C_GREY, 2);
}

static void render(const Status *c, const Status *p, bool force) {
    /* State banner: ESTOP (red) > CAUTION (amber) > NORMAL (green). */
    if (force || c->estop != p->estop || c->caution != p->caution || c->cm != p->cm) {
        uint16_t col = c->estop ? C_RED : c->caution ? C_AMBER : C_GREEN;
        const char *txt = c->estop ? "E-STOP" : c->caution ? "CAUTION" : "NORMAL";
        tft.fillRect(220, 44, W - 220, 92, C_BLACK);
        tft.fillRect(220, 44, W - 220, 92, col);
        tft.setTextColor(C_BLACK, col);
        tft.setTextSize(4);
        tft.setCursor(240, 70);
        tft.print(txt);
        char buf[20];
        snprintf(buf, sizeof buf, "speed %d%%", c->cm);
        tft.setTextSize(2);
        tft.setCursor(240, 110);
        tft.print(buf);
    }

    if (force || c->mode != p->mode)
        value_box(120, 70, 340, 24,
                  (c->mode >= 0 && c->mode <= 1) ? MODE_NAMES[c->mode] : "?",
                  c->mode == 0 ? C_GREEN : C_AMBER, 3);

    if (force || c->func != p->func)
        value_box(170, 120, 300, 24,
                  (c->func >= 0 && c->func <= 3) ? FUNC_NAMES[c->func] : "?", C_WHITE, 3);

    /* Battery bars + text. */
    if (force || c->p3 != p->p3 || c->b3 != p->b3) {
        char buf[24];
        if (c->p3 < 0) snprintf(buf, sizeof buf, "absent");
        else snprintf(buf, sizeof buf, "%d%%  %d.%02dV", c->p3, (int)(c->b3 / 1000), (int)((c->b3 % 1000) / 10));
        draw_bar(60, 196, 230, 22, c->p3, pct_color(c->p3));
        value_box(300, 198, 180, 20, buf, pct_color(c->p3), 2);
    }
    if (force || c->p6 != p->p6 || c->b6 != p->b6) {
        char buf[24];
        if (c->p6 < 0) snprintf(buf, sizeof buf, "absent");
        else snprintf(buf, sizeof buf, "%d%%  %d.%02dV", c->p6, (int)(c->b6 / 1000), (int)((c->b6 % 1000) / 10));
        draw_bar(60, 244, 230, 22, c->p6, pct_color(c->p6));
        value_box(300, 246, 180, 20, buf, pct_color(c->p6), 2);
    }

    /* TOF footer: F R L R (mm). */
    static const char *tlbl[4] = { "F", "R", "L", "Rt" };
    for (int i = 0; i < 4; i++) {
        if (force || c->tof[i] != p->tof[i]) {
            char buf[16];
            if (c->tof[i] >= 1200) snprintf(buf, sizeof buf, "%s:clr", tlbl[i]);
            else snprintf(buf, sizeof buf, "%s:%d", tlbl[i], c->tof[i]);
            value_box(60 + i * 108, 292, 104, 20, buf, tof_color(c->tof[i]), 2);
        }
    }
}

static void draw_no_link() {
    tft.fillRect(220, 44, W - 220, 92, C_DGREY);
    tft.setTextColor(C_WHITE, C_DGREY);
    tft.setTextSize(3);
    tft.setCursor(250, 78);
    tft.print("NO LINK");
}

void setup() {
    Serial.begin(115200);
    uint16_t id = tft.readID();
    if (id == 0x0 || id == 0xFFFF || id == 0xD3D3) id = 0x9486;   /* common 3.5" fallback */
    tft.begin(id);
    tft.setRotation(1);
    draw_chrome();
    memset(&s_prev, 0xFF, sizeof s_prev);   /* force first full render */
}

void loop() {
    /* Accumulate one line, then parse. */
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                Status tmp;
                if (parse_line(s_line, &tmp)) {
                    s_cur = tmp;
                    s_last_rx_ms = millis();
                    if (!s_linked || s_first_draw) { s_linked = true; }
                    render(&s_cur, &s_prev, s_first_draw);
                    s_prev = s_cur;
                    s_first_draw = false;
                }
                s_len = 0;
            }
        } else if (s_len < sizeof(s_line) - 1) {
            s_line[s_len++] = ch;
        } else {
            s_len = 0;   /* overflow: drop the runt line */
        }
    }

    /* Link watchdog. */
    if (s_linked && (millis() - s_last_rx_ms) > 2000) {
        s_linked = false;
        draw_no_link();
        s_first_draw = true;   /* repaint everything on reconnect */
    }
}
