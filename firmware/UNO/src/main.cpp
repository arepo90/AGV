/* =============================================================================
 *  AGV status panel — Arduino UNO + 3.5" TFT LCD shield (parallel, no touch).
 *  "Amber Industrial" face — EICAS / forklift-controller styling.
 *
 *  Display-only, single page. Reads a compact, checksummed ASCII status line
 *  from the Jetson over USB serial (driven by software/bridge panel_node) and
 *  renders one OVERVIEW screen: a state banner (NORMAL / CAUTION / E-STOP) with
 *  reason + speed cap, mode/function boxes, and a bottom strip with the battery
 *  voltages on the LEFT and total cargo load on the RIGHT. Shows NO LINK if the
 *  feed stops for >3 s.
 *
 *  Line format (newline-terminated), parsed field-by-field:
 *    AGV,<mode>,<func>,<estop>,<caution>,<cm>,<b3>,<p3>,<b6>,<p6>,
 *        <t0>,<t1>,<t2>,<t3>,<ir>,<c0>,<c1>,<c2>,<c3>,<reason>*<csum>
 *  csum = 8-bit XOR of every char between 'A' and '*', two hex digits.
 *    mode 0/1; func 0..3; estop/caution u16 masks; cm = caution modifier x100;
 *    bN = pack mV; pN = % (-1 absent); cN = corner load deci-kg (kg x10).
 *  The TOF (t0..t3) and IR (ir) fields are validated for field count but not
 *  shown on this face — only the totals matter here.
 *
 *  FONTS — one GFX font (FreeSansBold9pt7b, bundled with Adafruit_GFX) scaled
 *  x1..x3 for bold text, plus the classic 5x7 font for the tiniest labels.
 *
 *  Libraries: MCUFRIEND_kbv, Adafruit_GFX. Shield controller is auto-detected.
 * ===========================================================================*/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <Fonts/FreeSansBold9pt7b.h>

MCUFRIEND_kbv tft;
#define FONT_BOLD (&FreeSansBold9pt7b)   /* swap for a condensed font if desired */

/* ---- RGB565 palette (matches the "Amber Industrial" mock) ---------------- */
#define C_BLACK 0x0000
#define C_WHITE 0xFFFF
#define C_AMBER 0xFD60   /* #ffb000 */
#define C_GREEN 0x370D   /* #34e06a */
#define C_RED   0xF963   /* #ff2e1f */
#define C_DIM   0x7BCD   /* #7a7a6e */
#define C_LINE  0x39C6   /* #3a3a30 dark olive border */
#define C_OFF   0x18E3   /* unfilled segment */

static const int16_t W = 480;

static const char *MODE_NAMES[] = { "SUPERVISED", "UNSUPERVISED" };
static const char *FUNC_NAMES[] = { "STANDBY", "REMOTE CTRL", "LINE FOLLOW", "TRAJECTORY" };

/* cargo thresholds (mirror config.h) */
static const float CARGO_CAUTION_KG = 80.0f, CARGO_ESTOP_KG = 100.0f;

/* Link is declared lost after this much silence on the feed. */
static const uint32_t LINK_TIMEOUT_MS = 3000;

struct Status {
    int  mode, func;
    unsigned estop, caution;
    int  cm;                 /* caution modifier x100 (= speed cap %) */
    long b3, b6;             /* mV */
    int  p3, p6;             /* % or -1 (absent) */
    int  cargo[4];           /* deci-kg, FL FR RL RR */
    char reason[24];
};

static Status   s_cur, s_prev;
static char     s_line[200];
static uint8_t  s_len = 0;
static uint32_t s_last_rx_ms = 0;
static bool     s_linked = false;
static bool     s_full   = false;   /* a full (chrome + all values) repaint is pending */

/* ---- line parsing -------------------------------------------------------- */
static int8_t hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Validate the checksum and split the line into `out`. The 18 numeric fields
 * must all be present; TOF/IR (indices 9..13) are skipped, only the totals are
 * kept. Returns false on any malformed input (the previous frame is retained). */
static bool parse_line(char *line, Status *out) {
    char *star = strchr(line, '*');
    if (!star || line[0] != 'A') return false;
    uint8_t csum = 0;
    for (char *p = line; p < star; p++) csum ^= (uint8_t)*p;
    int8_t hi = hexval(star[1]), lo = hexval(star[2]);
    if (hi < 0 || lo < 0 || ((hi << 4) | lo) != csum) return false;

    *star = '\0';
    char *tok = strtok(line, ",");
    if (!tok || strcmp(tok, "AGV") != 0) return false;

    long f[18];
    int n = 0;
    while (n < 18 && (tok = strtok(NULL, ",")) != NULL) f[n++] = atol(tok);
    if (n != 18) return false;
    const char *rs = strtok(NULL, ",");        /* reason text (may be NULL) */

    out->mode = f[0];  out->func = f[1];
    out->estop = (unsigned)f[2];  out->caution = (unsigned)f[3];
    out->cm = f[4];
    out->b3 = f[5];  out->p3 = f[6];
    out->b6 = f[7];  out->p6 = f[8];
    for (int i = 0; i < 4; i++) out->cargo[i] = f[14 + i];
    strncpy(out->reason, rs ? rs : "", sizeof out->reason - 1);
    out->reason[sizeof out->reason - 1] = '\0';
    return true;
}

/* ---- small helpers ------------------------------------------------------- */
static uint16_t pct_color(int pct) {
    if (pct < 0)   return C_DIM;
    if (pct <= 15) return C_RED;
    if (pct <= 35) return C_AMBER;
    return C_GREEN;
}

/* Bold (GFX-font) text, positioned by TOP-LEFT. ascent ~= 13 px per size. */
static void boldText(int16_t x, int16_t yTop, const char *t, uint16_t col, uint8_t size) {
    tft.setFont(FONT_BOLD);
    tft.setTextColor(col);
    tft.setTextSize(size);
    tft.setCursor(x, yTop + 13 * size);
    tft.print(t);
    tft.setFont(NULL);
}
/* Tiny classic-font label (5x7), top-left. */
static void tinyText(int16_t x, int16_t y, const char *t, uint16_t col, uint8_t size = 1) {
    tft.setFont(NULL);
    tft.setTextColor(col);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    tft.print(t);
}
/* Erase a fixed box then draw bold text (anti-ghost). */
static void boldBox(int16_t x, int16_t y, int16_t w, int16_t h, const char *t, uint16_t col, uint8_t size) {
    tft.fillRect(x, y, w, h, C_BLACK);
    boldText(x, y, t, col, size);
}

/* Segmented bar (battery / load), n cells. */
static void drawSeg(int16_t x, int16_t y, int16_t w, int16_t h, int pct, uint16_t col, int n) {
    int filled = (pct <= 0) ? 0 : ((pct >= 100 ? 100 : pct) * n + 50) / 100;
    int gap = 2;
    int cw = (w - gap * (n - 1)) / n;
    for (int i = 0; i < n; i++)
        tft.fillRect(x + i * (cw + gap), y, cw, h, i < filled ? col : C_OFF);
}

/* Hazard stripes (amber/black diagonal) for the E-STOP / NO-LINK banners. */
static void drawHazard(int16_t x, int16_t y, int16_t w, int16_t h) {
    tft.fillRect(x, y, w, h, C_AMBER);
    for (int i = -h; i < w; i += 16)
        for (int k = 0; k < 8; k++)
            tft.drawLine(x + i + k, y + h - 1, x + i + k + h, y, C_BLACK);
}

/* ---- chrome: top + bottom bars, drawn once per full repaint -------------- */
static void draw_chrome() {
    tft.fillScreen(C_BLACK);
    boldText(8, 5, "AGV", C_AMBER, 1);
    tinyText(56, 12, "TEAM GTL", C_DIM, 1);
    tft.fillRect(0, 32, W, 2, C_AMBER);
    tft.drawFastHLine(0, 294, W, C_LINE);
    tinyText(10, 303, "OVERVIEW", C_DIM, 1);
}

/* Top-right link badge + uptime clock. Refreshed every second while linked and
 * once on each link-state change (so the badge turns red the moment the feed
 * drops, since the NO-LINK screen only repaints the body). */
static void draw_link(bool linked) {
    tft.fillRect(360, 4, 116, 26, C_BLACK);
    char up[10];
    uint32_t sec = millis() / 1000;
    snprintf(up, sizeof up, "%lu:%02lu", sec / 60, sec % 60);
    tinyText(366, 12, up, C_DIM, 1);
    uint16_t c = linked ? C_GREEN : C_RED;
    tft.drawRect(414, 6, 60, 22, c);
    tft.fillCircle(424, 17, 3, c);
    tinyText(432, 13, linked ? "LINK" : "NOLNK", c, 1);
}

/* ---- the OVERVIEW page ---------------------------------------------------- */
static void draw_banner(const Status *c) {
    bool filled = c->estop || c->caution;
    uint16_t col = c->estop ? C_RED : c->caution ? C_AMBER : C_GREEN;
    uint16_t tcol = filled ? C_BLACK : col;
    const char *word = c->estop ? "E-STOP" : c->caution ? "CAUTION" : "NORMAL";

    tft.fillRect(8, 40, 464, 92, filled ? col : C_BLACK);
    tft.drawRect(8, 40, 464, 92, col);
    tft.drawRect(9, 41, 462, 90, col);
    if (c->estop) { drawHazard(8, 40, 464, 7); drawHazard(8, 125, 464, 7); }
    tft.fillRect(8, 48, 10, 76, col);                 /* accent bar */
    boldText(30, 52, word, tcol, 2);
    tinyText(30, 104, c->reason[0] ? c->reason : "ALL SYSTEMS CLEAR", tcol, 1);
    /* speed cap */
    char cap[8]; snprintf(cap, sizeof cap, "%d%%", c->cm);
    tinyText(360, 52, "SPEED CAP", tcol, 1);
    tft.fillRect(360, 64, 108, 40, filled ? col : C_BLACK);
    boldText(360, 66, cap, tcol, 2);
}

static void draw_battery_row(int row, const Status *c) {
    int  y   = 206 + row * 42;
    long mv  = row ? c->b6 : c->b3;
    int  pct = row ? c->p6 : c->p3;
    uint16_t bc = pct_color(pct);

    tft.fillRect(8, y - 2, 228, 36, C_BLACK);           /* clear the row */
    boldText(8, y, row ? "6S" : "3S", C_WHITE, 1);
    char vv[12];
    if (pct < 0) snprintf(vv, sizeof vv, "ABSENT");
    else         snprintf(vv, sizeof vv, "%ld.%02ldV", mv / 1000, (mv % 1000) / 10);
    boldText(44, y, vv, bc, 1);
    if (pct >= 0) { char pp[6]; snprintf(pp, sizeof pp, "%d%%", pct); tinyText(180, y + 5, pp, bc, 1); }
    drawSeg(8, y + 22, 224, 8, pct, bc, 14);
}

static float cargo_total(const Status *s) {
    float t = 0;
    for (int i = 0; i < 4; i++) t += s->cargo[i] / 10.0f;
    return t;
}

static void draw_cargo(const Status *c) {
    float total = cargo_total(c);
    uint16_t tcol = total >= CARGO_ESTOP_KG ? C_RED
                  : total >= CARGO_CAUTION_KG ? C_AMBER : C_GREEN;
    char tot[12]; snprintf(tot, sizeof tot, "%d.%d", (int)total, ((int)(total * 10)) % 10);
    tft.fillRect(252, 204, 224, 48, C_BLACK);
    boldText(252, 204, tot, tcol, 3);
    tinyText(430, 230, "KG", C_DIM, 2);
    drawSeg(252, 262, 212, 12, (int)(total / CARGO_ESTOP_KG * 100), tcol, 16);
}

/* Renders the page. `full` forces every field; otherwise each block redraws
 * only when its inputs changed (the TFT write is the slow part). */
static void page_overview(const Status *c, const Status *p, bool full) {
    if (full) {
        tinyText(8, 192, "POWER", C_DIM, 1);
        tinyText(252, 192, "TOTAL LOAD", C_DIM, 1);
        tft.drawFastVLine(240, 192, 98, C_LINE);
    }

    if (full || c->estop != p->estop || c->caution != p->caution ||
        c->cm != p->cm || strcmp(c->reason, p->reason) != 0)
        draw_banner(c);

    if (full || c->mode != p->mode) {
        tft.drawRect(8, 142, 228, 44, C_LINE);
        tinyText(16, 150, "MODE", C_DIM, 1);
        boldBox(16, 160, 212, 22, (c->mode >= 0 && c->mode <= 1) ? MODE_NAMES[c->mode] : "?",
                c->mode == 0 ? C_GREEN : C_AMBER, 1);
    }
    if (full || c->func != p->func) {
        tft.drawRect(244, 142, 228, 44, C_LINE);
        tinyText(252, 150, "FUNCTION", C_DIM, 1);
        boldBox(252, 160, 212, 22, (c->func >= 0 && c->func <= 3) ? FUNC_NAMES[c->func] : "?", C_WHITE, 1);
    }

    if (full || c->b3 != p->b3 || c->p3 != p->p3) draw_battery_row(0, c);
    if (full || c->b6 != p->b6 || c->p6 != p->p6) draw_battery_row(1, c);

    if (full || cargo_total(c) != cargo_total(p)) draw_cargo(c);
}

static void draw_no_link() {
    tft.fillRect(0, 34, W, 260, C_BLACK);
    drawHazard(70, 150, 340, 8);
    boldText(150, 175, "NO LINK", C_RED, 2);
    tinyText(180, 220, "FEED LOST > 3s", C_DIM, 1);
    drawHazard(70, 240, 340, 8);
}

/* ---- read one newline-terminated line off the feed, parse on completion --- */
static void poll_serial() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                s_len = 0;
                if (parse_line(s_line, &s_cur)) {
                    s_last_rx_ms = millis();
                    if (!s_linked) { s_linked = true; s_full = true; }   /* (re)acquired */
                }
            }
        } else if (s_len < sizeof(s_line) - 1) {
            s_line[s_len++] = ch;
        } else {
            s_len = 0;   /* overrun — drop the runt line */
        }
    }
}

void setup() {
    Serial.begin(115200);
    uint16_t id = tft.readID();
    if (id == 0x0 || id == 0xFFFF || id == 0xD3D3) id = 0x9486;   /* common 3.5" fallback */
    tft.begin(id);
    tft.setRotation(1);

    /* Show NO LINK immediately so the panel isn't blank until the first frame.
     * Paint the chrome too, so boot looks identical to a mid-session dropout. */
    draw_chrome();
    draw_no_link();
    draw_link(false);
}

void loop() {
    poll_serial();

    if (s_linked) {
        bool full = s_full;
        if (full) {
            draw_chrome();
            memset(&s_prev, 0xFF, sizeof s_prev);   /* force every block to repaint */
            s_full = false;
        }
        page_overview(&s_cur, &s_prev, full);
        s_prev = s_cur;

        /* refresh the link badge + uptime clock once a second (and on full) */
        static uint32_t last_badge = 0;
        if (full || millis() - last_badge > 1000) { draw_link(true); last_badge = millis(); }
    }

    /* link watchdog: blank the body and flip the badge red on feed loss */
    if (s_linked && millis() - s_last_rx_ms > LINK_TIMEOUT_MS) {
        s_linked = false;
        draw_no_link();
        draw_link(false);
    }
}
