/* =============================================================================
 *  AGV Wi-Fi relay (ESP32-C3 SuperMini)
 *
 *  Transparent bidirectional bridge between the workstation (over Wi-Fi /
 *  WebSocket binary) and the STM32 (over UART). The ESP32 does no protocol
 *  interpretation beyond:
 *
 *    - validating frame magic, version, length, and CRC on UART RX
 *    - validating same on WS RX (paranoia — TCP already covers integrity)
 *    - NACKing the originator when CRC fails (so they can retry)
 *    - dropping unknown-version frames
 *
 *  AGV state lives entirely on the STM32. This relay never blocks a frame.
 * =============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "config.h"
#include "frame.h"

static AsyncWebServer  s_server(80);
static AsyncWebSocket  s_ws(WS_PATH);
static HardwareSerial  s_stm32(1);

static frame_parser_t  s_uart_parser;

/* WS callbacks run on the AsyncTCP task; loop() runs on the Arduino task. The
 * AsyncWebSocket library is designed to be safely called across tasks, and an
 * aligned pointer write/read is atomic on ESP32-C3 (RISC-V single-core), so we
 * don't need a mutex around s_client. */
static AsyncWebSocketClient *s_client = nullptr;

/* ---- UART side: forward complete frames to WS ---------------------------- */

static void uart_on_complete(const uint8_t *frame, size_t total_len) {
    AsyncWebSocketClient *c = s_client;
    if (c && c->status() == WS_CONNECTED) {
        c->binary(const_cast<uint8_t *>(frame), total_len);
    }
}

static void uart_on_error(frame_result_t err, uint8_t suspect_seq) {
    /* NACK back to STM32 so it knows we dropped a frame.
     * STM32-originated traffic is fire-and-forget (telemetry, log), so this is
     * mostly diagnostic — STM32 will log the inbound NACK. */
    if (err == FRAME_BAD_CRC) {
        uint8_t nack[PROTO_MAX_FRAME];
        size_t n = frame_build_nack(nack, suspect_seq, NACK_BAD_CRC);
        s_stm32.write(nack, n);
    }
#if DEBUG_LOG_BAD_FRAMES
    Serial.printf("[uart] frame error %d seq=0x%02X\n", (int)err, suspect_seq);
#endif
}

static void uart_drain(void) {
    while (s_stm32.available()) {
        frame_parser_feed(&s_uart_parser, (uint8_t)s_stm32.read());
    }
}

/* ---- WS side: validate, then forward to UART ---------------------------- */

static void ws_handle_binary(AsyncWebSocketClient *client,
                             const uint8_t *data, size_t len) {
    frame_header_t hdr;
    frame_result_t r = frame_validate(data, len, &hdr);

    if (r == FRAME_OK) {
        s_stm32.write(data, len);
        return;
    }

    /* Bad WS frame — NACK back over WS so the workstation can retry. */
#if DEBUG_LOG_BAD_FRAMES
    Serial.printf("[ws] frame error %d len=%u\n", (int)r, (unsigned)len);
#endif

    uint8_t err_code;
    switch (r) {
    case FRAME_BAD_CRC:     err_code = NACK_BAD_CRC;     break;
    case FRAME_BAD_VERSION: err_code = NACK_BAD_VERSION; break;
    default:                err_code = NACK_BAD_LENGTH;  break;
    }

    uint8_t nack[PROTO_MAX_FRAME];
    size_t n = frame_build_nack(nack, hdr.seq, err_code);
    client->binary(nack, n);
}

static void ws_event(AsyncWebSocket * /*server*/, AsyncWebSocketClient *client,
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
    case WS_EVT_CONNECT:
        if (s_client && s_client != client) {
            /* One workstation at a time — kick the new arrival. */
            client->close(1000, "busy");
            return;
        }
        s_client = client;
        Serial.printf("[ws] client connected id=%u from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
        break;

    case WS_EVT_DISCONNECT:
        if (s_client == client) s_client = nullptr;
        Serial.printf("[ws] client disconnected id=%u\n", client->id());
        break;

    case WS_EVT_DATA: {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len &&
            info->opcode == WS_BINARY) {
            ws_handle_binary(client, data, len);
        } else if (info->opcode == WS_TEXT) {
            /* Text frames not expected — ignored. */
        }
        break;
    }

    default:
        break;
    }
}

/* ---- Status page (minimal — just confirms the AP is up) ----------------- */

static const char STATUS_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>AGV Relay</title>
<style>body{font-family:system-ui;background:#0f172a;color:#e2e8f0;padding:24px}
code{background:#1e293b;padding:2px 6px;border-radius:4px}</style></head><body>
<h1>AGV Wi-Fi Relay</h1>
<p>Status: running</p>
<p>WebSocket endpoint: <code>ws://192.168.4.1/ws</code> (binary frames)</p>
<p>This page is for verifying the AP — control happens over the WebSocket.</p>
</body></html>
)html";

/* ---- setup / loop ------------------------------------------------------- */

void setup() {
    Serial.begin(115200);
    delay(50);

    Serial.println();
    Serial.println("[boot] AGV relay starting");

    /* UART to STM32 */
    s_stm32.setRxBufferSize(UART_RX_BUFSIZE);
    s_stm32.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    frame_parser_init(&s_uart_parser, uart_on_complete, uart_on_error);

    /* Wi-Fi AP */
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
    Serial.printf("[wifi] AP %s on channel %d: %s\n",
                  AP_SSID, AP_CHANNEL, ok ? "up" : "FAILED");
    Serial.printf("[wifi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    /* HTTP + WS */
    s_ws.onEvent(ws_event);
    s_server.addHandler(&s_ws);
    s_server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", STATUS_HTML);
    });
    s_server.begin();

    Serial.println("[boot] ready");
}

void loop() {
    uart_drain();
    s_ws.cleanupClients();
    /* No delay() — UART throughput at 921600 demands prompt draining. */
}
