#include "status_led.h"
#include <Arduino.h>

static int ledPin = -1;
static bool ledActiveLow = false;
static uint32_t lastUpdateMs = 0;
static bool estopActive = false;
static bool isSupervisedMode = true;
static uint8_t currentFunction = 0; // 0=STANDBY, 1=REMOTE, 2=LINE, 3=TRAJ

static inline void led_write(bool on) {
  digitalWrite(ledPin, (on ^ ledActiveLow) ? HIGH : LOW);
}

void status_led_init(int pin, bool active_low) {
  ledPin = pin;
  ledActiveLow = active_low;
  pinMode(ledPin, OUTPUT);
  led_write(false);
}

void status_led_update_estop(bool active) {
  estopActive = active;
}

void status_led_update_mode(bool supervised) {
  isSupervisedMode = supervised;
}

void status_led_update_function(uint8_t func) {
  currentFunction = func;
}

/* Flash pattern:
 *   ESTOP: continuous fast flashing (200ms on/off)
 *   Otherwise:
 *     - Mode indicator: 100ms flash (supervised) or 400ms flash (unsupervised), then 200ms pause
 *     - Function indicator: N fast flashes (50ms on/off), then 500ms pause
 *     - Total cycle: ~1s + N*100ms
 */
void status_led_tick(uint32_t nowMs) {
  if (ledPin < 0) return;

  if (estopActive) {
    // Fast continuous flashing (200ms period = 100ms on, 100ms off)
    uint32_t phase = (nowMs / 100) % 2;
    led_write(phase == 0);
    return;
  }

  // Non-ESTOP pattern: mode indicator (100-400ms) + pause (200ms) + function flashes (0-300ms) + final pause = ~600-900ms cycle
  uint32_t cycleTime = 1200u; // full cycle duration
  uint32_t offset = nowMs % cycleTime;

  uint32_t modeFlashDuration = isSupervisedMode ? 100u : 400u;
  uint32_t pauseAfterMode = 200u;
  uint32_t funcFlashDuration = currentFunction * 100u; // 100ms per flash (50ms on + 50ms off)
  uint32_t finalPauseDuration = cycleTime - (modeFlashDuration + pauseAfterMode + funcFlashDuration);

  // Stage 1: Mode indicator flash
  if (offset < modeFlashDuration) {
    led_write(true);
    return;
  }

  // Stage 2: Pause after mode
  if (offset < modeFlashDuration + pauseAfterMode) {
    led_write(false);
    return;
  }

  // Stage 3: Function indicator flashes
  uint32_t funcStart = modeFlashDuration + pauseAfterMode;
  if (offset < funcStart + funcFlashDuration) {
    uint32_t funcOffset = offset - funcStart;
    uint32_t flashPhase = (funcOffset / 50u) % 2;
    led_write(flashPhase == 0);
    return;
  }

  // Stage 4: Final pause before cycle repeats
  led_write(false);
}
