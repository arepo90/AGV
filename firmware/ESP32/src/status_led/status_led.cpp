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
  /* Documented values are 0..3 (STANDBY/REMOTE/LINE/TRAJ). Anything outside is
   * either a bad telemetry frame or a future addition we don't render yet —
   * clamp so the cycle math below can't underflow. */
  currentFunction = (func > 3) ? 0 : func;
}

/* Pattern (one ~1.2 s cycle, non-ESTOP):
 *   1. Mode flash: 100 ms (SUPERVISED) or 400 ms (UNSUPERVISED) on
 *   2. Pause: 200 ms off
 *   3. Function flashes: N × 100 ms (50 ms on / 50 ms off), N = function ID
 *   4. Final pause to fill the rest of the cycle.
 *
 * ESTOP: continuous 100 ms on / 100 ms off, no cycle structure. */
void status_led_tick(uint32_t nowMs) {
  if (ledPin < 0) return;

  if (estopActive) {
    led_write(((nowMs / 100u) % 2u) == 0u);
    return;
  }

  const uint32_t cycleTime = 1200u;
  uint32_t offset = nowMs % cycleTime;

  uint32_t modeFlashDuration = isSupervisedMode ? 100u : 400u;
  uint32_t pauseAfterMode = 200u;
  uint32_t funcFlashDuration = currentFunction * 100u;   /* 50 ms on + 50 ms off per flash */
  /* finalPauseDuration is implicit — anything past the function flashes stays off. */
  (void)cycleTime;  /* offset is already wrapped by cycleTime above */

  if (offset < modeFlashDuration) {
    led_write(true);
    return;
  }
  if (offset < modeFlashDuration + pauseAfterMode) {
    led_write(false);
    return;
  }
  uint32_t funcStart = modeFlashDuration + pauseAfterMode;
  if (offset < funcStart + funcFlashDuration) {
    uint32_t funcOffset = offset - funcStart;
    led_write(((funcOffset / 50u) % 2u) == 0u);
    return;
  }
  led_write(false);   /* final pause until offset wraps */
}
