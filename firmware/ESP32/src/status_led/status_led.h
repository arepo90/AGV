#pragma once

#include <stdint.h>

void status_led_init(int pin, bool active_low = false);
void status_led_update_estop(bool active);
void status_led_update_mode(bool supervised);
void status_led_update_function(uint8_t func); // 0=STANDBY, 1=REMOTE_CONTROL, 2=LINE_FOLLOW, 3=TRAJECTORY_FOLLOW
void status_led_tick(uint32_t nowMs); // Call regularly (e.g., every 50ms) with current time in ms
