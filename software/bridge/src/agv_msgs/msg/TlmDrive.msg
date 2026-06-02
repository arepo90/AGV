# PKT_TLM_DRIVE — per-wheel control internals (tuning stream).
# Mirrors firmware telemetry.c send_drive() exactly (32 bytes on the wire).
std_msgs/Header header

float32 v_left_target        # post-ramp, post-split target into the left PI
float32 v_right_target
float32 velocity_left        # measured wheel velocity (m/s)
float32 velocity_right
float32 duty_left            # applied signed PWM duty [-1, +1]
float32 duty_right
uint32 encoder_left_counts
uint32 encoder_right_counts
