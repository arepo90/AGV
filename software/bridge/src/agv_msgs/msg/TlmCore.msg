# PKT_TLM_CORE — operational state + pose (fast stream).
# Mirrors firmware telemetry.c send_core() exactly (42 bytes on the wire).
std_msgs/Header header

uint32 timestamp_ms
uint8 mode                  # 0=SUPERVISED, 1=UNSUPERVISED
uint8 function              # 0=STANDBY, 1=REMOTE_CONTROL, 2=LINE_FOLLOW
uint16 estop_sources        # bitmask (bit8=battery, bit9=LiDAR; bit7 retired)
uint16 caution_sources      # bitmask (bit6=battery, bit7=LiDAR; bit5 retired)
float32 caution_modifier
uint8 flags                 # bit0=adc, bit1=hx711, bit3=tare_in_progress

float32 velocity_linear
float32 velocity_angular
float32 odom_x
float32 odom_y
float32 odom_theta

uint16 current_left_ma
uint16 current_right_ma
uint16 proximity_obstructed  # bits 6-9 = PC6-PC9 (Front/Rear/Left/Right)
uint8 led_mode               # indicator-ring animation: 0=pulse, 1=snake
uint8 led_indicator_cfg      # distance-reactive ring: bit0 base (0=off,1=white), bit1 spread (0=fixed,1=responsive)
