# Firmware fault log entry — 12 bytes on the wire.
std_msgs/Header header
uint32 timestamp_ms
uint16 code
uint8 severity              # 0=INFO, 1=WARN, 2=ERROR, 3=CRITICAL
uint8 module
uint32 data
