# PKT_TLM_QTR — QTR-8A raw + computed line position (LINE_FOLLOW / calibration).
# Mirrors firmware telemetry.c send_qtr() exactly (20 bytes on the wire).
std_msgs/Header header

uint16[8] raw
float32 line_position       # firmware centroid, [-1, +1]
