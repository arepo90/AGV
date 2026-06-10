# PKT_TLM_SENSORS — load cells + battery + LiDAR echo (slow stream).
# Mirrors firmware telemetry.c send_sensors() exactly (18 fixed bytes on the wire).
std_msgs/Header header

float32[4] load_cells       # FL, FR, RL, RR (kg)

uint16 batt_3s_mv           # 3S pack bus voltage (mV); 0 if absent

uint16[] lidar_mm           # Jetson-segmented LaserScan distances echoed by the firmware
                            # (mm per angular interval); empty when no fresh LiDAR data
