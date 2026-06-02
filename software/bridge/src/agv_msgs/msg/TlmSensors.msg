# PKT_TLM_SENSORS — load cells + IMU orientation + TOF + battery (slow stream).
# Mirrors firmware telemetry.c send_sensors() exactly (41 bytes on the wire).
std_msgs/Header header

float32[4] load_cells       # FL, FR, RL, RR (kg)
float32 imu_yaw_deg
float32 imu_pitch_deg
float32 imu_roll_deg
uint8 imu_calib             # packed: (sys<<6) | (gyro<<4) | (accel<<2) | mag

uint16[4] tof_mm            # VL53L0X range per corner (Front, Rear, Left, Right); mm
uint16 batt_3s_mv           # 3S pack bus voltage (mV); 0 if absent
uint16 batt_6s_mv           # 6S pack bus voltage (mV); 0 if absent
