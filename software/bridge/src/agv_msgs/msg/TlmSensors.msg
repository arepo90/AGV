# PKT_TLM_SENSORS — load cells + IMU orientation + TOF + battery (slow stream).
# Mirrors firmware telemetry.c send_sensors() exactly (41 bytes on the wire).
std_msgs/Header header

float32[4] load_cells       # FL, FR, RL, RR (kg)
float32 imu_gyro_bias_dps   # heading-KF gyro-Z bias estimate (deg/s); MPU6050 has no absolute yaw
float32 imu_pitch_deg       # accel+gyro complementary tilt (board frame)
float32 imu_roll_deg
uint8 imu_status            # bit0 present, bit1 has_data, bit2 bias_converged, bit3 zupt_active

uint16[4] tof_mm            # VL53L0X range per corner (Front, Rear, Left, Right); mm
uint16 batt_3s_mv           # 3S pack bus voltage (mV); 0 if absent
uint16 batt_6s_mv           # 6S pack bus voltage (mV); 0 if absent

uint16[] lidar_mm           # Jetson-segmented LaserScan distances echoed by the firmware
                            # (mm per angular interval); empty when no fresh LiDAR data
