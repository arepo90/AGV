# LiDAR distance segments — output of lidar_node, forwarded to the firmware as
# PKT_LIDAR_SEGMENTS. The 2D LaserScan is masked over a hardcoded angular sector
# and binned into fixed angular intervals; each entry is the average range of one
# interval in millimetres (LIDAR_VALID_MAX_MM sentinel for an empty/clear bin),
# ordered from the sector start angle to the end.
std_msgs/Header header

uint16[] mm
