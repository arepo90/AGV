"""LiDAR segmentation node.

Consumes a 2D ``sensor_msgs/LaserScan`` on ``/agv/point_cloud``, drops the angular
sector occluded by the robot body, bins the remaining beams into fixed angular
intervals (mean range per interval), and republishes the per-interval distances
in millimetres as ``agv_msgs/LidarSegments`` on ``/agv/lidar_segments``.

The segments are *not* acted on here: ``uart_bridge_node`` forwards them to the
STM32 (PKT_LIDAR_SEGMENTS), which applies the same distance-band caution/E-STOP
distance-band policy and echoes them back up in TLM_SENSORS for the LED ring.
Keeping the STM32 as the single safety authority is deliberate — see
``reference/architecture.md``.

The window/mask/bin geometry is parameterised; the defaults are placeholders. Scan
angles are normalised to [0, 360) before masking/binning, so windows that wrap
through +-180° can be expressed as a plain range (e.g. 90°-270° for the rear half).
Set ``fov_min_deg`` / ``fov_max_deg`` to the usable field of view (these map to the
two LiDAR LED indicator points, "0°" and "MAX_FOV°"), ``mask_min_deg`` /
``mask_max_deg`` to the occluded sector to ignore (no-op while min > max), and
``bin_deg`` to the interval width Z°.
"""

from __future__ import annotations

import math
from typing import List, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from agv_msgs.msg import LidarSegments


def segment_scan(angles_deg: Sequence[float], ranges: Sequence[float],
                 range_min: float, range_max: float,
                 fov_min_deg: float, fov_max_deg: float, bin_deg: float,
                 mask_min_deg: float, mask_max_deg: float,
                 clear_mm: int, max_segments: int) -> List[int]:
    """Mask + bin a LaserScan into per-interval distances (mm).

    Returns one entry per Z°-wide bin spanning [fov_min_deg, fov_max_deg), ordered
    from fov_min to fov_max. A bin with no valid beam (out of range, masked, inf/nan)
    reports ``clear_mm`` so the segment count stays stable and aligned with the LED
    arc. Pure function (no ROS types) so it is unit-testable.
    """
    if bin_deg <= 0.0 or fov_max_deg <= fov_min_deg:
        return []
    n_seg = int(round((fov_max_deg - fov_min_deg) / bin_deg))
    n_seg = max(0, min(n_seg, max_segments))
    if n_seg == 0:
        return []

    sums = [0.0] * n_seg
    counts = [0] * n_seg
    masked = mask_min_deg <= mask_max_deg   # else mask disabled

    for a, r in zip(angles_deg, ranges):
        if a < fov_min_deg or a >= fov_max_deg:
            continue
        if masked and mask_min_deg <= a <= mask_max_deg:
            continue
        if math.isinf(r) or math.isnan(r) or r <= 0.0:
            continue
        if r < range_min or r > range_max:
            continue
        k = int((a - fov_min_deg) / bin_deg)
        if 0 <= k < n_seg:
            sums[k] += r
            counts[k] += 1

    out: List[int] = []
    for k in range(n_seg):
        if counts[k] == 0:
            out.append(clear_mm)
        else:
            mm = (sums[k] / counts[k]) * 1000.0
            out.append(int(max(0, min(0xFFFF, round(mm)))))
    return out


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_node')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/agv/lidar_segments')
        # Usable FOV → the two LiDAR LED indicator points (0° and MAX_FOV°).
        # Angles are normalised to [0, 360); 90-270 is the rear half (wraps +-180°).
        self.declare_parameter('fov_min_deg', 90.0)
        self.declare_parameter('fov_max_deg', 270.0)
        self.declare_parameter('bin_deg', 15.0)            # Z° interval width
        # Occluded sector to drop (robot body). No-op while min > max — set later.
        self.declare_parameter('mask_min_deg', 1.0)        # X°
        self.declare_parameter('mask_max_deg', -1.0)       # Y°
        self.declare_parameter('clear_mm', 8000)           # matches firmware LIDAR_VALID_MAX_MM
        self.declare_parameter('max_segments', 32)         # matches firmware LIDAR_MAX_SEGMENTS

        g = self.get_parameter
        self._input = g('input_topic').value
        self._fov_min = float(g('fov_min_deg').value)
        self._fov_max = float(g('fov_max_deg').value)
        self._bin = float(g('bin_deg').value)
        self._mask_min = float(g('mask_min_deg').value)
        self._mask_max = float(g('mask_max_deg').value)
        self._clear_mm = int(g('clear_mm').value)
        self._max_seg = int(g('max_segments').value)

        self._pub = self.create_publisher(LidarSegments, g('output_topic').value, 10)
        self.create_subscription(LaserScan, self._input, self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'lidar_node: {self._input} → segments '
            f'(fov [{self._fov_min}, {self._fov_max}]°, bin {self._bin}°, '
            f'mask [{self._mask_min}, {self._mask_max}]°)')

    def _on_scan(self, scan: LaserScan) -> None:
        # Normalised to [0, 360) so fov/mask windows can span +-180°
        # (e.g. fov_min_deg=90, fov_max_deg=270 for the rear half).
        angles = [math.degrees(scan.angle_min + i * scan.angle_increment) % 360.0
                  for i in range(len(scan.ranges))]
        seg = segment_scan(angles, scan.ranges, scan.range_min, scan.range_max,
                           self._fov_min, self._fov_max, self._bin,
                           self._mask_min, self._mask_max,
                           self._clear_mm, self._max_seg)
        msg = LidarSegments()
        msg.header = scan.header
        msg.mm = seg
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
