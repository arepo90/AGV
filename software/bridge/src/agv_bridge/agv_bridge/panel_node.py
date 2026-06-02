"""Panel node — drives the Arduino UNO + 3.5" TFT status display.

A display-only, one-way feed: this node subscribes to the telemetry topics the
uart_bridge already publishes (/agv/core, /agv/sensors), derives battery
percentage from the raw pack voltages, and writes a compact, checksummed ASCII
line to the Arduino's USB serial port at a few Hz. The Arduino renders mode,
function, E-STOP/caution state, and battery charge.

Line format (newline-terminated), parsed field-by-field on the UNO:

    AGV,<mode>,<func>,<estop>,<caution>,<cm_x100>,<b3_mv>,<p3>,<b6_mv>,<p6>,<t0>,<t1>,<t2>,<t3>*<csum>\\n

  * estop/caution are 16-bit source bitmasks (0 = clear).
  * cm_x100 is the caution modifier ×100 (0..100).
  * b3/b6 are pack millivolts (0 = absent); p3/p6 are 0..100 % (-1 = absent).
  * t0..t3 are TOF ranges in mm (Front, Rear, Left, Right).
  * csum is the 8-bit XOR of every character between 'A' and '*', as two hex digits.

The UNO resets when the port opens (DTR), so the node tolerates open/write errors
and lazily reopens — the sketch re-initialises and the stream resumes.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import serial

from agv_msgs.msg import TlmCore, TlmSensors


# LiPo per-cell open-circuit voltage → state-of-charge (%), piecewise-linear.
# Approximate and shared in spirit with the GUI; good enough for a panel gauge.
_LIPO_CURVE = [
    (3.20, 0.0), (3.30, 5.0), (3.40, 10.0), (3.50, 20.0), (3.60, 35.0),
    (3.70, 50.0), (3.80, 60.0), (3.90, 70.0), (4.00, 80.0), (4.10, 90.0),
    (4.20, 100.0),
]


def _lipo_percent(pack_mv: int, cells: int) -> int:
    """State-of-charge for a pack of `cells` series cells. -1 if absent."""
    if pack_mv <= 0 or cells <= 0:
        return -1
    v = (pack_mv / 1000.0) / cells
    if v <= _LIPO_CURVE[0][0]:
        return 0
    if v >= _LIPO_CURVE[-1][0]:
        return 100
    for (v0, p0), (v1, p1) in zip(_LIPO_CURVE, _LIPO_CURVE[1:]):
        if v <= v1:
            return int(round(p0 + (p1 - p0) * (v - v0) / (v1 - v0)))
    return 100


def _checksum(body: str) -> int:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return c & 0xFF


class PanelNode(Node):
    def __init__(self) -> None:
        super().__init__('panel_node')

        self.declare_parameter('uno_port', '/dev/agv-uno')
        self.declare_parameter('uno_baud', 115200)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('cells_3s', 3)
        self.declare_parameter('cells_6s', 6)

        self._port = self.get_parameter('uno_port').value
        self._baud = int(self.get_parameter('uno_baud').value)
        self._cells_3s = int(self.get_parameter('cells_3s').value)
        self._cells_6s = int(self.get_parameter('cells_6s').value)
        rate = float(self.get_parameter('rate_hz').value)

        self._ser: serial.Serial | None = None
        self._core: TlmCore | None = None
        self._sensors: TlmSensors | None = None

        self.create_subscription(TlmCore, '/agv/core', self._on_core, 10)
        self.create_subscription(TlmSensors, '/agv/sensors', self._on_sensors, 10)
        self.create_timer(1.0 / max(rate, 0.5), self._tick)

        self.get_logger().info(f'panel_node → {self._port} @ {self._baud} ({rate:.1f} Hz)')

    def _on_core(self, msg: TlmCore) -> None:
        self._core = msg

    def _on_sensors(self, msg: TlmSensors) -> None:
        self._sensors = msg

    def _ensure_open(self) -> bool:
        if self._ser is not None and self._ser.is_open:
            return True
        try:
            # write_timeout keeps a stalled/unplugged UNO from blocking the node.
            self._ser = serial.Serial(self._port, self._baud, timeout=0, write_timeout=0.2)
            self.get_logger().info(f'opened {self._port}')
            return True
        except (serial.SerialException, OSError) as e:
            self._ser = None
            self.get_logger().warning(f'panel port unavailable ({e}); retrying', throttle_duration_sec=5.0)
            return False

    def _tick(self) -> None:
        if self._core is None:
            return
        c = self._core
        s = self._sensors

        b3 = s.batt_3s_mv if s else 0
        b6 = s.batt_6s_mv if s else 0
        tof = list(s.tof_mm) if s else [0, 0, 0, 0]
        p3 = _lipo_percent(b3, self._cells_3s)
        p6 = _lipo_percent(b6, self._cells_6s)
        cm = int(round(max(0.0, min(1.0, c.caution_modifier)) * 100))

        body = (f'AGV,{c.mode},{c.function},{c.estop_sources},{c.caution_sources},'
                f'{cm},{b3},{p3},{b6},{p6},'
                f'{tof[0]},{tof[1]},{tof[2]},{tof[3]}')
        line = f'{body}*{_checksum(body):02X}\n'

        if not self._ensure_open():
            return
        try:
            self._ser.write(line.encode('ascii'))
        except (serial.SerialException, OSError, serial.SerialTimeoutException) as e:
            self.get_logger().warning(f'panel write failed ({e}); will reopen')
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


def main() -> None:
    rclpy.init()
    node = PanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._ser is not None:
            try:
                node._ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
