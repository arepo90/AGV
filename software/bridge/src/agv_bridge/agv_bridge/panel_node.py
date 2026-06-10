"""Panel node — drives the Arduino UNO + 3.5" TFT status display ("Amber
Industrial" face). One-way, display-only.

Subscribes to /agv/core + /agv/sensors, derives battery %, and writes a compact
checksummed ASCII line to the UNO at a few Hz. This version EXTENDS the original
line with the data the new panel face needs: IR proximity, per-corner cargo
load, and a short human-readable reason string decoded from the E-STOP/caution
source masks (so the panel can say *why* it stopped without bit-decoding on the
AVR).

Line format (newline-terminated):

  AGV,<mode>,<func>,<estop>,<caution>,<cm>,<b3>,<p3>,<b6>,<p6>,
      <t0>,<t1>,<t2>,<t3>,<ir>,<c0>,<c1>,<c2>,<c3>,<reason>*<csum>

  * estop/caution: 16-bit source bitmasks (0 = clear).
  * cm: caution modifier x100 (0..100) = speed cap %.
  * bN: pack mV (0 = absent); pN: 0..100 % (-1 = absent).
  * t0..t3: retired TOF fields — always 0 (kept so the field count stays stable).
  * ir: IR-proximity bitmask (bit0 FL, bit1 FR, bit2 RL, bit3 RR).
  * c0..c3: corner load deci-kg (kg x10, FL, FR, RL, RR).
  * reason: short text, NO comma / '*' (e.g. "LIDAR CLOSE", "CARGO IMBAL").
  * csum: 8-bit XOR of every char between 'A' and '*', two hex digits.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

import serial

from agv_msgs.msg import TlmCore, TlmSensors


# LiPo per-cell open-circuit voltage -> state-of-charge (%), piecewise-linear.
_LIPO_CURVE = [
    (3.20, 0.0), (3.30, 5.0), (3.40, 10.0), (3.50, 20.0), (3.60, 35.0),
    (3.70, 50.0), (3.80, 60.0), (3.90, 70.0), (4.00, 80.0), (4.10, 90.0),
    (4.20, 100.0),
]

# ---- source-mask -> short reason text -------------------------------------
# Bit positions mirror firmware types.h (estop_source_t / caution_source_t).
# Order matters: the FIRST set bit found wins (highest priority first).
_ESTOP_BITS = [
    (9, "LIDAR CLOSE"),
    (8, "BATTERY LOW"),
    (0, "OBSTACLE IR"),
    (1, "CARGO OVERLOAD"),
    (2, "CARGO IMBAL"),
    (5, "OVERCURRENT"),
    (3, "HEARTBEAT"),
    (4, "WORKSTATION"),
    (6, "FIRMWARE FAULT"),
]
_CAUTION_BITS = [
    (7, "LIDAR NEAR"),
    (6, "BATTERY LOW"),
    (0, "CARGO OVERWEIGHT"),
    (1, "CARGO IMBAL"),
    (2, "UNSUPERVISED"),
    (3, "OBSTACLE NEAR"),
    (4, "WS OVERRIDE"),
]


def _reason(estop: int, caution: int) -> str:
    if estop:
        for bit, name in _ESTOP_BITS:
            if estop & (1 << bit):
                return name
        return "E-STOP"
    if caution:
        for bit, name in _CAUTION_BITS:
            if caution & (1 << bit):
                return name
        return "CAUTION"
    return "ALL SYSTEMS CLEAR"


def _lipo_percent(pack_mv: int, cells: int) -> int:
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
        # IR proximity bits inside TlmCore.proximity_obstructed (PC6..PC9 -> FL,FR,RL,RR)
        self.declare_parameter('ir_shift', 6)

        self._port = self.get_parameter('uno_port').value
        self._baud = int(self.get_parameter('uno_baud').value)
        self._cells_3s = int(self.get_parameter('cells_3s').value)
        self._cells_6s = int(self.get_parameter('cells_6s').value)
        self._ir_shift = int(self.get_parameter('ir_shift').value)
        rate = float(self.get_parameter('rate_hz').value)

        self._ser: serial.Serial | None = None
        self._core: TlmCore | None = None
        self._sensors: TlmSensors | None = None

        self.create_subscription(TlmCore, '/agv/core', self._on_core, 10)
        self.create_subscription(TlmSensors, '/agv/sensors', self._on_sensors, 10)
        self.create_timer(1.0 / max(rate, 0.5), self._tick)

        self.get_logger().info(f'panel_node -> {self._port} @ {self._baud} ({rate:.1f} Hz)')

    def _on_core(self, msg: TlmCore) -> None:
        self._core = msg

    def _on_sensors(self, msg: TlmSensors) -> None:
        self._sensors = msg

    def _ensure_open(self) -> bool:
        if self._ser is not None and self._ser.is_open:
            return True
        try:
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
        # TOF + 6S monitoring removed from the firmware; the UNO line format
        # keeps the fields for compatibility (it renders 0 / -1 as absent).
        b6 = 0
        tof = [0, 0, 0, 0]
        load = list(s.load_cells) if s else [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR kg
        p3 = _lipo_percent(b3, self._cells_3s)
        p6 = -1
        cm = int(round(max(0.0, min(1.0, c.caution_modifier)) * 100))

        # IR proximity: pull the 4 corner bits out of proximity_obstructed.
        ir = (int(c.proximity_obstructed) >> self._ir_shift) & 0x0F
        # cargo corners as deci-kg ints
        cg = [int(round(max(0.0, v) * 10)) for v in load[:4]]
        reason = _reason(int(c.estop_sources), int(c.caution_sources))

        body = (f'AGV,{c.mode},{c.function},{c.estop_sources},{c.caution_sources},'
                f'{cm},{b3},{p3},{b6},{p6},'
                f'{tof[0]},{tof[1]},{tof[2]},{tof[3]},{ir},'
                f'{cg[0]},{cg[1]},{cg[2]},{cg[3]},{reason}')
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
