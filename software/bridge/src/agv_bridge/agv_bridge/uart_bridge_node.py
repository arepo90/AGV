"""UART bridge node — owns /dev/tty* to the ESP32-C3 and is the single source
of truth for the firmware protocol on the ROS side.

Inbound (firmware → ROS):
  * PKT_TLM_CORE    → /agv/core    + /agv/odom (nav_msgs/Odometry)
  * PKT_TLM_DRIVE   → /agv/drive
  * PKT_TLM_SENSORS → /agv/sensors + /agv/imu (sensor_msgs/Imu)
  * PKT_TLM_QTR     → /agv/qtr
  * PKT_LOG         → /agv/log
  * PKT_ACK/NACK    → resolves the matching pending command future

Outbound (ROS → firmware):
  * Topics: /agv/cmd_vel, /agv/heartbeat, /agv/param_update
  * Services: every PKT_CMD subtype + PKT_RESET variants

The serial reader runs in a dedicated thread (blocking pyserial). Writes are
serialized through _serial_lock. SEQ is a single 8-bit counter shared across
the node — wrap is fine since outstanding command count never approaches 256.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import serial

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

from agv_msgs.msg import (
    TlmCore, TlmDrive, TlmSensors, TlmQtr,
    LogEntry, ParamUpdate, ParamUpdateBatch,
)
from agv_msgs.srv import (
    SetMode, SetFunction, VirtualEstop, OverrideEstopSource, OverrideCaution,
    StartTare, ResetOdometry, QtrCalibrate, LoadTrajectory, LoadRampCurve,
    LogDump, LogClear, SoftReset, ClearAllEstop,
)

from . import frame
from . import protocol as proto


ACK_TIMEOUT_S = 0.5


@dataclass
class _PendingAck:
    event: threading.Event
    response_type: int = 0      # PKT_ACK or PKT_NACK
    nack_code: int = 0


class UartBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('uart_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('frame_id', 'base_link')

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(f'opening {port} @ {baud}')
        self._ser = serial.Serial(port, baudrate=baud, timeout=0.1)

        # ---- TX state ----------------------------------------------------
        self._serial_lock = threading.Lock()
        self._seq_lock = threading.Lock()
        self._seq = 0
        self._pending: dict[int, _PendingAck] = {}
        self._pending_lock = threading.Lock()

        # ---- RX state ----------------------------------------------------
        self._parser = frame.StreamingParser(self._on_frame, self._on_frame_error)
        self._stop = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop,
                                               name='uart-reader', daemon=True)

        # ---- Publishers --------------------------------------------------
        # Telemetry streams mirror the firmware packets 1:1; /agv/odom and
        # /agv/imu are standard ROS types derived from CORE/SENSORS for tooling.
        cb = ReentrantCallbackGroup()
        self._pub_core = self.create_publisher(TlmCore, '/agv/core', 50)
        self._pub_drive = self.create_publisher(TlmDrive, '/agv/drive', 50)
        self._pub_sensors = self.create_publisher(TlmSensors, '/agv/sensors', 20)
        self._pub_qtr = self.create_publisher(TlmQtr, '/agv/qtr', 50)
        self._pub_odom = self.create_publisher(Odometry, '/agv/odom', 50)
        self._pub_imu = self.create_publisher(Imu, '/agv/imu', 20)
        self._pub_log = self.create_publisher(LogEntry, '/agv/log', 100)

        # ---- Subscribers (ROS → UART) -----------------------------------
        self.create_subscription(Twist, '/agv/cmd_vel', self._on_cmd_vel, 10,
                                 callback_group=cb)
        self.create_subscription(Empty, '/agv/heartbeat', self._on_heartbeat, 10,
                                 callback_group=cb)
        self.create_subscription(ParamUpdateBatch, '/agv/param_update',
                                 self._on_param_update, 10, callback_group=cb)

        # ---- Services (ROS → UART, with ACK round-trip) -----------------
        self.create_service(SetMode, '/agv/set_mode', self._svc_set_mode, callback_group=cb)
        self.create_service(SetFunction, '/agv/set_function', self._svc_set_function, callback_group=cb)
        self.create_service(VirtualEstop, '/agv/virtual_estop', self._svc_virtual_estop, callback_group=cb)
        self.create_service(OverrideEstopSource, '/agv/override_estop_source',
                            self._svc_override_estop_source, callback_group=cb)
        self.create_service(OverrideCaution, '/agv/override_caution',
                            self._svc_override_caution, callback_group=cb)
        self.create_service(StartTare, '/agv/start_tare', self._svc_start_tare, callback_group=cb)
        self.create_service(ResetOdometry, '/agv/reset_odometry',
                            self._svc_reset_odometry, callback_group=cb)
        self.create_service(QtrCalibrate, '/agv/qtr_calibrate',
                            self._svc_qtr_calibrate, callback_group=cb)
        self.create_service(LoadTrajectory, '/agv/load_trajectory',
                            self._svc_load_trajectory, callback_group=cb)
        self.create_service(LoadRampCurve, '/agv/load_ramp_curve',
                            self._svc_load_ramp_curve, callback_group=cb)
        self.create_service(LogDump, '/agv/log_dump', self._svc_log_dump, callback_group=cb)
        self.create_service(LogClear, '/agv/log_clear', self._svc_log_clear, callback_group=cb)
        self.create_service(SoftReset, '/agv/soft_reset', self._svc_soft_reset, callback_group=cb)
        self.create_service(ClearAllEstop, '/agv/clear_all_estop',
                            self._svc_clear_all_estop, callback_group=cb)

        self._reader_thread.start()
        self.get_logger().info('uart_bridge ready')

    # =====================================================================
    # Serial I/O
    # =====================================================================

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'serial read failed: {e}')
                time.sleep(0.5)
                continue
            if data:
                self._parser.feed(data)

    def _next_seq(self) -> int:
        with self._seq_lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return s

    def _send_frame(self, ptype: int, payload: bytes, *, expect_ack: bool) -> Optional[_PendingAck]:
        seq = self._next_seq()
        f = frame.Frame(seq=seq, type=ptype, payload=payload)
        encoded = f.encode()

        pending: Optional[_PendingAck] = None
        if expect_ack:
            pending = _PendingAck(event=threading.Event())
            with self._pending_lock:
                self._pending[seq] = pending

        with self._serial_lock:
            try:
                self._ser.write(encoded)
            except serial.SerialException as e:
                self.get_logger().error(f'serial write failed: {e}')
                if pending is not None:
                    with self._pending_lock:
                        self._pending.pop(seq, None)
                return None

        return pending

    def _await(self, pending: Optional[_PendingAck], timeout: float = ACK_TIMEOUT_S) -> tuple[bool, int]:
        if pending is None:
            return False, 0
        ok = pending.event.wait(timeout=timeout)
        if not ok:
            return False, 0
        return pending.response_type == proto.PKT_ACK, pending.nack_code

    # =====================================================================
    # Inbound frame dispatch
    # =====================================================================

    def _on_frame(self, f: frame.Frame) -> None:
        if f.type == proto.PKT_TLM_CORE:
            self._handle_core(f.payload)
        elif f.type == proto.PKT_TLM_DRIVE:
            self._handle_drive(f.payload)
        elif f.type == proto.PKT_TLM_SENSORS:
            self._handle_sensors(f.payload)
        elif f.type == proto.PKT_TLM_QTR:
            self._handle_qtr(f.payload)
        elif f.type == proto.PKT_LOG:
            self._handle_log(f.payload)
        elif f.type in (proto.PKT_ACK, proto.PKT_NACK):
            self._handle_ack_nack(f)
        # Other inbound types (HEARTBEAT, CMD echo) aren't expected from firmware.

    def _on_frame_error(self, err: frame.FrameError, suspect_seq: int) -> None:
        self.get_logger().warning(f'frame error {err.name} seq={suspect_seq}')

    def _handle_ack_nack(self, f: frame.Frame) -> None:
        if len(f.payload) < 1:
            return
        echoed_seq = f.payload[0]
        code = f.payload[1] if len(f.payload) >= 2 else 0
        with self._pending_lock:
            pending = self._pending.pop(echoed_seq, None)
        if pending is None:
            return
        pending.response_type = f.type
        pending.nack_code = code
        pending.event.set()

    # =====================================================================
    # Telemetry fan-out
    # =====================================================================

    def _handle_core(self, payload: bytes) -> None:
        try:
            c = proto.TlmCore.from_bytes(payload)
        except ValueError as e:
            self.get_logger().warning(f'malformed core telemetry: {e}')
            return
        now = self.get_clock().now().to_msg()

        msg = TlmCore()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.timestamp_ms = c.timestamp_ms
        msg.mode = c.mode
        msg.function = c.function
        msg.estop_sources = c.estop_sources
        msg.caution_sources = c.caution_sources
        msg.caution_modifier = c.caution_modifier
        msg.flags = c.flags
        msg.velocity_linear = c.velocity_linear
        msg.velocity_angular = c.velocity_angular
        msg.odom_x = c.odom_x
        msg.odom_y = c.odom_y
        msg.odom_theta = c.odom_theta
        msg.current_left_ma = c.current_left_ma
        msg.current_right_ma = c.current_right_ma
        msg.proximity_obstructed = c.proximity_obstructed
        msg.led_mode = c.led_mode
        self._pub_core.publish(msg)

        # Standard ROS odometry for rviz/SLAM tooling.
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self._frame_id
        odom.pose.pose.position.x = float(c.odom_x)
        odom.pose.pose.position.y = float(c.odom_y)
        odom.pose.pose.orientation = _yaw_to_quat(c.odom_theta)
        odom.twist.twist.linear.x = float(c.velocity_linear)
        odom.twist.twist.angular.z = float(c.velocity_angular)
        self._pub_odom.publish(odom)

    def _handle_drive(self, payload: bytes) -> None:
        try:
            d = proto.TlmDrive.from_bytes(payload)
        except ValueError as e:
            self.get_logger().warning(f'malformed drive telemetry: {e}')
            return
        msg = TlmDrive()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.v_left_target = d.v_left_target
        msg.v_right_target = d.v_right_target
        msg.velocity_left = d.velocity_left
        msg.velocity_right = d.velocity_right
        msg.duty_left = d.duty_left
        msg.duty_right = d.duty_right
        msg.encoder_left_counts = d.encoder_left_counts
        msg.encoder_right_counts = d.encoder_right_counts
        self._pub_drive.publish(msg)

    def _handle_sensors(self, payload: bytes) -> None:
        try:
            s = proto.TlmSensors.from_bytes(payload)
        except ValueError as e:
            self.get_logger().warning(f'malformed sensors telemetry: {e}')
            return
        now = self.get_clock().now().to_msg()

        msg = TlmSensors()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.load_cells = list(s.load_cells)
        msg.imu_yaw_deg = s.imu_yaw_deg
        msg.imu_pitch_deg = s.imu_pitch_deg
        msg.imu_roll_deg = s.imu_roll_deg
        msg.imu_calib = s.imu_calib
        msg.tof_mm = list(s.tof_mm)
        msg.batt_3s_mv = s.batt_3s_mv
        msg.batt_6s_mv = s.batt_6s_mv
        self._pub_sensors.publish(msg)

        # Standard ROS IMU (Euler-derived yaw quaternion; covariances unknown).
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self._frame_id
        imu.orientation = _yaw_to_quat(math.radians(s.imu_yaw_deg))
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self._pub_imu.publish(imu)

    def _handle_qtr(self, payload: bytes) -> None:
        try:
            q = proto.TlmQtr.from_bytes(payload)
        except ValueError as e:
            self.get_logger().warning(f'malformed qtr telemetry: {e}')
            return
        msg = TlmQtr()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.raw = list(q.raw)
        msg.line_position = q.line_position
        self._pub_qtr.publish(msg)

    def _handle_log(self, payload: bytes) -> None:
        try:
            e = proto.LogEntry.from_bytes(payload)
        except ValueError as exc:
            self.get_logger().warning(f'malformed log entry: {exc}')
            return
        msg = LogEntry(
            timestamp_ms=e.timestamp_ms, code=e.code,
            severity=e.severity, module=e.module, data=e.data,
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub_log.publish(msg)

    # =====================================================================
    # ROS → UART (subscribers)
    # =====================================================================

    def _on_cmd_vel(self, msg: Twist) -> None:
        payload = bytes([proto.CMD_VEL_CMD]) + struct.pack(
            '<ff', float(msg.linear.x), float(msg.angular.z))
        # cmd_vel is high-rate; don't wait on ACK — fire-and-forget keeps the
        # control path lean. ACKs will still come back but the dispatcher will
        # find no pending entry and silently drop them.
        self._send_frame(proto.PKT_CMD, payload, expect_ack=False)

    def _on_heartbeat(self, _msg: Empty) -> None:
        self._send_frame(proto.PKT_HEARTBEAT, b'', expect_ack=False)

    def _on_param_update(self, msg: ParamUpdateBatch) -> None:
        payload = b''.join(
            bytes([p.param_id]) + struct.pack('<f', float(p.value))
            for p in msg.params
        )
        self._send_frame(proto.PKT_PARAM_UPDATE, payload, expect_ack=False)

    # =====================================================================
    # ROS → UART (services with ACK round-trip)
    # =====================================================================

    def _do_cmd(self, payload: bytes, response, ptype: int = proto.PKT_CMD):
        pending = self._send_frame(ptype, payload, expect_ack=True)
        ok, nack = self._await(pending)
        response.success = ok
        response.nack_code = nack
        return response

    def _svc_set_mode(self, request, response):
        return self._do_cmd(bytes([proto.CMD_SET_MODE, request.mode & 0xFF]), response)

    def _svc_set_function(self, request, response):
        return self._do_cmd(bytes([proto.CMD_SET_FUNCTION, request.function & 0xFF]), response)

    def _svc_virtual_estop(self, _request, response):
        return self._do_cmd(bytes([proto.CMD_VIRTUAL_ESTOP]), response)

    def _svc_override_estop_source(self, request, response):
        m = request.source_mask & 0xFFFF
        return self._do_cmd(
            bytes([proto.CMD_OVERRIDE_ESTOP_SOURCE, m & 0xFF, (m >> 8) & 0xFF]), response)

    def _svc_override_caution(self, request, response):
        return self._do_cmd(
            bytes([proto.CMD_OVERRIDE_CAUTION]) + struct.pack('<f', float(request.scalar)),
            response)

    def _svc_start_tare(self, _request, response):
        return self._do_cmd(bytes([proto.CMD_START_TARE]), response)

    def _svc_reset_odometry(self, _request, response):
        return self._do_cmd(bytes([proto.CMD_RESET_ODOMETRY]), response)

    def _svc_qtr_calibrate(self, request, response):
        return self._do_cmd(bytes([proto.CMD_QTR_CALIBRATE, request.op & 0xFF]), response)

    def _svc_load_trajectory(self, request, response):
        if request.op == 0:
            payload = bytes([proto.CMD_LOAD_TRAJECTORY, 0])
        elif request.op == 1:
            payload = bytes([proto.CMD_LOAD_TRAJECTORY, 1]) + struct.pack(
                '<ff', float(request.point.x), float(request.point.y))
        else:
            response.success = False
            response.nack_code = proto.NACK_UNKNOWN_SUBTYPE
            return response
        return self._do_cmd(payload, response)

    def _svc_load_ramp_curve(self, request, response):
        op = request.op
        if op in (0, 2, 3):
            payload = bytes([proto.CMD_LOAD_RAMP_CURVE, op])
        elif op == 1:
            payload = bytes([proto.CMD_LOAD_RAMP_CURVE, 1]) + struct.pack(
                '<ff', float(request.point.s), float(request.point.f))
        else:
            response.success = False
            response.nack_code = proto.NACK_UNKNOWN_SUBTYPE
            return response
        return self._do_cmd(payload, response)

    def _svc_log_dump(self, _request, response):
        return self._do_cmd(bytes([proto.CMD_LOG_DUMP_REQUEST]), response)

    def _svc_log_clear(self, _request, response):
        return self._do_cmd(bytes([proto.CMD_LOG_CLEAR]), response)

    def _svc_soft_reset(self, _request, response):
        return self._do_cmd(bytes([0x01]), response, ptype=proto.PKT_RESET)

    def _svc_clear_all_estop(self, _request, response):
        return self._do_cmd(bytes([0x00]), response, ptype=proto.PKT_RESET)

    # =====================================================================
    # Shutdown
    # =====================================================================

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass


def _yaw_to_quat(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UartBridgeNode()
    # Multi-threaded executor so service callbacks can block on ACK without
    # starving the rest of the node (subscriptions, telemetry callbacks).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
