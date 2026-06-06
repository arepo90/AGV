"""WebSocket bridge node — speaks the same binary frame protocol the GUI used
to talk to the ESP32 AP, but now over Wi-Fi to the Jetson. Translates frames
↔ ROS topics/services. Adding a second WS path (e.g. /pointcloud) later is a
one-block change in _ws_handler().

ROS integration runs on its own thread (MultiThreadedExecutor); the asyncio
loop owns the WebSocket server. Service calls from asyncio bridge into rclpy
via rclpy.task.Future → asyncio.Future adapters.
"""

from __future__ import annotations

import asyncio
import struct
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import websockets
from websockets.server import WebSocketServerProtocol

from geometry_msgs.msg import Twist
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
from agv_msgs.msg import TrajectoryPoint, RampPoint

from . import frame
from . import protocol as proto


SERVICE_CALL_TIMEOUT_S = 0.8


class WsBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('ws_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('path', '/ws')
        self._host = self.get_parameter('host').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self._path = self.get_parameter('path').get_parameter_value().string_value

        # Owned by the asyncio loop; populated when start_server() runs.
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_clients: set[WebSocketServerProtocol] = set()

        # Bridge-side SEQ counter for frames we originate (telemetry, ACK/NACK
        # echoes to the GUI). The GUI only correlates ACK SEQ for its outbound
        # frames, so it's safe to pick any wrapping counter here.
        self._telem_seq = 0
        self._telem_seq_lock = threading.Lock()

        # ROS publishers (commands going firmware-ward).
        self._pub_heartbeat = self.create_publisher(Empty, '/agv/heartbeat', 10)
        self._pub_param = self.create_publisher(ParamUpdateBatch, '/agv/param_update', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/agv/cmd_vel', 10)

        # ROS subscriptions (data going GUI-ward) — re-encode and broadcast.
        # Each telemetry stream is forwarded verbatim as its own frame type.
        self.create_subscription(TlmCore, '/agv/core', self._on_core, 50)
        self.create_subscription(TlmDrive, '/agv/drive', self._on_drive, 50)
        self.create_subscription(TlmSensors, '/agv/sensors', self._on_sensors, 20)
        self.create_subscription(TlmQtr, '/agv/qtr', self._on_qtr, 50)
        self.create_subscription(LogEntry, '/agv/log', self._on_log, 100)

        # ROS service clients (commands going firmware-ward, ACK round-trip).
        self._svc = {
            proto.CMD_SET_MODE: self.create_client(SetMode, '/agv/set_mode'),
            proto.CMD_SET_FUNCTION: self.create_client(SetFunction, '/agv/set_function'),
            proto.CMD_VIRTUAL_ESTOP: self.create_client(VirtualEstop, '/agv/virtual_estop'),
            proto.CMD_OVERRIDE_ESTOP_SOURCE: self.create_client(
                OverrideEstopSource, '/agv/override_estop_source'),
            proto.CMD_OVERRIDE_CAUTION: self.create_client(
                OverrideCaution, '/agv/override_caution'),
            proto.CMD_START_TARE: self.create_client(StartTare, '/agv/start_tare'),
            proto.CMD_RESET_ODOMETRY: self.create_client(ResetOdometry, '/agv/reset_odometry'),
            proto.CMD_QTR_CALIBRATE: self.create_client(QtrCalibrate, '/agv/qtr_calibrate'),
            proto.CMD_LOAD_TRAJECTORY: self.create_client(LoadTrajectory, '/agv/load_trajectory'),
            proto.CMD_LOAD_RAMP_CURVE: self.create_client(LoadRampCurve, '/agv/load_ramp_curve'),
            proto.CMD_LOG_DUMP_REQUEST: self.create_client(LogDump, '/agv/log_dump'),
            proto.CMD_LOG_CLEAR: self.create_client(LogClear, '/agv/log_clear'),
        }
        self._svc_soft_reset = self.create_client(SoftReset, '/agv/soft_reset')
        self._svc_clear_estop = self.create_client(ClearAllEstop, '/agv/clear_all_estop')

        self.get_logger().info(f'ws_bridge ready ({self._host}:{self._port}{self._path})')

    # =====================================================================
    # ROS → WS broadcast (subscriber callbacks run on rclpy executor thread)
    # =====================================================================

    def _broadcast(self, ptype: int, payload: bytes) -> None:
        loop = self._loop
        if loop is None:
            return
        # Use the bridge's own SEQ stream for STM32→WS frames. The GUI keeps
        # its own seq counter for inbound traffic and only correlates on ACK
        # SEQ (which we don't generate here), so any wrapping counter works.
        seq = self._next_telem_seq()
        encoded = frame.Frame(seq=seq, type=ptype, payload=payload).encode()
        loop.call_soon_threadsafe(self._enqueue_broadcast, encoded)

    def _next_telem_seq(self) -> int:
        with self._telem_seq_lock:
            s = self._telem_seq
            self._telem_seq = (s + 1) & 0xFF
            return s

    def _enqueue_broadcast(self, data: bytes) -> None:
        # Called on the asyncio loop; fire-and-forget send to all clients.
        for client in list(self._ws_clients):
            asyncio.ensure_future(self._safe_send(client, data))

    @staticmethod
    async def _safe_send(client: WebSocketServerProtocol, data: bytes) -> None:
        try:
            await client.send(data)
        except websockets.ConnectionClosed:
            pass

    def _on_core(self, msg: TlmCore) -> None:
        c = proto.TlmCore(
            timestamp_ms=msg.timestamp_ms,
            mode=msg.mode, function=msg.function,
            estop_sources=msg.estop_sources, caution_sources=msg.caution_sources,
            caution_modifier=msg.caution_modifier, flags=msg.flags,
            velocity_linear=msg.velocity_linear, velocity_angular=msg.velocity_angular,
            odom_x=msg.odom_x, odom_y=msg.odom_y, odom_theta=msg.odom_theta,
            current_left_ma=msg.current_left_ma, current_right_ma=msg.current_right_ma,
            proximity_obstructed=msg.proximity_obstructed, led_mode=msg.led_mode,
            led_indicator_cfg=msg.led_indicator_cfg,
        )
        self._broadcast(proto.PKT_TLM_CORE, c.to_bytes())

    def _on_drive(self, msg: TlmDrive) -> None:
        d = proto.TlmDrive(
            v_left_target=msg.v_left_target, v_right_target=msg.v_right_target,
            velocity_left=msg.velocity_left, velocity_right=msg.velocity_right,
            duty_left=msg.duty_left, duty_right=msg.duty_right,
            encoder_left_counts=msg.encoder_left_counts,
            encoder_right_counts=msg.encoder_right_counts,
        )
        self._broadcast(proto.PKT_TLM_DRIVE, d.to_bytes())

    def _on_sensors(self, msg: TlmSensors) -> None:
        s = proto.TlmSensors(
            load_cells=tuple(msg.load_cells),
            imu_gyro_bias_dps=msg.imu_gyro_bias_dps, imu_pitch_deg=msg.imu_pitch_deg,
            imu_roll_deg=msg.imu_roll_deg, imu_status=msg.imu_status,
            tof_mm=tuple(msg.tof_mm),
            batt_3s_mv=msg.batt_3s_mv, batt_6s_mv=msg.batt_6s_mv,
            lidar_mm=tuple(msg.lidar_mm),
        )
        self._broadcast(proto.PKT_TLM_SENSORS, s.to_bytes())

    def _on_qtr(self, msg: TlmQtr) -> None:
        q = proto.TlmQtr(raw=tuple(msg.raw), line_position=msg.line_position)
        self._broadcast(proto.PKT_TLM_QTR, q.to_bytes())

    def _on_log(self, msg: LogEntry) -> None:
        e = proto.LogEntry(timestamp_ms=msg.timestamp_ms, code=msg.code,
                           severity=msg.severity, module=msg.module, data=msg.data)
        self._broadcast(proto.PKT_LOG, e.to_bytes())

    # =====================================================================
    # WS → ROS (incoming frames)
    # =====================================================================

    async def _handle_frame(self, ws: WebSocketServerProtocol, f: frame.Frame) -> None:
        if f.type == proto.PKT_HEARTBEAT:
            # Fire-and-forget toward firmware (uart_bridge subscribes), ACK
            # the GUI locally — see design notes in module docstring.
            self._pub_heartbeat.publish(Empty())
            await self._send_ack(ws, f.seq, 0)
            return

        if f.type == proto.PKT_PARAM_UPDATE:
            params = _parse_param_payload(f.payload)
            if params is None:
                await self._send_nack(ws, f.seq, proto.NACK_BAD_LENGTH)
                return
            batch = ParamUpdateBatch(params=[
                ParamUpdate(param_id=pid, value=val) for pid, val in params
            ])
            self._pub_param.publish(batch)
            # uart_bridge fires PKT_PARAM_UPDATE to firmware fire-and-forget;
            # we can't synchronously know the firmware ACK. Optimistic ACK is
            # consistent with the heartbeat path and keeps the GUI's flow.
            await self._send_ack(ws, f.seq, 0)
            return

        if f.type == proto.PKT_CMD:
            await self._dispatch_cmd(ws, f)
            return

        if f.type == proto.PKT_RESET:
            mode = f.payload[0] if len(f.payload) > 0 else 0
            client = self._svc_soft_reset if mode == 1 else self._svc_clear_estop
            ok, nack = await self._call_service(client, _empty_request_for(client))
            await self._reply(ws, f.seq, ok, nack)
            return

        await self._send_nack(ws, f.seq, proto.NACK_UNKNOWN_TYPE)

    async def _dispatch_cmd(self, ws: WebSocketServerProtocol, f: frame.Frame) -> None:
        if len(f.payload) < 1:
            await self._send_nack(ws, f.seq, proto.NACK_BAD_LENGTH)
            return
        sub = f.payload[0]
        body = f.payload[1:]

        # CMD_VEL_CMD is the high-rate path. ACK is not awaited — we publish
        # to /agv/cmd_vel and ACK the GUI immediately so the wireless link
        # round-trip matches the original ESP32 behavior.
        if sub == proto.CMD_VEL_CMD:
            if len(body) < 8:
                await self._send_nack(ws, f.seq, proto.NACK_BAD_LENGTH)
                return
            linear, angular = struct.unpack('<ff', body[:8])
            t = Twist()
            t.linear.x = float(linear)
            t.angular.z = float(angular)
            self._pub_cmd_vel.publish(t)
            await self._send_ack(ws, f.seq, 0)
            return

        client = self._svc.get(sub)
        if client is None:
            await self._send_nack(ws, f.seq, proto.NACK_UNKNOWN_SUBTYPE)
            return

        request = _build_request_for_subtype(sub, body)
        if request is None:
            await self._send_nack(ws, f.seq, proto.NACK_BAD_LENGTH)
            return

        ok, nack = await self._call_service(client, request)
        await self._reply(ws, f.seq, ok, nack)

    # =====================================================================
    # asyncio ↔ rclpy service bridge
    # =====================================================================

    async def _call_service(self, client, request) -> tuple[bool, int]:
        if not client.service_is_ready():
            ready = client.wait_for_service(timeout_sec=0.0)
            if not ready:
                return False, proto.NACK_BUSY
        rclpy_future = client.call_async(request)
        aio_future: asyncio.Future = asyncio.get_running_loop().create_future()

        def _done(_fut, loop=asyncio.get_running_loop(), aio=aio_future) -> None:
            try:
                result = _fut.result()
                loop.call_soon_threadsafe(aio.set_result, result)
            except Exception as exc:  # pragma: no cover — defensive
                loop.call_soon_threadsafe(aio.set_exception, exc)

        rclpy_future.add_done_callback(_done)
        try:
            result = await asyncio.wait_for(aio_future, timeout=SERVICE_CALL_TIMEOUT_S)
        except asyncio.TimeoutError:
            return False, proto.NACK_BUSY
        return bool(result.success), int(getattr(result, 'nack_code', 0))

    async def _reply(self, ws: WebSocketServerProtocol, seq: int, ok: bool, nack: int) -> None:
        if ok:
            await self._send_ack(ws, seq, 0)
        else:
            await self._send_nack(ws, seq, nack or proto.NACK_BUSY)

    async def _send_ack(self, ws: WebSocketServerProtocol, seq: int, status: int) -> None:
        # Per firmware comms_send_ack(): payload = [echoed_seq, status]
        f = frame.Frame(seq=self._next_telem_seq(), type=proto.PKT_ACK,
                        payload=bytes([seq, status]))
        await self._safe_send(ws, f.encode())

    async def _send_nack(self, ws: WebSocketServerProtocol, seq: int, code: int) -> None:
        f = frame.Frame(seq=self._next_telem_seq(), type=proto.PKT_NACK,
                        payload=bytes([seq, code]))
        await self._safe_send(ws, f.encode())

    # =====================================================================
    # WebSocket server
    # =====================================================================

    async def _ws_handler(self, ws: WebSocketServerProtocol) -> None:
        # Path is exposed differently across websockets versions; this works
        # on both 10.x (ws.path) and 12+ (ws.request.path).
        client_path = getattr(ws, 'path', None) or getattr(
            getattr(ws, 'request', None), 'path', None) or '/'
        if client_path != self._path:
            # Future LiDAR / point-cloud channel goes here as another branch.
            await ws.close(code=1008, reason='unknown path')
            return
        self.get_logger().info(f'ws client connected: {ws.remote_address}')
        self._ws_clients.add(ws)
        try:
            async for message in ws:
                if not isinstance(message, (bytes, bytearray)):
                    continue
                err, parsed = frame.validate(bytes(message))
                if err != frame.FrameError.OK or parsed is None:
                    # No SEQ to echo if the frame is unparseable.
                    continue
                await self._handle_frame(ws, parsed)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._ws_clients.discard(ws)
            self.get_logger().info(f'ws client gone: {ws.remote_address}')

    async def run_server(self) -> None:
        self._loop = asyncio.get_running_loop()
        async with websockets.serve(self._ws_handler, self._host, self._port,
                                    max_size=2**20):
            await asyncio.Future()  # run forever


# =============================================================================
# Service-request construction (sub-type → request object)
# =============================================================================

def _build_request_for_subtype(sub: int, body: bytes):
    if sub == proto.CMD_SET_MODE:
        if len(body) < 1:
            return None
        req = SetMode.Request()
        req.mode = body[0]
        return req
    if sub == proto.CMD_SET_FUNCTION:
        if len(body) < 1:
            return None
        req = SetFunction.Request()
        req.function = body[0]
        return req
    if sub == proto.CMD_VIRTUAL_ESTOP:
        return VirtualEstop.Request()
    if sub == proto.CMD_OVERRIDE_ESTOP_SOURCE:
        if len(body) < 1:
            return None
        req = OverrideEstopSource.Request()
        req.source_mask = body[0]
        return req
    if sub == proto.CMD_OVERRIDE_CAUTION:
        if len(body) < 4:
            return None
        req = OverrideCaution.Request()
        (req.scalar,) = struct.unpack('<f', body[:4])
        return req
    if sub == proto.CMD_START_TARE:
        return StartTare.Request()
    if sub == proto.CMD_RESET_ODOMETRY:
        return ResetOdometry.Request()
    if sub == proto.CMD_QTR_CALIBRATE:
        if len(body) < 1:
            return None
        req = QtrCalibrate.Request()
        req.op = body[0]
        return req
    if sub == proto.CMD_LOAD_TRAJECTORY:
        if len(body) < 1:
            return None
        op = body[0]
        req = LoadTrajectory.Request()
        req.op = op
        if op == 1:
            if len(body) < 1 + 8:
                return None
            x, y = struct.unpack('<ff', body[1:9])
            req.point = TrajectoryPoint(x=float(x), y=float(y))
        return req
    if sub == proto.CMD_LOAD_RAMP_CURVE:
        if len(body) < 1:
            return None
        op = body[0]
        req = LoadRampCurve.Request()
        req.op = op
        if op == 1:
            if len(body) < 1 + 8:
                return None
            s, fval = struct.unpack('<ff', body[1:9])
            req.point = RampPoint(s=float(s), f=float(fval))
        return req
    if sub == proto.CMD_LOG_DUMP_REQUEST:
        return LogDump.Request()
    if sub == proto.CMD_LOG_CLEAR:
        return LogClear.Request()
    return None


def _empty_request_for(client):
    return client.srv_type.Request()


def _parse_param_payload(payload: bytes) -> Optional[list[tuple[int, float]]]:
    if len(payload) % 5 != 0:
        return None
    out: list[tuple[int, float]] = []
    for i in range(0, len(payload), 5):
        pid = payload[i]
        (val,) = struct.unpack('<f', payload[i + 1:i + 5])
        out.append((pid, val))
    return out


# =============================================================================
# Entry point — rclpy on a thread, asyncio on the main thread.
# =============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WsBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, name='rclpy-spin', daemon=True)
    spin_thread.start()

    try:
        asyncio.run(node.run_server())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
