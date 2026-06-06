"""Protocol constants + telemetry/log codecs.

Mirrors firmware/STM32_new/src/app/proto.h and the telemetry stream packers in
telemetry.c. Telemetry is now four rate-grouped streams (CORE/DRIVE/SENSORS/QTR)
instead of one monolithic frame; each has its own packet type and codec here.
Field offsets/types must stay in lockstep with the firmware.
"""

import struct
from dataclasses import dataclass
from typing import Tuple

# ---- Packet types --------------------------------------------------------
PKT_CMD = 0x01
PKT_PARAM_UPDATE = 0x02
PKT_TLM_CORE = 0x03      # operational state + pose (fast)
PKT_HEARTBEAT = 0x04
PKT_LIDAR_SEGMENTS = 0x07  # WS/Jetson→STM32: segmented LaserScan distances (u16 mm[])
PKT_ACK = 0x05
PKT_NACK = 0x06
PKT_LOG = 0x08
PKT_TLM_DRIVE = 0x09     # per-wheel control internals
PKT_TLM_SENSORS = 0x0A   # load cells + IMU orientation
PKT_TLM_QTR = 0x0B       # QTR raw + line position
PKT_RESET = 0xFF

# ---- CMD subtypes --------------------------------------------------------
CMD_SET_FUNCTION = 0x01
CMD_SET_MODE = 0x02
CMD_VEL_CMD = 0x03
CMD_VIRTUAL_ESTOP = 0x04
CMD_OVERRIDE_ESTOP_SOURCE = 0x05
CMD_OVERRIDE_CAUTION = 0x06
CMD_READ_SENSOR = 0x07
CMD_LOAD_TRAJECTORY = 0x08
CMD_START_TARE = 0x09
CMD_LOG_DUMP_REQUEST = 0x0A
CMD_LOG_CLEAR = 0x0B
CMD_QTR_CALIBRATE = 0x0C
CMD_RESET_ODOMETRY = 0x0D
CMD_LOAD_RAMP_CURVE = 0x0E

# ---- NACK codes ----------------------------------------------------------
NACK_BAD_CRC = 0x01
NACK_BAD_LENGTH = 0x02
NACK_UNKNOWN_TYPE = 0x03
NACK_UNKNOWN_SUBTYPE = 0x04
NACK_UNKNOWN_PARAM = 0x05
NACK_ILLEGAL_TRANSITION = 0x06
NACK_BUSY = 0x07


# =============================================================================
#  Telemetry streams — each codec mirrors a packer in firmware telemetry.c
# =============================================================================

# PKT_TLM_CORE: u32 ts, u8 mode, u8 func, u16 estop, u16 caution, f32 modifier,
#               u8 flags, f32 v, f32 w, f32 x, f32 y, f32 theta,
#               u16 cur_l, u16 cur_r, u16 proximity, u8 led_mode,
#               u8 led_indicator_cfg  → 43 bytes
# estop/caution widened to 16 bits to carry the TOF + battery sources.
_CORE_FMT = '<IBBHHfBfffffHHHBB'
TLM_CORE_LEN = struct.calcsize(_CORE_FMT)
assert TLM_CORE_LEN == 43, TLM_CORE_LEN


@dataclass
class TlmCore:
    timestamp_ms: int
    mode: int
    function: int
    estop_sources: int
    caution_sources: int
    caution_modifier: float
    flags: int
    velocity_linear: float
    velocity_angular: float
    odom_x: float
    odom_y: float
    odom_theta: float
    current_left_ma: int
    current_right_ma: int
    proximity_obstructed: int
    led_mode: int
    led_indicator_cfg: int

    @classmethod
    def from_bytes(cls, payload: bytes) -> 'TlmCore':
        if len(payload) < TLM_CORE_LEN:
            raise ValueError(f'core payload {len(payload)} < {TLM_CORE_LEN}')
        return cls(*struct.unpack(_CORE_FMT, payload[:TLM_CORE_LEN]))

    def to_bytes(self) -> bytes:
        return struct.pack(_CORE_FMT,
                           self.timestamp_ms, self.mode, self.function,
                           self.estop_sources, self.caution_sources,
                           self.caution_modifier, self.flags,
                           self.velocity_linear, self.velocity_angular,
                           self.odom_x, self.odom_y, self.odom_theta,
                           self.current_left_ma, self.current_right_ma,
                           self.proximity_obstructed, self.led_mode,
                           self.led_indicator_cfg)


# PKT_TLM_DRIVE: f32 vl_tgt, vr_tgt, vl_meas, vr_meas, duty_l, duty_r,
#                u32 enc_l, u32 enc_r  → 32 bytes
_DRIVE_FMT = '<ffffffII'
TLM_DRIVE_LEN = struct.calcsize(_DRIVE_FMT)
assert TLM_DRIVE_LEN == 32, TLM_DRIVE_LEN


@dataclass
class TlmDrive:
    v_left_target: float
    v_right_target: float
    velocity_left: float
    velocity_right: float
    duty_left: float
    duty_right: float
    encoder_left_counts: int
    encoder_right_counts: int

    @classmethod
    def from_bytes(cls, payload: bytes) -> 'TlmDrive':
        if len(payload) < TLM_DRIVE_LEN:
            raise ValueError(f'drive payload {len(payload)} < {TLM_DRIVE_LEN}')
        return cls(*struct.unpack(_DRIVE_FMT, payload[:TLM_DRIVE_LEN]))

    def to_bytes(self) -> bytes:
        return struct.pack(_DRIVE_FMT,
                           self.v_left_target, self.v_right_target,
                           self.velocity_left, self.velocity_right,
                           self.duty_left, self.duty_right,
                           self.encoder_left_counts, self.encoder_right_counts)


# PKT_TLM_SENSORS: f32 load[4], f32 gyro_bias_dps, pitch, roll, u8 imu_status,
#                  u16 tof_mm[4] (F,R,L,R), u16 batt_3s_mv, u16 batt_6s_mv  → 41 fixed bytes,
#                  then a variable u16 LiDAR tail (one mm per angular interval; may be empty).
# MPU6050 (6-DOF, no magnetometer): no absolute yaw — the fused heading is in
# TLM_CORE (odom_theta). The former yaw slot now carries the heading-KF gyro-bias
# estimate (deg/s), and the former BNO055 calib byte is now imu_status bits:
#   bit0 present, bit1 has_data, bit2 bias_converged, bit3 zupt_active.
_SENSORS_FMT = '<4f3fB4HHH'
TLM_SENSORS_LEN = struct.calcsize(_SENSORS_FMT)
assert TLM_SENSORS_LEN == 41, TLM_SENSORS_LEN

IMU_STATUS_PRESENT        = 0x01
IMU_STATUS_HAS_DATA       = 0x02
IMU_STATUS_BIAS_CONVERGED = 0x04
IMU_STATUS_ZUPT_ACTIVE    = 0x08


@dataclass
class TlmSensors:
    load_cells: Tuple[float, float, float, float]
    imu_gyro_bias_dps: float
    imu_pitch_deg: float
    imu_roll_deg: float
    imu_status: int
    tof_mm: Tuple[int, int, int, int]
    batt_3s_mv: int
    batt_6s_mv: int
    lidar_mm: Tuple[int, ...] = ()

    @classmethod
    def from_bytes(cls, payload: bytes) -> 'TlmSensors':
        if len(payload) < TLM_SENSORS_LEN:
            raise ValueError(f'sensors payload {len(payload)} < {TLM_SENSORS_LEN}')
        (lc0, lc1, lc2, lc3, bias, p, r, status,
         t0, t1, t2, t3, b3, b6) = struct.unpack(_SENSORS_FMT, payload[:TLM_SENSORS_LEN])
        tail = payload[TLM_SENSORS_LEN:]
        n = len(tail) // 2
        lidar = struct.unpack(f'<{n}H', tail[:n * 2]) if n else ()
        return cls(load_cells=(lc0, lc1, lc2, lc3),
                   imu_gyro_bias_dps=bias, imu_pitch_deg=p, imu_roll_deg=r,
                   imu_status=status,
                   tof_mm=(t0, t1, t2, t3), batt_3s_mv=b3, batt_6s_mv=b6,
                   lidar_mm=tuple(lidar))

    def to_bytes(self) -> bytes:
        fixed = struct.pack(_SENSORS_FMT, *self.load_cells,
                            self.imu_gyro_bias_dps, self.imu_pitch_deg, self.imu_roll_deg,
                            self.imu_status, *self.tof_mm,
                            self.batt_3s_mv, self.batt_6s_mv)
        tail = struct.pack(f'<{len(self.lidar_mm)}H', *self.lidar_mm)
        return fixed + tail


def pack_lidar_segments(mm) -> bytes:
    """Pack LiDAR segment distances (mm) into the PKT_LIDAR_SEGMENTS payload:
    little-endian u16 per segment, clamped to the u16 range."""
    vals = [max(0, min(0xFFFF, int(round(v)))) for v in mm]
    return struct.pack(f'<{len(vals)}H', *vals)


# PKT_TLM_QTR: u16 raw[8], f32 line_position  → 20 bytes
_QTR_FMT = '<8Hf'
TLM_QTR_LEN = struct.calcsize(_QTR_FMT)
assert TLM_QTR_LEN == 20, TLM_QTR_LEN


@dataclass
class TlmQtr:
    raw: Tuple[int, ...]
    line_position: float

    @classmethod
    def from_bytes(cls, payload: bytes) -> 'TlmQtr':
        if len(payload) < TLM_QTR_LEN:
            raise ValueError(f'qtr payload {len(payload)} < {TLM_QTR_LEN}')
        vals = struct.unpack(_QTR_FMT, payload[:TLM_QTR_LEN])
        return cls(raw=vals[:8], line_position=vals[8])

    def to_bytes(self) -> bytes:
        return struct.pack(_QTR_FMT, *self.raw, self.line_position)


# =============================================================================
#  Fault log — unchanged 12-byte entry
# =============================================================================

_LOG_FMT = '<IHBBI'
LOG_ENTRY_LEN = struct.calcsize(_LOG_FMT)
assert LOG_ENTRY_LEN == 12


@dataclass
class LogEntry:
    timestamp_ms: int
    code: int
    severity: int
    module: int
    data: int

    @classmethod
    def from_bytes(cls, payload: bytes) -> 'LogEntry':
        if len(payload) < LOG_ENTRY_LEN:
            raise ValueError(f'log payload {len(payload)} < {LOG_ENTRY_LEN}')
        ts, code, sev, mod, data = struct.unpack(_LOG_FMT, payload[:LOG_ENTRY_LEN])
        return cls(timestamp_ms=ts, code=code, severity=sev, module=mod, data=data)

    def to_bytes(self) -> bytes:
        return struct.pack(_LOG_FMT, self.timestamp_ms, self.code,
                           self.severity, self.module, self.data)
