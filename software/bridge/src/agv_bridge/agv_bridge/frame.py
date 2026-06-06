"""Frame codec for the AGV wire protocol.

Mirrors firmware/STM32/src/comms/comms.c and firmware/ESP32/src/frame/frame.cpp.

Wire layout (8-byte overhead + 0..255 payload):

    MAG0 MAG1 VER SEQ TYPE LEN PAYLOAD... CRC_LO CRC_HI

CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection, no xorout) covers
[VER, SEQ, TYPE, LEN, PAYLOAD...].
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional

MAGIC0 = 0xAA
MAGIC1 = 0x56
VERSION = 0x03   # v3: +led_indicator_cfg in CORE, lidar tail in SENSORS, PKT_LIDAR_SEGMENTS
MAX_PAYLOAD = 255
FRAME_OVERHEAD = 8
MAX_FRAME = MAX_PAYLOAD + FRAME_OVERHEAD


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE — matches firmware crc.cpp byte-for-byte."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class FrameError(IntEnum):
    OK = 0
    BAD_MAGIC = 1
    BAD_VERSION = 2
    BAD_CRC = 3
    BAD_LENGTH = 4
    INCOMPLETE = 5


@dataclass
class Frame:
    seq: int
    type: int
    payload: bytes

    def __post_init__(self) -> None:
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError(f'payload {len(self.payload)} > MAX_PAYLOAD')

    def encode(self) -> bytes:
        seq = self.seq & 0xFF
        ptype = self.type & 0xFF
        length = len(self.payload)
        crc_input = bytes([VERSION, seq, ptype, length]) + self.payload
        crc = crc16_ccitt(crc_input)
        return bytes([MAGIC0, MAGIC1, VERSION, seq, ptype, length]) + self.payload + bytes([
            crc & 0xFF, (crc >> 8) & 0xFF,
        ])


def validate(buf: bytes) -> tuple[FrameError, Optional[Frame]]:
    """Validate a complete frame buffer (no partials)."""
    if len(buf) < FRAME_OVERHEAD:
        return FrameError.INCOMPLETE, None
    if buf[0] != MAGIC0 or buf[1] != MAGIC1:
        return FrameError.BAD_MAGIC, None
    if buf[2] != VERSION:
        return FrameError.BAD_VERSION, None
    length = buf[5]
    expected = FRAME_OVERHEAD + length
    if len(buf) != expected:
        return FrameError.BAD_LENGTH, None
    crc_input = buf[2:6 + length]
    expected_crc = crc16_ccitt(crc_input)
    rx_crc = buf[6 + length] | (buf[7 + length] << 8)
    if expected_crc != rx_crc:
        return FrameError.BAD_CRC, None
    return FrameError.OK, Frame(seq=buf[3], type=buf[4], payload=bytes(buf[6:6 + length]))


class StreamingParser:
    """Byte-at-a-time frame parser for UART RX. Mirrors firmware frame_parser_feed()."""

    def __init__(self,
                 on_frame: Callable[[Frame], None],
                 on_error: Optional[Callable[[FrameError, int], None]] = None) -> None:
        self._on_frame = on_frame
        self._on_error = on_error
        self._reset()

    def _reset(self) -> None:
        self._state = 0   # 0=M0 1=M1 2=VER 3=SEQ 4=TYPE 5=LEN 6=PAYLOAD 7=CRC_LO 8=CRC_HI
        self._buf = bytearray()
        self._len = 0
        self._payload_idx = 0
        self._seq = 0

    def feed(self, data: bytes) -> None:
        for b in data:
            self._feed_byte(b)

    def _feed_byte(self, b: int) -> None:
        s = self._state
        if s == 0:
            if b == MAGIC0:
                self._buf = bytearray([b])
                self._state = 1
        elif s == 1:
            if b == MAGIC1:
                self._buf.append(b)
                self._state = 2
            else:
                self._reset()
        elif s == 2:
            if b != VERSION:
                if self._on_error:
                    self._on_error(FrameError.BAD_VERSION, 0)
                self._reset()
                return
            self._buf.append(b)
            self._state = 3
        elif s == 3:
            self._seq = b
            self._buf.append(b)
            self._state = 4
        elif s == 4:
            self._buf.append(b)
            self._state = 5
        elif s == 5:
            self._len = b
            self._buf.append(b)
            self._payload_idx = 0
            self._state = 6 if b > 0 else 7
        elif s == 6:
            self._buf.append(b)
            self._payload_idx += 1
            if self._payload_idx >= self._len:
                self._state = 7
        elif s == 7:
            self._buf.append(b)
            self._state = 8
        elif s == 8:
            self._buf.append(b)
            err, frame = validate(bytes(self._buf))
            if err == FrameError.OK and frame is not None:
                self._on_frame(frame)
            elif self._on_error:
                self._on_error(err, self._seq)
            self._reset()
