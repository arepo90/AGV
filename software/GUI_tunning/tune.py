#!/usr/bin/env python3
"""
PID tuning GUI for the AGV — talks to the *production* firmware natively.

This drives the real firmware over the production binary protocol (VER 0x03), not
a separate bench rig. It plugs into the same USB-CDC link the ESP32 byte-pump
exposes, so you can tune the per-wheel velocity loop on the actual robot:

  * Gains   → PKT_PARAM_UPDATE with the per-wheel PI + feedforward ids
              (Kp/Ki/Kff, NOT Kp/Ki/Kd — the firmware has no derivative term).
  * Targets → mode SUPERVISED + function REMOTE_CONTROL, then CMD_VEL_CMD (v, ω).
              Per-wheel step targets are converted to a chassis (v, ω) so the
              firmware's kinematic split reproduces them (needs WHEEL_BASE_M to
              match the firmware).
  * Telemetry → the TLM_DRIVE stream (per-wheel target/measured velocity + duty),
              plus TLM_CORE for mode/function/E-STOP status.
  * Heartbeat → PKT_HEARTBEAT keeps the firmware's watchdog from degrading to
              UNSUPERVISED (which drops REMOTE_CONTROL to STANDBY) and then E-STOP.

Notes:
  * `duty ≈ Kff·v_target` at steady state — that feedforward is why duty has a
    floor even with Kp/Ki near zero. Zero Kff to see the loop without it.
  * TLM_DRIVE defaults to 20 Hz; raise TLM_DRIVE_HZ in firmware config.h for finer
    step-response resolution.
  * The chassis ramp shapes the target before the PI sees it; raise "Ramp accel"
    for crisper velocity steps.

Run: python3 tune.py [--port /dev/ttyACM0] [--baud 921600]
Deps: pyqtgraph, pyserial, PyQt5 (or PyQt6).
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

# Force pyqtgraph onto PyQt5 — PyQt6 strict enum namespaces (Qt.PenStyle.X,
# Qt.Key.X) would force every constant in this file to be qualified, and
# downgrading PyQt6 → PyQt5 isn't always available on the user's box.
os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

import serial
import serial.tools.list_ports
import pyqtgraph as pg
# Use pyqtgraph's Qt wrapper so we share whatever binding it picked (PyQt5,
# PyQt6, PySide2, PySide6). Otherwise enum namespaces (Qt.DashLine vs
# Qt.PenStyle.DashLine) and signal classes drift between bindings.
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
Qt = QtCore.Qt
QTimer = QtCore.QTimer
QKeyEvent = QtGui.QKeyEvent
QApplication = QtWidgets.QApplication
QCheckBox = QtWidgets.QCheckBox
QComboBox = QtWidgets.QComboBox
QDoubleSpinBox = QtWidgets.QDoubleSpinBox
QFrame = QtWidgets.QFrame
QGridLayout = QtWidgets.QGridLayout
QGroupBox = QtWidgets.QGroupBox
QHBoxLayout = QtWidgets.QHBoxLayout
QLabel = QtWidgets.QLabel
QMainWindow = QtWidgets.QMainWindow
QPushButton = QtWidgets.QPushButton
QVBoxLayout = QtWidgets.QVBoxLayout
QWidget = QtWidgets.QWidget

# ─── Production protocol constants (mirror firmware/STM32/src/app/proto/proto.h) ─

PROTO_MAGIC0  = 0xAA
PROTO_MAGIC1  = 0x56
PROTO_VERSION = 0x03            # v3: streamed telemetry + PI+FF gains

# Packet types
PKT_CMD          = 0x01
PKT_PARAM_UPDATE = 0x02
PKT_TLM_CORE     = 0x03
PKT_HEARTBEAT    = 0x04
PKT_ACK          = 0x05
PKT_NACK         = 0x06
PKT_TLM_DRIVE    = 0x09
PKT_RESET        = 0xFF

# CMD sub-types (first payload byte)
CMD_SET_FUNCTION = 0x01
CMD_SET_MODE     = 0x02
CMD_VEL_CMD      = 0x03         # f32 v, f32 ω

# PARAM ids — per-wheel velocity PI + feedforward, and ramp accel.
PARAM_MAX_LINEAR_ACCEL = 0x03
PARAM_LEFT_KP   = 0x10
PARAM_LEFT_KI   = 0x11
PARAM_LEFT_KFF  = 0x12
PARAM_RIGHT_KP  = 0x13
PARAM_RIGHT_KI  = 0x14
PARAM_RIGHT_KFF = 0x15

# Mode / function enum values
MODE_SUPERVISED     = 0x00
FUNC_STANDBY        = 0x00
FUNC_REMOTE_CONTROL = 0x01

# TLM_DRIVE payload (firmware telemetry.c send_drive): f32 vl_tgt, vr_tgt,
# vl_meas, vr_meas, duty_l, duty_r, u32 enc_l, enc_r  → 32 bytes.
DRIVE_STRUCT = struct.Struct('<ffffffII')
assert DRIVE_STRUCT.size == 32

# TLM_CORE: we only need a few leading fields — u32 ts, u8 mode, u8 func,
# u16 estop, u16 caution ... (43 bytes total). Parse the prefix only.
CORE_MIN_LEN = 12

# Replay buffer length in samples. At 20 Hz TLM_DRIVE that's ~50 s of history.
HISTORY_LEN = 1000

# Wheel base used to translate (v_L, v_R) step targets and joystick (v, ω) into a
# chassis (v, ω). MUST match firmware WHEEL_BASE_M for per-wheel steps to land.
WHEEL_BASE_M = 0.20


# ─── CRC16/CCITT-FALSE ───────────────────────────────────────────────────────

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def encode_frame(type_byte: int, payload: bytes = b'', seq: int = 0) -> bytes:
    body = bytes([PROTO_VERSION, seq & 0xFF, type_byte, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([PROTO_MAGIC0, PROTO_MAGIC1]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


# ─── Streaming frame parser (PC side, mirrors STM32 parser) ──────────────────

@dataclass
class Frame:
    type: int
    seq: int
    payload: bytes


class FrameParser:
    """Byte-at-a-time parser. Yields complete Frame objects via feed()."""

    S_MAGIC0, S_MAGIC1, S_VER, S_SEQ, S_TYPE, S_LEN, S_PAYLOAD, S_CRC_LO, S_CRC_HI = range(9)

    def __init__(self) -> None:
        self.state = self.S_MAGIC0
        self._ver = 0
        self._seq = 0
        self._type = 0
        self._len = 0
        self._payload = bytearray()
        self._crc_recv = 0

    def feed(self, data: bytes):
        for b in data:
            f = self._feed_byte(b)
            if f is not None:
                yield f

    def _feed_byte(self, b: int) -> Optional[Frame]:
        S = self
        if S.state == S.S_MAGIC0:
            if b == PROTO_MAGIC0: S.state = S.S_MAGIC1
        elif S.state == S.S_MAGIC1:
            if b == PROTO_MAGIC1:    S.state = S.S_VER
            elif b == PROTO_MAGIC0:  S.state = S.S_MAGIC1
            else:                    S.state = S.S_MAGIC0
        elif S.state == S.S_VER:
            S._ver = b
            if b != PROTO_VERSION:
                S.state = S.S_MAGIC0
            else:
                S.state = S.S_SEQ
        elif S.state == S.S_SEQ:    S._seq = b;  S.state = S.S_TYPE
        elif S.state == S.S_TYPE:   S._type = b; S.state = S.S_LEN
        elif S.state == S.S_LEN:
            S._len = b
            S._payload = bytearray()
            S.state = S.S_CRC_LO if b == 0 else S.S_PAYLOAD
        elif S.state == S.S_PAYLOAD:
            S._payload.append(b)
            if len(S._payload) >= S._len:
                S.state = S.S_CRC_LO
        elif S.state == S.S_CRC_LO:
            S._crc_recv = b
            S.state = S.S_CRC_HI
        elif S.state == S.S_CRC_HI:
            S._crc_recv |= b << 8
            body = bytes([S._ver, S._seq, S._type, S._len]) + bytes(S._payload)
            crc = crc16_ccitt(body)
            S.state = S.S_MAGIC0
            if crc == S._crc_recv:
                return Frame(S._type, S._seq, bytes(S._payload))
        return None


# ─── Serial connection ───────────────────────────────────────────────────────

class Link:
    def __init__(self) -> None:
        self.ser: Optional[serial.Serial] = None
        self.parser = FrameParser()
        self._seq = 0
        self._rx_bytes = 0
        self._rx_frames = 0

    def open(self, port: str, baud: int) -> Optional[str]:
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.05)
            self.parser = FrameParser()
            return None
        except serial.SerialException as e:
            self.ser = None
            return str(e)

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def poll(self):
        """Read whatever's pending; yield each completed Frame."""
        if not self.is_open():
            return
        try:
            data = self.ser.read(8192)
        except serial.SerialException:
            self.close()
            return
        if not data:
            return
        self._rx_bytes += len(data)
        for f in self.parser.feed(data):
            self._rx_frames += 1
            yield f

    def send(self, type_byte: int, payload: bytes = b'') -> None:
        if not self.is_open():
            return
        try:
            self.ser.write(encode_frame(type_byte, payload, self._seq))
        except serial.SerialException:
            self.close()
            return
        self._seq = (self._seq + 1) & 0xFF


# ─── Main window ─────────────────────────────────────────────────────────────

@dataclass
class JoystickState:
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    max_v: float = 0.40       # m/s when forward/backward fully pressed
    max_w: float = 1.50       # rad/s when left/right fully pressed


class MainWindow(QMainWindow):
    def __init__(self, default_port: str, baud: int) -> None:
        super().__init__()
        self.setWindowTitle('AGV PID Tuning (production protocol v3)')
        self.resize(1200, 800)

        self.link = Link()
        self.baud = baud
        self.t0 = time.monotonic()
        self.joy = JoystickState()
        self.telem_count = 0
        self.telem_count_t0 = time.monotonic()

        # Commanded chassis target and whether we've put the firmware in
        # REMOTE_CONTROL. Sent every beat tick (also serves as the heartbeat).
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.enabled = False

        self._build_ui(default_port)

        # Two timers: a fast one for serial drain + plot redraw, and a slower
        # one for heartbeat + joystick command updates so the firmware
        # watchdog never trips even when nothing else is being sent.
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll)
        self.poll_timer.start(20)        # 50 Hz UI / RX drain

        self.beat_timer = QTimer(self)
        self.beat_timer.timeout.connect(self._heartbeat_and_joy)
        self.beat_timer.start(50)        # 20 Hz joystick / heartbeat

    # ──────────────── UI construction ───────────────────────────────────────

    def _build_ui(self, default_port: str) -> None:
        central = QWidget(self)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        # Connection bar
        bar = QHBoxLayout()
        bar.addWidget(QLabel('Port:'))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self._refresh_ports(default_port)
        bar.addWidget(self.port_combo, 1)
        self.refresh_btn = QPushButton('Rescan')
        self.refresh_btn.clicked.connect(lambda: self._refresh_ports(self.port_combo.currentText()))
        bar.addWidget(self.refresh_btn)
        self.conn_btn = QPushButton('Connect')
        self.conn_btn.clicked.connect(self._toggle_connect)
        bar.addWidget(self.conn_btn)
        self.status_lbl = QLabel('● Disconnected')
        self.status_lbl.setStyleSheet('color: #b00;')
        bar.addWidget(self.status_lbl)
        bar.addStretch(1)
        self.rate_lbl = QLabel('Drive: —')
        bar.addWidget(self.rate_lbl)
        self.enabled_lbl = QLabel('Motors: OFF')
        self.enabled_lbl.setStyleSheet('color: #888;')
        bar.addWidget(self.enabled_lbl)
        root.addLayout(bar)

        # Gains + test signal row
        controls = QHBoxLayout()
        controls.addWidget(self._build_gain_box('Left wheel',  is_left=True))
        controls.addWidget(self._build_gain_box('Right wheel', is_left=False))
        controls.addWidget(self._build_test_box(), 1)
        root.addLayout(controls)

        # Action row
        actions = QHBoxLayout()
        self.enable_btn  = QPushButton('ENABLE (REMOTE_CONTROL)'); self.enable_btn.clicked.connect(lambda: self._send_enable(True))
        self.disable_btn = QPushButton('DISABLE (STANDBY)');       self.disable_btn.clicked.connect(lambda: self._send_enable(False))
        self.clear_btn   = QPushButton('Clear E-STOP');            self.clear_btn.clicked.connect(self._clear_estop)
        self.enable_btn.setStyleSheet('background:#1a7; color:white; font-weight:bold;')
        self.disable_btn.setStyleSheet('background:#a22; color:white; font-weight:bold;')
        for b in (self.enable_btn, self.disable_btn, self.clear_btn):
            actions.addWidget(b)
        actions.addStretch(1)
        root.addLayout(actions)

        # Plots
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('w')
        pg.setConfigOptions(antialias=True)

        pL = pg.mkPen((30, 100, 200), width=1.6)
        pL_dash = pg.mkPen((30, 100, 200), width=1.4, style=Qt.DashLine)
        pR = pg.mkPen((200, 80, 30), width=1.6)
        pR_dash = pg.mkPen((200, 80, 30), width=1.4, style=Qt.DashLine)

        def newplot(row: int) -> pg.PlotItem:
            p = self.plot_widget.addPlot(row=row, col=0)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.addLegend(offset=(-10, 10), labelTextSize='8pt')
            p.setLabel('bottom', 'time (s)')
            return p

        self.p_vel  = newplot(0); self.p_vel.setTitle('Velocity (m/s) — TLM_DRIVE target vs measured')
        self.p_err  = newplot(1); self.p_err.setTitle('Tracking error = target − measured (m/s)')
        self.p_duty = newplot(2); self.p_duty.setTitle('PWM duty [-1, +1]  (steady-state ≈ Kff × v_target)')
        # Link x-axes so panning one pans all.
        for p in (self.p_err, self.p_duty):
            p.setXLink(self.p_vel)

        self.c_vt_L = self.p_vel.plot([], [], name='L target',   pen=pL_dash)
        self.c_vm_L = self.p_vel.plot([], [], name='L measured', pen=pL)
        self.c_vt_R = self.p_vel.plot([], [], name='R target',   pen=pR_dash)
        self.c_vm_R = self.p_vel.plot([], [], name='R measured', pen=pR)

        self.c_e_L = self.p_err.plot([], [], name='L', pen=pL)
        self.c_e_R = self.p_err.plot([], [], name='R', pen=pR)

        self.c_d_L = self.p_duty.plot([], [], name='L', pen=pL)
        self.c_d_R = self.p_duty.plot([], [], name='R', pen=pR)

        root.addWidget(self.plot_widget, 1)

        # Rolling buffers
        self.t_buf   = deque(maxlen=HISTORY_LEN)
        self.vt_L = deque(maxlen=HISTORY_LEN); self.vm_L = deque(maxlen=HISTORY_LEN)
        self.vt_R = deque(maxlen=HISTORY_LEN); self.vm_R = deque(maxlen=HISTORY_LEN)
        self.e_L  = deque(maxlen=HISTORY_LEN); self.e_R  = deque(maxlen=HISTORY_LEN)
        self.d_L  = deque(maxlen=HISTORY_LEN); self.d_R  = deque(maxlen=HISTORY_LEN)

        self.setCentralWidget(central)

    def _build_gain_box(self, title: str, is_left: bool) -> QGroupBox:
        box = QGroupBox(title)
        lay = QGridLayout(box)

        # Production loop is PI + feedforward (no derivative). Defaults mirror
        # firmware config.h: Kp 0.5, Ki 2.0, Kff 1.0.
        kp  = QDoubleSpinBox(); kp.setDecimals(4);  kp.setRange(0.0, 50.0); kp.setSingleStep(0.05);  kp.setValue(0.5)
        ki  = QDoubleSpinBox(); ki.setDecimals(4);  ki.setRange(0.0, 50.0); ki.setSingleStep(0.05);  ki.setValue(2.0)
        kff = QDoubleSpinBox(); kff.setDecimals(4); kff.setRange(0.0, 5.0); kff.setSingleStep(0.05); kff.setValue(1.0)
        kff.setToolTip('Feedforward duty per m/s. Steady-state duty ≈ Kff × v_target — '
                       'this is the "duty floor" you see with Kp/Ki near zero.')

        lay.addWidget(QLabel('Kp:'),  0, 0); lay.addWidget(kp,  0, 1)
        lay.addWidget(QLabel('Ki:'),  1, 0); lay.addWidget(ki,  1, 1)
        lay.addWidget(QLabel('Kff:'), 2, 0); lay.addWidget(kff, 2, 1)

        # Auto-apply on change (debounced through editingFinished)
        for s in (kp, ki, kff):
            s.editingFinished.connect(self._maybe_apply_gains)

        if is_left:
            self.kp_L_spin = kp; self.ki_L_spin = ki; self.kff_L_spin = kff
        else:
            self.kp_R_spin = kp; self.ki_R_spin = ki; self.kff_R_spin = kff
        return box

    def _build_test_box(self) -> QGroupBox:
        box = QGroupBox('Test signal')
        lay = QGridLayout(box)

        # Step (per-wheel targets, converted to chassis v/ω for REMOTE_CONTROL)
        self.v_L_spin = QDoubleSpinBox(); self.v_L_spin.setDecimals(3); self.v_L_spin.setRange(-2.0, 2.0); self.v_L_spin.setSingleStep(0.05); self.v_L_spin.setValue(0.30)
        self.v_R_spin = QDoubleSpinBox(); self.v_R_spin.setDecimals(3); self.v_R_spin.setRange(-2.0, 2.0); self.v_R_spin.setSingleStep(0.05); self.v_R_spin.setValue(0.30)
        lay.addWidget(QLabel('v_L target (m/s):'), 0, 0); lay.addWidget(self.v_L_spin, 0, 1)
        lay.addWidget(QLabel('v_R target (m/s):'), 1, 0); lay.addWidget(self.v_R_spin, 1, 1)
        step_btn = QPushButton('Apply step'); step_btn.clicked.connect(self._send_step)
        zero_btn = QPushButton('Zero targets'); zero_btn.clicked.connect(self._send_zero)
        lay.addWidget(step_btn, 0, 2); lay.addWidget(zero_btn, 1, 2)

        # Ramp accel — the chassis slew limiter shapes the target before the PI
        # sees it; raise this for a crisper step. Mirrors firmware default 0.8.
        self.accel_spin = QDoubleSpinBox(); self.accel_spin.setDecimals(2); self.accel_spin.setRange(0.05, 20.0); self.accel_spin.setSingleStep(0.5); self.accel_spin.setValue(0.80)
        self.accel_spin.setToolTip('PARAM_MAX_LINEAR_ACCEL (m/s²). Raise for crisper velocity steps.')
        self.accel_spin.editingFinished.connect(self._apply_accel)
        lay.addWidget(QLabel('Ramp accel (m/s²):'), 2, 0); lay.addWidget(self.accel_spin, 2, 1)

        # Joystick
        sep = QFrame(); sep.setFrameShape(QFrame.HLine); sep.setFrameShadow(QFrame.Sunken)
        lay.addWidget(sep, 3, 0, 1, 3)

        self.joy_chk = QCheckBox('Joystick mode (W/A/S/D or arrows)')
        self.joy_chk.toggled.connect(self._joy_toggled)
        lay.addWidget(self.joy_chk, 4, 0, 1, 3)

        max_v = QDoubleSpinBox(); max_v.setDecimals(2); max_v.setRange(0.05, 2.0); max_v.setSingleStep(0.05); max_v.setValue(self.joy.max_v)
        max_w = QDoubleSpinBox(); max_w.setDecimals(2); max_w.setRange(0.05, 5.0); max_w.setSingleStep(0.1);  max_w.setValue(self.joy.max_w)
        max_v.valueChanged.connect(lambda v: setattr(self.joy, 'max_v', v))
        max_w.valueChanged.connect(lambda v: setattr(self.joy, 'max_w', v))
        lay.addWidget(QLabel('Joy max v:'),  5, 0); lay.addWidget(max_v, 5, 1)
        lay.addWidget(QLabel('Joy max ω:'),  6, 0); lay.addWidget(max_w, 6, 1)

        self.joy_status = QLabel('—')
        self.joy_status.setStyleSheet('color: #555;')
        lay.addWidget(self.joy_status, 5, 2, 2, 1)

        return box

    # ──────────────── Connection ────────────────────────────────────────────

    def _refresh_ports(self, prefer: str = '') -> None:
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if prefer and prefer not in ports:
            ports.insert(0, prefer)
        for p in ports:
            self.port_combo.addItem(p)
        if prefer:
            i = self.port_combo.findText(prefer)
            if i >= 0:
                self.port_combo.setCurrentIndex(i)
        self.port_combo.blockSignals(False)

    def _toggle_connect(self) -> None:
        if self.link.is_open():
            self.link.close()
            self._set_status(False)
            return
        port = self.port_combo.currentText().strip()
        if not port:
            self._set_status(False, 'no port selected')
            return
        err = self.link.open(port, self.baud)
        if err:
            self._set_status(False, err)
            return
        self._set_status(True)
        # Fresh session: reset plot time origin + buffers, push current UI gains
        # and ramp accel so the firmware matches what's on screen. Motors stay
        # OFF (STANDBY) until ENABLE is pressed.
        self.t0 = time.monotonic()
        for d in (self.t_buf, self.vt_L, self.vm_L, self.vt_R, self.vm_R,
                  self.e_L, self.e_R, self.d_L, self.d_R):
            d.clear()
        self.enabled = False
        self.cmd_v = self.cmd_w = 0.0
        self._maybe_apply_gains()
        self._apply_accel()

    def _set_status(self, connected: bool, msg: str = '') -> None:
        if connected:
            self.status_lbl.setText('● Connected')
            self.status_lbl.setStyleSheet('color: #0a0;')
            self.conn_btn.setText('Disconnect')
        else:
            tail = f' — {msg}' if msg else ''
            self.status_lbl.setText(f'● Disconnected{tail}')
            self.status_lbl.setStyleSheet('color: #b00;')
            self.conn_btn.setText('Connect')

    # ──────────────── Commands ──────────────────────────────────────────────

    def _maybe_apply_gains(self) -> None:
        # One PKT_PARAM_UPDATE batch of [u8 id][f32 value] tuples. The firmware
        # applies Kp/Ki as a pair and Kff independently, per wheel.
        pairs = [
            (PARAM_LEFT_KP,   self.kp_L_spin.value()),
            (PARAM_LEFT_KI,   self.ki_L_spin.value()),
            (PARAM_LEFT_KFF,  self.kff_L_spin.value()),
            (PARAM_RIGHT_KP,  self.kp_R_spin.value()),
            (PARAM_RIGHT_KI,  self.ki_R_spin.value()),
            (PARAM_RIGHT_KFF, self.kff_R_spin.value()),
        ]
        payload = b''.join(bytes([pid]) + struct.pack('<f', float(v)) for pid, v in pairs)
        self.link.send(PKT_PARAM_UPDATE, payload)

    def _apply_accel(self) -> None:
        self.link.send(PKT_PARAM_UPDATE,
                       bytes([PARAM_MAX_LINEAR_ACCEL]) + struct.pack('<f', float(self.accel_spin.value())))

    def _set_target_from_wheels(self, v_L: float, v_R: float) -> None:
        # Inverse of the firmware kinematic split: v_L/v_R → chassis (v, ω).
        self.cmd_v = 0.5 * (v_L + v_R)
        self.cmd_w = (v_R - v_L) / WHEEL_BASE_M

    def _send_step(self) -> None:
        self._set_target_from_wheels(float(self.v_L_spin.value()), float(self.v_R_spin.value()))
        if self.enabled:
            self.link.send(PKT_CMD, bytes([CMD_VEL_CMD]) + struct.pack('<ff', self.cmd_v, self.cmd_w))

    def _send_zero(self) -> None:
        self.v_L_spin.setValue(0.0)
        self.v_R_spin.setValue(0.0)
        self.cmd_v = self.cmd_w = 0.0
        self.link.send(PKT_CMD, bytes([CMD_VEL_CMD]) + struct.pack('<ff', 0.0, 0.0))

    def _send_enable(self, enabled: bool) -> None:
        if enabled:
            # Clear any latched E-STOP, take SUPERVISED, then REMOTE_CONTROL.
            self.link.send(PKT_RESET, bytes([0x00]))
            self.link.send(PKT_CMD, bytes([CMD_SET_MODE, MODE_SUPERVISED]))
            self.link.send(PKT_CMD, bytes([CMD_SET_FUNCTION, FUNC_REMOTE_CONTROL]))
            self.enabled = True
        else:
            self.link.send(PKT_CMD, bytes([CMD_SET_FUNCTION, FUNC_STANDBY]))
            self.enabled = False
            self.cmd_v = self.cmd_w = 0.0

    def _clear_estop(self) -> None:
        # PKT_RESET with 0x00 clears all E-STOP sources (0x01 would soft-reset).
        self.link.send(PKT_RESET, bytes([0x00]))

    # ──────────────── Joystick ──────────────────────────────────────────────

    def _joy_toggled(self, on: bool) -> None:
        # Reset key state when toggling so a stuck-down key from before
        # doesn't keep driving after re-enable.
        self.joy.forward = self.joy.backward = self.joy.left = self.joy.right = False
        if on:
            self.setFocus()
            self.joy_status.setText('Joystick on. WASD or ↑↓←→')
        else:
            self.joy_status.setText('—')

    def keyPressEvent(self, e: QKeyEvent) -> None:
        if not self.joy_chk.isChecked():
            super().keyPressEvent(e)
            return
        if e.isAutoRepeat():
            return
        k = e.key()
        if   k in (Qt.Key_W, Qt.Key_Up):    self.joy.forward  = True
        elif k in (Qt.Key_S, Qt.Key_Down):  self.joy.backward = True
        elif k in (Qt.Key_A, Qt.Key_Left):  self.joy.left     = True
        elif k in (Qt.Key_D, Qt.Key_Right): self.joy.right    = True
        elif k == Qt.Key_Space:
            self._send_zero()
        else:
            super().keyPressEvent(e); return
        self._update_joy_status()

    def keyReleaseEvent(self, e: QKeyEvent) -> None:
        if not self.joy_chk.isChecked():
            super().keyReleaseEvent(e)
            return
        if e.isAutoRepeat():
            return
        k = e.key()
        if   k in (Qt.Key_W, Qt.Key_Up):    self.joy.forward  = False
        elif k in (Qt.Key_S, Qt.Key_Down):  self.joy.backward = False
        elif k in (Qt.Key_A, Qt.Key_Left):  self.joy.left     = False
        elif k in (Qt.Key_D, Qt.Key_Right): self.joy.right    = False
        else:
            super().keyReleaseEvent(e); return
        self._update_joy_status()

    def _update_joy_status(self) -> None:
        v, w = self._joy_vw()
        self.joy_status.setText(f'v={v:+.2f}  ω={w:+.2f}')

    def _joy_vw(self) -> tuple[float, float]:
        v = (self.joy.max_v if self.joy.forward  else 0.0) \
          - (self.joy.max_v if self.joy.backward else 0.0)
        w = (self.joy.max_w if self.joy.left     else 0.0) \
          - (self.joy.max_w if self.joy.right    else 0.0)
        return v, w

    # ──────────────── Timers ────────────────────────────────────────────────

    def _poll(self) -> None:
        for frame in self.link.poll():
            if frame.type == PKT_TLM_DRIVE and len(frame.payload) >= DRIVE_STRUCT.size:
                self._on_drive(frame.payload)
            elif frame.type == PKT_TLM_CORE and len(frame.payload) >= CORE_MIN_LEN:
                self._on_core(frame.payload)
            # ACK/NACK and other streams are ignored.
        self._redraw()

        # Connection status: detect drops the poll caused
        if not self.link.is_open() and self.conn_btn.text() == 'Disconnect':
            self._set_status(False, 'link closed')

        # Drive-stream rate display
        now = time.monotonic()
        elapsed = now - self.telem_count_t0
        if elapsed >= 1.0:
            hz = self.telem_count / elapsed
            self.rate_lbl.setText(f'Drive: {hz:.0f} Hz')
            self.telem_count = 0
            self.telem_count_t0 = now

    def _heartbeat_and_joy(self) -> None:
        if not self.link.is_open():
            return
        if self.joy_chk.isChecked():
            self.cmd_v, self.cmd_w = self._joy_vw()
        if self.enabled:
            # The velocity command doubles as proof-of-life for the watchdog.
            self.link.send(PKT_CMD, bytes([CMD_VEL_CMD]) + struct.pack('<ff', self.cmd_v, self.cmd_w))
        else:
            self.link.send(PKT_HEARTBEAT)

    # ──────────────── Telemetry handling ────────────────────────────────────

    def _on_drive(self, payload: bytes) -> None:
        (vt_L, vt_R, vm_L, vm_R, d_L, d_R, _enc_L, _enc_R) = DRIVE_STRUCT.unpack(payload[:DRIVE_STRUCT.size])

        # TLM_DRIVE carries no timestamp; use PC arrival time since connect.
        t = time.monotonic() - self.t0
        self.telem_count += 1

        self.t_buf.append(t)
        self.vt_L.append(vt_L); self.vm_L.append(vm_L)
        self.vt_R.append(vt_R); self.vm_R.append(vm_R)
        self.e_L.append(vt_L - vm_L); self.e_R.append(vt_R - vm_R)
        self.d_L.append(d_L); self.d_R.append(d_R)

    def _on_core(self, payload: bytes) -> None:
        # u32 ts, u8 mode, u8 func, u16 estop, ...
        mode      = payload[4]
        function  = payload[5]
        estop     = payload[6] | (payload[7] << 8)
        if estop:
            self.enabled_lbl.setText('E-STOP active')
            self.enabled_lbl.setStyleSheet('color: #b00; font-weight: bold;')
        elif function == FUNC_REMOTE_CONTROL:
            sup = 'SUPERVISED' if mode == MODE_SUPERVISED else 'UNSUPERVISED'
            self.enabled_lbl.setText(f'Motors: REMOTE_CONTROL ({sup})')
            self.enabled_lbl.setStyleSheet('color: #0a0; font-weight: bold;')
        else:
            self.enabled_lbl.setText('Motors: OFF (STANDBY)')
            self.enabled_lbl.setStyleSheet('color: #888;')

    def _redraw(self) -> None:
        if not self.t_buf:
            return
        t = list(self.t_buf)
        self.c_vt_L.setData(t, list(self.vt_L))
        self.c_vm_L.setData(t, list(self.vm_L))
        self.c_vt_R.setData(t, list(self.vt_R))
        self.c_vm_R.setData(t, list(self.vm_R))
        self.c_e_L.setData(t, list(self.e_L))
        self.c_e_R.setData(t, list(self.e_R))
        self.c_d_L.setData(t, list(self.d_L))
        self.c_d_R.setData(t, list(self.d_R))

    # ──────────────── Shutdown ──────────────────────────────────────────────

    def closeEvent(self, e):
        # Best-effort: drop to STANDBY on graceful exit. If the link is already
        # dead the firmware's heartbeat watchdog will stop the motors anyway.
        try:
            self._send_enable(False)
        except Exception:
            pass
        self.link.close()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0',
                    help='serial port (default: /dev/ttyACM0; udev may expose /dev/ESP)')
    ap.add_argument('--baud', type=int, default=921600)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(args.port, args.baud)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
