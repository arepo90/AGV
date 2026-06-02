#!/usr/bin/env python3
"""
PID tuning GUI for the AGV bench rig.

Talks to the STM32_tunning firmware over the same binary protocol used in
production — only TYPE bytes 0x10 (telemetry) and 0x11 (command) are used,
so the existing ESP32 USB-CDC byte pump works unchanged.

Plots per-wheel target/measured velocity, error, PID integrator state, and
PWM duty in real time. Live-editable kp/ki/kd gains per wheel. Step commands
and WASD/arrow-key joystick.

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

# ─── Protocol constants (mirror STM32_tunning/src/config.h) ──────────────────

PROTO_MAGIC0   = 0xAA
PROTO_MAGIC1   = 0x56
PROTO_VERSION  = 0x01

PKT_TUNE_TELEM = 0x10
PKT_TUNE_CMD   = 0x11

TCMD_SET_TARGETS    = 0x01
TCMD_SET_GAINS      = 0x02
TCMD_ENABLE         = 0x03
TCMD_RESET_ENCODERS = 0x04
TCMD_RESET_PID      = 0x05
TCMD_HEARTBEAT      = 0x06

# Telemetry payload layout — must match send_telemetry() in main.c.
# u32 timestamp, u8 enabled, u8 reserved, u16 pad,
# 5x f32 left (v_target, v_meas, error, integral, duty),
# 5x f32 right,
# 6x f32 gains (kpL, kiL, kdL, kpR, kiR, kdR) = 4+1+1+2 + 20 + 20 + 24 = 72 bytes
TELEM_STRUCT = struct.Struct('<I B B H 5f 5f 6f')
assert TELEM_STRUCT.size == 72

# Replay buffer length in samples. At 100 Hz telemetry that's 10 s of history.
HISTORY_LEN = 1000

# Wheel geometry used only to translate joystick (v, ω) into per-wheel targets.
# Doesn't need to match the firmware exactly — change here if your rig differs.
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
        self.setWindowTitle('AGV PID Tuning')
        self.resize(1200, 800)

        self.link = Link()
        self.baud = baud
        self.t0 = time.monotonic()
        self.joy = JoystickState()
        self.last_telem_t = 0.0
        self.telem_count = 0
        self.telem_count_t0 = time.monotonic()

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
        self.rate_lbl = QLabel('Telem: —')
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
        self.enable_btn  = QPushButton('ENABLE motors');   self.enable_btn.clicked.connect(lambda: self._send_enable(True))
        self.disable_btn = QPushButton('DISABLE motors');  self.disable_btn.clicked.connect(lambda: self._send_enable(False))
        self.reset_pid_btn = QPushButton('Reset PID');              self.reset_pid_btn.clicked.connect(lambda: self.link.send(PKT_TUNE_CMD, bytes([TCMD_RESET_PID])))
        self.reset_enc_btn = QPushButton('Reset encoders');         self.reset_enc_btn.clicked.connect(lambda: self.link.send(PKT_TUNE_CMD, bytes([TCMD_RESET_ENCODERS])))
        self.enable_btn.setStyleSheet('background:#1a7; color:white; font-weight:bold;')
        self.disable_btn.setStyleSheet('background:#a22; color:white; font-weight:bold;')
        for b in (self.enable_btn, self.disable_btn, self.reset_pid_btn, self.reset_enc_btn):
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

        self.p_vel  = newplot(0); self.p_vel.setTitle('Velocity (m/s)')
        self.p_err  = newplot(1); self.p_err.setTitle('Tracking error = target − measured (m/s)')
        self.p_int  = newplot(2); self.p_int.setTitle('PID integrator state ∫e dt (i-contribution = ki × this)')
        self.p_duty = newplot(3); self.p_duty.setTitle('PWM duty [-1, +1]')
        # Link x-axes so panning one pans all.
        for p in (self.p_err, self.p_int, self.p_duty):
            p.setXLink(self.p_vel)

        self.c_vt_L = self.p_vel.plot([], [], name='L target',   pen=pL_dash)
        self.c_vm_L = self.p_vel.plot([], [], name='L measured', pen=pL)
        self.c_vt_R = self.p_vel.plot([], [], name='R target',   pen=pR_dash)
        self.c_vm_R = self.p_vel.plot([], [], name='R measured', pen=pR)

        self.c_e_L = self.p_err.plot([], [], name='L', pen=pL)
        self.c_e_R = self.p_err.plot([], [], name='R', pen=pR)

        self.c_i_L = self.p_int.plot([], [], name='L', pen=pL)
        self.c_i_R = self.p_int.plot([], [], name='R', pen=pR)

        self.c_d_L = self.p_duty.plot([], [], name='L', pen=pL)
        self.c_d_R = self.p_duty.plot([], [], name='R', pen=pR)

        root.addWidget(self.plot_widget, 1)

        # Rolling buffers
        self.t_buf   = deque(maxlen=HISTORY_LEN)
        self.vt_L = deque(maxlen=HISTORY_LEN); self.vm_L = deque(maxlen=HISTORY_LEN)
        self.vt_R = deque(maxlen=HISTORY_LEN); self.vm_R = deque(maxlen=HISTORY_LEN)
        self.e_L  = deque(maxlen=HISTORY_LEN); self.e_R  = deque(maxlen=HISTORY_LEN)
        self.i_L  = deque(maxlen=HISTORY_LEN); self.i_R  = deque(maxlen=HISTORY_LEN)
        self.d_L  = deque(maxlen=HISTORY_LEN); self.d_R  = deque(maxlen=HISTORY_LEN)

        self.setCentralWidget(central)

    def _build_gain_box(self, title: str, is_left: bool) -> QGroupBox:
        box = QGroupBox(title)
        lay = QGridLayout(box)

        kp = QDoubleSpinBox(); kp.setDecimals(4); kp.setRange(0.0, 50.0); kp.setSingleStep(0.05); kp.setValue(1.0)
        ki = QDoubleSpinBox(); ki.setDecimals(4); ki.setRange(0.0, 50.0); ki.setSingleStep(0.05); ki.setValue(0.0)
        kd = QDoubleSpinBox(); kd.setDecimals(4); kd.setRange(0.0, 50.0); kd.setSingleStep(0.005); kd.setValue(0.0)

        lay.addWidget(QLabel('Kp:'), 0, 0); lay.addWidget(kp, 0, 1)
        lay.addWidget(QLabel('Ki:'), 1, 0); lay.addWidget(ki, 1, 1)
        lay.addWidget(QLabel('Kd:'), 2, 0); lay.addWidget(kd, 2, 1)

        # Auto-apply on change (debounced through editingFinished)
        for s in (kp, ki, kd):
            s.editingFinished.connect(self._maybe_apply_gains)

        if is_left:
            self.kp_L_spin = kp; self.ki_L_spin = ki; self.kd_L_spin = kd
        else:
            self.kp_R_spin = kp; self.ki_R_spin = ki; self.kd_R_spin = kd
        return box

    def _build_test_box(self) -> QGroupBox:
        box = QGroupBox('Test signal')
        lay = QGridLayout(box)

        # Step
        self.v_L_spin = QDoubleSpinBox(); self.v_L_spin.setDecimals(3); self.v_L_spin.setRange(-2.0, 2.0); self.v_L_spin.setSingleStep(0.05); self.v_L_spin.setValue(0.30)
        self.v_R_spin = QDoubleSpinBox(); self.v_R_spin.setDecimals(3); self.v_R_spin.setRange(-2.0, 2.0); self.v_R_spin.setSingleStep(0.05); self.v_R_spin.setValue(0.30)
        lay.addWidget(QLabel('v_L target (m/s):'), 0, 0); lay.addWidget(self.v_L_spin, 0, 1)
        lay.addWidget(QLabel('v_R target (m/s):'), 1, 0); lay.addWidget(self.v_R_spin, 1, 1)
        step_btn = QPushButton('Apply step'); step_btn.clicked.connect(self._send_step)
        zero_btn = QPushButton('Zero targets'); zero_btn.clicked.connect(self._send_zero)
        lay.addWidget(step_btn, 0, 2); lay.addWidget(zero_btn, 1, 2)

        # Joystick
        sep = QFrame(); sep.setFrameShape(QFrame.HLine); sep.setFrameShadow(QFrame.Sunken)
        lay.addWidget(sep, 2, 0, 1, 3)

        self.joy_chk = QCheckBox('Joystick mode (W/A/S/D or arrows)')
        self.joy_chk.toggled.connect(self._joy_toggled)
        lay.addWidget(self.joy_chk, 3, 0, 1, 3)

        max_v = QDoubleSpinBox(); max_v.setDecimals(2); max_v.setRange(0.05, 2.0); max_v.setSingleStep(0.05); max_v.setValue(self.joy.max_v)
        max_w = QDoubleSpinBox(); max_w.setDecimals(2); max_w.setRange(0.05, 5.0); max_w.setSingleStep(0.1);  max_w.setValue(self.joy.max_w)
        max_v.valueChanged.connect(lambda v: setattr(self.joy, 'max_v', v))
        max_w.valueChanged.connect(lambda v: setattr(self.joy, 'max_w', v))
        lay.addWidget(QLabel('Joy max v:'),  4, 0); lay.addWidget(max_v, 4, 1)
        lay.addWidget(QLabel('Joy max ω:'),  5, 0); lay.addWidget(max_w, 5, 1)

        self.joy_status = QLabel('—')
        self.joy_status.setStyleSheet('color: #555;')
        lay.addWidget(self.joy_status, 4, 2, 2, 1)

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
        else:
            self._set_status(True)

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
        # Always apply on any field commit; cheap.
        payload = bytes([TCMD_SET_GAINS]) + struct.pack(
            '<ffffff',
            float(self.kp_L_spin.value()), float(self.ki_L_spin.value()), float(self.kd_L_spin.value()),
            float(self.kp_R_spin.value()), float(self.ki_R_spin.value()), float(self.kd_R_spin.value()),
        )
        self.link.send(PKT_TUNE_CMD, payload)

    def _send_step(self) -> None:
        v_L = float(self.v_L_spin.value())
        v_R = float(self.v_R_spin.value())
        self.link.send(PKT_TUNE_CMD, bytes([TCMD_SET_TARGETS]) + struct.pack('<ff', v_L, v_R))

    def _send_zero(self) -> None:
        self.v_L_spin.setValue(0.0)
        self.v_R_spin.setValue(0.0)
        self.link.send(PKT_TUNE_CMD, bytes([TCMD_SET_TARGETS]) + struct.pack('<ff', 0.0, 0.0))

    def _send_enable(self, enabled: bool) -> None:
        self.link.send(PKT_TUNE_CMD, bytes([TCMD_ENABLE, 1 if enabled else 0]))

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
            if frame.type == PKT_TUNE_TELEM and len(frame.payload) == TELEM_STRUCT.size:
                self._on_telem(frame.payload)
        self._redraw()

        # Connection status: detect drops the poll caused
        if not self.link.is_open() and self.conn_btn.text() == 'Disconnect':
            self._set_status(False, 'link closed')

        # Telemetry rate display
        now = time.monotonic()
        elapsed = now - self.telem_count_t0
        if elapsed >= 1.0:
            hz = self.telem_count / elapsed
            self.rate_lbl.setText(f'Telem: {hz:.0f} Hz')
            self.telem_count = 0
            self.telem_count_t0 = now

    def _heartbeat_and_joy(self) -> None:
        if not self.link.is_open():
            return
        if self.joy_chk.isChecked():
            v, w = self._joy_vw()
            v_L = v - (WHEEL_BASE_M / 2.0) * w
            v_R = v + (WHEEL_BASE_M / 2.0) * w
            self.link.send(PKT_TUNE_CMD, bytes([TCMD_SET_TARGETS]) + struct.pack('<ff', v_L, v_R))
        else:
            self.link.send(PKT_TUNE_CMD, bytes([TCMD_HEARTBEAT]))

    # ──────────────── Telemetry handling ────────────────────────────────────

    def _on_telem(self, payload: bytes) -> None:
        (ts_ms, enabled, _res, _pad,
         vt_L, vm_L, e_L, i_L, d_L,
         vt_R, vm_R, e_R, i_R, d_R,
         _kpL, _kiL, _kdL, _kpR, _kiR, _kdR) = TELEM_STRUCT.unpack(payload)

        # Firmware timestamp is monotonic ms since boot; reset our t0 the first
        # frame so the x-axis starts at 0 each session.
        if self.last_telem_t == 0.0:
            self.t0 = time.monotonic() - ts_ms / 1000.0
        self.last_telem_t = ts_ms / 1000.0
        t = ts_ms / 1000.0
        self.telem_count += 1

        self.t_buf.append(t)
        self.vt_L.append(vt_L); self.vm_L.append(vm_L)
        self.vt_R.append(vt_R); self.vm_R.append(vm_R)
        self.e_L.append(e_L);   self.e_R.append(e_R)
        self.i_L.append(i_L);   self.i_R.append(i_R)
        self.d_L.append(d_L);   self.d_R.append(d_R)

        if enabled:
            self.enabled_lbl.setText('Motors: ON')
            self.enabled_lbl.setStyleSheet('color: #0a0; font-weight: bold;')
        else:
            self.enabled_lbl.setText('Motors: OFF')
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
        self.c_i_L.setData(t, list(self.i_L))
        self.c_i_R.setData(t, list(self.i_R))
        self.c_d_L.setData(t, list(self.d_L))
        self.c_d_R.setData(t, list(self.d_R))

    # ──────────────── Shutdown ──────────────────────────────────────────────

    def closeEvent(self, e):
        # Best-effort: tell the firmware to disable on graceful exit. If the
        # link is already dead the firmware's watchdog will catch it anyway.
        try:
            self._send_enable(False)
        except Exception:
            pass
        self.link.close()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0', help='serial port (default: /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=921600)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(args.port, args.baud)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
