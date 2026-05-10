
// ── AGV Wire Protocol (matches firmware exactly) ────────────────────────────
// Frame: 0xAA 0x56 VER(0x01) SEQ TYPE LEN PAYLOAD[0..255] CRC16-CCITT(2)
// CRC covers: [VER, SEQ, TYPE, LEN, PAYLOAD...]

const AGV_PROTO = {
  MAGIC0: 0xAA, MAGIC1: 0x56, VERSION: 0x01,

  // Packet types
  PKT: {
    CMD:          0x01,
    PARAM_UPDATE: 0x02,
    TELEMETRY:    0x03,
    HEARTBEAT:    0x04,
    ACK:          0x05,
    NACK:         0x06,
    FRAG:         0x07,
    LOG:          0x08,
    RESET:        0xFF,
  },

  // CMD sub-types
  CMD: {
    SET_FUNCTION:           0x01,
    SET_MODE:               0x02,
    VEL_CMD:                0x03,
    VIRTUAL_ESTOP:          0x04,
    OVERRIDE_ESTOP_SOURCE:  0x05,
    OVERRIDE_CAUTION:       0x06,
    READ_SENSOR:            0x07,
    LOAD_TRAJECTORY:        0x08,
    START_TARE:             0x09,
    LOG_DUMP_REQUEST:       0x0A,
    LOG_CLEAR:              0x0B,
    QTR_CALIBRATE:          0x0C,
  },

  // PARAM_UPDATE IDs
  PARAM: {
    MAX_LINEAR_SPEED:     0x01,
    MAX_ANGULAR_SPEED:    0x02,
    MAX_LINEAR_ACCEL:     0x03,
    MAX_ANGULAR_ACCEL:    0x04,
    TELEMETRY_RATE_HZ:    0x05,
    HEARTBEAT_TIMEOUT_MS: 0x06,
    INNER_KP_LEFT:        0x10,
    INNER_KI_LEFT:        0x11,
    INNER_KD_LEFT:        0x12,
    INNER_KP_RIGHT:       0x13,
    INNER_KI_RIGHT:       0x14,
    INNER_KD_RIGHT:       0x15,
    OUTER_LIN_KP:         0x16,
    OUTER_LIN_KI:         0x17,
    OUTER_LIN_KD:         0x18,
    OUTER_ANG_KP:         0x19,
    OUTER_ANG_KI:         0x1A,
    OUTER_ANG_KD:         0x1B,
    LINE_KP:              0x20,
    LINE_KI:              0x21,
    LINE_KD:              0x22,
    LINE_CRUISE_MPS:      0x23,
    TRAJ_CRUISE_MPS:      0x24,
    TRAJ_LOOKAHEAD_M:     0x25,
    WEIGHT_CAUTION_KG:    0x30,
    WEIGHT_ESTOP_KG:      0x31,
    IMBALANCE_CAUTION:    0x32,
    IMBALANCE_ESTOP:      0x33,
  },

  // Mode & function IDs (match firmware enums)
  MODE: { SUPERVISED: 0x00, UNSUPERVISED: 0x01 },
  FUNC: { STANDBY: 0x00, REMOTE_CONTROL: 0x01, LINE_FOLLOW: 0x02, TRAJECTORY_FOLLOW: 0x03 },

  // E-STOP source bitmask
  ESTOP_SRC: {
    PROXIMITY:         0x01,
    CARGO_OVERLOAD:    0x02,
    CARGO_IMBALANCE:   0x04,
    HEARTBEAT_TIMEOUT: 0x08,
    WORKSTATION:       0x10,
    OVERCURRENT:       0x20,
    FIRMWARE_FAULT:    0x40,
  },
  ESTOP_AUTOCLEAR_MASK: 0x07, // proximity | cargo_overload | cargo_imbalance

  // NACK codes
  NACK: {
    BAD_CRC:            0x01,
    BAD_LENGTH:         0x02,
    UNKNOWN_TYPE:       0x03,
    UNKNOWN_SUBTYPE:    0x04,
    UNKNOWN_PARAM:      0x05,
    ILLEGAL_TRANSITION: 0x06,
    BUSY:               0x07,
  },

  // Log severity
  LOG_SEV: { INFO: 0, WARN: 1, ERROR: 2, CRITICAL: 3 },
  LOG_SEV_NAMES: ['INFO', 'WARN', 'ERROR', 'CRITICAL'],

  // Log module names
  LOG_MOD_NAMES: {
    0: 'SYSTEM', 1: 'COMMS', 2: 'MOTORS', 3: 'ENCODERS', 4: 'ADC',
    5: 'HX711', 6: 'IMU', 7: 'PROXIMITY', 8: 'ESTOP', 9: 'HEARTBEAT',
    10: 'STATE', 11: 'NAV', 12: 'ODOMETRY',
  },

  // Log code names (mirrors types.h log_code_t)
  LOG_CODE_NAMES: {
    0x0001: 'BOOT',                   0x0002: 'WATCHDOG_RESET',
    0x0003: 'BROWNOUT_RESET',         0x0004: 'SOFT_RESET',
    0x0100: 'BAD_MAGIC',              0x0101: 'BAD_VERSION',
    0x0102: 'BAD_CRC',                0x0103: 'BAD_LENGTH',
    0x0104: 'UART_OVERRUN',           0x0105: 'UART_FRAMING',
    0x0106: 'UART_NOISE',             0x0107: 'TX_QUEUE_FULL',
    0x0108: 'UNKNOWN_PKT_TYPE',       0x0109: 'UNKNOWN_CMD',
    0x010A: 'UNKNOWN_PARAM',          0x010B: 'SEQ_GAP',
    0x010C: 'REASSEMBLY_ERR',
    0x0200: 'OVERCURRENT_M1',         0x0201: 'OVERCURRENT_M2',
    0x0202: 'PWM_SATURATED',
    0x0300: 'ENCODER_OVERFLOW',
    0x0500: 'HX711_TIMEOUT',          0x0501: 'HX711_OUT_OF_RANGE',
    0x0600: 'IMU_I2C_NACK',           0x0601: 'IMU_I2C_TIMEOUT',
    0x0602: 'IMU_BUS_RESET',          0x0603: 'IMU_CALIB_LOST',
    0x0700: 'PROX_TRIGGERED',         0x0701: 'PROX_CLEARED',
    0x0800: 'ESTOP_ASSERTED',         0x0801: 'ESTOP_CLEARED',
    0x0802: 'ESTOP_OVERRIDE',
    0x0900: 'HEARTBEAT_LOST',         0x0901: 'HEARTBEAT_GRACE_EXPIRED',
    0x0902: 'HEARTBEAT_RESTORED',
    0x0A00: 'MODE_TRANSITION',        0x0A01: 'FUNCTION_TRANSITION',
    0x0A02: 'ILLEGAL_TRANSITION',
    0x0B00: 'TRAJECTORY_LOADED',      0x0B01: 'WAYPOINT_REACHED',
    0x0B02: 'TRAJECTORY_COMPLETE',    0x0B03: 'LINE_LOST',
    0x0C00: 'ODOMETRY_RESET',
    0x0D00: 'QTR_CAL_BEGIN',          0x0D01: 'QTR_CAL_END',
    0x0D02: 'QTR_CAL_CANCELED',       0x0D03: 'QTR_CAL_INSUFFICIENT',
    0x0D04: 'FLASH_WRITE_FAIL',       0x0D05: 'FLASH_LOAD_FAIL',
    0x0D06: 'FLASH_LOAD_OK',          0x0D07: 'PARAM_APPLIED',
  },
};

// ── CRC16-CCITT (matching firmware's crc.c) ──────────────────────────────────
function crc16_ccitt(bytes) {
  let crc = 0xFFFF;
  for (const b of bytes) {
    crc ^= (b << 8);
    for (let i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
      crc &= 0xFFFF;
    }
  }
  return crc;
}

// ── Frame builder ─────────────────────────────────────────────────────────────
let _seq = 0;
function buildFrame(type, payload = []) {
  const seq = (_seq++) & 0xFF;
  const len = payload.length;
  // CRC covers [VER, SEQ, TYPE, LEN, ...PAYLOAD]
  const crcInput = [AGV_PROTO.VERSION, seq, type, len, ...payload];
  const crc = crc16_ccitt(crcInput);
  return new Uint8Array([
    AGV_PROTO.MAGIC0, AGV_PROTO.MAGIC1,
    AGV_PROTO.VERSION, seq, type, len,
    ...payload,
    crc & 0xFF, (crc >> 8) & 0xFF,
  ]);
}

// ── Float → 4 bytes (little-endian, matching C memcpy of float) ───────────────
function floatToBytes(f) {
  const buf = new ArrayBuffer(4);
  new DataView(buf).setFloat32(0, f, true);
  return [...new Uint8Array(buf)];
}
function bytesToFloat(bytes, offset = 0) {
  const buf = new ArrayBuffer(4);
  new Uint8Array(buf).set(bytes.slice(offset, offset + 4));
  return new DataView(buf).getFloat32(0, true);
}

// ── Frame parser (mirrors ESP32 frame_parser logic) ───────────────────────────
function createFrameParser(onFrame, onError) {
  let state = 'MAGIC0';
  let ver, seq, type, len, payloadIdx;
  let buf = [];

  function feed(byte) {
    switch (state) {
      case 'MAGIC0':
        if (byte === AGV_PROTO.MAGIC0) { buf = [byte]; state = 'MAGIC1'; }
        break;
      case 'MAGIC1':
        if (byte === AGV_PROTO.MAGIC1) { buf.push(byte); state = 'VER'; }
        else { state = 'MAGIC0'; }
        break;
      case 'VER':
        ver = byte; buf.push(byte); state = 'SEQ';
        break;
      case 'SEQ':
        seq = byte; buf.push(byte); state = 'TYPE';
        break;
      case 'TYPE':
        type = byte; buf.push(byte); state = 'LEN';
        break;
      case 'LEN':
        len = byte; buf.push(byte);
        state = len > 0 ? 'PAYLOAD' : 'CRC_HI';
        payloadIdx = 0;
        break;
      case 'PAYLOAD':
        buf.push(byte);
        if (++payloadIdx >= len) state = 'CRC_HI';
        break;
      case 'CRC_HI':
        buf.push(byte); state = 'CRC_LO';
        break;
      case 'CRC_LO': {
        buf.push(byte);
        // Verify CRC: covers [VER, SEQ, TYPE, LEN, PAYLOAD...]
        const crcInput = buf.slice(2, buf.length - 2); // skip MAGIC, skip CRC
        const calcCrc = crc16_ccitt(crcInput);
        const rxCrc = buf[buf.length - 2] | (buf[buf.length - 1] << 8);
        if (calcCrc === rxCrc) {
          const payload = buf.slice(6, 6 + len);
          onFrame({ type, seq, len, payload, ver });
        } else {
          onError && onError('BAD_CRC', seq);
        }
        state = 'MAGIC0';
        break;
      }
    }
  }

  return { feed };
}

// ── WebSocket connection manager ──────────────────────────────────────────────
function useAGVWebSocket({ url, onTelemetry, onLog, onAck, onNack, onConnected, onDisconnected, onStats }) {
  const wsRef = React.useRef(null);
  const parserRef = React.useRef(null);
  const heartbeatRef = React.useRef(null);
  const reconnectRef = React.useRef(null);
  const intentionalRef = React.useRef(false);
  const pendingAcks = React.useRef({}); // seq → { resolve, reject, timer }

  function connect() {
    if (reconnectRef.current) { clearTimeout(reconnectRef.current); reconnectRef.current = null; }
    if (wsRef.current) return;
    intentionalRef.current = false;
    const ws = new WebSocket(url);
    ws.binaryType = 'arraybuffer';
    wsRef.current = ws;

    parserRef.current = createFrameParser(
      (frame) => handleFrame(frame),
      (err, seq) => console.warn('[ws] frame error', err, 'seq', seq)
    );

    ws.onopen = () => {
      onConnected && onConnected();
      heartbeatRef.current = setInterval(() => sendHeartbeat(), 500);
    };

    ws.onmessage = (e) => {
      const bytes = new Uint8Array(e.data);
      for (const b of bytes) parserRef.current.feed(b);
    };

    ws.onclose = () => {
      cleanup();
      onDisconnected && onDisconnected();
      if (!intentionalRef.current) {
        reconnectRef.current = setTimeout(connect, 1000);
      }
    };

    ws.onerror = () => {
      // onclose fires after onerror — let it handle cleanup and reconnect
    };
  }

  function cleanup() {
    if (heartbeatRef.current) { clearInterval(heartbeatRef.current); heartbeatRef.current = null; }
    // Cancel all pending ACK waits so they don't resolve against a new connection's frames
    Object.values(pendingAcks.current).forEach(p => { clearTimeout(p.timer); p.reject(new Error('disconnected')); });
    pendingAcks.current = {};
    wsRef.current = null;
  }

  function disconnect() {
    intentionalRef.current = true;
    if (reconnectRef.current) { clearTimeout(reconnectRef.current); reconnectRef.current = null; }
    if (wsRef.current) wsRef.current.close();
    cleanup();
    onDisconnected && onDisconnected();
  }

  function send(type, payload = []) {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return false;
    const frame = buildFrame(type, payload);
    wsRef.current.send(frame.buffer);
    onStats && onStats({ tx: 1 });
    return frame[3]; // return seq
  }

  function sendWithAck(type, payload = [], timeoutMs = 1000) {
    return new Promise((resolve, reject) => {
      const seq = send(type, payload);
      if (seq === false) { reject(new Error('not connected')); return; }
      const t0 = Date.now();
      const timer = setTimeout(() => {
        delete pendingAcks.current[seq];
        onStats && onStats({ drop: 1 });
        reject(new Error('ACK timeout'));
      }, timeoutMs);
      pendingAcks.current[seq] = {
        resolve: (result) => { onStats && onStats({ latency: Date.now() - t0 }); resolve(result); },
        reject,
        timer,
      };
    });
  }

  function handleFrame(frame) {
    const { type, payload } = frame;
    switch (type) {
      case AGV_PROTO.PKT.ACK:
        if (pendingAcks.current[payload[0]]) {
          const p = pendingAcks.current[payload[0]];
          clearTimeout(p.timer);
          delete pendingAcks.current[payload[0]];
          p.resolve({ seq: payload[0], status: payload[1] });
        }
        onAck && onAck(payload[0], payload[1]);
        break;
      case AGV_PROTO.PKT.NACK:
        if (pendingAcks.current[payload[0]]) {
          const p = pendingAcks.current[payload[0]];
          clearTimeout(p.timer);
          delete pendingAcks.current[payload[0]];
          p.reject(new Error('NACK ' + payload[1]));
        }
        onNack && onNack(payload[0], payload[1]);
        break;
      case AGV_PROTO.PKT.TELEMETRY:
        onTelemetry && onTelemetry(parseTelemetry(payload));
        break;
      case AGV_PROTO.PKT.LOG:
        onLog && onLog(parseLogEntry(payload));
        break;
    }
  }

  function sendHeartbeat() {
    const seq = send(AGV_PROTO.PKT.HEARTBEAT, []);
    if (seq === false) return;
    const t0 = Date.now();
    const timer = setTimeout(() => { delete pendingAcks.current[seq]; }, 900);
    pendingAcks.current[seq] = {
      resolve: () => { onStats && onStats({ latency: Date.now() - t0 }); },
      reject: () => {},
      timer,
    };
  }

  // ── Command helpers ────────────────────────────────────────────────────────
  function cmdSetFunction(funcId) {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.SET_FUNCTION, funcId]);
  }
  function cmdSetMode(modeId) {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.SET_MODE, modeId]);
  }
  function cmdVelCmd(linear, angular) {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.VEL_CMD, ...floatToBytes(linear), ...floatToBytes(angular)]);
  }
  function cmdVirtualEstop() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.VIRTUAL_ESTOP]);
  }
  function cmdOverrideEstop(srcMask) {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.OVERRIDE_ESTOP_SOURCE, srcMask]);
  }
  function cmdOverrideCaution(scalar) {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.OVERRIDE_CAUTION, ...floatToBytes(scalar)]);
  }
  function cmdStartTare() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.START_TARE]);
  }
  function cmdLogClear() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.LOG_CLEAR]);
  }
  function cmdQtrCalibrate() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.QTR_CALIBRATE]);
  }
  function cmdSoftReset() {
    return sendWithAck(AGV_PROTO.PKT.RESET, [0x01]);
  }
  function cmdClearAllEstop() {
    return sendWithAck(AGV_PROTO.PKT.RESET, [0x00]);
  }
  function sendParamUpdate(paramId, value) {
    return sendWithAck(AGV_PROTO.PKT.PARAM_UPDATE, [paramId, ...floatToBytes(value)]);
  }
  function sendParamBatch(params) {
    // params: [{id, value}, ...]  — pack into one PARAM_UPDATE frame
    const payload = [];
    for (const { id, value } of params) {
      payload.push(id, ...floatToBytes(value));
    }
    return sendWithAck(AGV_PROTO.PKT.PARAM_UPDATE, payload);
  }

  return {
    connect, disconnect,
    cmdSetFunction, cmdSetMode, cmdVelCmd,
    cmdVirtualEstop, cmdOverrideEstop, cmdOverrideCaution,
    cmdStartTare, cmdLogClear, cmdQtrCalibrate,
    cmdSoftReset, cmdClearAllEstop,
    sendParamUpdate, sendParamBatch,
  };
}

// ── Telemetry parser ─────────────────────────────────────────────────────────
// Mirrors the byte layout in firmware/STM32/src/main.c → send_telemetry().
// Total: 111 bytes. All multi-byte values are little-endian.
//
//  off  size  field
//  ---  ----  -----------------------------------------------------------
//    0   u32  timestamp_ms
//    4   u8   mode                       (0=SUPERVISED, 1=UNSUPERVISED)
//    5   u8   function                   (0=STBY, 1=REMOTE, 2=LINE, 3=TRAJ)
//    6   u8   estop_sources              (bitmask, see ESTOP_SRC)
//    7   u8   caution_active_sources     (bitmask)
//    8   f32  caution_modifier           [0..1]
//   12   u16  log_pending  / u16 log_dropped
//   16   u32  enc_left  / u32 enc_right
//   24   f32  enc_left_mps / f32 enc_right_mps
//   32   f32  odom_x / f32 odom_y / f32 odom_theta / f32 odom_v / f32 odom_omega
//   52   f32  control_v_target / f32 control_omega_target
//   60   u16  motor_current_ma[0] / u16 motor_current_ma[1]
//   64   8 × u16  qtr[0..7]                              (raw ADC counts)
//   80   4 × f32  hx711_kg[FL,FR,RL,RR]
//   96   f32  imu_yaw_deg / f32 imu_pitch_deg / f32 imu_roll_deg
//  108   u8   imu_calib (sys|gyro|accel|mag, 2 bits each)
//  109   u8   proximity_obstructed (bit0=F, bit1=R, bit2=L, bit3=Rt)
//  110   u8   flags (bit0=adc, bit1=hx711, bit2=imu, bit3=tare_in_progress)
//  111   end
function parseTelemetry(payload) {
  if (!payload || payload.length < 111) return null;
  const buf = new Uint8Array(payload);
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  const u8  = (o) => buf[o];
  const u16 = (o) => dv.getUint16(o, true);
  const u32 = (o) => dv.getUint32(o, true);
  const f32 = (o) => dv.getFloat32(o, true);

  const mode = u8(4) === 0 ? 'SUPERVISED' : 'UNSUPERVISED';
  const FUNC_NAMES = ['STANDBY','REMOTE_CONTROL','LINE_FOLLOW','TRAJECTORY_FOLLOW'];
  const func = FUNC_NAMES[u8(5)] ?? 'STANDBY';

  const estopSrc = u8(6);
  const cautSrc  = u8(7);
  const cautMod  = f32(8);

  // Decode E-STOP sources to human strings
  const estopSourceLabels = [];
  if (estopSrc & 0x01) estopSourceLabels.push('Proximity sensor');
  if (estopSrc & 0x02) estopSourceLabels.push('Cargo overload');
  if (estopSrc & 0x04) estopSourceLabels.push('Cargo imbalance');
  if (estopSrc & 0x08) estopSourceLabels.push('Heartbeat timeout');
  if (estopSrc & 0x10) estopSourceLabels.push('Workstation command');
  if (estopSrc & 0x20) estopSourceLabels.push('Motor overcurrent');
  if (estopSrc & 0x40) estopSourceLabels.push('Firmware fault');

  const cautLevel =
    cautMod >= 0.95 ? 'NORMAL' :
    cautMod >= 0.35 ? 'CAUTION' :
    cautMod > 0.0   ? 'CRITICAL' : 'ESTOP';

  const fl = f32(80), fr = f32(84), rl = f32(88), rr = f32(92);
  const total = fl + fr + rl + rr;
  const cogX = total > 0.001 ? (fr + rr - fl - rl) / total : 0;
  const cogY = total > 0.001 ? (rl + rr - fl - fr) / total : 0;

  const proxBits = u8(109);
  const flags    = u8(110);
  const calib    = u8(108);

  const vLeft  = f32(24);
  const vRight = f32(28);

  // Correct odometry block: x@32, y@36, theta@40, v@44, omega@48
  const odomX     = f32(32);
  const odomY     = f32(36);
  const odomTheta = f32(40);
  const odomV     = f32(44);
  const odomOmega = f32(48);
  // Encoder velocities @24/28 — used for wheel-RPM derivation
  const wheelR = 0.10;

  return {
    timestamp_ms: u32(0),
    mode,
    func,
    estop: {
      active:   estopSrc !== 0,
      virtual:  estopSrc !== 0,
      physical: false, // physical E-STOP cuts driver power; MCU keeps running but driver is dark
      sources:  estopSourceLabels,
      sourceMask: estopSrc,
    },
    caution: { modifier: cautMod, level: cautLevel, sources: cautSrc },
    velocity: { v: odomV, omega: odomOmega, vLeft, vRight },
    position: { x: odomX, y: odomY, theta: odomTheta },
    encoders: {
      left:  u32(16),
      right: u32(20),
      leftRpm:  vLeft  * 60.0 / (2 * Math.PI * wheelR),
      rightRpm: vRight * 60.0 / (2 * Math.PI * wheelR),
    },
    loadCells: { fl, fr, rl, rr, total, cog: { x: cogX, y: cogY } },
    imu: {
      yaw: f32(96), pitch: f32(100), roll: f32(104),
      ax: 0, ay: 0, az: 9.81, // accel/gyro vectors not in telemetry payload yet
      calib_sys:   (calib >> 6) & 0x3,
      calib_gyro:  (calib >> 4) & 0x3,
      calib_accel: (calib >> 2) & 0x3,
      calib_mag:   (calib     ) & 0x3,
    },
    proximity: {
      front: !!(proxBits & 0x40),  // PC6
      rear:  !!(proxBits & 0x80),  // PC7
      left:  false,                // PC8 — truncated by uint8 cast in firmware
      right: false,                // PC9 — truncated by uint8 cast in firmware
    },
    current: { left: u16(60) / 1000.0, right: u16(62) / 1000.0 },
    qtr: Array.from({ length: 8 }, (_, i) => u16(64 + i * 2)),
    control: { v_target: f32(52), omega_target: f32(56) },
    flags: {
      adc_data:         !!(flags & 0x01),
      hx711_data:       !!(flags & 0x02),
      imu_data:         !!(flags & 0x04),
      tare_in_progress: !!(flags & 0x08),
    },
    log: { pending: u16(12), dropped: u16(14) },
    uptime: u32(0) / 1000.0,
  };
}

// ── Log entry parser ───────────────────────────────────────────────────────────
// Firmware drain_logs() layout (12 bytes):
//   [0..3]  timestamp_ms  (u32 LE)
//   [4..5]  code          (u16 LE, lo first)
//   [6]     severity
//   [7]     module
//   [8..11] data          (u32 LE)
function parseLogEntry(payload) {
  if (!payload || payload.length < 8) return null;
  const timestamp_ms = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
  const code = payload[4] | (payload[5] << 8);
  const sev  = payload[6];
  const mod  = payload[7];
  const data = payload.length >= 12
    ? (payload[8] | (payload[9] << 8) | (payload[10] << 16) | (payload[11] << 24))
    : 0;
  const level  = AGV_PROTO.LOG_SEV_NAMES[sev] || 'INFO';
  const module = AGV_PROTO.LOG_MOD_NAMES[mod] || `MOD_${mod}`;
  const codeName = AGV_PROTO.LOG_CODE_NAMES[code]
    || `0x${code.toString(16).toUpperCase().padStart(4, '0')}`;
  const msg = data
    ? `[${module}] ${codeName} (data=0x${data.toString(16).toUpperCase()})`
    : `[${module}] ${codeName}`;
  return { timestamp_ms, level, module, code, msg, data };
}

Object.assign(window, {
  AGV_PROTO, crc16_ccitt, buildFrame, floatToBytes, bytesToFloat,
  createFrameParser, useAGVWebSocket, parseTelemetry, parseLogEntry,
});
