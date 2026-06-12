
// ── AGV Wire Protocol (matches firmware exactly) ────────────────────────────
// Frame: 0xAA 0x56 VER(0x03) SEQ TYPE LEN PAYLOAD[0..255] CRC16-CCITT(2)
// CRC covers: [VER, SEQ, TYPE, LEN, PAYLOAD...]
// v2: telemetry is four rate-grouped streams (CORE/DRIVE/SENSORS/QTR).

const AGV_PROTO = {
  MAGIC0: 0xAA, MAGIC1: 0x56, VERSION: 0x04,

  // Packet types
  PKT: {
    CMD:          0x01,
    PARAM_UPDATE: 0x02,
    TLM_CORE:     0x03,   // operational state + pose (fast)
    HEARTBEAT:    0x04,
    ACK:          0x05,
    NACK:         0x06,
    LOG:          0x08,
    TLM_DRIVE:    0x09,   // per-wheel control internals
    TLM_SENSORS:  0x0A,   // load cells + battery + LiDAR tail
    TLM_QTR:      0x0B,   // QTR raw + line position
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
    START_TARE:             0x09,
    LOG_DUMP_REQUEST:       0x0A,
    LOG_CLEAR:              0x0B,
    /* 0x0C retired (was QTR_CALIBRATE — QTR is now self-calibrating) */
    RESET_ODOMETRY:         0x0D,   /* firmware must zero x/y/θ */
    LOAD_RAMP_CURVE:        0x0E,   /* upload custom ramp shape (op=0/1/2/3) */
  },

  // PARAM_UPDATE IDs
  PARAM: {
    MAX_LINEAR_SPEED:     0x01,
    MAX_ANGULAR_SPEED:    0x02,
    MAX_LINEAR_ACCEL:     0x03,
    MAX_ANGULAR_ACCEL:    0x04,
    TELEMETRY_RATE_HZ:    0x05,
    HEARTBEAT_TIMEOUT_MS: 0x06,
    // Per-wheel velocity PI + feedforward (replaces the old super-twisting cascade).
    LEFT_KP:              0x10,
    LEFT_KI:              0x11,
    LEFT_KFF:             0x12,
    RIGHT_KP:             0x13,
    RIGHT_KI:             0x14,
    RIGHT_KFF:            0x15,
    LINE_KP:              0x20,
    LINE_KI:              0x21,
    LINE_KD:              0x22,
    LINE_CRUISE_MPS:      0x23,
    QTR_LINE_LOST_THRESH: 0x26,
    LINE_T_BLACK:         0x27,   // T-bar "black" ADC threshold (counts)
    WEIGHT_CAUTION_KG:    0x30,
    WEIGHT_ESTOP_KG:      0x31,
    IMBALANCE_CAUTION:    0x32,
    IMBALANCE_ESTOP:      0x33,
    RAMP_SHAPE:           0x40,
    RAMP_JERK_LIN:        0x41,
    RAMP_JERK_ANG:        0x42,
    RAMP_TAU_LIN:         0x43,
    RAMP_TAU_ANG:         0x44,
    LED_MODE:             0x50,   // indicator-ring animation (see LED_MODE enum)
    LED_BASE:             0x51,   // reactive ring base: 0=off, 1=white
    LED_INDICATOR_MODE:   0x52,   // reactive ring spread bit (currently unused)
    /* 0x60–0x62 retired (were TOF distance bands — hardware removed) */
    BATT_3S_CAUTION_MV:   0x63,   // 3S low-voltage thresholds (mV)
    BATT_3S_ESTOP_MV:     0x64,
    LIDAR_CAUTION_MM:     0x65,   // LiDAR distance bands (mm)
    LIDAR_CRITICAL_MM:    0x66,
    LIDAR_ESTOP_MM:       0x67,
  },

  // Ramp shape enum (matches ramp.h)
  RAMP_SHAPE: { LINEAR: 0, SCURVE: 1, EXPONENTIAL: 2, CUSTOM: 3 },

  // Indicator-ring animation enum (matches firmware LED_MODE_*)
  LED_MODE: { PULSE: 0, SNAKE: 1 },
  // Reactive-ring config enums (matches firmware LED_IND_*_BIT in led_indicator_cfg)
  LED_BASE: { OFF: 0, WHITE: 1 },
  LED_INDICATOR_MODE: { FIXED: 0, RESPONSIVE: 1 },

  // Mode & function IDs (match firmware enums)
  MODE: { SUPERVISED: 0x00, UNSUPERVISED: 0x01 },
  FUNC: { STANDBY: 0x00, REMOTE_CONTROL: 0x01, LINE_FOLLOW: 0x02 },

  // E-STOP source bitmask (16-bit; bit 7 retired — was TOF)
  ESTOP_SRC: {
    PROXIMITY:         0x01,
    CARGO_OVERLOAD:    0x02,
    CARGO_IMBALANCE:   0x04,
    HEARTBEAT_TIMEOUT: 0x08,
    WORKSTATION:       0x10,
    OVERCURRENT:       0x20,
    FIRMWARE_FAULT:    0x40,
    BATTERY_LOW:       0x100,
    LIDAR:             0x200,
  },
  // proximity | cargo_overload | cargo_imbalance | battery_low | lidar
  ESTOP_AUTOCLEAR_MASK: 0x307,

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
    5: 'HX711', 7: 'PROXIMITY', 8: 'ESTOP', 9: 'HEARTBEAT',
    10: 'STATE', 11: 'NAV', 12: 'ODOMETRY', 14: 'BATTERY', 15: 'LIDAR',
  },

  // Log code names (mirrors firmware/STM32/src/types.h log_code_t)
  LOG_CODE_NAMES: {
    0x0001: 'BOOT',                   0x0002: 'WATCHDOG_RESET',
    0x0003: 'BROWNOUT_RESET',         0x0004: 'SOFT_RESET',
    0x0100: 'BAD_MAGIC',              0x0101: 'BAD_VERSION',
    0x0102: 'BAD_CRC',                0x0103: 'BAD_LENGTH',
    0x0104: 'UART_OVERRUN',           0x0105: 'UART_FRAMING',
    0x0106: 'UART_NOISE',             0x0107: 'TX_QUEUE_FULL',
    0x0108: 'UNKNOWN_PKT_TYPE',       0x0109: 'UNKNOWN_CMD',
    0x010A: 'UNKNOWN_PARAM',          0x010B: 'SEQ_GAP',
    0x010C: 'REMOTE_NACK',
    0x0200: 'OVERCURRENT_M1',         0x0201: 'OVERCURRENT_M2',
    0x0500: 'HX711_TIMEOUT',          0x0501: 'HX711_TARE_COMPLETE',
    0x0700: 'PROX_TRIGGERED',         0x0701: 'PROX_CLEARED',
    0x0800: 'ESTOP_ASSERTED',         0x0801: 'ESTOP_CLEARED',
    0x0802: 'ESTOP_OVERRIDE',
    0x0900: 'HEARTBEAT_LOST',         0x0901: 'HEARTBEAT_GRACE_EXPIRED',
    0x0902: 'HEARTBEAT_RESTORED',
    0x0A00: 'MODE_TRANSITION',        0x0A01: 'FUNCTION_TRANSITION',
    0x0A02: 'ILLEGAL_TRANSITION',
    0x0B03: 'LINE_LOST',              0x0B04: 'LINE_T_DETECTED',
    0x0B05: 'LINE_REACQUIRED',        0x0B06: 'LINE_TURN_FAILED',
    0x0C00: 'ODOMETRY_RESET',
    0x0D07: 'PARAM_APPLIED',
    0x0F00: 'BATTERY_LOW',            0x0F01: 'BATTERY_ESTOP',
    0x0F02: 'BATTERY_RESTORED',       0x0F04: 'BATTERY_I2C_FAIL',
    0x0F05: 'BATTERY_STALE',
    0x1000: 'LIDAR_TRIGGERED',        0x1001: 'LIDAR_CLEARED',
    0x1002: 'LIDAR_STALE',
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
        state = len > 0 ? 'PAYLOAD' : 'CRC_LO';
        payloadIdx = 0;
        break;
      case 'PAYLOAD':
        buf.push(byte);
        if (++payloadIdx >= len) state = 'CRC_LO';
        break;
      case 'CRC_LO':
        // Wire order is LE: low byte first.
        buf.push(byte); state = 'CRC_HI';
        break;
      case 'CRC_HI': {
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
// Returns a stable handle (memoized) so consumers' useEffect dependencies don't
// retrigger on every render of the calling component.
function useAGVWebSocket({ url, onTelemetry, onLog, onAck, onNack, onConnected, onDisconnected, onStats }) {
  const wsRef = React.useRef(null);
  const parserRef = React.useRef(null);
  const heartbeatRef = React.useRef(null);
  const reconnectRef = React.useRef(null);
  const intentionalRef = React.useRef(false);
  const pendingAcks = React.useRef({}); // seq → { resolve, reject, timer }
  const telemRef = React.useRef(makeEmptyTelem()); // accumulates the 4 streams

  // Stash callbacks behind refs so memoized methods can call the latest
  // versions without depending on closure identity.
  const cbsRef = React.useRef({});
  cbsRef.current = { onTelemetry, onLog, onAck, onNack, onConnected, onDisconnected, onStats };
  const urlRef = React.useRef(url);
  urlRef.current = url;

  function connect() {
    if (reconnectRef.current) { clearTimeout(reconnectRef.current); reconnectRef.current = null; }
    if (wsRef.current) return;
    intentionalRef.current = false;
    const ws = new WebSocket(urlRef.current);
    ws.binaryType = 'arraybuffer';
    wsRef.current = ws;

    parserRef.current = createFrameParser(
      (frame) => handleFrame(frame),
      (err, seq) => console.warn('[ws] frame error', err, 'seq', seq)
    );

    ws.onopen = () => {
      cbsRef.current.onConnected && cbsRef.current.onConnected();
      heartbeatRef.current = setInterval(() => sendHeartbeat(), 500);
    };

    ws.onmessage = (e) => {
      const bytes = new Uint8Array(e.data);
      for (const b of bytes) parserRef.current.feed(b);
    };

    ws.onclose = () => {
      cleanup();
      cbsRef.current.onDisconnected && cbsRef.current.onDisconnected();
      if (!intentionalRef.current) {
        reconnectRef.current = setTimeout(connect, 1000);
      }
    };

    // onclose fires after onerror, so let it handle cleanup + reconnect.
    ws.onerror = () => {};
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
    cbsRef.current.onDisconnected && cbsRef.current.onDisconnected();
  }

  function send(type, payload = []) {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return false;
    const frame = buildFrame(type, payload);
    wsRef.current.send(frame.buffer);
    const cbs = cbsRef.current;
    cbs.onStats && cbs.onStats({ tx: 1 });
    return frame[3]; // return seq
  }

  function sendWithAck(type, payload = [], timeoutMs = 1000) {
    return new Promise((resolve, reject) => {
      const seq = send(type, payload);
      if (seq === false) { reject(new Error('not connected')); return; }
      const t0 = Date.now();
      const timer = setTimeout(() => {
        delete pendingAcks.current[seq];
        const cbs = cbsRef.current;
        cbs.onStats && cbs.onStats({ drop: 1 });
        reject(new Error('ACK timeout'));
      }, timeoutMs);
      pendingAcks.current[seq] = {
        resolve: (result) => {
          const cbs = cbsRef.current;
          cbs.onStats && cbs.onStats({ latency: Date.now() - t0 });
          resolve(result);
        },
        reject,
        timer,
      };
    });
  }

  function handleFrame(frame) {
    const { type, payload } = frame;
    const cbs = cbsRef.current;
    switch (type) {
      case AGV_PROTO.PKT.ACK:
        if (pendingAcks.current[payload[0]]) {
          const p = pendingAcks.current[payload[0]];
          clearTimeout(p.timer);
          delete pendingAcks.current[payload[0]];
          p.resolve({ seq: payload[0], status: payload[1] });
        }
        cbs.onAck && cbs.onAck(payload[0], payload[1]);
        break;
      case AGV_PROTO.PKT.NACK:
        if (pendingAcks.current[payload[0]]) {
          const p = pendingAcks.current[payload[0]];
          clearTimeout(p.timer);
          delete pendingAcks.current[payload[0]];
          p.reject(new Error('NACK ' + payload[1]));
        }
        cbs.onNack && cbs.onNack(payload[0], payload[1]);
        break;
      case AGV_PROTO.PKT.TLM_CORE:
        telemRef.current = mergeCore(telemRef.current, payload);
        cbs.onTelemetry && cbs.onTelemetry(telemRef.current);
        break;
      case AGV_PROTO.PKT.TLM_DRIVE:
        telemRef.current = mergeDrive(telemRef.current, payload);
        cbs.onTelemetry && cbs.onTelemetry(telemRef.current);
        break;
      case AGV_PROTO.PKT.TLM_SENSORS:
        telemRef.current = mergeSensors(telemRef.current, payload);
        cbs.onTelemetry && cbs.onTelemetry(telemRef.current);
        break;
      case AGV_PROTO.PKT.TLM_QTR:
        telemRef.current = mergeQtr(telemRef.current, payload);
        cbs.onTelemetry && cbs.onTelemetry(telemRef.current);
        break;
      case AGV_PROTO.PKT.LOG:
        cbs.onLog && cbs.onLog(parseLogEntry(payload));
        break;
    }
  }

  function sendHeartbeat() {
    const seq = send(AGV_PROTO.PKT.HEARTBEAT, []);
    if (seq === false) return;
    const t0 = Date.now();
    const timer = setTimeout(() => { delete pendingAcks.current[seq]; }, 900);
    pendingAcks.current[seq] = {
      resolve: () => {
        const cbs = cbsRef.current;
        cbs.onStats && cbs.onStats({ latency: Date.now() - t0 });
      },
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
    // 16-bit mask, little-endian (high byte reaches battery bit8 / LiDAR bit9).
    return sendWithAck(AGV_PROTO.PKT.CMD,
      [AGV_PROTO.CMD.OVERRIDE_ESTOP_SOURCE, srcMask & 0xFF, (srcMask >> 8) & 0xFF]);
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
  function cmdResetOdometry() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.RESET_ODOMETRY]);
  }
  // Custom ramp curve upload — sequence: begin → addPoint×N → commit.
  function cmdRampCurveBegin() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.LOAD_RAMP_CURVE, 0x00]);
  }
  function cmdRampCurveAddPoint(s, f) {
    return sendWithAck(AGV_PROTO.PKT.CMD, [
      AGV_PROTO.CMD.LOAD_RAMP_CURVE, 0x01,
      ...floatToBytes(s), ...floatToBytes(f),
    ]);
  }
  function cmdRampCurveCommit() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.LOAD_RAMP_CURVE, 0x02]);
  }
  function cmdRampCurveCancel() {
    return sendWithAck(AGV_PROTO.PKT.CMD, [AGV_PROTO.CMD.LOAD_RAMP_CURVE, 0x03]);
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

  // Memoize so caller's effect deps stay referentially stable across renders.
  return React.useMemo(() => ({
    connect, disconnect,
    cmdSetFunction, cmdSetMode, cmdVelCmd,
    cmdVirtualEstop, cmdOverrideEstop, cmdOverrideCaution,
    cmdStartTare, cmdLogClear, cmdResetOdometry,
    cmdRampCurveBegin, cmdRampCurveAddPoint, cmdRampCurveCommit, cmdRampCurveCancel,
    cmdSoftReset, cmdClearAllEstop,
    sendParamUpdate, sendParamBatch,
  }), []);
}

// ── Telemetry streams → merged state ─────────────────────────────────────────
// Firmware (v2) emits four rate-grouped streams; we accumulate them into one
// object whose shape matches createMockTelemetry() so the tabs need no changes.
//   CORE    (0x03): state, caution, chassis v/ω, pose, currents, proximity, flags
//   DRIVE   (0x09): per-wheel target/measured velocity, duty, encoder counts
//   SENSORS (0x0A): load cells, battery, LiDAR
//   QTR     (0x0B): 8 raw reflectance values + firmware line position
const WHEEL_RADIUS_M = 0.10;
const WHEEL_BASE_M   = 0.40;   // matches firmware config.h WHEEL_BASE_M

function makeEmptyTelem() {
  return {
    timestamp_ms: 0, uptime: 0,
    mode: 'SUPERVISED', func: 'STANDBY',
    estop: { active: false, virtual: false, sources: [], sourceMask: 0 },
    caution: { modifier: 1.0, level: 'NORMAL', sources: 0 },
    velocity: { v: 0, omega: 0, vLeft: 0, vRight: 0 },
    position: { x: 0, y: 0, theta: 0 },
    encoders: { left: 0, right: 0, leftRpm: 0, rightRpm: 0 },
    loadCells: { fl: 0, fr: 0, rl: 0, rr: 0, total: 0, cog: { x: 0, y: 0 } },
    proximity: { fl: false, fr: false, rl: false, rr: false },
    lidar: [],                                       // segmented LaserScan distances (mm)
    battery: { v3s: 0, pct3s: null },
    current: { left: 0, right: 0 },
    qtr: [0,0,0,0,0,0,0,0],
    control: { v_target: 0, omega_target: 0 },
    motors: { dutyLeft: 0, dutyRight: 0, vLeftTarget: 0, vRightTarget: 0 },
    flags: { adc_data: false, hx711_data: false, tare_in_progress: false },
    log: { pending: 0, dropped: 0 },
    ledMode: 0,
    ledIndicatorCfg: 0,    // bit0 base (0=off,1=white), bit1 spread (0=fixed,1=responsive)
  };
}

function _telemDataView(payload) {
  const buf = new Uint8Array(payload);
  return new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
}

function _estopSourceLabels(mask) {
  const out = [];
  if (mask & 0x01) out.push('Proximity sensor');
  if (mask & 0x02) out.push('Cargo overload');
  if (mask & 0x04) out.push('Cargo imbalance');
  if (mask & 0x08) out.push('Heartbeat timeout');
  if (mask & 0x10) out.push('Workstation command');
  if (mask & 0x20) out.push('Motor overcurrent');
  if (mask & 0x40) out.push('Firmware fault');
  if (mask & 0x100) out.push('Battery low (3S)');
  if (mask & 0x200) out.push('LiDAR obstacle');
  return out;
}

function _cautionLevel(m) {
  return m >= 0.95 ? 'NORMAL' : m >= 0.35 ? 'CAUTION' : m > 0.0 ? 'CRITICAL' : 'ESTOP';
}

// LiPo per-cell voltage → state-of-charge (%). Mirrors panel_node._lipo_percent.
const _LIPO_CURVE = [
  [3.20, 0], [3.30, 5], [3.40, 10], [3.50, 20], [3.60, 35],
  [3.70, 50], [3.80, 60], [3.90, 70], [4.00, 80], [4.10, 90], [4.20, 100],
];
function lipoPercent(packMv, cells) {
  if (packMv <= 0 || cells <= 0) return null;          // absent
  const v = (packMv / 1000) / cells;
  if (v <= _LIPO_CURVE[0][0]) return 0;
  if (v >= _LIPO_CURVE[_LIPO_CURVE.length - 1][0]) return 100;
  for (let i = 0; i < _LIPO_CURVE.length - 1; i++) {
    const [v0, p0] = _LIPO_CURVE[i], [v1, p1] = _LIPO_CURVE[i + 1];
    if (v <= v1) return Math.round(p0 + (p1 - p0) * (v - v0) / (v1 - v0));
  }
  return 100;
}

// PKT_TLM_CORE — 43 bytes (firmware telemetry.c send_core()).
// estop/caution widened to u16 (offsets 6 & 8) — everything after shifts +2.
function mergeCore(prev, payload) {
  if (!payload || payload.length < 43) return prev;
  const dv = _telemDataView(payload);
  const u8 = (o) => dv.getUint8(o), u16 = (o) => dv.getUint16(o, true);
  const u32 = (o) => dv.getUint32(o, true), f32 = (o) => dv.getFloat32(o, true);

  const estopMask = u16(6);
  const cautMod = f32(10);
  const proxBits = u16(39);
  const flags = u8(14);
  const FUNC_NAMES = ['STANDBY','REMOTE_CONTROL','LINE_FOLLOW'];

  return {
    ...prev,
    timestamp_ms: u32(0),
    uptime: u32(0) / 1000.0,
    mode: u8(4) === 0 ? 'SUPERVISED' : 'UNSUPERVISED',
    func: FUNC_NAMES[u8(5)] ?? 'STANDBY',
    ledMode: u8(41),
    ledIndicatorCfg: u8(42),
    estop: { active: estopMask !== 0, virtual: estopMask !== 0,
             sources: _estopSourceLabels(estopMask), sourceMask: estopMask },
    caution: { modifier: cautMod, level: _cautionLevel(cautMod), sources: u16(8) },
    velocity: { ...prev.velocity, v: f32(15), omega: f32(19) },
    position: { x: f32(23), y: f32(27), theta: f32(31) },
    current: { left: u16(35) / 1000.0, right: u16(37) / 1000.0 },
    proximity: {
      fl: !!(proxBits & 0x40), fr: !!(proxBits & 0x80),
      rl: !!(proxBits & 0x100), rr: !!(proxBits & 0x200),
    },
    flags: {
      adc_data: !!(flags & 0x01), hx711_data: !!(flags & 0x02),
      tare_in_progress: !!(flags & 0x08),
    },
  };
}

// PKT_TLM_DRIVE — 32 bytes (firmware telemetry.c send_drive()).
function mergeDrive(prev, payload) {
  if (!payload || payload.length < 32) return prev;
  const dv = _telemDataView(payload);
  const f32 = (o) => dv.getFloat32(o, true), u32 = (o) => dv.getUint32(o, true);

  const vlt = f32(0), vrt = f32(4), vl = f32(8), vr = f32(12);
  const rpm = (mps) => mps * 60.0 / (2 * Math.PI * WHEEL_RADIUS_M);
  return {
    ...prev,
    velocity: { ...prev.velocity, vLeft: vl, vRight: vr },
    encoders: { left: u32(24), right: u32(28), leftRpm: rpm(vl), rightRpm: rpm(vr) },
    motors: { dutyLeft: f32(16), dutyRight: f32(20), vLeftTarget: vlt, vRightTarget: vrt },
    // Reconstruct the chassis setpoint from the per-wheel targets.
    control: { v_target: 0.5 * (vlt + vrt), omega_target: (vrt - vlt) / WHEEL_BASE_M },
  };
}

// PKT_TLM_SENSORS — 18 fixed bytes + variable u16 LiDAR tail (firmware send_sensors()).
// Layout: f32 load_cells[4] (0..15), u16 batt_3s_mv (16),
//         then (len-18)/2 LiDAR segment distances (mm) from offset 18.
function mergeSensors(prev, payload) {
  if (!payload || payload.length < 18) return prev;
  const dv = _telemDataView(payload);
  const f32 = (o) => dv.getFloat32(o, true), u16 = (o) => dv.getUint16(o, true);

  const fl = f32(0), fr = f32(4), rl = f32(8), rr = f32(12);
  const total = fl + fr + rl + rr;
  const v3s = u16(16);
  const nLidar = Math.floor((payload.length - 18) / 2);
  const lidar = Array.from({ length: nLidar }, (_, i) => u16(18 + i * 2));
  return {
    ...prev,
    loadCells: {
      fl, fr, rl, rr, total,
      cog: {
        x: total > 0.001 ? (fr + rr - fl - rl) / total : 0,
        y: total > 0.001 ? (rl + rr - fl - fr) / total : 0,
      },
    },
    lidar,
    battery: { v3s: v3s / 1000.0, pct3s: lipoPercent(v3s, 3) },
  };
}

// PKT_TLM_QTR — 20 bytes (firmware telemetry.c send_qtr()).
function mergeQtr(prev, payload) {
  if (!payload || payload.length < 20) return prev;
  const dv = _telemDataView(payload);
  return {
    ...prev,
    qtr: Array.from({ length: 8 }, (_, i) => dv.getUint16(i * 2, true)),
    qtrLinePosition: dv.getFloat32(16, true),
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
  createFrameParser, useAGVWebSocket, parseLogEntry, lipoPercent,
  makeEmptyTelem, mergeCore, mergeDrive, mergeSensors, mergeQtr,
});
