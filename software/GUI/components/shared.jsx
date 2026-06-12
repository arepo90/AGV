
// Shared utilities, hooks, and base components

// ── Persistent panel state ──────────────────────────────────────────────────
// Tuning panels live in tabs that unmount when you navigate away, which would
// snap their React.useState back to defaults on return. usePersistentState keeps
// the value in a module-level store (so it survives tab switches) backed by
// localStorage (so it survives page reloads). Same API as useState, including
// updater functions. Values must be JSON-serialisable.
const _persistentStore = {};
const _LS_PREFIX = 'agv.';

function readPersistent(key, fallback) {
  if (key in _persistentStore) return _persistentStore[key];
  try {
    const raw = window.localStorage.getItem(_LS_PREFIX + key);
    if (raw !== null) {
      const v = JSON.parse(raw);
      _persistentStore[key] = v;
      return v;
    }
  } catch (_) { /* unavailable/corrupt localStorage → fall back */ }
  return fallback;
}

function usePersistentState(key, initial) {
  const [val, setVal] = React.useState(() => readPersistent(key, initial));
  const set = React.useCallback((next) => {
    setVal(prev => {
      const resolved = typeof next === 'function' ? next(prev) : next;
      _persistentStore[key] = resolved;
      try { window.localStorage.setItem(_LS_PREFIX + key, JSON.stringify(resolved)); } catch (_) {}
      return resolved;
    });
  }, [key]);
  return [val, set];
}

// ── Firmware param sync ─────────────────────────────────────────────────────
// Firmware params live in RAM only; the GUI is the persistence layer and must
// re-push everything on each connect, else firmware reboots (or GUI reloads)
// leave the two silently out of sync. One entry per firmware-bound tunable:
// the persisted key, its default (mirrors the owning control), and how the
// stored value expands to [PARAM name, number] pairs.
// Not synced: ramp custom-curve points (command flow, not params) and LED
// mode/base (firmware-owned, echoed back in TLM_CORE).
const PARAM_SYNC = [
  { key: 'standby.weightCaution', def: 80,    pairs: v => [['WEIGHT_CAUTION_KG', v]] },
  { key: 'standby.weightEstop',   def: 100,   pairs: v => [['WEIGHT_ESTOP_KG', v]] },
  { key: 'standby.lidarCaution',  def: 800,   pairs: v => [['LIDAR_CAUTION_MM', v]] },
  { key: 'standby.lidarCritical', def: 400,   pairs: v => [['LIDAR_CRITICAL_MM', v]] },
  { key: 'standby.lidarEstop',    def: 200,   pairs: v => [['LIDAR_ESTOP_MM', v]] },
  { key: 'standby.battCaution',   def: 10500, pairs: v => [['BATT_3S_CAUTION_MV', v]] },
  { key: 'standby.battEstop',     def: 9900,  pairs: v => [['BATT_3S_ESTOP_MV', v]] },
  { key: 'pid.gains',
    def: { left_kp: 5, left_ki: 0.5, left_kff: 0, right_kp: 5, right_ki: 0.5, right_kff: 0,
           line_kp: 3.0, line_ki: 0.0, line_kd: 0.0, line_cruise: 0.2 },
    pairs: g => [
      ['LEFT_KP', g.left_kp], ['LEFT_KI', g.left_ki], ['LEFT_KFF', g.left_kff],
      ['RIGHT_KP', g.right_kp], ['RIGHT_KI', g.right_ki], ['RIGHT_KFF', g.right_kff],
      ['LINE_KP', g.line_kp], ['LINE_KI', g.line_ki], ['LINE_KD', g.line_kd],
      ['LINE_CRUISE_MPS', g.line_cruise],
    ] },
  // After pid.gains: the Line Follow tab's cruise slider duplicates 0x23 and
  // should win (last tuple applied wins on the firmware side).
  { key: 'lineFollow.cruise',     def: 0.3,   pairs: v => [['LINE_CRUISE_MPS', v]] },
  { key: 'lineFollow.lostThresh', def: 300,   pairs: v => [['QTR_LINE_LOST_THRESH', v]] },
  { key: 'lineFollow.tBlack',     def: 2500,  pairs: v => [['LINE_T_BLACK', v]] },
  { key: 'ramp.shape', def: 'EXPONENTIAL',
    pairs: s => [['RAMP_SHAPE', ({ LINEAR: 0, SCURVE: 1, EXPONENTIAL: 2, CUSTOM: 3 })[s] ?? 2]] },
  { key: 'ramp.params',
    def: { accel_lin: 0.8, accel_ang: 2.0, jerk_lin: 4.0, jerk_ang: 10.0, tau_lin: 0.01, tau_ang: 0.01 },
    pairs: p => [
      ['MAX_LINEAR_ACCEL', p.accel_lin], ['MAX_ANGULAR_ACCEL', p.accel_ang],
      ['RAMP_JERK_LIN', p.jerk_lin], ['RAMP_JERK_ANG', p.jerk_ang],
      ['RAMP_TAU_LIN', p.tau_lin], ['RAMP_TAU_ANG', p.tau_ang],
    ] },
];

async function syncParamsToFirmware(agv) {
  const P = (window.AGV_PROTO || {}).PARAM || {};
  const tuples = [];
  for (const ent of PARAM_SYNC) {
    let list;
    try { list = ent.pairs(readPersistent(ent.key, ent.def)); } catch (_) { continue; }
    for (const [name, value] of list) {
      if (P[name] === undefined || typeof value !== 'number' || !isFinite(value)) continue;
      tuples.push({ id: P[name], value });
    }
  }
  // 5 bytes per tuple; chunk well under the 255-byte payload cap.
  for (let i = 0; i < tuples.length; i += 40)
    await agv.sendParamBatch(tuples.slice(i, i + 40));
}

// ── Mock telemetry data stream ──────────────────────────────────────────────
// Mock-state defaults. Anything here either lives in the wire telemetry or is
// a workstation-only field (comms.*). Wire-backed fields are overwritten in
// applyRealTelemetry as soon as the first telemetry frame arrives.
function createMockTelemetry() {
  return {
    mode: 'SUPERVISED',
    func: 'STANDBY',
    estop: { active: false, virtual: false, sources: [], sourceMask: 0 },
    caution: { modifier: 1.0, level: 'NORMAL', sources: 0 },
    velocity: { v: 0.0, omega: 0.0, vLeft: 0.0, vRight: 0.0 },
    position: { x: 0.0, y: 0.0, theta: 0.0 },
    encoders: { left: 0, right: 0, leftRpm: 0, rightRpm: 0 },
    loadCells: { fl: 0, fr: 0, rl: 0, rr: 0, total: 0, cog: { x: 0, y: 0 } },
    proximity: { fl: false, fr: false, rl: false, rr: false },
    battery: { v3s: 0, pct3s: null },
    current: { left: 0, right: 0 },
    qtr: [0,0,0,0,0,0,0,0],
    control: { v_target: 0, omega_target: 0 },
    motors: { dutyLeft: 0, dutyRight: 0 },
    flags: { adc_data: false, hx711_data: false, tare_in_progress: false },
    log: { pending: 0, dropped: 0 },
    ledMode: 0,
    ledIndicatorCfg: 0,
    lidar: [],
    comms: { connected: false, latency: 0, rxPackets: 0, txPackets: 0, drops: 0 },
    uptime: 0,
  };
}

function useTelemetry(connected) {
  const [telem, setTelem] = React.useState(createMockTelemetry());
  const lastTelemMsRef = React.useRef(0);
  const [telemTimeout, setTelemTimeout] = React.useState(false);

  function applyRealTelemetry(t) {
    if (!t) return;
    lastTelemMsRef.current = Date.now();
    setTelemTimeout(false);
    setTelem(prev => ({
      ...prev,
      mode: t.mode, func: t.func,
      estop: t.estop, caution: t.caution,
      velocity: t.velocity, position: t.position,
      encoders: t.encoders, loadCells: t.loadCells,
      proximity: t.proximity, battery: t.battery, current: t.current,
      qtr: t.qtr, qtrLinePosition: t.qtrLinePosition, control: t.control, flags: t.flags,
      motors: t.motors,
      log: t.log,
      ledMode: t.ledMode,
      ledIndicatorCfg: t.ledIndicatorCfg,
      lidar: t.lidar,
      uptime: (t.timestamp_ms ?? 0) / 1000.0,
      comms: { ...prev.comms, connected: true, rxPackets: prev.comms.rxPackets + 1 },
    }));
  }

  React.useEffect(() => {
    if (!connected) {
      lastTelemMsRef.current = 0;
      setTelemTimeout(false);
      setTelem(t => ({ ...t, comms: { ...t.comms, connected: false } }));
      return;
    }

    const watchdog = setInterval(() => {
      if (lastTelemMsRef.current === 0) return;
      if (Date.now() - lastTelemMsRef.current > 2000) {
        setTelemTimeout(true);
        setTelem(t => ({ ...t, comms: { ...t.comms, connected: false } }));
      }
    }, 500);

    return () => clearInterval(watchdog);
  }, [connected]);

  function setFunc(f) { setTelem(t => ({ ...t, func: f })); }
  function setMode(m) { setTelem(t => ({ ...t, mode: m })); }
  function triggerEstop() {
    setTelem(t => ({
      ...t,
      func: 'STANDBY',
      estop: { active: true, virtual: true, sources: ['Workstation command'], sourceMask: 0x10 },
    }));
  }
  function clearEstop() {
    setTelem(t => ({
      ...t,
      estop: { active: false, virtual: false, sources: [], sourceMask: 0 },
    }));
  }
  function setCaution(level) {
    const map = { NORMAL: 1.0, CAUTION: 0.5, CRITICAL: 0.2 };
    setTelem(t => ({ ...t, caution: { ...t.caution, level, modifier: map[level] ?? 1.0 } }));
  }

  function updateCommsStats({ latency, tx, drop }) {
    setTelem(t => ({
      ...t,
      comms: {
        ...t.comms,
        // First sample sets directly; subsequent samples use EMA (α=0.25) to smooth jitter
        latency:   latency !== undefined
          ? (t.comms.latency === 0 ? latency : Math.round(0.25 * latency + 0.75 * t.comms.latency))
          : t.comms.latency,
        txPackets: tx   !== undefined ? t.comms.txPackets + tx   : t.comms.txPackets,
        drops:     drop !== undefined ? t.comms.drops    + drop  : t.comms.drops,
      },
    }));
  }

  return { telem, telemTimeout, setFunc, setMode, triggerEstop, clearEstop, setCaution, applyRealTelemetry, updateCommsStats };
}

// ── Format helpers ─────────────────────────────────────────────────────────
function fmt(v, decimals = 2, unit = '') {
  if (v === null || v === undefined) return '—';
  return Number(v).toFixed(decimals) + (unit ? ' ' + unit : '');
}
function fmtUptime(s) {
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = Math.floor(s % 60);
  return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(sec).padStart(2,'0')}`;
}

// Export
Object.assign(window, { useTelemetry, createMockTelemetry, fmt, fmtUptime });
