
// Shared utilities, hooks, and base components

// ── Mock telemetry data stream ──────────────────────────────────────────────
function createMockTelemetry() {
  return {
    mode: 'SUPERVISED',
    func: 'STANDBY',
    estop: { active: false, virtual: false, physical: false, sources: [] },
    caution: { modifier: 1.0, level: 'NORMAL', sources: [] },
    velocity: { v: 0.0, omega: 0.0, vLeft: 0.0, vRight: 0.0 },
    position: { x: 0.0, y: 0.0, theta: 0.0 },
    encoders: { left: 0, right: 0, leftRpm: 0, rightRpm: 0 },
    loadCells: { fl: 0, fr: 0, rl: 0, rr: 0, total: 0, cog: { x: 0, y: 0 } },
    imu: { roll: 0, pitch: 0, yaw: 0, ax: 0, ay: 0, az: 9.81 },
    proximity: { front: false, rear: false, left: false, right: false },
    current: { left: 0, right: 0 },
    comms: { connected: false, latency: 0, rxPackets: 0, txPackets: 0, drops: 0 },
    lineError: 0,
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
      imu: { ...prev.imu, ...t.imu },
      proximity: t.proximity, current: t.current,
      qtr: t.qtr, control: t.control, flags: t.flags,
      motors: t.motors,
      log: t.log,
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
      estop: { active: true, virtual: true, physical: false, sources: ['Workstation command'] },
    }));
  }
  function clearEstop() {
    setTelem(t => ({
      ...t,
      estop: { active: false, virtual: false, physical: false, sources: [] },
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
