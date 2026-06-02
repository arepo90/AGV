
// ── GainRow — number input that keeps string state while editing ──────────────
function GainRow({ gainKey, label, min, max, step, value, onChange, theme }) {
  const [draft, setDraft] = React.useState(null); // null = not editing

  function handleFocus() { setDraft(String(value)); }
  function handleChange(e) { setDraft(e.target.value); }
  function handleBlur() {
    if (draft !== null) {
      const n = parseFloat(draft);
      if (!isNaN(n)) onChange(gainKey, n);
      setDraft(null);
    }
  }
  function handleKeyDown(e) {
    if (e.key === 'Enter') e.target.blur();
    if (e.key === 'Escape') { setDraft(null); e.target.blur(); }
  }

  const displayValue = draft !== null ? draft : value;

  return (
    <div style={{ marginBottom: '14px' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '5px' }}>
        <span style={{ fontSize: '11px', fontFamily: theme.monoFont, color: theme.muted }}>{label}</span>
        <input
          type="text"
          inputMode="decimal"
          value={displayValue}
          onFocus={handleFocus}
          onChange={handleChange}
          onBlur={handleBlur}
          onKeyDown={handleKeyDown}
          style={{
            width: '86px', background: theme.bg, border: `1px solid ${draft !== null ? theme.accent : theme.border}`,
            borderRadius: '5px', color: theme.accent, fontFamily: theme.monoFont,
            fontSize: '12px', fontWeight: 700, padding: '3px 6px', textAlign: 'right',
            outline: 'none', transition: 'border-color 0.15s',
          }} />
      </div>
      <input type="range" min={min} max={Math.max(max, value)} step={step} value={value}
        onChange={e => { onChange(gainKey, parseFloat(e.target.value)); setDraft(null); }}
        style={{ width: '100%', accentColor: theme.accent }} />
      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
        <span style={{ fontSize: '9px', fontFamily: theme.monoFont, color: theme.muted + '80' }}>{min}</span>
        <span style={{ fontSize: '9px', fontFamily: theme.monoFont, color: theme.muted + '80' }}>{max}</span>
      </div>
    </div>
  );
}

// ── Velocity PI Tuning Tab ───────────────────────────────────────────────────
// Each wheel runs an independent velocity PI with feedforward (duty = Kff·v_tgt
// + Kp·e + Ki·∫e). Independent left/right gains compensate for mechanical
// asymmetry. There is no outer chassis cascade — (v, ω) splits algebraically.
// Per-wheel live readout — shows what each wheel loop is doing without flipping
// to the Telemetry tab while tuning gains.
function WheelLiveReadout({ telem, theme }) {
  const m = (telem && telem.motors) || {};
  const v = (telem && telem.velocity) || {};
  const rows = [
    { side: 'LEFT',  target: m.vLeftTarget,  actual: v.vLeft,  duty: m.dutyLeft },
    { side: 'RIGHT', target: m.vRightTarget, actual: v.vRight, duty: m.dutyRight },
  ];
  const fmt = (x, unit, digits = 3) =>
    (x === undefined || x === null || Number.isNaN(x)) ? '—' : `${Number(x).toFixed(digits)}${unit}`;
  const Cell = ({ label, value, color }) => (
    <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: '2px' }}>
      <span style={{ fontSize: '9px', fontFamily: theme.monoFont, letterSpacing: '0.1em',
        color: theme.muted, textTransform: 'uppercase' }}>{label}</span>
      <span style={{ fontSize: '14px', fontFamily: theme.monoFont, fontWeight: 700,
        color: color || theme.fg }}>{value}</span>
    </div>
  );
  return (
    <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`,
      borderRadius: '10px', padding: '12px 16px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
      <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted,
        fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>
        Live wheel-loop signals (per wheel)
      </div>
      {rows.map(r => (
        <div key={r.side} style={{ display: 'flex', gap: '14px', alignItems: 'center',
          padding: '8px 10px', background: theme.bg, borderRadius: '7px' }}>
          <span style={{ fontSize: '11px', fontFamily: theme.monoFont, fontWeight: 700,
            color: theme.accent, width: '48px' }}>{r.side}</span>
          <Cell label="Target (m/s)" value={fmt(r.target, '')} color={theme.accent} />
          <Cell label="Encoder (m/s)" value={fmt(r.actual, '')} />
          <Cell label="Error (m/s)"
            value={fmt((r.target !== undefined && r.actual !== undefined) ? (r.target - r.actual) : undefined, '')}
            color={theme.warn} />
          <Cell label="Duty" value={fmt(r.duty, '', 3)} />
        </div>
      ))}
    </div>
  );
}

// Module-scope (stable identity) so the high-rate telemetry re-renders don't
// remount these and steal focus / drop keystrokes from the gain inputs inside.
function GainSection({ title, paramNote, keys, theme, gains, setGain }) {
  return (
    <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
      <div style={{ marginBottom: '14px' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>{title}</div>
        {paramNote && <div style={{ fontSize: '9px', color: theme.accent, fontFamily: theme.monoFont, marginTop: '2px', opacity: 0.8 }}>{paramNote}</div>}
      </div>
      {keys.map(([key, label, min, max, step]) => (
        <GainRow key={key} gainKey={key} label={label} min={min} max={max} step={step}
          value={gains[key]} onChange={setGain} theme={theme} />
      ))}
    </div>
  );
}

function PIDTab({ theme, agv, connected, telem }) {
  // Defaults match config.h WHEEL_*/LINE_*/TRAJECTORY_* macros
  const [gains, setGains] = React.useState({
    left_kp:  0.5,  left_ki:  2.0,  left_kff:  1.0,
    right_kp: 0.5,  right_ki: 2.0,  right_kff: 1.0,
    line_kp:        1.0,  line_ki:        0.0,  line_kd:        0.0,
    line_cruise:    0.3,  traj_cruise:    0.3,  traj_lookahead: 0.50, traj_curv_slowdown: 0.50,
  });
  const [sent, setSent] = React.useState(null);

  // Map gain key → PARAM_UPDATE ID (from proto.h)
  const PARAM_MAP = {
    left_kp:  0x10, left_ki:  0x11, left_kff:  0x12,
    right_kp: 0x13, right_ki: 0x14, right_kff: 0x15,
    line_kp:        0x20, line_ki:        0x21, line_kd:        0x22,
    line_cruise:    0x23, traj_cruise:    0x24, traj_lookahead: 0x25, traj_curv_slowdown: 0x27,
  };

  function setGain(k, v) {
    const n = parseFloat(v);
    if (!isNaN(n)) setGains(g => ({ ...g, [k]: n }));
  }

  async function sendGains() {
    setSent('sending');
    if (connected && agv) {
      try {
        const params = Object.entries(PARAM_MAP).map(([key, id]) => ({ id, value: gains[key] }));
        await agv.sendParamBatch(params);
        setSent('ok');
      } catch(e) {
        console.warn('[agv] param update failed:', e.message);
        setSent('err');
      }
    } else {
      await new Promise(r => setTimeout(r, 800));
      setSent('ok');
    }
    setTimeout(() => setSent(null), 2800);
  }

  return (
    <div style={{ padding: '16px', display: 'flex', flexDirection: 'column', gap: '12px', height: '100%', boxSizing: 'border-box', overflowY: 'auto' }}>
      <WheelLiveReadout telem={telem} theme={theme} />
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '12px' }}>
        <GainSection theme={theme} gains={gains} setGain={setGain} title="Left Wheel — Velocity PI + FF" paramNote="PARAM_LEFT_KP/KI/KFF (0x10–0x12)" keys={[
          ['left_kp',  'Kp',           0, 5,  0.01],
          ['left_ki',  'Ki',           0, 10, 0.01],
          ['left_kff', 'Kff (× v_tgt)', 0, 3,  0.01],
        ]} />
        <GainSection theme={theme} gains={gains} setGain={setGain} title="Right Wheel — Velocity PI + FF" paramNote="PARAM_RIGHT_KP/KI/KFF (0x13–0x15)" keys={[
          ['right_kp',  'Kp',           0, 5,  0.01],
          ['right_ki',  'Ki',           0, 10, 0.01],
          ['right_kff', 'Kff (× v_tgt)', 0, 3,  0.01],
        ]} />
        <GainSection theme={theme} gains={gains} setGain={setGain} title="Line Follow Navigator (PID)" paramNote="PARAM_LINE_K* (0x20–0x22)" keys={[
          ['line_kp',     'Kp',           0, 10,  0.01],
          ['line_ki',     'Ki',           0, 2,   0.01],
          ['line_kd',     'Kd',           0, 2,   0.001],
          ['line_cruise', 'Cruise (m/s)', 0, 1.0, 0.01],
        ]} />
        <div style={{ display: 'flex', flexDirection: 'column', gap: '10px', justifyContent: 'flex-end' }}>
          <GainSection theme={theme} gains={gains} setGain={setGain} title="Trajectory Navigator (Pure Pursuit)" paramNote="PARAM_TRAJ_* (0x24, 0x25, 0x27)" keys={[
            ['traj_cruise',          'Cruise (m/s)',          0, 1.0, 0.01],
            ['traj_lookahead',       'Lookahead Lᴅ (m)',     0.1, 2.0, 0.01],
            ['traj_curv_slowdown',   'Curv. slowdown g (m)',  0,  2.0, 0.01],
          ]} />
          <button onClick={sendGains} style={{
            padding: '14px',
            background: sent === 'ok' ? theme.success : sent === 'err' ? theme.danger : theme.accent,
            border: 'none', borderRadius: '9px', color: '#fff', fontSize: '13px',
            fontFamily: theme.monoFont, fontWeight: 700, cursor: 'pointer', transition: 'background 0.3s',
          }}>
            {sent === 'sending' ? 'Sending PARAM_UPDATE…'
              : sent === 'ok' ? '✓ Applied to firmware'
              : sent === 'err' ? '✗ Send failed — check connection'
              : connected ? 'Send to AGV (PARAM_UPDATE)' : 'Apply (simulated — not connected)'}
          </button>
          <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, lineHeight: 1.6 }}>
            All gains packed into a single <code>PKT_PARAM_UPDATE</code> frame.<br />
            Applied on the next 100 Hz control tick.<br />
            Each wheel runs an independent velocity PI with feedforward (Kff × target).<br />
            No outer cascade — chassis (v, ω) splits algebraically to wheel targets.
          </div>
        </div>
      </div>
    </div>
  );
}

// ── Logs Tab ─────────────────────────────────────────────────────────────────
function LogsTab({ logs, theme, agv, connected }) {
  const endRef = React.useRef(null);
  const [autoScroll, setAutoScroll] = React.useState(true);
  React.useEffect(() => {
    if (autoScroll && endRef.current) endRef.current.scrollIntoView({ block: 'nearest' });
  }, [logs, autoScroll]);

  const levelColor = {
    INFO: theme.fg, WARN: theme.warn, ERROR: theme.danger,
    DEBUG: theme.muted, ESTOP: theme.danger, OK: theme.success,
    CRITICAL: theme.danger,
  };

  async function clearLogs() {
    if (connected && agv) {
      try { await agv.cmdLogClear(); } catch(e) {}
    }
  }

  return (
    <div style={{ padding: '16px', height: '100%', boxSizing: 'border-box', display: 'flex', flexDirection: 'column', gap: '10px' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <span style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase' }}>Event Log</span>
        <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
          <label style={{ display: 'flex', alignItems: 'center', gap: '5px', fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, cursor: 'pointer' }}>
            <input type="checkbox" checked={autoScroll} onChange={e => setAutoScroll(e.target.checked)} style={{ accentColor: theme.accent }} />
            Auto-scroll
          </label>
          <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>{logs.length} events</span>
          <button onClick={clearLogs} style={{ background: 'transparent', border: `1px solid ${theme.border}`, borderRadius: '5px', color: theme.muted, fontSize: '10px', fontFamily: theme.monoFont, padding: '3px 10px', cursor: 'pointer' }}>
            {connected ? 'Clear (firmware + local)' : 'Clear local'}
          </button>
        </div>
      </div>
      <div style={{ flex: 1, overflowY: 'auto', background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '12px', fontFamily: theme.monoFont, fontSize: '11px' }}>
        {logs.map((entry, i) => (
          <div key={i} style={{ display: 'flex', gap: '10px', marginBottom: '4px', alignItems: 'baseline' }}>
            <span style={{ color: theme.muted, minWidth: '72px', flexShrink: 0 }}>{entry.ts}</span>
            <span style={{ color: levelColor[entry.level] || theme.fg, minWidth: '52px', flexShrink: 0, fontWeight: 600 }}>{entry.level}</span>
            <span style={{ color: theme.fg }}>{entry.msg}</span>
          </div>
        ))}
        <div ref={endRef} />
      </div>
    </div>
  );
}

// ── Unsupervised Tab ──────────────────────────────────────────────────────────
function UnsupervisedTab({ telem, setFunc, setMode, theme }) {
  const [selectedFunc, setSelectedFunc] = React.useState('LINE_FOLLOW');
  const [confirmed, setConfirmed] = React.useState(false);
  const [deployed, setDeployed] = React.useState(false);

  const eligibleFuncs = ['LINE_FOLLOW', 'TRAJECTORY_FOLLOW', 'STANDBY'];

  async function deploy() {
    setDeployed(true);
    await setMode('UNSUPERVISED');
    await setFunc(selectedFunc);
  }
  async function recall() {
    setDeployed(false);
    setConfirmed(false);
    await setMode('SUPERVISED');
    await setFunc('STANDBY');
  }

  return (
    <div style={{ padding: '24px', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', height: '100%', gap: '20px', boxSizing: 'border-box' }}>
      <div style={{ maxWidth: '560px', width: '100%', display: 'flex', flexDirection: 'column', gap: '16px' }}>
        <div style={{ background: theme.warn + '18', border: `1.5px solid ${theme.warn}`, borderRadius: '10px', padding: '16px 18px' }}>
          <div style={{ fontSize: '12px', fontFamily: theme.monoFont, fontWeight: 700, color: theme.warn, marginBottom: '6px' }}>⚠ Unsupervised Deployment</div>
          <div style={{ fontSize: '11px', fontFamily: theme.sansFont, color: theme.fg, lineHeight: 1.6 }}>
            The AGV will switch to <code>MODE_UNSUPERVISED</code>. Heartbeat watch is disabled — closing the app will <strong>not</strong> trigger timeout. <code>REMOTE_CONTROL</code> unavailable. Caution modifier baseline <strong>0.5×</strong> (firmware-enforced). Heartbeat timeout: <strong>1000 ms</strong>, grace: <strong>3000 ms</strong>.
          </div>
        </div>

        <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
          <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', marginBottom: '12px' }}>Select Navigation Function</div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
            {eligibleFuncs.map(f => (
              <label key={f} style={{ display: 'flex', alignItems: 'center', gap: '12px', padding: '10px 14px', borderRadius: '7px', border: `1.5px solid ${selectedFunc === f ? theme.accent : theme.border}`, background: selectedFunc === f ? theme.accent + '14' : 'transparent', cursor: deployed ? 'not-allowed' : 'pointer' }}>
                <input type="radio" name="func" value={f} checked={selectedFunc === f} onChange={() => !deployed && setSelectedFunc(f)} style={{ accentColor: theme.accent }} />
                <span style={{ fontFamily: theme.monoFont, fontWeight: 600, fontSize: '12px', color: selectedFunc === f ? theme.accent : theme.fg }}>{f}</span>
                <span style={{ fontSize: '9px', fontFamily: theme.monoFont, color: theme.muted, marginLeft: 'auto' }}>
                  {f === 'LINE_FOLLOW' ? 'FUNC_LINE_FOLLOW (0x02)' : f === 'TRAJECTORY_FOLLOW' ? 'FUNC_TRAJECTORY_FOLLOW (0x03)' : 'FUNC_STANDBY (0x00)'}
                </span>
              </label>
            ))}
          </div>
        </div>

        {!deployed ? (
          <>
            <label style={{ display: 'flex', alignItems: 'center', gap: '10px', cursor: 'pointer' }}>
              <input type="checkbox" checked={confirmed} onChange={e => setConfirmed(e.target.checked)} style={{ accentColor: theme.accent, width: '16px', height: '16px' }} />
              <span style={{ fontSize: '11px', fontFamily: theme.sansFont, color: theme.fg }}>I understand the AGV will operate autonomously. I have verified the environment is clear.</span>
            </label>
            <button disabled={!confirmed} onClick={deploy} style={{
              padding: '14px', background: confirmed ? theme.accent : theme.muted + '40',
              border: 'none', borderRadius: '9px', color: confirmed ? '#fff' : theme.muted,
              fontSize: '13px', fontFamily: theme.monoFont, fontWeight: 700,
              cursor: confirmed ? 'pointer' : 'not-allowed', transition: 'all 0.2s',
            }}>Deploy — CMD_SET_MODE(UNSUPERVISED) + CMD_SET_FUNCTION</button>
          </>
        ) : (
          <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
            <div style={{ background: theme.success + '18', border: `1.5px solid ${theme.success}`, borderRadius: '10px', padding: '14px 18px', fontSize: '12px', fontFamily: theme.monoFont, color: theme.success, fontWeight: 700, textAlign: 'center' }}>
              ● AGV running {selectedFunc} in UNSUPERVISED mode — safe to close app
            </div>
            <button onClick={recall} style={{ padding: '12px', background: 'transparent', border: `1.5px solid ${theme.accent}`, borderRadius: '9px', color: theme.accent, fontSize: '12px', fontFamily: theme.monoFont, fontWeight: 700, cursor: 'pointer' }}>
              Recall — CMD_SET_MODE(SUPERVISED) + CMD_SET_FUNCTION(STANDBY)
            </button>
          </div>
        )}
      </div>
    </div>
  );
}

// ── Comms Tab ────────────────────────────────────────────────────────────────
function CommsTab({ telem, connected, onConnect, onDisconnect, wsUrl, setWsUrl, theme, agv }) {
  const { comms } = telem;
  const [resetting, setResetting] = React.useState(false);

  async function softReset() {
    if (!connected || !agv) return;
    setResetting(true);
    try { await agv.cmdSoftReset(); } catch(e) {}
    setTimeout(() => setResetting(false), 2000);
  }

  return (
    <div style={{ padding: '20px', display: 'flex', flexDirection: 'column', alignItems: 'center', height: '100%', gap: '14px', boxSizing: 'border-box', overflowY: 'auto' }}>
      <div style={{ maxWidth: '560px', width: '100%', display: 'flex', flexDirection: 'column', gap: '14px' }}>

        {/* Connection */}
        <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '18px' }}>
          <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', marginBottom: '14px' }}>WebSocket — ESP32 Bridge</div>
          <div style={{ display: 'flex', gap: '8px', marginBottom: '12px' }}>
            <input value={wsUrl} onChange={e => setWsUrl(e.target.value)} disabled={connected}
              placeholder="ws://192.168.4.1/ws"
              style={{ flex: 1, background: theme.bg, border: `1px solid ${theme.border}`, borderRadius: '7px', color: theme.fg, fontFamily: theme.monoFont, fontSize: '13px', padding: '10px 12px' }} />
            <button onClick={connected ? onDisconnect : onConnect}
              style={{ padding: '10px 18px', background: connected ? theme.danger : theme.accent, border: 'none', borderRadius: '7px', color: '#fff', fontFamily: theme.monoFont, fontSize: '12px', fontWeight: 700, cursor: 'pointer' }}>
              {connected ? 'Disconnect' : 'Connect'}
            </button>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: '8px', marginBottom: connected ? '14px' : '0' }}>
            <div style={{ width: '8px', height: '8px', borderRadius: '50%', background: connected ? theme.success : theme.muted, boxShadow: connected ? `0 0 6px ${theme.success}` : 'none' }}></div>
            <span style={{ fontSize: '12px', fontFamily: theme.monoFont, color: connected ? theme.success : theme.muted }}>
              {connected ? 'Connected — ESP32 relay active · WS binary frames' : 'Not connected'}
            </span>
          </div>
          {connected && (
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr 1fr', gap: '8px' }}>
              {[['Latency', comms.latency + ' ms'], ['RX', comms.rxPackets], ['TX', comms.txPackets], ['Drops', comms.drops]].map(([label, val]) => (
                <div key={label} style={{ background: theme.bg, borderRadius: '7px', padding: '8px 10px' }}>
                  <div style={{ fontSize: '9px', fontFamily: theme.monoFont, color: theme.muted, textTransform: 'uppercase', letterSpacing: '0.1em', marginBottom: '3px' }}>{label}</div>
                  <div style={{ fontSize: '16px', fontFamily: theme.monoFont, fontWeight: 700, color: theme.fg }}>{val}</div>
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Protocol reference */}
        <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
          <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', marginBottom: '12px' }}>Frame Format</div>
          <div style={{ fontFamily: theme.monoFont, fontSize: '11px', color: theme.fg, lineHeight: 2, display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0 20px' }}>
            <div>Magic: <span style={{ color: theme.accent }}>0xAA 0x56</span></div>
            <div>Version: <span style={{ color: theme.accent }}>0x01</span></div>
            <div>Overhead: <span style={{ color: theme.accent }}>8 bytes</span></div>
            <div>Max payload: <span style={{ color: theme.accent }}>255 bytes</span></div>
            <div>CRC: <span style={{ color: theme.accent }}>CRC16-CCITT</span></div>
            <div>CRC covers: <span style={{ color: theme.accent }}>VER+SEQ+TYPE+LEN+PL</span></div>
            <div>Heartbeat: <span style={{ color: theme.accent }}>500 ms interval</span></div>
            <div>HB timeout: <span style={{ color: theme.accent }}>1000 ms</span></div>
            <div>Grace period: <span style={{ color: theme.accent }}>3000 ms</span></div>
            <div>UART baud: <span style={{ color: theme.accent }}>921600</span></div>
            <div>ACK timeout: <span style={{ color: theme.accent }}>50 ms</span></div>
            <div>ACK retries: <span style={{ color: theme.accent }}>3</span></div>
          </div>
        </div>

        {/* Firmware control */}
        {connected && (
          <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
            <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', marginBottom: '12px' }}>Firmware Control</div>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button onClick={softReset} disabled={resetting}
                style={{ flex: 1, padding: '10px', background: 'transparent', border: `1px solid ${theme.danger}`, borderRadius: '7px', color: theme.danger, fontFamily: theme.monoFont, fontSize: '11px', fontWeight: 700, cursor: 'pointer' }}>
                {resetting ? 'Resetting…' : 'PKT_RESET — Soft Reset firmware'}
              </button>
              <button onClick={() => agv && agv.cmdClearAllEstop()}
                style={{ flex: 1, padding: '10px', background: 'transparent', border: `1px solid ${theme.warn}`, borderRadius: '7px', color: theme.warn, fontFamily: theme.monoFont, fontSize: '11px', fontWeight: 700, cursor: 'pointer' }}>
                PKT_RESET(0x00) — Clear all E-STOP
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

// ── Ramp Tab ─────────────────────────────────────────────────────────────────
// Motion-profile editor. Shapes the chassis-level (v, ω) setpoint between the
// navigator and the cascade controller. Applies uniformly to REMOTE_CONTROL,
// LINE_FOLLOW, TRAJECTORY_FOLLOW. See architecture.md §Motion Profiling.
function RampTab({ theme, agv, connected }) {
  const SHAPES = ['LINEAR', 'SCURVE', 'EXPONENTIAL', 'CUSTOM'];
  const SHAPE_ID = { LINEAR: 0, SCURVE: 1, EXPONENTIAL: 2, CUSTOM: 3 };

  const [shape, setShape] = React.useState('LINEAR');
  const [params, setParams] = React.useState({
    accel_lin: 0.8,   // m/s²    PARAM_MAX_LINEAR_ACCEL  (0x03)
    accel_ang: 2.0,   // rad/s²  PARAM_MAX_ANGULAR_ACCEL (0x04)
    jerk_lin:  4.0,   // m/s³    PARAM_RAMP_JERK_LIN     (0x41)
    jerk_ang: 10.0,   // rad/s³  PARAM_RAMP_JERK_ANG     (0x42)
    tau_lin:   0.30,  // s       PARAM_RAMP_TAU_LIN      (0x43)
    tau_ang:   0.20,  // s       PARAM_RAMP_TAU_ANG      (0x44)
  });
  // Custom curve: control points (s, f) including endpoints (0,0) and (1,1).
  const [points, setPoints] = React.useState([
    { s: 0.00, f: 0.00 },
    { s: 0.30, f: 0.10 },
    { s: 0.70, f: 0.90 },
    { s: 1.00, f: 1.00 },
  ]);
  const [sent, setSent] = React.useState(null);
  const [curveSent, setCurveSent] = React.useState(null);

  function setParam(k, v) {
    const n = parseFloat(v);
    if (!isNaN(n)) setParams(p => ({ ...p, [k]: n }));
  }

  // ── Simulate v(t) for a unit step (0 → 1) using the same algorithm as the
  //    firmware. Used to render the live preview chart. axis: 'lin' | 'ang'.
  function simulate(axis) {
    const dt = 0.005;
    const Tmax = 4.0;
    const N = Math.ceil(Tmax / dt);
    const out = new Array(N + 1);
    const v_target = 1.0;          // unit step — preview is normalised
    let v = 0, a = 0;
    const max_a = axis === 'lin' ? params.accel_lin : params.accel_ang;
    const max_j = axis === 'lin' ? params.jerk_lin  : params.jerk_ang;
    const tau   = axis === 'lin' ? params.tau_lin   : params.tau_ang;

    // Custom: ramp duration is derived from |Δv|·peak_slope / max_a (same as fw).
    let segT = 0, segS = 0, peakSlope = 1.0;
    if (shape === 'CUSTOM') {
      const sorted = [...points].sort((p, q) => p.s - q.s);
      let peak = 0;
      for (let i = 1; i < sorted.length; i++) {
        const df = sorted[i].f - sorted[i-1].f;
        const ds = sorted[i].s - sorted[i-1].s;
        if (ds > 0 && df > 0) {
          const slope = df / ds;
          if (slope > peak) peak = slope;
        }
      }
      peakSlope = peak > 1e-3 ? peak : 1.0;
      segT = Math.max(0.001, v_target * peakSlope / Math.max(max_a, 1e-6));
    }
    const evalCurve = (s) => {
      if (s <= 0) return 0;
      if (s >= 1) return 1;
      const sorted = [...points].sort((p, q) => p.s - q.s);
      for (let i = 1; i < sorted.length; i++) {
        if (s <= sorted[i].s) {
          const ds = sorted[i].s - sorted[i-1].s;
          if (ds <= 0) return sorted[i].f;
          const t = (s - sorted[i-1].s) / ds;
          return sorted[i-1].f + t * (sorted[i].f - sorted[i-1].f);
        }
      }
      return 1;
    };

    for (let i = 0; i <= N; i++) {
      out[i] = { t: i * dt, v };
      if (shape === 'LINEAR') {
        const step = max_a * dt;
        const diff = v_target - v;
        v = Math.abs(diff) <= step ? v_target : v + Math.sign(diff) * step;
      } else if (shape === 'SCURVE') {
        const rem = v_target - v;
        let v_drift = (a * a) / (2 * Math.max(max_j, 1e-6));
        if (a < 0) v_drift = -v_drift;
        const rem_after = rem - v_drift;
        let target_a = 0;
        if (Math.abs(rem_after) >= 1e-4) {
          const cap = Math.min(max_a, Math.sqrt(2 * max_j * Math.abs(rem_after)));
          target_a = Math.sign(rem_after) * cap;
        }
        const ad = target_a - a;
        const as = max_j * dt;
        a = Math.abs(ad) <= as ? target_a : a + Math.sign(ad) * as;
        v += a * dt;
      } else if (shape === 'EXPONENTIAL') {
        const alpha = tau < 1e-4 ? 1 : (1 - Math.exp(-dt / tau));
        v += alpha * (v_target - v);
      } else if (shape === 'CUSTOM') {
        segS += dt / segT;
        if (segS >= 1) { segS = 1; v = v_target; }
        else            v = evalCurve(segS);
      }
    }
    return out;
  }

  const linTrace = React.useMemo(() => simulate('lin'),
    [shape, params, points]);                     // eslint-disable-line
  const angTrace = React.useMemo(() => simulate('ang'),
    [shape, params, points]);                     // eslint-disable-line

  // ── Plot the two traces side by side with axes. ─────────────────────────────
  const W = 520, H = 280, PAD = 36;
  const xMax = 4.0, yMax = 1.05;
  const px = (t) => PAD + (t / xMax) * (W - 2 * PAD);
  const py = (v) => (H - PAD) - (v / yMax) * (H - 2 * PAD);
  const tracePath = (trace) => trace.map((p, i) =>
    (i === 0 ? 'M' : 'L') + px(p.t).toFixed(1) + ',' + py(p.v).toFixed(1)).join(' ');

  // ── Curve editor: draggable control points (only enabled when CUSTOM). ──────
  const CW = 320, CH = 240, CPAD = 24;
  const cx = (s) => CPAD + s * (CW - 2 * CPAD);
  const cy = (f) => (CH - CPAD) - f * (CH - 2 * CPAD);
  const fromPx = (px_, py_) => ({
    s: (px_ - CPAD) / (CW - 2 * CPAD),
    f: ((CH - CPAD) - py_) / (CH - 2 * CPAD),
  });
  const [dragIdx, setDragIdx] = React.useState(null);
  const svgRef = React.useRef(null);

  function onPtPointerDown(i, e) {
    if (shape !== 'CUSTOM') return;
    e.preventDefault();
    setDragIdx(i);
  }
  function onSvgPointerMove(e) {
    if (dragIdx === null || !svgRef.current) return;
    const rect = svgRef.current.getBoundingClientRect();
    const { s, f } = fromPx(e.clientX - rect.left, e.clientY - rect.top);
    setPoints(prev => {
      const next = [...prev];
      let ns = Math.max(0, Math.min(1, s));
      let nf = Math.max(0, Math.min(1, f));
      // Endpoints locked at (0,0) and (1,1).
      if (dragIdx === 0)                { ns = 0; nf = 0; }
      if (dragIdx === next.length - 1)  { ns = 1; nf = 1; }
      // Keep s monotonic — clamp between neighbours so the curve stays valid.
      if (dragIdx > 0)                  ns = Math.max(ns, next[dragIdx - 1].s + 0.01);
      if (dragIdx < next.length - 1)    ns = Math.min(ns, next[dragIdx + 1].s - 0.01);
      // Keep f monotonic non-decreasing (firmware rejects otherwise).
      if (dragIdx > 0)                  nf = Math.max(nf, next[dragIdx - 1].f);
      if (dragIdx < next.length - 1)    nf = Math.min(nf, next[dragIdx + 1].f);
      next[dragIdx] = { s: ns, f: nf };
      return next;
    });
  }
  function onSvgPointerUp() { setDragIdx(null); }

  function addPoint() {
    if (points.length >= 8) return;
    setPoints(prev => {
      // Insert at the largest gap.
      let bestGap = 0, bestIdx = 0;
      for (let i = 1; i < prev.length; i++) {
        const gap = prev[i].s - prev[i-1].s;
        if (gap > bestGap) { bestGap = gap; bestIdx = i; }
      }
      const ns = (prev[bestIdx - 1].s + prev[bestIdx].s) / 2;
      const nf = (prev[bestIdx - 1].f + prev[bestIdx].f) / 2;
      return [...prev.slice(0, bestIdx), { s: ns, f: nf }, ...prev.slice(bestIdx)];
    });
  }
  function removePoint() {
    if (points.length <= 2) return;
    // Drop the middle-most interior point.
    setPoints(prev => prev.length <= 2 ? prev :
      [...prev.slice(0, Math.floor(prev.length / 2)),
       ...prev.slice(Math.floor(prev.length / 2) + 1)]);
  }
  function resetCurve() {
    setPoints([{ s: 0, f: 0 }, { s: 0.3, f: 0.1 }, { s: 0.7, f: 0.9 }, { s: 1, f: 1 }]);
  }

  async function sendParams() {
    setSent('sending');
    if (connected && agv) {
      try {
        const batch = [
          { id: AGV_PROTO.PARAM.RAMP_SHAPE,          value: SHAPE_ID[shape] },
          { id: AGV_PROTO.PARAM.MAX_LINEAR_ACCEL,    value: params.accel_lin },
          { id: AGV_PROTO.PARAM.MAX_ANGULAR_ACCEL,   value: params.accel_ang },
          { id: AGV_PROTO.PARAM.RAMP_JERK_LIN,       value: params.jerk_lin },
          { id: AGV_PROTO.PARAM.RAMP_JERK_ANG,       value: params.jerk_ang },
          { id: AGV_PROTO.PARAM.RAMP_TAU_LIN,        value: params.tau_lin },
          { id: AGV_PROTO.PARAM.RAMP_TAU_ANG,        value: params.tau_ang },
        ];
        await agv.sendParamBatch(batch);
        setSent('ok');
      } catch (e) {
        console.warn('[agv] ramp params failed:', e.message);
        setSent('err');
      }
    } else {
      await new Promise(r => setTimeout(r, 600));
      setSent('ok');
    }
    setTimeout(() => setSent(null), 2400);
  }

  async function sendCurve() {
    setCurveSent('sending');
    if (!(connected && agv)) {
      await new Promise(r => setTimeout(r, 600));
      setCurveSent('ok');
      setTimeout(() => setCurveSent(null), 2400);
      return;
    }
    try {
      await agv.cmdRampCurveBegin();
      // Send sorted to satisfy firmware monotonicity check.
      const sorted = [...points].sort((p, q) => p.s - q.s);
      for (const pt of sorted) await agv.cmdRampCurveAddPoint(pt.s, pt.f);
      await agv.cmdRampCurveCommit();
      setCurveSent('ok');
    } catch (e) {
      console.warn('[agv] ramp curve upload failed:', e.message);
      setCurveSent('err');
    }
    setTimeout(() => setCurveSent(null), 2800);
  }

  // ── Param rows shown for each shape. ────────────────────────────────────────
  const rowsForShape = {
    LINEAR: [
      ['accel_lin', 'Max accel — linear (m/s²)',    0, 5,  0.01],
      ['accel_ang', 'Max accel — angular (rad/s²)', 0, 10, 0.05],
    ],
    SCURVE: [
      ['accel_lin', 'Max accel — linear (m/s²)',    0, 5,  0.01],
      ['accel_ang', 'Max accel — angular (rad/s²)', 0, 10, 0.05],
      ['jerk_lin',  'Max jerk — linear (m/s³)',     0, 20, 0.05],
      ['jerk_ang',  'Max jerk — angular (rad/s³)',  0, 40, 0.1],
    ],
    EXPONENTIAL: [
      ['tau_lin', 'Time constant — linear (s)',  0.01, 2.0, 0.01],
      ['tau_ang', 'Time constant — angular (s)', 0.01, 2.0, 0.01],
    ],
    CUSTOM: [
      ['accel_lin', 'Max accel — linear (m/s²)',    0, 5,  0.01],
      ['accel_ang', 'Max accel — angular (rad/s²)', 0, 10, 0.05],
    ],
  };

  return (
    <div style={{ padding: '16px', display: 'flex', flexDirection: 'column', gap: '12px', height: '100%', boxSizing: 'border-box', overflowY: 'auto' }}>
      <div style={{ display: 'grid', gridTemplateColumns: '320px 1fr', gap: '12px' }}>
        {/* Left column: shape + params */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
          <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
            <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600, marginBottom: '10px' }}>
              Profile shape
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px' }}>
              {SHAPES.map(s => (
                <button key={s} onClick={() => setShape(s)} style={{
                  padding: '8px 10px', borderRadius: '6px', cursor: 'pointer',
                  background: shape === s ? theme.accent : theme.bg,
                  color: shape === s ? '#fff' : theme.fg,
                  border: `1px solid ${shape === s ? theme.accent : theme.border}`,
                  fontFamily: theme.monoFont, fontSize: '11px', fontWeight: 700,
                  letterSpacing: '0.04em',
                }}>{s}</button>
              ))}
            </div>
            <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, marginTop: '10px', lineHeight: 1.55 }}>
              {shape === 'LINEAR'      && 'Constant max accel. Trapezoidal v(t) on a step. Simplest.'}
              {shape === 'SCURVE'      && 'Jerk-limited. Accel itself ramps. Smoothest on cargo / wheels.'}
              {shape === 'EXPONENTIAL' && '1st-order filter. Asymptotic — never quite reaches target.'}
              {shape === 'CUSTOM'      && 'Operator-drawn normalised curve. Drag control points →'}
            </div>
          </div>
          <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
            <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600, marginBottom: '12px' }}>
              Magnitude
            </div>
            {rowsForShape[shape].map(([key, label, min, max, step]) => (
              <GainRow key={key} gainKey={key} label={label} min={min} max={max} step={step}
                value={params[key]} onChange={setParam} theme={theme} />
            ))}
          </div>
          <button onClick={sendParams} style={{
            padding: '12px',
            background: sent === 'ok' ? theme.success : sent === 'err' ? theme.danger : theme.accent,
            border: 'none', borderRadius: '9px', color: '#fff', fontSize: '12px',
            fontFamily: theme.monoFont, fontWeight: 700, cursor: 'pointer', transition: 'background 0.3s',
          }}>
            {sent === 'sending' ? 'Sending PARAM_UPDATE…'
              : sent === 'ok' ? '✓ Profile applied'
              : sent === 'err' ? '✗ Send failed'
              : connected ? 'Send profile to AGV' : 'Apply (simulated)'}
          </button>
        </div>

        {/* Right column: live preview + (custom) curve editor */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
          <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
              <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>
                Step response (0 → 1 normalised)
              </div>
              <div style={{ display: 'flex', gap: '14px', fontSize: '10px', fontFamily: theme.monoFont }}>
                <span style={{ color: theme.accent }}>● linear</span>
                <span style={{ color: theme.warning || '#ffb020' }}>● angular</span>
              </div>
            </div>
            <svg width={W} height={H} style={{ width: '100%', height: 'auto', maxWidth: W }}>
              {/* Grid */}
              {[0, 0.25, 0.5, 0.75, 1].map(g => (
                <line key={'h' + g} x1={PAD} x2={W - PAD} y1={py(g)} y2={py(g)}
                      stroke={theme.border} strokeDasharray="2 3" />
              ))}
              {[0, 1, 2, 3, 4].map(g => (
                <line key={'v' + g} x1={px(g)} x2={px(g)} y1={PAD} y2={H - PAD}
                      stroke={theme.border} strokeDasharray="2 3" />
              ))}
              {/* Target */}
              <line x1={PAD} x2={W - PAD} y1={py(1)} y2={py(1)} stroke={theme.muted} strokeDasharray="4 4" opacity="0.5" />
              {/* Traces */}
              <path d={tracePath(linTrace)} fill="none" stroke={theme.accent} strokeWidth="2" />
              <path d={tracePath(angTrace)} fill="none" stroke={theme.warning || '#ffb020'} strokeWidth="2" strokeDasharray="5 3" />
              {/* Axes */}
              <line x1={PAD} y1={H - PAD} x2={W - PAD} y2={H - PAD} stroke={theme.fg} />
              <line x1={PAD} y1={PAD}     x2={PAD}     y2={H - PAD} stroke={theme.fg} />
              {[0, 1, 2, 3, 4].map(g => (
                <text key={'tx' + g} x={px(g)} y={H - PAD + 14}
                      textAnchor="middle" fontSize="9" fontFamily={theme.monoFont} fill={theme.muted}>{g}s</text>
              ))}
              {[0, 0.5, 1].map(g => (
                <text key={'ty' + g} x={PAD - 6} y={py(g) + 3}
                      textAnchor="end" fontSize="9" fontFamily={theme.monoFont} fill={theme.muted}>{g.toFixed(1)}</text>
              ))}
            </svg>
          </div>

          {shape === 'CUSTOM' && (
            <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>
                  Velocity curve f(s) — drag points
                </div>
                <div style={{ display: 'flex', gap: '6px' }}>
                  <button onClick={addPoint} disabled={points.length >= 8} style={{
                    padding: '5px 10px', borderRadius: '5px', cursor: points.length >= 8 ? 'not-allowed' : 'pointer',
                    background: theme.bg, border: `1px solid ${theme.border}`, color: theme.fg,
                    fontFamily: theme.monoFont, fontSize: '10px',
                  }}>+ point</button>
                  <button onClick={removePoint} disabled={points.length <= 2} style={{
                    padding: '5px 10px', borderRadius: '5px', cursor: points.length <= 2 ? 'not-allowed' : 'pointer',
                    background: theme.bg, border: `1px solid ${theme.border}`, color: theme.fg,
                    fontFamily: theme.monoFont, fontSize: '10px',
                  }}>− point</button>
                  <button onClick={resetCurve} style={{
                    padding: '5px 10px', borderRadius: '5px', cursor: 'pointer',
                    background: theme.bg, border: `1px solid ${theme.border}`, color: theme.fg,
                    fontFamily: theme.monoFont, fontSize: '10px',
                  }}>reset</button>
                </div>
              </div>
              <svg ref={svgRef} width={CW} height={CH}
                onPointerMove={onSvgPointerMove}
                onPointerUp={onSvgPointerUp}
                onPointerLeave={onSvgPointerUp}
                style={{ width: '100%', height: 'auto', maxWidth: CW, touchAction: 'none', cursor: dragIdx !== null ? 'grabbing' : 'crosshair' }}>
                {/* Grid */}
                {[0, 0.25, 0.5, 0.75, 1].map(g => (
                  <React.Fragment key={'g' + g}>
                    <line x1={cx(0)} x2={cx(1)} y1={cy(g)} y2={cy(g)} stroke={theme.border} strokeDasharray="2 3" />
                    <line x1={cx(g)} x2={cx(g)} y1={cy(0)} y2={cy(1)} stroke={theme.border} strokeDasharray="2 3" />
                  </React.Fragment>
                ))}
                {/* Diagonal (linear reference) */}
                <line x1={cx(0)} y1={cy(0)} x2={cx(1)} y2={cy(1)} stroke={theme.muted} strokeDasharray="4 4" opacity="0.5" />
                {/* Curve */}
                <path d={[...points].sort((a, b) => a.s - b.s).map((p, i) =>
                  (i === 0 ? 'M' : 'L') + cx(p.s).toFixed(1) + ',' + cy(p.f).toFixed(1)).join(' ')}
                  fill="none" stroke={theme.accent} strokeWidth="2.5" />
                {/* Axes */}
                <line x1={cx(0)} y1={cy(0)} x2={cx(1)} y2={cy(0)} stroke={theme.fg} />
                <line x1={cx(0)} y1={cy(0)} x2={cx(0)} y2={cy(1)} stroke={theme.fg} />
                <text x={cx(0.5)} y={CH - 4} textAnchor="middle" fontSize="9" fontFamily={theme.monoFont} fill={theme.muted}>progress s</text>
                <text x={6} y={cy(0.5)} fontSize="9" fontFamily={theme.monoFont} fill={theme.muted} transform={`rotate(-90, 6, ${cy(0.5)})`}>f(s)</text>
                {/* Control points */}
                {points.map((p, i) => {
                  const locked = i === 0 || i === points.length - 1;
                  return (
                    <circle key={i} cx={cx(p.s)} cy={cy(p.f)} r="7"
                      fill={locked ? theme.muted : theme.accent}
                      stroke="#fff" strokeWidth="2"
                      style={{ cursor: locked ? 'not-allowed' : 'grab' }}
                      onPointerDown={e => onPtPointerDown(i, e)} />
                  );
                })}
              </svg>
              <button onClick={sendCurve} style={{
                marginTop: '10px', width: '100%', padding: '10px',
                background: curveSent === 'ok' ? theme.success : curveSent === 'err' ? theme.danger : theme.accent,
                border: 'none', borderRadius: '7px', color: '#fff', fontSize: '11px',
                fontFamily: theme.monoFont, fontWeight: 700, cursor: 'pointer', transition: 'background 0.3s',
              }}>
                {curveSent === 'sending' ? 'Uploading curve…'
                  : curveSent === 'ok' ? '✓ Curve uploaded'
                  : curveSent === 'err' ? '✗ Upload failed'
                  : connected ? 'Upload curve to AGV (CMD_LOAD_RAMP_CURVE)' : 'Upload (simulated)'}
              </button>
              <div style={{ fontSize: '9px', fontFamily: theme.monoFont, color: theme.muted, marginTop: '8px', lineHeight: 1.55 }}>
                f(s) maps normalised time (s) → normalised velocity. Steepest segment ≡ <code>max_accel</code>.
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

Object.assign(window, { GainRow, PIDTab, RampTab, LogsTab, UnsupervisedTab, CommsTab });
