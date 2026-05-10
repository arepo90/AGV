
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

// ── PID Tuning Tab ───────────────────────────────────────────────────────────
function PIDTab({ theme, agv, connected }) {
  // Real defaults from config.h
  const [gains, setGains] = React.useState({
    outer_lin_kp:   1.0,  outer_lin_ki:   0.5,  outer_lin_kd:   0.0,
    outer_ang_kp:   1.0,  outer_ang_ki:   0.5,  outer_ang_kd:   0.0,
    inner_left_kp:  80.0, inner_left_ki:  200.0, inner_left_kd:  0.0,
    inner_right_kp: 80.0, inner_right_ki: 200.0, inner_right_kd: 0.0,
    line_kp:        4.0,  line_ki:        0.0,  line_kd:        0.2,
    line_cruise:    0.3,  traj_cruise:    0.3,  traj_lookahead: 0.20,
  });
  const [sent, setSent] = React.useState(null);

  // Map gain key → PARAM_UPDATE ID (from comms.h)
  const PARAM_MAP = {
    outer_lin_kp:   0x16, outer_lin_ki:   0x17, outer_lin_kd:   0x18,
    outer_ang_kp:   0x19, outer_ang_ki:   0x1A, outer_ang_kd:   0x1B,
    inner_left_kp:  0x10, inner_left_ki:  0x11, inner_left_kd:  0x12,
    inner_right_kp: 0x13, inner_right_ki: 0x14, inner_right_kd: 0x15,
    line_kp:        0x20, line_ki:        0x21, line_kd:        0x22,
    line_cruise:    0x23, traj_cruise:    0x24, traj_lookahead: 0x25,
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

  const Section = ({ title, paramNote, keys }) => (
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

  return (
    <div style={{ padding: '16px', display: 'flex', flexDirection: 'column', gap: '12px', height: '100%', boxSizing: 'border-box', overflowY: 'auto' }}>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '12px' }}>
        <Section title="Outer Loop — Linear (PI)" paramNote="PARAM_OUTER_LIN_K* (0x16–0x18)" keys={[
          ['outer_lin_kp', 'Kp', 0, 10, 0.01],
          ['outer_lin_ki', 'Ki', 0, 5,  0.01],
          ['outer_lin_kd', 'Kd', 0, 2,  0.001],
        ]} />
        <Section title="Outer Loop — Angular (PI)" paramNote="PARAM_OUTER_ANG_K* (0x19–0x1B)" keys={[
          ['outer_ang_kp', 'Kp', 0, 10, 0.01],
          ['outer_ang_ki', 'Ki', 0, 5,  0.01],
          ['outer_ang_kd', 'Kd', 0, 2,  0.001],
        ]} />
        <Section title="Line Follow Navigator (PID)" paramNote="PARAM_LINE_K* (0x20–0x22)" keys={[
          ['line_kp',     'Kp',           0, 10,  0.01],
          ['line_ki',     'Ki',           0, 2,   0.01],
          ['line_kd',     'Kd',           0, 2,   0.001],
          ['line_cruise', 'Cruise (m/s)', 0, 1.0, 0.01],
        ]} />
        <Section title="Inner Loop — Left Wheel (PID)" paramNote="PARAM_INNER_K*_LEFT (0x10–0x12)" keys={[
          ['inner_left_kp', 'Kp', 0, 500, 1],
          ['inner_left_ki', 'Ki', 0, 500, 1],
          ['inner_left_kd', 'Kd', 0, 10,  0.001],
        ]} />
        <Section title="Inner Loop — Right Wheel (PID)" paramNote="PARAM_INNER_K*_RIGHT (0x13–0x15)" keys={[
          ['inner_right_kp', 'Kp', 0, 500, 1],
          ['inner_right_ki', 'Ki', 0, 500, 1],
          ['inner_right_kd', 'Kd', 0, 10,  0.001],
        ]} />
        <div style={{ display: 'flex', flexDirection: 'column', gap: '10px', justifyContent: 'flex-end' }}>
          <Section title="Trajectory Navigator" paramNote="PARAM_TRAJ_* (0x24–0x25)" keys={[
            ['traj_cruise',    'Cruise (m/s)',    0, 1.0, 0.01],
            ['traj_lookahead', 'Lookahead (m)',   0, 1.0, 0.01],
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
            Applied on the next 200 Hz control tick.<br />
            Inner loop gains are large by design (80/200 default).
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

Object.assign(window, { GainRow, PIDTab, LogsTab, UnsupervisedTab, CommsTab });
