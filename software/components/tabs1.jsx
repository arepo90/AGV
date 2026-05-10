
// ── Dashboard Tab ───────────────────────────────────────────────────────────
function DashboardTab({ telem, setFunc, setMode, theme }) {
  const { func, mode, estop, caution, comms } = telem;

  const functions = ['STANDBY', 'REMOTE_CONTROL', 'LINE_FOLLOW', 'TRAJECTORY_FOLLOW'];
  const funcColors = {
    STANDBY: theme.accent,
    REMOTE_CONTROL: theme.warn,
    LINE_FOLLOW: theme.success,
    TRAJECTORY_FOLLOW: theme.success,
  };
  const funcDescriptions = {
    STANDBY: 'Motors idle. System active, awaiting command.',
    REMOTE_CONTROL: 'Direct velocity commands from workstation.',
    LINE_FOLLOW: 'Following floor line via QTR-8A array.',
    TRAJECTORY_FOLLOW: 'Executing pre-loaded waypoint sequence.',
  };

  const bgCard = theme.cardBg;
  const border = theme.border;

  return (
    <div style={{ padding: '16px', display: 'grid', gridTemplateColumns: '1fr 1fr', gridTemplateRows: 'auto auto', gap: '14px', height: '100%', boxSizing: 'border-box' }}>

      {/* Mode card */}
      <div style={{ background: bgCard, border: `1px solid ${border}`, borderRadius: '10px', padding: '18px' }}>
        <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, marginBottom: '10px', fontFamily: theme.monoFont, textTransform: 'uppercase' }}>Operating Mode</div>
        <div style={{ display: 'flex', gap: '8px' }}>
          {['SUPERVISED', 'UNSUPERVISED'].map(m => (
            <button key={m} onClick={() => setMode(m)}
              style={{
                flex: 1, padding: '10px 6px', borderRadius: '7px', fontSize: '11px',
                fontFamily: theme.monoFont, fontWeight: 600, letterSpacing: '0.06em',
                border: `1.5px solid ${mode === m ? theme.accent : border}`,
                background: mode === m ? theme.accent + '18' : 'transparent',
                color: mode === m ? theme.accent : theme.muted,
                cursor: 'pointer', transition: 'all 0.15s',
              }}>{m}</button>
          ))}
        </div>
        <div style={{ marginTop: '10px', fontSize: '11px', color: theme.muted, fontFamily: theme.monoFont }}>
          {mode === 'SUPERVISED' ? '● Heartbeat expected. Remote functions active.' : '○ Autonomous. No heartbeat required.'}
        </div>
      </div>

      {/* Caution modifier */}
      <div style={{ background: bgCard, border: `1px solid ${border}`, borderRadius: '10px', padding: '18px' }}>
        <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, marginBottom: '10px', fontFamily: theme.monoFont, textTransform: 'uppercase' }}>Caution Modifier</div>
        <div style={{ display: 'flex', alignItems: 'flex-end', gap: '10px' }}>
          <div style={{ fontSize: '38px', fontFamily: theme.monoFont, fontWeight: 700, color: caution.level === 'NORMAL' ? theme.success : caution.level === 'CAUTION' ? theme.warn : theme.danger, lineHeight: 1 }}>
            {(caution.modifier * 100).toFixed(0)}<span style={{ fontSize: '16px' }}>%</span>
          </div>
          <div style={{ fontSize: '12px', fontFamily: theme.monoFont, color: caution.level === 'NORMAL' ? theme.success : caution.level === 'CAUTION' ? theme.warn : theme.danger, marginBottom: '4px', fontWeight: 600 }}>{caution.level}</div>
        </div>
        <div style={{ marginTop: '10px', display: 'flex', gap: '6px' }}>
          {['NORMAL','CAUTION','CRITICAL'].map(lvl => {
            const c = lvl === 'NORMAL' ? theme.success : lvl === 'CAUTION' ? theme.warn : theme.danger;
            return (
              <button key={lvl} onClick={() => window._setCaution && window._setCaution(lvl)}
                style={{ flex: 1, padding: '6px 2px', fontSize: '9px', fontFamily: theme.monoFont, fontWeight: 600,
                  borderRadius: '5px', letterSpacing: '0.06em',
                  border: `1.5px solid ${caution.level === lvl ? c : border}`,
                  background: caution.level === lvl ? c + '22' : 'transparent',
                  color: caution.level === lvl ? c : theme.muted, cursor: 'pointer' }}>{lvl}</button>
            );
          })}
        </div>
      </div>

      {/* Function selector — full width */}
      <div style={{ gridColumn: '1 / -1', background: bgCard, border: `1px solid ${border}`, borderRadius: '10px', padding: '18px' }}>
        <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, marginBottom: '14px', fontFamily: theme.monoFont, textTransform: 'uppercase' }}>Navigation Function</div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '10px' }}>
          {functions.map(f => {
            const active = func === f;
            const col = funcColors[f];
            const disabled = estop.active && f !== 'STANDBY';
            return (
              <button key={f} onClick={() => !disabled && setFunc(f)}
                style={{
                  padding: '14px 8px', borderRadius: '8px', cursor: disabled ? 'not-allowed' : 'pointer',
                  border: `1.5px solid ${active ? col : border}`,
                  background: active ? col + '1A' : 'transparent',
                  opacity: disabled ? 0.4 : 1,
                  transition: 'all 0.15s',
                }}>
                <div style={{ fontSize: '11px', fontFamily: theme.monoFont, fontWeight: 700, color: active ? col : theme.fg, letterSpacing: '0.05em', marginBottom: '6px' }}>{f}</div>
                <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.sansFont, lineHeight: 1.4, textAlign: 'left' }}>{funcDescriptions[f]}</div>
              </button>
            );
          })}
        </div>
      </div>
    </div>
  );
}

// ── Remote Control Tab ──────────────────────────────────────────────────────
function RemoteControlTab({ telem, setFunc, theme, agv, connected }) {
  const [pressed, setPressed] = React.useState({ fwd: false, back: false, left: false, right: false });
  const [speed, setSpeed] = React.useState(0.3);
  const { estop, func } = telem;
  const active = func === 'REMOTE_CONTROL' && !estop.active;

  React.useEffect(() => {
    const onKey = (e) => {
      const map = { ArrowUp: 'fwd', w: 'fwd', ArrowDown: 'back', s: 'back', ArrowLeft: 'left', a: 'left', ArrowRight: 'right', d: 'right' };
      const dir = map[e.key];
      if (!dir) return;
      e.preventDefault();
      setPressed(p => ({ ...p, [dir]: e.type === 'keydown' }));
    };
    window.addEventListener('keydown', onKey);
    window.addEventListener('keyup', onKey);
    return () => { window.removeEventListener('keydown', onKey); window.removeEventListener('keyup', onKey); };
  }, []);

  // Stream CMD_VEL_CMD to firmware at 20 Hz while the tab is active. We send
  // even (0,0) so firmware sees a fresh setpoint and the navigator's stale
  // timeout (200 ms) doesn't fall back to zero between key presses. The
  // command is also fire-and-forget — we don't await ACKs so dropped frames
  // don't queue up.
  const vRef = React.useRef(0);
  const wRef = React.useRef(0);
  React.useEffect(() => {
    if (!active || !connected || !agv) return;
    const handle = setInterval(() => {
      // Best-effort send; navigator gates on supervised+remote+!estop already.
      try { agv.cmdVelCmd(vRef.current, wRef.current); } catch (_) {}
    }, 50); // 20 Hz — faster than firmware's 5 Hz stale threshold
    return () => clearInterval(handle);
  }, [active, connected, agv]);

  const btn = (dir, label, row, col) => {
    const isPressed = pressed[dir];
    return (
      <button
        onPointerDown={() => active && setPressed(p => ({ ...p, [dir]: true }))}
        onPointerUp={() => setPressed(p => ({ ...p, [dir]: false }))}
        onPointerLeave={() => setPressed(p => ({ ...p, [dir]: false }))}
        style={{
          gridRow: row, gridColumn: col,
          width: '80px', height: '80px', borderRadius: '12px',
          border: `2px solid ${isPressed && active ? theme.accent : theme.border}`,
          background: isPressed && active ? theme.accent + '22' : theme.cardBg,
          color: active ? theme.fg : theme.muted,
          fontSize: '28px', cursor: active ? 'pointer' : 'not-allowed',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          transition: 'all 0.08s', userSelect: 'none',
          transform: isPressed && active ? 'scale(0.94)' : 'scale(1)',
        }}
        aria-label={dir}
      >{label}</button>
    );
  };

  const v = (pressed.fwd ? speed : 0) - (pressed.back ? speed : 0);
  const omega = (pressed.right ? speed * 2 : 0) - (pressed.left ? speed * 2 : 0);
  // Mirror into refs for the streaming setInterval above
  vRef.current = active ? v : 0;
  wRef.current = active ? omega : 0;

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', height: '100%', gap: '28px', padding: '16px' }}>
      {!active && (
        <div style={{ background: theme.warn + '22', border: `1px solid ${theme.warn}`, borderRadius: '8px', padding: '10px 20px', fontSize: '12px', fontFamily: theme.monoFont, color: theme.warn }}>
          {estop.active ? '⚠ E-STOP active — clear before driving' : 'Activate REMOTE_CONTROL function to drive'}
          {!estop.active && <button onClick={() => setFunc('REMOTE_CONTROL')} style={{ marginLeft: '14px', background: theme.accent, border: 'none', borderRadius: '6px', color: '#fff', fontSize: '11px', padding: '4px 12px', cursor: 'pointer', fontFamily: theme.monoFont, fontWeight: 600 }}>Activate</button>}
        </div>
      )}

      {/* D-pad */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 80px)', gridTemplateRows: 'repeat(3, 80px)', gap: '8px' }}>
        <div style={{ gridRow: 1, gridColumn: 2 }}>{btn('fwd', '▲', 1, 2)}</div>
        <div style={{ gridRow: 2, gridColumn: 1 }}>{btn('left', '◀', 2, 1)}</div>
        <div style={{ gridRow: 2, gridColumn: 3 }}>{btn('right', '▶', 2, 3)}</div>
        <div style={{ gridRow: 3, gridColumn: 2 }}>{btn('back', '▼', 3, 2)}</div>
        <div style={{ gridRow: 2, gridColumn: 2, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          <div style={{ width: '24px', height: '24px', borderRadius: '50%', background: theme.border }}></div>
        </div>
      </div>

      {/* Speed slider */}
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '8px', width: '260px' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', width: '100%' }}>
          <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, textTransform: 'uppercase', letterSpacing: '0.1em' }}>Speed limit</span>
          <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.accent, fontWeight: 700 }}>{speed.toFixed(2)} m/s</span>
        </div>
        <input type="range" min={0.05} max={1.0} step={0.05} value={speed} onChange={e => setSpeed(parseFloat(e.target.value))}
          style={{ width: '100%', accentColor: theme.accent }} />
      </div>

      {/* Live velocity readout */}
      <div style={{ display: 'flex', gap: '24px' }}>
        {[['v', fmt(v, 3, 'm/s')], ['ω', fmt(omega, 3, 'rad/s')]].map(([label, val]) => (
          <div key={label} style={{ textAlign: 'center', background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '8px', padding: '10px 20px' }}>
            <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', marginBottom: '4px' }}>{label}</div>
            <div style={{ fontSize: '22px', fontFamily: theme.monoFont, fontWeight: 700, color: theme.accent }}>{val}</div>
          </div>
        ))}
      </div>

      <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>Keyboard: WASD / Arrow keys also work</div>
    </div>
  );
}

// ── Telemetry Tab ───────────────────────────────────────────────────────────
function TelemetryTab({ telem, theme }) {
  const { velocity, encoders, loadCells, imu, current, caution } = telem;

  const Card = ({ title, children, style: extraStyle }) => (
    <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px', ...extraStyle }}>
      <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '12px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>{title}</div>
      {children}
    </div>
  );

  const Row = ({ label, value, unit, color }) => (
    <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', marginBottom: '6px' }}>
      <span style={{ fontSize: '11px', fontFamily: theme.monoFont, color: theme.muted }}>{label}</span>
      <span style={{ fontSize: '14px', fontFamily: theme.monoFont, fontWeight: 600, color: color || theme.fg }}>{value}{unit && <span style={{ fontSize: '10px', color: theme.muted, marginLeft: '3px' }}>{unit}</span>}</span>
    </div>
  );

  const cogX = loadCells.cog.x * 100;
  const cogY = loadCells.cog.y * 100;

  return (
    <div style={{ padding: '16px', display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '12px', overflowY: 'auto', height: '100%', boxSizing: 'border-box', alignContent: 'start' }}>
      <Card title="Velocity">
        <Row label="Linear v" value={fmt(velocity.v, 3)} unit="m/s" color={theme.accent} />
        <Row label="Angular ω" value={fmt(velocity.omega, 3)} unit="rad/s" />
        <Row label="Left wheel" value={fmt(velocity.vLeft, 3)} unit="m/s" />
        <Row label="Right wheel" value={fmt(velocity.vRight, 3)} unit="m/s" />
      </Card>
      <Card title="Encoders">
        <Row label="Left count" value={encoders.left} />
        <Row label="Right count" value={encoders.right} />
        <Row label="Left RPM" value={fmt(encoders.leftRpm, 1)} unit="rpm" color={theme.accent} />
        <Row label="Right RPM" value={fmt(encoders.rightRpm, 1)} unit="rpm" color={theme.accent} />
      </Card>
      <Card title="Motor Current">
        <Row label="Left" value={fmt(current.left, 2)} unit="A" color={current.left > 2.5 ? theme.danger : theme.fg} />
        <Row label="Right" value={fmt(current.right, 2)} unit="A" color={current.right > 2.5 ? theme.danger : theme.fg} />
        <Row label="Caution mod." value={(caution.modifier * 100).toFixed(0)} unit="%" />
      </Card>
      <Card title="Load Cells">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px', marginBottom: '10px' }}>
          {[['FL', loadCells.fl], ['FR', loadCells.fr], ['RL', loadCells.rl], ['RR', loadCells.rr]].map(([pos, val]) => (
            <div key={pos} style={{ background: theme.bg, borderRadius: '6px', padding: '8px', textAlign: 'center' }}>
              <div style={{ fontSize: '9px', color: theme.muted, fontFamily: theme.monoFont, marginBottom: '3px' }}>{pos}</div>
              <div style={{ fontSize: '15px', fontFamily: theme.monoFont, fontWeight: 700, color: theme.fg }}>{fmt(val, 1)}</div>
              <div style={{ fontSize: '9px', color: theme.muted, fontFamily: theme.monoFont }}>kg</div>
            </div>
          ))}
        </div>
        <Row label="Total" value={fmt(loadCells.total, 1)} unit="kg" color={theme.accent} />
      </Card>
      <Card title="IMU — BNO055">
        <Row label="Roll" value={fmt(imu.roll, 2)} unit="°" />
        <Row label="Pitch" value={fmt(imu.pitch, 2)} unit="°" />
        <Row label="Yaw" value={fmt(imu.yaw, 1)} unit="°" color={theme.accent} />
        <Row label="Accel X" value={fmt(imu.ax, 3)} unit="m/s²" />
        <Row label="Accel Y" value={fmt(imu.ay, 3)} unit="m/s²" />
        <Row label="Accel Z" value={fmt(imu.az, 2)} unit="m/s²" />
      </Card>
      <Card title="Proximity Sensors — E18-D80NK" style={{ gridColumn: 'span 2' }}>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px', marginBottom: '8px' }}>
          {[['Front', telem.proximity.front], ['Rear', telem.proximity.rear], ['Left', telem.proximity.left], ['Right', telem.proximity.right]].map(([pos, active]) => (
            <div key={pos} style={{ display: 'flex', alignItems: 'center', gap: '8px', background: active ? theme.danger + '18' : theme.bg, borderRadius: '6px', padding: '8px 10px', border: `1px solid ${active ? theme.danger : theme.border}`, transition: 'all 0.1s' }}>
              <div style={{ width: '9px', height: '9px', borderRadius: '50%', background: active ? theme.danger : theme.border, boxShadow: active ? `0 0 6px ${theme.danger}` : 'none', flexShrink: 0, transition: 'all 0.1s' }}></div>
              <span style={{ fontSize: '11px', fontFamily: theme.monoFont, color: active ? theme.danger : theme.muted, fontWeight: active ? 700 : 400 }}>{pos}</span>
              <span style={{ marginLeft: 'auto', fontSize: '10px', fontFamily: theme.monoFont, color: active ? theme.danger : theme.muted }}>{active ? 'BLOCKED' : 'CLEAR'}</span>
            </div>
          ))}
        </div>
        <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, lineHeight: 1.5 }}>NPN digital, interrupt-driven (EXTI). Auto-clears E-STOP when all sensors deassert.</div>
      </Card>
      <Card title="Center of Gravity">
        <div style={{ position: 'relative', width: '100%', paddingBottom: '66%', marginBottom: '8px' }}>
          <div style={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <div style={{ position: 'relative', width: '110px', height: '70px', border: `2px solid ${theme.border}`, borderRadius: '8px', background: theme.bg }}>
              {/* Corner dots */}
              {[['FL', 0, 0], ['FR', 0, 1], ['RL', 1, 0], ['RR', 1, 1]].map(([pos, r, c]) => (
                <div key={pos} style={{ position: 'absolute', top: r === 0 ? '6px' : undefined, bottom: r === 1 ? '6px' : undefined, left: c === 0 ? '8px' : undefined, right: c === 1 ? '8px' : undefined, fontSize: '8px', fontFamily: theme.monoFont, color: theme.muted }}>{pos}</div>
              ))}
              {/* CoG dot */}
              <div style={{ position: 'absolute', width: '10px', height: '10px', borderRadius: '50%', background: theme.accent, top: `calc(50% + ${cogY * 0.3}px - 5px)`, left: `calc(50% + ${cogX * 0.4}px - 5px)`, transition: 'all 0.1s', boxShadow: `0 0 6px ${theme.accent}` }}></div>
            </div>
          </div>
        </div>
        <Row label="CoG X offset" value={fmt(cogX, 1)} unit="%" />
        <Row label="CoG Y offset" value={fmt(cogY, 1)} unit="%" />
      </Card>
    </div>
  );
}

Object.assign(window, { DashboardTab, RemoteControlTab, TelemetryTab });
