
// ── Function-specific control panels (rendered inside Dashboard) ─────────────

// Module-scope so its identity is stable across the ~50 Hz telemetry re-renders.
// A component defined *inside* a render gets a new type every frame, so React
// unmounts/remounts its <button> — destroying the DOM node mid-click and
// dropping the in-progress press. Keep interactive components at module scope.
function PanelBtn({ onClick, children, color, dis, theme, disabled }) {
  const c = color || theme.accent;
  const off = dis || disabled;
  return (
    <button onClick={onClick} disabled={off}
      style={{
        padding: '10px 14px', borderRadius: '8px', fontSize: '11px', fontFamily: theme.monoFont, fontWeight: 600,
        border: `1.5px solid ${c}`, background: c + '15', color: c,
        cursor: off ? 'not-allowed' : 'pointer', opacity: off ? 0.45 : 1,
        letterSpacing: '0.04em', transition: 'all 0.12s',
      }}>{children}</button>
  );
}

function StandbyPanel({ theme, agv, connected, telem }) {
  const [qtrPhase, setQtrPhase] = React.useState('idle'); // idle | sweeping
  const [weightCaution, setWeightCaution] = React.useState(80);
  const [weightEstop, setWeightEstop] = React.useState(100);
  const [tofCaution, setTofCaution] = React.useState(800);
  const [tofCritical, setTofCritical] = React.useState(400);
  const [tofEstop, setTofEstop] = React.useState(200);
  const [battCaution, setBattCaution] = React.useState(10500);
  const [battEstop, setBattEstop] = React.useState(9900);
  const disabled = !connected;

  const send = (fn) => { if (!connected || !agv) return; try { fn(); } catch (_) {} };

  return (
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '14px' }}>
      {/* Load cells */}
      <div>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>Load Cells</div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
          <PanelBtn theme={theme} disabled={disabled} onClick={() => send(() => agv.cmdStartTare())}>⊘ Tare (zero load cells)</PanelBtn>
          <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5 }}>
            Empty the platform, then tare. ~1 s; <code>flags</code> bit 3 indicates in-progress.
          </div>
        </div>
      </div>

      {/* Odometry */}
      <div>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>Odometry</div>
        <PanelBtn theme={theme} disabled={disabled} onClick={() => send(() => agv.cmdResetOdometry())}>⟲ Reset odometry (x=y=θ=0)</PanelBtn>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          Position now: x {fmt(telem.position.x, 2)} m, y {fmt(telem.position.y, 2)} m, θ {fmt(telem.position.theta, 2)} rad
        </div>
      </div>

      {/* QTR calibration */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>QTR Line Sensor Calibration</div>
        <div style={{ display: 'flex', gap: '8px', flexWrap: 'wrap' }}>
          <PanelBtn theme={theme} disabled={disabled} onClick={() => { send(() => agv.cmdQtrCalibrate(0)); setQtrPhase('sweeping'); }} dis={qtrPhase === 'sweeping'}>▶ Begin sweep</PanelBtn>
          <PanelBtn theme={theme} disabled={disabled} color={theme.success} onClick={() => { send(() => agv.cmdQtrCalibrate(1)); setQtrPhase('idle'); }} dis={qtrPhase !== 'sweeping'}>✓ Save + persist</PanelBtn>
          <PanelBtn theme={theme} disabled={disabled} color={theme.warn} onClick={() => { send(() => agv.cmdQtrCalibrate(2)); setQtrPhase('idle'); }} dis={qtrPhase !== 'sweeping'}>✗ Cancel</PanelBtn>
          <PanelBtn theme={theme} disabled={disabled} color={theme.danger} onClick={() => send(() => agv.cmdQtrCalibrate(3))}>⌫ Reset to defaults</PanelBtn>
        </div>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          {qtrPhase === 'sweeping'
            ? <span style={{ color: theme.warn }}>● Sweeping — move the AGV side-to-side across the line for ~3-5 s, then Save.</span>
            : 'Begin a sweep, manually pass the array over the line a few times, then save to flash.'}
        </div>
      </div>

      {/* Cargo limits */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>Cargo Limits</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '14px' }}>
          {[
            ['Caution at', weightCaution, setWeightCaution, 'WEIGHT_CAUTION_KG', theme.warn, 200],
            ['E-STOP at', weightEstop, setWeightEstop, 'WEIGHT_ESTOP_KG', theme.danger, 200],
          ].map(([label, val, setVal, paramName, col, max]) => (
            <div key={label}>
              <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                <span style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont }}>{label}</span>
                <span style={{ fontSize: '12px', color: col, fontFamily: theme.monoFont, fontWeight: 700 }}>{val.toFixed(0)} kg</span>
              </div>
              <input type="range" min={10} max={max} step={5} value={val}
                onChange={e => setVal(parseFloat(e.target.value))}
                onMouseUp={() => send(() => agv.sendParamUpdate(window.AGV_PROTO.PARAM[paramName], parseFloat(val)))}
                style={{ width: '100%', accentColor: col }} />
            </div>
          ))}
        </div>
      </div>

      {/* TOF distance bands */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>TOF Distance Bands (closest of 4 sensors)</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '14px' }}>
          {[
            ['Caution <', tofCaution, setTofCaution, 'TOF_CAUTION_MM', theme.warn],
            ['Critical <', tofCritical, setTofCritical, 'TOF_CRITICAL_MM', theme.warn],
            ['E-STOP <', tofEstop, setTofEstop, 'TOF_ESTOP_MM', theme.danger],
          ].map(([label, val, setVal, paramName, col]) => (
            <div key={label}>
              <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                <span style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont }}>{label}</span>
                <span style={{ fontSize: '12px', color: col, fontFamily: theme.monoFont, fontWeight: 700 }}>{val} mm</span>
              </div>
              <input type="range" min={50} max={1200} step={10} value={val}
                onChange={e => setVal(parseInt(e.target.value))}
                onMouseUp={() => send(() => agv.sendParamUpdate(window.AGV_PROTO.PARAM[paramName], val))}
                style={{ width: '100%', accentColor: col }} />
            </div>
          ))}
        </div>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          Auto-clearing, like the IR ring. Keep E-STOP &lt; Critical &lt; Caution. <code>PARAM 0x60–0x62</code>.
        </div>
      </div>

      {/* Battery (3S) low-voltage limits */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>3S Battery Low-Voltage Limits</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '14px' }}>
          {[
            ['Caution <', battCaution, setBattCaution, 'BATT_3S_CAUTION_MV', theme.warn],
            ['E-STOP <', battEstop, setBattEstop, 'BATT_3S_ESTOP_MV', theme.danger],
          ].map(([label, val, setVal, paramName, col]) => (
            <div key={label}>
              <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                <span style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont }}>{label}</span>
                <span style={{ fontSize: '12px', color: col, fontFamily: theme.monoFont, fontWeight: 700 }}>{(val / 1000).toFixed(1)} V</span>
              </div>
              <input type="range" min={9000} max={12000} step={100} value={val}
                onChange={e => setVal(parseInt(e.target.value))}
                onMouseUp={() => send(() => agv.sendParamUpdate(window.AGV_PROTO.PARAM[paramName], val))}
                style={{ width: '100%', accentColor: col }} />
            </div>
          ))}
        </div>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          3S only (motors/logic); auto-clears with hysteresis. The 6S rail is display-only. <code>PARAM 0x63–0x64</code>.
        </div>
      </div>

      {/* Indicator lights */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>Indicator Lights (LED Rings)</div>
        <div style={{ display: 'flex', gap: '8px' }}>
          {[['◍ Pulse', window.AGV_PROTO.LED_MODE.PULSE], ['→ Snake', window.AGV_PROTO.LED_MODE.SNAKE]].map(([lbl, m]) => (
            <PanelBtn theme={theme} disabled={disabled} key={m} color={telem.ledMode === m ? theme.success : theme.accent}
              onClick={() => send(() => agv.sendParamUpdate(window.AGV_PROTO.PARAM.LED_MODE, m))}>
              {telem.ledMode === m ? '● ' : ''}{lbl}
            </PanelBtn>
          ))}
        </div>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          Animation for the three state rings (two small + one big); colour follows system state (green / yellow / red). <code>PARAM_LED_MODE (0x50)</code>.
        </div>

        {/* Reactive obstacle ring — one big ring shows per-sensor distance instead of state. */}
        <div style={{ marginTop: '14px' }}>
          <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>Obstacle Ring</div>
          <div style={{ display: 'flex', gap: '20px', flexWrap: 'wrap' }}>
            <div>
              <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, marginBottom: '4px' }}>Base</div>
              <div style={{ display: 'flex', gap: '8px' }}>
                {[['○ Off', window.AGV_PROTO.LED_BASE.OFF], ['◌ White', window.AGV_PROTO.LED_BASE.WHITE]].map(([lbl, v]) => (
                  <PanelBtn theme={theme} disabled={disabled} key={v} color={((telem.ledIndicatorCfg >> 0) & 1) === v ? theme.success : theme.accent}
                    onClick={() => send(() => agv.sendParamUpdate(window.AGV_PROTO.PARAM.LED_BASE, v))}>
                    {((telem.ledIndicatorCfg >> 0) & 1) === v ? '● ' : ''}{lbl}
                  </PanelBtn>
                ))}
              </div>
            </div>
            <div>
              <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, marginBottom: '4px' }}>Spread</div>
              <div style={{ display: 'flex', gap: '8px' }}>
                {[['▭ Fixed', window.AGV_PROTO.LED_INDICATOR_MODE.FIXED], ['◈ Responsive', window.AGV_PROTO.LED_INDICATOR_MODE.RESPONSIVE]].map(([lbl, v]) => (
                  <PanelBtn theme={theme} disabled={disabled} key={v} color={((telem.ledIndicatorCfg >> 1) & 1) === v ? theme.success : theme.accent}
                    onClick={() => send(() => agv.sendParamUpdate(window.AGV_PROTO.PARAM.LED_INDICATOR_MODE, v))}>
                    {((telem.ledIndicatorCfg >> 1) & 1) === v ? '● ' : ''}{lbl}
                  </PanelBtn>
                ))}
              </div>
            </div>
          </div>
          <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
            One big ring maps each distance sensor to LEDs: IR red on detection, TOF / LiDAR yellow→red by range. Responsive widens the TOF span as range falls. <code>PARAM 0x51–0x52</code>.
          </div>
        </div>
      </div>
    </div>
  );
}

function RemoteControlPanel({ theme, telem, pressed, setPressed, speed, setSpeed, rcV, rcOmega }) {
  const { estop } = telem;
  const active = !estop.active;

  const btn = (dir, label) => {
    const isPressed = pressed[dir];
    return (
      <button
        onPointerDown={() => active && setPressed(p => ({ ...p, [dir]: true }))}
        onPointerUp={() => setPressed(p => ({ ...p, [dir]: false }))}
        onPointerLeave={() => setPressed(p => ({ ...p, [dir]: false }))}
        style={{
          width: '66px', height: '66px', borderRadius: '11px',
          border: `2px solid ${isPressed && active ? theme.accent : theme.border}`,
          background: isPressed && active ? theme.accent + '22' : theme.bg,
          color: active ? theme.fg : theme.muted,
          fontSize: '22px', cursor: active ? 'pointer' : 'not-allowed',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          transition: 'all 0.08s', userSelect: 'none',
          transform: isPressed && active ? 'scale(0.94)' : 'scale(1)',
        }}
        aria-label={dir}>{label}</button>
    );
  };

  return (
    <div style={{ display: 'flex', alignItems: 'flex-start', gap: '24px' }}>
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 66px)', gridTemplateRows: 'repeat(3, 66px)', gap: '6px', flexShrink: 0 }}>
        <div style={{ gridRow: 1, gridColumn: 2 }}>{btn('fwd', '▲')}</div>
        <div style={{ gridRow: 2, gridColumn: 1 }}>{btn('left', '◀')}</div>
        <div style={{ gridRow: 2, gridColumn: 3 }}>{btn('right', '▶')}</div>
        <div style={{ gridRow: 3, gridColumn: 2 }}>{btn('back', '▼')}</div>
      </div>

      <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: '14px' }}>
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
            <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>Speed limit</span>
            <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.accent, fontWeight: 700 }}>{speed.toFixed(2)} m/s</span>
          </div>
          <input type="range" min={0.05} max={1.0} step={0.05} value={speed} onChange={e => setSpeed(parseFloat(e.target.value))}
            style={{ width: '100%', accentColor: theme.accent }} />
        </div>

        <div style={{ display: 'flex', gap: '14px' }}>
          {[['v', fmt(rcV, 3), 'm/s'], ['ω', fmt(rcOmega, 3), 'rad/s']].map(([label, val, unit]) => (
            <div key={label} style={{ flex: 1, background: theme.bg, border: `1px solid ${theme.border}`, borderRadius: '8px', padding: '8px 12px', textAlign: 'center' }}>
              <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', marginBottom: '2px' }}>{label}</div>
              <div style={{ fontSize: '17px', fontFamily: theme.monoFont, fontWeight: 700, color: theme.accent }}>{val}<span style={{ fontSize: '10px', color: theme.muted, marginLeft: '3px' }}>{unit}</span></div>
            </div>
          ))}
        </div>

        {estop.active && (
          <div style={{ fontSize: '11px', color: theme.danger, fontFamily: theme.monoFont }}>⚠ E-STOP active — clear before driving</div>
        )}
        <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>WASD / Arrow keys also work.</div>
      </div>
    </div>
  );
}

function LineFollowPanel({ theme, agv, connected, telem }) {
  const [cruise, setCruise] = React.useState(0.3);
  const [lostThresh, setLostThresh] = React.useState(0.5);
  const send = (id, val) => { if (connected && agv) try { agv.sendParamUpdate(id, val); } catch (_) {} };
  const P = window.AGV_PROTO?.PARAM || {};

  return (
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '16px' }}>
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
          <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>Cruise speed</span>
          <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.success, fontWeight: 700 }}>{cruise.toFixed(2)} m/s</span>
        </div>
        <input type="range" min={0.05} max={1.0} step={0.05} value={cruise}
          onChange={e => setCruise(parseFloat(e.target.value))}
          onMouseUp={() => send(P.LINE_CRUISE_MPS, cruise)}
          style={{ width: '100%', accentColor: theme.success }} />
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, marginTop: '4px' }}>PARAM_LINE_CRUISE_MPS</div>
      </div>

      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
          <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>Line-lost threshold</span>
          <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.warn, fontWeight: 700 }}>{lostThresh.toFixed(2)}</span>
        </div>
        <input type="range" min={0.1} max={2.0} step={0.05} value={lostThresh}
          onChange={e => setLostThresh(parseFloat(e.target.value))}
          onMouseUp={() => send(P.QTR_LINE_LOST_THRESH, lostThresh)}
          style={{ width: '100%', accentColor: theme.warn }} />
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, marginTop: '4px' }}>Sum-of-normalised below this → halt + log LINE_LOST</div>
      </div>

      <div style={{ gridColumn: '1 / -1', display: 'flex', gap: '14px', background: theme.bg, border: `1px solid ${theme.border}`, borderRadius: '8px', padding: '10px 14px' }}>
        <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, textTransform: 'uppercase', letterSpacing: '0.1em' }}>Line position</span>
        <span style={{ fontSize: '12px', fontFamily: theme.monoFont, color: theme.accent, fontWeight: 700 }}>—</span>
        <span style={{ marginLeft: 'auto', fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>Not in telemetry yet · PID gains tunable on PID tab</span>
      </div>
    </div>
  );
}

function TrajectoryPanel({ theme, agv, connected }) {
  const [cruise, setCruise] = React.useState(0.3);
  const [lookahead, setLookahead] = React.useState(0.5);
  const [curvSlowdown, setCurvSlowdown] = React.useState(0.5);
  const send = (id, val) => { if (connected && agv) try { agv.sendParamUpdate(id, val); } catch (_) {} };
  const P = window.AGV_PROTO?.PARAM || {};

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '14px' }}>
      <div style={{ background: theme.warn + '12', border: `1px solid ${theme.warn}55`, borderRadius: '8px', padding: '12px 14px', fontSize: '11px', fontFamily: theme.monoFont, color: theme.fg, lineHeight: 1.5 }}>
        Pure-pursuit follower. Load and edit waypoints on the <strong style={{ color: theme.warn }}>Trajectory</strong> tab.
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '16px' }}>
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
            <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>Cruise speed</span>
            <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.success, fontWeight: 700 }}>{cruise.toFixed(2)} m/s</span>
          </div>
          <input type="range" min={0.05} max={1.0} step={0.05} value={cruise}
            onChange={e => setCruise(parseFloat(e.target.value))}
            onMouseUp={() => send(P.TRAJ_CRUISE_MPS, cruise)}
            style={{ width: '100%', accentColor: theme.success }} />
        </div>
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
            <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>Lookahead Lᴅ</span>
            <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.accent, fontWeight: 700 }}>{lookahead.toFixed(2)} m</span>
          </div>
          <input type="range" min={0.1} max={2.0} step={0.05} value={lookahead}
            onChange={e => setLookahead(parseFloat(e.target.value))}
            onMouseUp={() => send(P.TRAJ_LOOKAHEAD_M, lookahead)}
            style={{ width: '100%', accentColor: theme.accent }} />
        </div>
        <div>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
            <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>Curv. slowdown g</span>
            <span style={{ fontSize: '13px', fontFamily: theme.monoFont, color: theme.warn, fontWeight: 700 }}>{curvSlowdown.toFixed(2)} m</span>
          </div>
          <input type="range" min={0} max={2.0} step={0.05} value={curvSlowdown}
            onChange={e => setCurvSlowdown(parseFloat(e.target.value))}
            onMouseUp={() => send(P.TRAJ_CURV_SLOWDOWN, curvSlowdown)}
            style={{ width: '100%', accentColor: theme.warn }} />
          <div style={{ fontSize: '9px', color: theme.muted, fontFamily: theme.monoFont, marginTop: '2px' }}>v = cruise/(1+g·|κ|) · 0 disables</div>
        </div>
      </div>
    </div>
  );
}

// ── Dashboard Tab ───────────────────────────────────────────────────────────
function DashboardTab({ telem, setFunc, setMode, theme, agv, connected, rcPressed, setRcPressed, rcSpeed, setRcSpeed, rcV, rcOmega }) {
  const { func, mode, estop, caution } = telem;

  const functions = ['STANDBY', 'REMOTE_CONTROL', 'LINE_FOLLOW', 'TRAJECTORY_FOLLOW'];
  const funcColors = {
    STANDBY: theme.accent,
    REMOTE_CONTROL: theme.warn,
    LINE_FOLLOW: theme.success,
    TRAJECTORY_FOLLOW: theme.success,
  };
  const funcDescriptions = {
    STANDBY: 'Motors idle. Calibration & setup available.',
    REMOTE_CONTROL: 'Direct velocity commands from workstation.',
    LINE_FOLLOW: 'Following floor line via QTR-8A array.',
    TRAJECTORY_FOLLOW: 'Executing pre-loaded waypoint sequence.',
  };

  const bgCard = theme.cardBg;
  const border = theme.border;

  function FunctionPanel() {
    if (func === 'STANDBY')          return <StandbyPanel theme={theme} agv={agv} connected={connected} telem={telem} />;
    if (func === 'REMOTE_CONTROL')   return <RemoteControlPanel theme={theme} telem={telem} pressed={rcPressed} setPressed={setRcPressed} speed={rcSpeed} setSpeed={setRcSpeed} rcV={rcV} rcOmega={rcOmega} />;
    if (func === 'LINE_FOLLOW')      return <LineFollowPanel theme={theme} agv={agv} connected={connected} telem={telem} />;
    if (func === 'TRAJECTORY_FOLLOW')return <TrajectoryPanel theme={theme} agv={agv} connected={connected} />;
    return null;
  }

  return (
    <div style={{ padding: '16px', display: 'grid', gridTemplateColumns: '1fr 1fr', gridTemplateRows: 'auto auto 1fr', gap: '14px', height: '100%', boxSizing: 'border-box' }}>

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

      {/* Function-specific control panel — swaps based on selected function */}
      <div style={{ gridColumn: '1 / -1', background: bgCard, border: `1px solid ${funcColors[func]}55`, borderRadius: '10px', padding: '18px', overflowY: 'auto' }}>
        <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: funcColors[func], marginBottom: '14px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 700 }}>
          {func.replace('_', ' ')} controls
        </div>
        <FunctionPanel />
      </div>
    </div>
  );
}

// ── Weight heat map — CSS radial gradients (GPU-smooth, no blinking) ─────────
// QTR line sensors: 8 raw 12-bit ADC readings rendered as a horizontal heat strip.
// Higher raw value = darker surface = line. CoM is computed per-frame after
// in-frame min/max normalisation so the marker still tracks the line under
// uneven ambient light (the firmware does this against calibrated white/black,
// but we don't have those values here).
function QtrHeatmap({ values }) {
  const n = values.length;
  const adcMax = 4095;

  function heatRgb(t) {
    t = Math.max(0, Math.min(1, t));
    const s = [[20,40,130],[0,170,180],[240,180,0],[220,20,0]];
    const i = Math.min(Math.floor(t * (s.length - 1)), s.length - 2);
    const u = t * (s.length - 1) - i;
    return s[i].map((a, j) => Math.round(a + (s[i+1][j] - a) * u));
  }

  const minV = Math.min(...values);
  const maxV = Math.max(...values);
  const span = Math.max(maxV - minV, 1);
  const norm = values.map(v => (v - minV) / span);

  let wsum = 0, wtot = 0;
  norm.forEach((w, i) => { wsum += w * i; wtot += w; });
  const valid = wtot > 0.05;
  const com = valid ? wsum / wtot : (n - 1) / 2;
  const comPct = (com / (n - 1)) * 100;

  return (
    <div>
      <div style={{ position: 'relative', height: '46px', display: 'flex', gap: '2px' }}>
        {values.map((v, i) => {
          const t = v / adcMax;
          const [r, g, b] = heatRgb(t);
          return (
            <div key={i} style={{ flex: 1, background: `rgb(${r},${g},${b})`, borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'center', fontFamily: 'monospace', fontSize: '10px', fontWeight: 700, color: t > 0.55 ? '#fff' : 'rgba(0,0,0,0.75)', textShadow: t > 0.55 ? '0 1px 2px rgba(0,0,0,0.6)' : 'none', transition: 'background 0.07s linear' }}>
              {v}
            </div>
          );
        })}
        <div style={{ position: 'absolute', top: '-4px', bottom: '-4px', left: `${comPct}%`, width: '2px', background: valid ? 'white' : 'rgba(255,255,255,0.25)', boxShadow: valid ? '0 0 6px rgba(255,255,255,0.9)' : 'none', transform: 'translateX(-50%)', transition: 'left 0.1s', pointerEvents: 'none' }} />
      </div>
      <div style={{ display: 'flex', marginTop: '4px', fontFamily: 'monospace', fontSize: '9px', color: 'rgba(255,255,255,0.45)' }}>
        {values.map((_, i) => <span key={i} style={{ flex: 1, textAlign: 'center' }}>S{i}</span>)}
      </div>
      <div style={{ fontSize: '10px', fontFamily: 'monospace', color: 'rgba(255,255,255,0.55)', textAlign: 'center', marginTop: '6px' }}>
        Line position: {valid ? com.toFixed(2) : '— '} / {n - 1}
        &nbsp;&nbsp;|&nbsp;&nbsp;
        ADC min {minV} &nbsp; max {maxV}
      </div>
    </div>
  );
}

// Load cells are rated 100 kg per corner. Palette and legend are pinned to
// [0, 100] kg so the colours are absolute — saturating red = full capacity,
// not "the heaviest of four near-empty corners".
const WEIGHT_PALETTE_MAX_KG = 100;

function HeatLegend() {
  const gradient = 'linear-gradient(to top, rgb(20,40,130) 0%, rgb(0,170,180) 33%, rgb(240,180,0) 66%, rgb(220,20,0) 100%)';
  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', width: '28px', flexShrink: 0, height: '158px' }}>
      <span style={{ fontSize: '8px', fontFamily: 'monospace', color: 'rgba(255,255,255,0.55)', lineHeight: 1, marginBottom: '3px', whiteSpace: 'nowrap' }}>{WEIGHT_PALETTE_MAX_KG} kg</span>
      <div style={{ flex: 1, width: '10px', borderRadius: '5px', background: gradient }} />
      <span style={{ fontSize: '8px', fontFamily: 'monospace', color: 'rgba(255,255,255,0.55)', lineHeight: 1, marginTop: '3px' }}>0 kg</span>
    </div>
  );
}

// cogX/cogY here are normalised fractions in [-1, +1] (0 = centre of the
// platform, ±1 = full deviation toward one edge). The TelemetryTab converts
// them to real-world cm for the readout below the heatmap.
function WeightHeatmap({ fl, fr, rl, rr, cogX, cogY }) {
  // Map normalised weight [0,1] → thermal RGB: deep-blue → cyan → yellow → red
  function heatRgb(t) {
    t = Math.max(0, Math.min(1, t));
    const s = [[20,40,130],[0,170,180],[240,180,0],[220,20,0]];
    const i = Math.min(Math.floor(t * (s.length - 1)), s.length - 2);
    const u = t * (s.length - 1) - i;
    return s[i].map((a, j) => Math.round(a + (s[i+1][j] - a) * u));
  }

  const corners = [
    { pos: 'FL', val: fl, gx: '0%',   gy: '0%',   label: { top: '7px',    left:  '9px' } },
    { pos: 'FR', val: fr, gx: '100%', gy: '0%',   label: { top: '7px',    right: '9px' } },
    { pos: 'RL', val: rl, gx: '0%',   gy: '100%', label: { bottom: '7px', left:  '9px' } },
    { pos: 'RR', val: rr, gx: '100%', gy: '100%', label: { bottom: '7px', right: '9px' } },
  ];

  const gradients = corners.map(({ val, gx, gy }) => {
    const t = Math.max(0, Math.min(1, val / WEIGHT_PALETTE_MAX_KG));
    const [r, g, b] = heatRgb(t);
    const alpha = (0.10 + t * 0.80).toFixed(2);
    return `radial-gradient(ellipse 135% 135% at ${gx} ${gy}, rgba(${r},${g},${b},${alpha}) 0%, transparent 66%)`;
  });
  const bg = [...gradients, 'linear-gradient(160deg,#080918 0%,#060710 100%)'].join(', ');

  // cogX/cogY: -1…+1 where 0 = centre. Maps linearly to 0…100% of the box.
  const cx = `${(50 + cogX * 50).toFixed(2)}%`;
  const cy = `${(50 + cogY * 50).toFixed(2)}%`;

  return (
    <div style={{ position: 'relative', background: bg, borderRadius: '8px', height: '158px', overflow: 'hidden', transition: 'background 0.07s linear' }}>
      {corners.map(({ pos, val, label }) => (
        <div key={pos} style={{ position: 'absolute', ...label, fontFamily: 'monospace', lineHeight: 1.25, textShadow: '0 1px 4px rgba(0,0,0,0.9)', pointerEvents: 'none' }}>
          <div style={{ fontSize: '10px', fontWeight: 700, color: 'rgba(255,255,255,0.9)' }}>{pos}</div>
          <div style={{ fontSize: '9px', color: 'rgba(255,255,255,0.62)' }}>{val.toFixed(2)} kg</div>
        </div>
      ))}
      <div style={{ position: 'absolute', left: cx, top: 0, bottom: 0, width: '1px', background: 'rgba(255,255,255,0.25)', transform: 'translateX(-50%)', transition: 'left 0.1s', pointerEvents: 'none' }} />
      <div style={{ position: 'absolute', top: cy, left: 0, right: 0, height: '1px', background: 'rgba(255,255,255,0.25)', transform: 'translateY(-50%)', transition: 'top 0.1s', pointerEvents: 'none' }} />
      <div style={{ position: 'absolute', left: cx, top: cy, transform: 'translate(-50%,-50%)', width: '13px', height: '13px', borderRadius: '50%', background: 'white', boxShadow: '0 0 10px rgba(255,255,255,0.65), 0 0 3px rgba(0,0,0,0.5)', transition: 'left 0.1s, top 0.1s', zIndex: 2, pointerEvents: 'none' }} />
    </div>
  );
}

// ── Telemetry Tab ───────────────────────────────────────────────────────────
function TelemetryTab({ telem, theme, rcActive, rcV, rcOmega, rcPressed }) {
  const { velocity, encoders, loadCells, imu, current, caution, motors, qtr } = telem;
  const tof = telem.tof || { front: 0, rear: 0, left: 0, right: 0 };
  const battery = telem.battery || { v3s: 0, v6s: 0, pct3s: null, pct6s: null };

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

  // loadCells.cog.{x,y} are normalised fractions in [-1, +1] (0 = centre).
  // Load cells sit on the corners of a 30 cm × 30 cm square, so each corner
  // is at ±15 cm from centre. Multiplying the normalised fraction by 15
  // gives the real-world CoM offset in cm.
  const PLATFORM_HALF_CM = 15;
  const cogXNorm = loadCells.cog.x;
  const cogYNorm = loadCells.cog.y;
  const cogXcm = cogXNorm * PLATFORM_HALF_CM;
  const cogYcm = cogYNorm * PLATFORM_HALF_CM;

  return (
    <div style={{ padding: '16px', display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '12px', overflowY: 'auto', height: '100%', boxSizing: 'border-box', alignContent: 'start' }}>

      {/* RC indicator banner — only when REMOTE_CONTROL is active */}
      {rcActive && (
        <div style={{ gridColumn: 'span 3', background: theme.warn + '15', border: `1px solid ${theme.warn}`, borderRadius: '8px', padding: '8px 14px', display: 'flex', alignItems: 'center', gap: '14px' }}>
          <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.warn, fontWeight: 700, letterSpacing: '0.12em', flexShrink: 0 }}>RC ACTIVE</span>
          <div style={{ display: 'flex', gap: '4px' }}>
            {[['▲', rcPressed?.fwd], ['◀', rcPressed?.left], ['▶', rcPressed?.right], ['▼', rcPressed?.back]].map(([lbl, on]) => (
              <div key={lbl} style={{ width: '22px', height: '22px', borderRadius: '4px', background: on ? theme.warn : theme.border, color: on ? '#fff' : theme.muted, fontSize: '11px', display: 'flex', alignItems: 'center', justifyContent: 'center', transition: 'all 0.06s', fontWeight: 700 }}>{lbl}</div>
            ))}
          </div>
          <span style={{ fontSize: '12px', fontFamily: theme.monoFont, color: theme.warn }}>v: {fmt(rcV, 2)} m/s</span>
          <span style={{ fontSize: '12px', fontFamily: theme.monoFont, color: theme.warn }}>ω: {fmt(rcOmega, 2)} rad/s</span>
          <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, marginLeft: 'auto' }}>WASD / Arrow keys active</span>
        </div>
      )}

      {/* Row 1: Motor Output | Motor Current | Proximity */}
      <Card title="Motor Output">
        {[['Left', motors?.dutyLeft ?? 0], ['Right', motors?.dutyRight ?? 0]].map(([side, duty]) => {
          const pct = (Math.abs(duty) * 100).toFixed(1);
          const dir = duty > 0.005 ? 'FWD' : duty < -0.005 ? 'REV' : 'BRAKE';
          const dirColor = dir === 'FWD' ? theme.success : dir === 'REV' ? theme.danger : theme.muted;
          return (
            <div key={side} style={{ marginBottom: '10px' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                <span style={{ fontSize: '11px', fontFamily: theme.monoFont, color: theme.muted }}>{side}</span>
                <span style={{ fontSize: '11px', fontFamily: theme.monoFont, fontWeight: 700, color: dirColor }}>{dir} {pct}%</span>
              </div>
              <div style={{ height: '6px', background: theme.bg, borderRadius: '3px', overflow: 'hidden' }}>
                <div style={{ height: '100%', width: pct + '%', background: dirColor, borderRadius: '3px', transition: 'width 0.05s' }} />
              </div>
            </div>
          );
        })}
      </Card>
      <Card title="Motor Current">
        <Row label="Left" value={fmt(current.left, 2)} unit="A" color={current.left > 2.5 ? theme.danger : theme.fg} />
        <Row label="Right" value={fmt(current.right, 2)} unit="A" color={current.right > 2.5 ? theme.danger : theme.fg} />
        <Row label="Caution mod." value={(caution.modifier * 100).toFixed(0)} unit="%" />
      </Card>
      <Card title="Proximity">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px' }}>
          {[['Front', telem.proximity.front], ['Rear', telem.proximity.rear], ['Left', telem.proximity.left], ['Right', telem.proximity.right]].map(([pos, active]) => (
            <div key={pos} style={{ display: 'flex', alignItems: 'center', gap: '6px', background: active ? theme.danger + '18' : theme.bg, borderRadius: '6px', padding: '6px 8px', border: `1px solid ${active ? theme.danger : theme.border}`, transition: 'all 0.1s' }}>
              <div style={{ width: '8px', height: '8px', borderRadius: '50%', background: active ? theme.danger : theme.border, boxShadow: active ? `0 0 5px ${theme.danger}` : 'none', flexShrink: 0 }} />
              <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: active ? theme.danger : theme.muted, fontWeight: active ? 700 : 400 }}>{pos}</span>
            </div>
          ))}
        </div>
      </Card>

      {/* Row 2: TOF Distance | Battery | (Velocity continues below) */}
      <Card title="TOF Distance (VL53L0X)">
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px' }}>
          {[['Front', tof.front], ['Rear', tof.rear], ['Left', tof.left], ['Right', tof.right]].map(([pos, mm]) => {
            // Colour by band: red ≤200, amber ≤800, else fg. >=1200 reads as "clear".
            const col = mm <= 200 ? theme.danger : mm <= 800 ? theme.warn : theme.fg;
            const txt = mm >= 1200 ? '≥1200' : String(mm);
            return (
              <div key={pos} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', background: theme.bg, borderRadius: '6px', padding: '6px 8px', border: `1px solid ${theme.border}` }}>
                <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>{pos}</span>
                <span style={{ fontSize: '13px', fontFamily: theme.monoFont, fontWeight: 700, color: col }}>{txt}<span style={{ fontSize: '9px', color: theme.muted, marginLeft: '2px' }}>mm</span></span>
              </div>
            );
          })}
        </div>
      </Card>
      <Card title="Battery">
        {[['3S', battery.v3s, battery.pct3s], ['6S', battery.v6s, battery.pct6s]].map(([name, v, pct]) => {
          const absent = pct === null || pct === undefined;
          const col = absent ? theme.muted : pct <= 15 ? theme.danger : pct <= 35 ? theme.warn : theme.success;
          return (
            <div key={name} style={{ marginBottom: '10px' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
                <span style={{ fontSize: '11px', fontFamily: theme.monoFont, color: theme.muted }}>{name} pack</span>
                <span style={{ fontSize: '13px', fontFamily: theme.monoFont, fontWeight: 700, color: col }}>
                  {absent ? '— ' : fmt(v, 2) + ' V · ' + pct + '%'}
                </span>
              </div>
              <div style={{ height: '6px', background: theme.bg, borderRadius: '3px', overflow: 'hidden' }}>
                <div style={{ height: '100%', width: (absent ? 0 : pct) + '%', background: col, borderRadius: '3px', transition: 'width 0.2s' }} />
              </div>
            </div>
          );
        })}
      </Card>

      {/* Velocity | Encoders | IMU */}
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
      <Card title="IMU — MPU6050">
        <Row label="Roll" value={fmt(imu.roll, 2)} unit="°" />
        <Row label="Pitch" value={fmt(imu.pitch, 2)} unit="°" />
        <Row label="Gyro bias" value={fmt(imu.gyroBias, 3)} unit="°/s" color={theme.accent} />
        <Row label="Bias conv." value={imu.biasConverged ? 'yes' : 'no'}
             color={imu.biasConverged ? theme.success : theme.warn} />
        <Row label="ZUPT" value={imu.zuptActive ? 'active' : '—'}
             color={imu.zuptActive ? theme.success : theme.muted} />
      </Card>

      {/* Row 3: Weight Distribution | Load Cells | (empty) */}
      <Card title="Weight Distribution">
        <div style={{ display: 'flex', gap: '6px', alignItems: 'flex-start' }}>
          <div style={{ flex: 1 }}>
            <WeightHeatmap fl={loadCells.fl} fr={loadCells.fr} rl={loadCells.rl} rr={loadCells.rr} cogX={cogXNorm} cogY={cogYNorm} />
          </div>
          <HeatLegend />
        </div>
        <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, textAlign: 'center', marginTop: '8px' }}>
          CoM: X {cogXcm >= 0 ? '+' : ''}{fmt(cogXcm, 1)} cm &nbsp; Y {cogYcm >= 0 ? '+' : ''}{fmt(cogYcm, 1)} cm
        </div>
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
      <div />

      {/* Row 4: Line Sensors (span 2) | (empty) */}
      <Card title="Line Sensors (QTR)" style={{ gridColumn: 'span 2' }}>
        <QtrHeatmap values={qtr && qtr.length === 8 ? qtr : [0,0,0,0,0,0,0,0]} />
      </Card>
      <div />
    </div>
  );
}

Object.assign(window, { DashboardTab, TelemetryTab });
