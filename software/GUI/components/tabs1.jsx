
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
  const [weightCaution, setWeightCaution] = usePersistentState('standby.weightCaution', 80);
  const [weightEstop, setWeightEstop] = usePersistentState('standby.weightEstop', 100);
  const [imbCaution, setImbCaution] = usePersistentState('standby.imbCaution', 0.6);
  const [imbEstop, setImbEstop] = usePersistentState('standby.imbEstop', 1.5);
  const [lidarCaution, setLidarCaution] = usePersistentState('standby.lidarCaution', 800);
  const [lidarCritical, setLidarCritical] = usePersistentState('standby.lidarCritical', 400);
  const [lidarEstop, setLidarEstop] = usePersistentState('standby.lidarEstop', 200);
  const [battCaution, setBattCaution] = usePersistentState('standby.battCaution', 10500);
  const [battEstop, setBattEstop] = usePersistentState('standby.battEstop', 9900);
  const disabled = !connected;
  const P = window.AGV_PROTO?.PARAM || {};

  const send = (fn) => { if (!connected || !agv) return; try { fn(); } catch (_) {} };
  const sendParam = (id, val) => send(() => agv.sendParamUpdate(id, val));

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

      {/* Cargo limits */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>Cargo Limits</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '14px' }}>
          <ParamSlider theme={theme} label="Weight caution at" value={weightCaution} setValue={setWeightCaution}
            min={10} max={200} step={5} color={theme.warn} unit="kg"
            onCommit={v => sendParam(P.WEIGHT_CAUTION_KG, v)} />
          <ParamSlider theme={theme} label="Weight E-STOP at" value={weightEstop} setValue={setWeightEstop}
            min={10} max={200} step={5} color={theme.danger} unit="kg"
            onCommit={v => sendParam(P.WEIGHT_ESTOP_KG, v)} />
          <ParamSlider theme={theme} label="Imbalance caution" value={imbCaution} setValue={setImbCaution}
            min={0.05} max={3.0} step={0.05} color={theme.warn} unit="frac"
            onCommit={v => sendParam(P.IMBALANCE_CAUTION, v)} />
          <ParamSlider theme={theme} label="Imbalance E-STOP" value={imbEstop} setValue={setImbEstop}
            min={0.05} max={3.0} step={0.05} color={theme.danger} unit="frac"
            onCommit={v => sendParam(P.IMBALANCE_ESTOP, v)} />
        </div>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          Imbalance metric = worst corner's |weight − mean| / mean — NOT the CoM offset the
          Telemetry heatmap shows. Ignored below 5 kg total. <code>PARAM 0x30–0x33</code>.
        </div>
      </div>

      {/* LiDAR distance bands */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>LiDAR Distance Bands (closest fresh segment)</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '14px' }}>
          <ParamSlider theme={theme} label="Caution <" value={lidarCaution} setValue={setLidarCaution}
            min={50} max={3000} step={10} color={theme.warn} unit="mm"
            onCommit={v => sendParam(P.LIDAR_CAUTION_MM, v)} />
          <ParamSlider theme={theme} label="Critical <" value={lidarCritical} setValue={setLidarCritical}
            min={50} max={3000} step={10} color={theme.warn} unit="mm"
            onCommit={v => sendParam(P.LIDAR_CRITICAL_MM, v)} />
          <ParamSlider theme={theme} label="E-STOP <" value={lidarEstop} setValue={setLidarEstop}
            min={50} max={3000} step={10} color={theme.danger} unit="mm"
            onCommit={v => sendParam(P.LIDAR_ESTOP_MM, v)} />
        </div>
        <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
          Auto-clearing, like the IR ring; stale LiDAR (&gt;500 ms) reads as clear. Keep E-STOP &lt; Critical &lt; Caution. <code>PARAM 0x65–0x67</code>.
        </div>
      </div>

      {/* Battery (3S) low-voltage limits */}
      <div style={{ gridColumn: '1 / -1' }}>
        <div style={{ fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, marginBottom: '8px', fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600 }}>3S Battery Low-Voltage Limits</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '14px' }}>
          <ParamSlider theme={theme} label="Caution <" value={battCaution} setValue={setBattCaution}
            min={9000} max={12000} step={100} color={theme.warn} unit="mV"
            onCommit={v => sendParam(P.BATT_3S_CAUTION_MV, v)} />
          <ParamSlider theme={theme} label="E-STOP <" value={battEstop} setValue={setBattEstop}
            min={9000} max={12000} step={100} color={theme.danger} unit="mV"
            onCommit={v => sendParam(P.BATT_3S_ESTOP_MV, v)} />
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
          </div>
          <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, lineHeight: 1.5, marginTop: '8px' }}>
            One big ring maps each sensor to LEDs: IR red on detection, LiDAR arc yellow→red by range. <code>PARAM 0x51</code>.
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
        <ParamSlider theme={theme} label="Speed limit" value={speed} setValue={setSpeed}
          min={0.05} max={1.0} step={0.05} color={theme.accent} unit="m/s" />

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

// Module-scope (like PanelBtn): a stable identity across the telemetry-rate
// re-renders so in-progress slider drags and text edits aren't interrupted.
// Slider commits the param on release; the text field (GainRow-style draft
// state) commits on Enter/blur and accepts values beyond the slider range —
// the slider max stretches to follow, and the firmware setters clamp anyway.
function ParamSlider({ theme, label, value, setValue, min, max, step, color, unit = '', note, onCommit }) {
  const [draft, setDraft] = React.useState(null);   // null = not editing
  const cancelRef = React.useRef(false);
  const commit = (v) => { if (onCommit) onCommit(v); };
  const acceptDraft = () => {
    if (cancelRef.current) { cancelRef.current = false; setDraft(null); return; }
    if (draft === null) return;
    const n = parseFloat(draft);
    setDraft(null);
    if (!isNaN(n)) { setValue(n); commit(n); }
  };
  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
        <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase' }}>{label}</span>
        <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          <input type="text" inputMode="decimal" value={draft !== null ? draft : String(value)}
            onFocus={() => setDraft(String(value))}
            onChange={e => setDraft(e.target.value)}
            onBlur={acceptDraft}
            onKeyDown={e => {
              if (e.key === 'Enter') e.target.blur();
              if (e.key === 'Escape') { cancelRef.current = true; e.target.blur(); }
            }}
            style={{
              width: '64px', background: theme.bg, border: `1px solid ${draft !== null ? color : theme.border}`,
              borderRadius: '5px', color, fontFamily: theme.monoFont, fontSize: '12px', fontWeight: 700,
              padding: '3px 6px', textAlign: 'right', outline: 'none', transition: 'border-color 0.15s',
            }} />
          {unit && <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>{unit}</span>}
        </span>
      </div>
      <input type="range" min={min} max={Math.max(max, value)} step={step} value={value}
        onChange={e => { setValue(parseFloat(e.target.value)); setDraft(null); }}
        onMouseUp={() => commit(value)}
        style={{ width: '100%', accentColor: color }} />
      {note && <div style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont, marginTop: '4px' }}>{note}</div>}
    </div>
  );
}

function LineFollowPanel({ theme, agv, connected, telem }) {
  const [cruise, setCruise] = usePersistentState('lineFollow.cruise', 0.3);
  const [lostThresh, setLostThresh] = usePersistentState('lineFollow.lostThresh', 100);
  const [tBlack, setTBlack] = usePersistentState('lineFollow.tBlack', 2500);
  const [tMinSensors, setTMinSensors] = usePersistentState('lineFollow.tMinSensors', 4);
  const [tDebounce, setTDebounce] = usePersistentState('lineFollow.tDebounce', 3);
  const [reacqTicks, setReacqTicks] = usePersistentState('lineFollow.reacqTicks', 5);
  const [turnCcw, setTurnCcw] = usePersistentState('lineFollow.turnCcw', 1);
  const [turnOmega, setTurnOmega] = usePersistentState('lineFollow.turnOmega', 1.0);
  const [blindDeg, setBlindDeg] = usePersistentState('lineFollow.blindDeg', 150);
  const [maxSweepDeg, setMaxSweepDeg] = usePersistentState('lineFollow.maxSweepDeg', 345);
  const [timeoutS, setTimeoutS] = usePersistentState('lineFollow.timeoutS', 8);
  const send = (id, val) => { if (connected && agv) try { agv.sendParamUpdate(id, val); } catch (_) {} };
  const P = window.AGV_PROTO?.PARAM || {};
  const DEG = Math.PI / 180;   // firmware angles are radians; sliders show degrees

  const secHd = (label) => (
    <div style={{ gridColumn: '1 / -1', fontSize: '9px', letterSpacing: '0.14em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', fontWeight: 600, marginBottom: '-6px' }}>{label}</div>
  );

  return (
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '16px' }}>
      {secHd('Following')}
      <ParamSlider theme={theme} label="Cruise speed" value={cruise} setValue={setCruise}
        min={0.05} max={1.0} step={0.05} color={theme.success}
        unit="m/s" note="PARAM_LINE_CRUISE_MPS"
        onCommit={v => send(P.LINE_CRUISE_MPS, v)} />
      <ParamSlider theme={theme} label="Line-lost threshold" value={lostThresh} setValue={setLostThresh}
        min={0} max={1000} step={10} color={theme.warn}
        unit="cts"
        note="Array contrast (max−min ADC counts) below this → line not visible: halt + LINE_LOST while following, keep searching during the T-turn (0 disables)"
        onCommit={v => send(P.QTR_LINE_LOST_THRESH, v)} />

      {secHd('T-junction detection')}
      <ParamSlider theme={theme} label="T-bar black threshold" value={tBlack} setValue={setTBlack}
        min={500} max={4000} step={50} color={theme.accent}
        unit="cts"
        note='Sensor reads "black" above this (raw ADC)'
        onCommit={v => send(P.LINE_T_BLACK, v)} />
      <ParamSlider theme={theme} label="T-bar min sensors" value={tMinSensors} setValue={setTMinSensors}
        min={2} max={8} step={1} color={theme.accent}
        unit="/ 8"
        note="≥ this many black sensors = T bar; also: search phase only reacquires with fewer black than this"
        onCommit={v => send(P.LINE_T_MIN_SENSORS, v)} />
      <ParamSlider theme={theme} label="T-bar debounce" value={tDebounce} setValue={setTDebounce}
        min={1} max={10} step={1} color={theme.accent}
        unit="ticks"
        note="Consecutive control frames (100 Hz) over the bar before the turn triggers"
        onCommit={v => send(P.LINE_T_DEBOUNCE, v)} />

      {secHd('180° turnaround')}
      <div>
        <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.1em', textTransform: 'uppercase', marginBottom: '4px' }}>Turn direction</div>
        <div style={{ display: 'flex', gap: '8px' }}>
          {[['↺ CCW (left)', 1], ['↻ CW (right)', 0]].map(([lbl, v]) => (
            <PanelBtn theme={theme} key={v} color={turnCcw === v ? theme.success : theme.accent}
              onClick={() => { setTurnCcw(v); send(P.LINE_TURN_CCW, v); }}>
              {turnCcw === v ? '● ' : ''}{lbl}
            </PanelBtn>
          ))}
        </div>
      </div>
      <ParamSlider theme={theme} label="Turn rate" value={turnOmega} setValue={setTurnOmega}
        min={0.2} max={3.0} step={0.05} color={theme.success}
        unit="rad/s"
        note="On-axis ω during the turn (caution modifier still clamps it)"
        onCommit={v => send(P.LINE_TURN_OMEGA, v)} />
      <ParamSlider theme={theme} label="Blind sweep" value={blindDeg} setValue={setBlindDeg}
        min={30} max={330} step={5} color={theme.warn}
        unit="°"
        note="Rotate at least this far (encoder odometry) before searching for the line — must clear the T bar from view"
        onCommit={v => send(P.LINE_TURN_BLIND_RAD, v * DEG)} />
      <ParamSlider theme={theme} label="Reacquire debounce" value={reacqTicks} setValue={setReacqTicks}
        min={1} max={20} step={1} color={theme.warn}
        unit="ticks"
        note="Consecutive line-visible frames (100 Hz) before the search ends — rejects single-frame floor-sheen false positives"
        onCommit={v => send(P.LINE_REACQ_TICKS, v)} />
      <ParamSlider theme={theme} label="Max sweep" value={maxSweepDeg} setValue={setMaxSweepDeg}
        min={180} max={700} step={5} color={theme.danger}
        unit="°"
        note="Swept this far without reacquiring → give up (LINE_TURN_FAILED + lost stop)"
        onCommit={v => send(P.LINE_TURN_MAX_RAD, v * DEG)} />
      <ParamSlider theme={theme} label="Turn timeout" value={timeoutS} setValue={setTimeoutS}
        min={1} max={20} step={0.5} color={theme.danger}
        unit="s"
        note="Hard time cap on the whole turn (covers frozen odometry)"
        onCommit={v => send(P.LINE_TURN_TIMEOUT_MS, v * 1000)} />

      <div style={{ gridColumn: '1 / -1', display: 'flex', gap: '14px', background: theme.bg, border: `1px solid ${theme.border}`, borderRadius: '8px', padding: '10px 14px' }}>
        <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, textTransform: 'uppercase', letterSpacing: '0.1em' }}>Line position</span>
        <span style={{ fontSize: '12px', fontFamily: theme.monoFont, color: theme.accent, fontWeight: 700 }}>
          {typeof telem.qtrLinePosition === 'number' ? telem.qtrLinePosition.toFixed(3) : '—'}
        </span>
        <span style={{ marginLeft: 'auto', fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted }}>From TLM_QTR (streams during LINE_FOLLOW) · PID gains tunable on PID tab</span>
      </div>
    </div>
  );
}

// ── Dashboard Tab ───────────────────────────────────────────────────────────
function DashboardTab({ telem, setFunc, setMode, theme, agv, connected, rcPressed, setRcPressed, rcSpeed, setRcSpeed, rcV, rcOmega }) {
  const { func, mode, estop, caution } = telem;

  const functions = ['STANDBY', 'REMOTE_CONTROL', 'LINE_FOLLOW'];
  const funcColors = {
    STANDBY: theme.accent,
    REMOTE_CONTROL: theme.warn,
    LINE_FOLLOW: theme.success,
  };
  const funcDescriptions = {
    STANDBY: 'Motors idle. Calibration & setup available.',
    REMOTE_CONTROL: 'Direct velocity commands from workstation.',
    LINE_FOLLOW: 'Following floor line via QTR-8A array.',
  };

  const bgCard = theme.cardBg;
  const border = theme.border;

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
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '10px' }}>
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
        {/* Inlined (not a nested component) so the active panel keeps its identity
            and in-progress edits across DashboardTab's telemetry-rate re-renders. */}
        {func === 'STANDBY'           && <StandbyPanel theme={theme} agv={agv} connected={connected} telem={telem} />}
        {func === 'REMOTE_CONTROL'    && <RemoteControlPanel theme={theme} telem={telem} pressed={rcPressed} setPressed={setRcPressed} speed={rcSpeed} setSpeed={setRcSpeed} rcV={rcV} rcOmega={rcOmega} />}
        {func === 'LINE_FOLLOW'       && <LineFollowPanel theme={theme} agv={agv} connected={connected} telem={telem} />}
      </div>
    </div>
  );
}

// ── Weight heat map — CSS radial gradients (GPU-smooth, no blinking) ─────────
// QTR line sensors: 8 raw 12-bit ADC readings rendered as a horizontal heat strip.
// Higher raw value = darker surface = line. CoM is computed per-frame after
// in-frame min/max normalisation so the marker tracks the line under uneven
// ambient light without calibration — the same auto-ranging the firmware's
// nav_line uses, so this marker matches the firmware's line_position.
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
  const { velocity, encoders, loadCells, current, caution, motors, qtr } = telem;
  const lidar = telem.lidar || [];
  const battery = telem.battery || { v3s: 0, pct3s: null };

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

  // The firmware's imbalance metric (safety.c cargo_tick): worst corner's
  // |weight − mean| / mean, ignored below the 5 kg floor. This is what the
  // imbalance caution/E-STOP limits compare against — NOT the CoM above.
  const IMBALANCE_FLOOR_KG = 5;
  const [imbCautionLim] = usePersistentState('standby.imbCaution', 0.6);
  const [imbEstopLim] = usePersistentState('standby.imbEstop', 1.5);
  const imbMean = loadCells.total / 4;
  const imbFrac = (loadCells.total >= IMBALANCE_FLOOR_KG && imbMean > 0)
    ? Math.max(...[loadCells.fl, loadCells.fr, loadCells.rl, loadCells.rr].map(w => Math.abs(w - imbMean))) / imbMean
    : null;
  const imbColor = imbFrac === null ? theme.muted
    : imbFrac >= imbEstopLim ? theme.danger
    : imbFrac >= imbCautionLim ? theme.warn : theme.success;

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
          {[['FL', telem.proximity.fl], ['FR', telem.proximity.fr], ['RL', telem.proximity.rl], ['RR', telem.proximity.rr]].map(([pos, active]) => (
            <div key={pos} style={{ display: 'flex', alignItems: 'center', gap: '6px', background: active ? theme.danger + '18' : theme.bg, borderRadius: '6px', padding: '6px 8px', border: `1px solid ${active ? theme.danger : theme.border}`, transition: 'all 0.1s' }}>
              <div style={{ width: '8px', height: '8px', borderRadius: '50%', background: active ? theme.danger : theme.border, boxShadow: active ? `0 0 5px ${theme.danger}` : 'none', flexShrink: 0 }} />
              <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: active ? theme.danger : theme.muted, fontWeight: active ? 700 : 400 }}>{pos}</span>
            </div>
          ))}
        </div>
      </Card>

      {/* Row 2: LiDAR Segments | Battery | (Velocity continues below) */}
      <Card title="LiDAR Segments (Jetson)">
        {lidar.length === 0 ? (
          <div style={{ fontSize: '11px', fontFamily: theme.monoFont, color: theme.muted }}>No fresh segments (stale / LiDAR off)</div>
        ) : (() => {
          const min = Math.min(...lidar);
          const minCol = min <= 200 ? theme.danger : min <= 800 ? theme.warn : theme.fg;
          return (
            <div>
              <Row label="Min distance" value={min >= 8000 ? 'clear' : min} unit={min >= 8000 ? '' : 'mm'} color={minCol} />
              <Row label="Segments" value={lidar.length} />
              <div style={{ display: 'flex', gap: '2px', alignItems: 'flex-end', height: '36px', marginTop: '6px' }}>
                {lidar.map((mm, i) => {
                  const t = Math.max(0, Math.min(1, mm / 2000));
                  const col = mm <= 200 ? theme.danger : mm <= 800 ? theme.warn : theme.success;
                  return <div key={i} style={{ flex: 1, height: `${Math.max(8, (1 - t) * 100)}%`, background: col, borderRadius: '2px', opacity: mm >= 8000 ? 0.2 : 0.9 }} title={`${mm} mm`} />;
                })}
              </div>
              <div style={{ fontSize: '9px', fontFamily: theme.monoFont, color: theme.muted, textAlign: 'center', marginTop: '4px' }}>0° → MAX FOV (taller = closer)</div>
            </div>
          );
        })()}
      </Card>
      <Card title="Battery">
        {[['3S', battery.v3s, battery.pct3s]].map(([name, v, pct]) => {
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

      {/* Velocity | Encoders */}
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

      {/* Row 3: Weight Distribution | Line Sensors (span 2) */}
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
        <div style={{ fontSize: '10px', fontFamily: theme.monoFont, textAlign: 'center', marginTop: '4px', color: imbColor }}>
          Imbalance (firmware metric): {imbFrac === null
            ? `— (under ${IMBALANCE_FLOOR_KG} kg floor)`
            : `${(imbFrac * 100).toFixed(0)}% · limits ${(imbCautionLim * 100).toFixed(0)}/${(imbEstopLim * 100).toFixed(0)}%`}
        </div>
      </Card>
      <Card title="Line Sensors (QTR)" style={{ gridColumn: 'span 2' }}>
        <QtrHeatmap values={qtr && qtr.length === 8 ? qtr : [0,0,0,0,0,0,0,0]} />
      </Card>
    </div>
  );
}

Object.assign(window, { DashboardTab, TelemetryTab });
