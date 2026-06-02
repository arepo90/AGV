
// ── Map Tab (placeholder) ───────────────────────────────────────────────────
function MapTab({ telem, theme }) {
  const { position } = telem;
  const canvasRef = React.useRef(null);
  const trailRef = React.useRef([]);

  React.useEffect(() => {
    trailRef.current.push({ x: position.x, y: position.y });
    if (trailRef.current.length > 400) trailRef.current.shift();
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const scale = 80;
    const cx = W / 2, cy = H / 2;
    ctx.clearRect(0, 0, W, H);
    // grid
    ctx.strokeStyle = theme.border;
    ctx.lineWidth = 0.5;
    for (let gx = cx % scale; gx < W; gx += scale) { ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, H); ctx.stroke(); }
    for (let gy = cy % scale; gy < H; gy += scale) { ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(W, gy); ctx.stroke(); }
    ctx.strokeStyle = theme.border; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();
    // trail
    const trail = trailRef.current;
    if (trail.length > 1) {
      ctx.strokeStyle = theme.accent + '66'; ctx.lineWidth = 2;
      ctx.beginPath();
      trail.forEach((p, i) => {
        const px = cx + p.x * scale, py = cy - p.y * scale;
        i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    // robot
    const rx = cx + position.x * scale, ry = cy - position.y * scale;
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(-position.theta);
    ctx.fillStyle = theme.accent;
    ctx.beginPath(); ctx.moveTo(12, 0); ctx.lineTo(-8, -7); ctx.lineTo(-8, 7); ctx.closePath(); ctx.fill();
    ctx.restore();
    // position label
    ctx.fillStyle = theme.muted; ctx.font = `10px ${theme.monoFont}`; ctx.fillText(`x: ${position.x.toFixed(2)}m  y: ${position.y.toFixed(2)}m  θ: ${(position.theta * 180 / Math.PI).toFixed(1)}°`, 12, H - 12);
  });

  return (
    <div style={{ height: '100%', display: 'flex', flexDirection: 'column', padding: '16px', gap: '10px', boxSizing: 'border-box' }}>
      <div style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, letterSpacing: '0.12em', textTransform: 'uppercase' }}>
        Odometry — 2D Position Trace &nbsp;·&nbsp; <span style={{ color: theme.warn }}>Map features coming in future scope</span>
      </div>
      <canvas ref={canvasRef} width={800} height={440}
        style={{ flex: 1, width: '100%', borderRadius: '10px', border: `1px solid ${theme.border}`, background: theme.cardBg }} />
    </div>
  );
}

// ── Trajectory Tab ──────────────────────────────────────────────────────────
function TrajectoryTab({ telem, theme }) {
  const [waypoints, setWaypoints] = React.useState([
    { id: 1, x: '0.50', y: '0.00', label: 'WP-1' },
    { id: 2, x: '0.50', y: '0.50', label: 'WP-2' },
    { id: 3, x: '0.00', y: '0.50', label: 'WP-3' },
  ]);
  const [nextId, setNextId] = React.useState(4);
  const [status, setStatus] = React.useState(null);

  function addWP() {
    setWaypoints(w => [...w, { id: nextId, x: '0.00', y: '0.00', label: `WP-${nextId}` }]);
    setNextId(n => n + 1);
  }
  function removeWP(id) { setWaypoints(w => w.filter(p => p.id !== id)); }
  function updateWP(id, field, val) { setWaypoints(w => w.map(p => p.id === id ? { ...p, [field]: val } : p)); }
  function upload() {
    setStatus('uploading');
    setTimeout(() => setStatus('ok'), 1200);
    setTimeout(() => setStatus(null), 3500);
  }

  return (
    <div style={{ padding: '16px', display: 'grid', gridTemplateColumns: '1fr 1.4fr', gap: '14px', height: '100%', boxSizing: 'border-box' }}>
      {/* Waypoint list */}
      <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px', display: 'flex', flexDirection: 'column', gap: '8px', overflowY: 'auto' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
          <span style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase' }}>Waypoints</span>
          <button onClick={addWP} style={{ background: theme.accent, border: 'none', borderRadius: '6px', color: '#fff', fontSize: '11px', padding: '5px 12px', cursor: 'pointer', fontFamily: theme.monoFont, fontWeight: 600 }}>+ Add</button>
        </div>
        {waypoints.map((wp, i) => (
          <div key={wp.id} style={{ display: 'flex', gap: '6px', alignItems: 'center', background: theme.bg, borderRadius: '7px', padding: '8px 10px' }}>
            <span style={{ fontSize: '10px', fontFamily: theme.monoFont, color: theme.muted, width: '30px' }}>{i + 1}</span>
            <input value={wp.label} onChange={e => updateWP(wp.id, 'label', e.target.value)}
              style={{ width: '60px', background: 'transparent', border: `1px solid ${theme.border}`, borderRadius: '4px', color: theme.fg, fontFamily: theme.monoFont, fontSize: '11px', padding: '3px 6px' }} />
            <span style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont }}>X</span>
            <input value={wp.x} onChange={e => updateWP(wp.id, 'x', e.target.value)}
              style={{ width: '52px', background: 'transparent', border: `1px solid ${theme.border}`, borderRadius: '4px', color: theme.fg, fontFamily: theme.monoFont, fontSize: '11px', padding: '3px 6px' }} />
            <span style={{ fontSize: '10px', color: theme.muted, fontFamily: theme.monoFont }}>Y</span>
            <input value={wp.y} onChange={e => updateWP(wp.id, 'y', e.target.value)}
              style={{ width: '52px', background: 'transparent', border: `1px solid ${theme.border}`, borderRadius: '4px', color: theme.fg, fontFamily: theme.monoFont, fontSize: '11px', padding: '3px 6px' }} />
            <button onClick={() => removeWP(wp.id)} style={{ marginLeft: 'auto', background: 'transparent', border: 'none', color: theme.muted, cursor: 'pointer', fontSize: '14px', padding: '0 4px' }}>×</button>
          </div>
        ))}
        <div style={{ marginTop: 'auto', display: 'flex', gap: '8px' }}>
          <button onClick={upload} style={{ flex: 1, background: status === 'ok' ? theme.success : theme.accent, border: 'none', borderRadius: '7px', color: '#fff', fontSize: '12px', padding: '10px', cursor: 'pointer', fontFamily: theme.monoFont, fontWeight: 700, transition: 'background 0.3s' }}>
            {status === 'uploading' ? 'Uploading…' : status === 'ok' ? '✓ Uploaded' : 'Upload to AGV'}
          </button>
        </div>
      </div>
      {/* Preview canvas */}
      <div style={{ background: theme.cardBg, border: `1px solid ${theme.border}`, borderRadius: '10px', padding: '16px', display: 'flex', flexDirection: 'column' }}>
        <div style={{ fontSize: '10px', letterSpacing: '0.12em', color: theme.muted, fontFamily: theme.monoFont, textTransform: 'uppercase', marginBottom: '12px' }}>Path Preview</div>
        <TrajectoryCanvas waypoints={waypoints} theme={theme} />
      </div>
    </div>
  );
}

function TrajectoryCanvas({ waypoints, theme }) {
  const canvasRef = React.useRef(null);
  React.useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const PAD = 40;
    ctx.clearRect(0, 0, W, H);
    const pts = waypoints.map(w => ({ x: parseFloat(w.x) || 0, y: parseFloat(w.y) || 0 }));
    if (pts.length === 0) return;
    const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
    const minX = Math.min(...xs, 0), maxX = Math.max(...xs, 0.01);
    const minY = Math.min(...ys, 0), maxY = Math.max(...ys, 0.01);
    const rangeX = maxX - minX || 1, rangeY = maxY - minY || 1;
    const scale = Math.min((W - PAD * 2) / rangeX, (H - PAD * 2) / rangeY);
    const ox = PAD + (W - PAD * 2 - rangeX * scale) / 2 - minX * scale;
    const oy = H - PAD - (H - PAD * 2 - rangeY * scale) / 2 + minY * scale;
    const tx = x => ox + x * scale, ty = y => oy - y * scale;
    // grid
    ctx.strokeStyle = theme.border; ctx.lineWidth = 0.5;
    for (let gx = 0; gx <= rangeX; gx += 0.25) { ctx.beginPath(); ctx.moveTo(tx(minX + gx), 0); ctx.lineTo(tx(minX + gx), H); ctx.stroke(); }
    for (let gy = 0; gy <= rangeY; gy += 0.25) { ctx.beginPath(); ctx.moveTo(0, ty(minY + gy)); ctx.lineTo(W, ty(minY + gy)); ctx.stroke(); }
    // path
    if (pts.length > 1) {
      ctx.strokeStyle = theme.accent + 'AA'; ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath(); pts.forEach((p, i) => i === 0 ? ctx.moveTo(tx(p.x), ty(p.y)) : ctx.lineTo(tx(p.x), ty(p.y))); ctx.stroke();
      ctx.setLineDash([]);
    }
    // waypoints
    pts.forEach((p, i) => {
      ctx.fillStyle = theme.accent; ctx.beginPath(); ctx.arc(tx(p.x), ty(p.y), 7, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = `bold 9px monospace`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(i + 1, tx(p.x), ty(p.y));
      ctx.fillStyle = theme.muted; ctx.font = `9px monospace`; ctx.textAlign = 'left';
      ctx.fillText(waypoints[i].label, tx(p.x) + 10, ty(p.y) - 8);
    });
    // origin
    ctx.strokeStyle = theme.muted + '80'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(tx(0), ty(0), 5, 0, Math.PI * 2); ctx.stroke();
  }, [waypoints, theme]);
  return <canvas ref={canvasRef} width={440} height={360} style={{ width: '100%', flex: 1, borderRadius: '6px' }} />;
}

Object.assign(window, { MapTab, TrajectoryTab, TrajectoryCanvas });
