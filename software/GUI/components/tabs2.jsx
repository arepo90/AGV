
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

Object.assign(window, { MapTab });
