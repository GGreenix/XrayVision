"""WebSocket server exposing PC-side detections to Unity.

Same payload format as the Pi server so Unity's PiClient works unchanged,
just pointed at the PC's IP and port instead.
"""

import asyncio
import json

from aiohttp import WSMsgType, web


class PCServer:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self._latest: dict = {"t": 0.0, "objects": []}
        self._subscribers: set[web.WebSocketResponse] = set()
        self._lock = asyncio.Lock()

        self._pi_url = ""
        self._depth_subscribers: set[asyncio.Queue] = set()
        self._latest_depth_jpg: bytes = b""

        self.app = web.Application()
        self.app.router.add_get("/", self._index)
        self.app.router.add_get("/stream", self._stream_ws)
        self.app.router.add_get("/healthz", self._healthz)
        self.app.router.add_get("/pose", self._pose_endpoint)
        self.app.router.add_get("/video/depth.mjpg", self._depth_mjpg)

    async def run(self):
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        print(f"[pc_server] listening on http://{self.host}:{self.port}", flush=True)
        while True:
            await asyncio.sleep(1)

    async def publish(self, payload: dict):
        async with self._lock:
            self._latest = payload
        text = json.dumps(payload)
        dead = []
        for ws in list(self._subscribers):
            try:
                await ws.send_str(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._subscribers.discard(ws)

    async def publish_pose(self, pose):
        self._current_pose = pose

    async def publish_depth(self, jpg: bytes):
        self._latest_depth_jpg = jpg
        dead = []
        for q in list(self._depth_subscribers):
            try:
                if q.full():
                    try: q.get_nowait()
                    except Exception: pass
                q.put_nowait(jpg)
            except Exception:
                dead.append(q)
        for q in dead:
            self._depth_subscribers.discard(q)

    async def _depth_mjpg(self, request: web.Request) -> web.StreamResponse:
        response = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame"
        })
        await response.prepare(request)
        q: asyncio.Queue = asyncio.Queue(maxsize=2)
        self._depth_subscribers.add(q)
        if self._latest_depth_jpg:
            await q.put(self._latest_depth_jpg)
        try:
            while True:
                jpg = await q.get()
                await response.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                )
        except Exception:
            pass
        finally:
            self._depth_subscribers.discard(q)
        return response

    async def _stream_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)
        self._subscribers.add(ws)
        async with self._lock:
            await ws.send_str(json.dumps(self._latest))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.ERROR:
                    break
        finally:
            self._subscribers.discard(ws)
        return ws

    async def _healthz(self, request: web.Request) -> web.Response:
        return web.json_response({"ok": True, "subscribers": len(self._subscribers)})

    async def _index(self, request: web.Request) -> web.Response:
        pi_url = self._pi_url
        html = f"""<!doctype html>
<html><head><title>XrayVision</title>
<style>
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{ background: #0d0d0d; color: #eee; font-family: monospace; height: 100vh; display: flex; flex-direction: column; }}
header {{ padding: 10px 16px; background: #111; border-bottom: 1px solid #222; display: flex; align-items: center; gap: 20px; }}
h1 {{ font-size: 14px; color: #8af; }}
.field {{ font-size: 12px; display: flex; flex-direction: column; gap: 1px; }}
.field-label {{ color: #555; font-size: 10px; text-transform: uppercase; }}
.field-value {{ color: #eee; }}
#status {{ color: #f88; }}
#fps {{ color: #8f8; }}
#objects {{ color: #fa8; }}
#classes {{ color: #c8f; }}
.main {{ display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr; gap: 6px; padding: 6px; flex: 1; min-height: 0; }}
.panel-wide {{ grid-column: span 2; }}
.panel {{ background: #111; border: 1px solid #222; border-radius: 4px; display: flex; flex-direction: column; overflow: hidden; }}
.panel-title {{ font-size: 11px; color: #888; padding: 4px 8px; border-bottom: 1px solid #1a1a1a; }}
.panel img {{ width: 100%; height: 100%; object-fit: contain; background: #000; }}
#map {{ width: 100%; height: 100%; }}
#detections {{ flex: 1; overflow-y: auto; padding: 6px; }}
table {{ width: 100%; border-collapse: collapse; font-size: 11px; }}
th {{ color: #666; text-align: left; padding: 3px 6px; border-bottom: 1px solid #222; position: sticky; top: 0; background: #111; }}
td {{ padding: 3px 6px; border-bottom: 1px solid #1a1a1a; }}
tr:hover td {{ background: #1a1a1a; }}
.conf {{ color: #8f8; }}
.pos {{ color: #8af; }}
.id {{ color: #fa8; }}
</style>
</head>
<body>
<header>
  <h1>XrayVision</h1>
  <div class="field"><span class="field-label">Status</span><span class="field-value" id="status">connecting...</span></div>
  <div class="field"><span class="field-label">Rate</span><span class="field-value" id="fps">— Hz</span></div>
  <div class="field"><span class="field-label">Objects</span><span class="field-value" id="objects">0</span></div>
  <div class="field"><span class="field-label">Classes</span><span class="field-value" id="classes">—</span></div>
  <div class="field"><span class="field-label">Camera Pos</span><span class="field-value" id="campos">—</span></div>
</header>
<div class="main">
  <div class="panel">
    <div class="panel-title">Left Camera</div>
    <img src="{pi_url}/video/left.mjpg" onerror="this.alt='No signal'">
  </div>
  <div class="panel">
    <div class="panel-title">Right Camera</div>
    <img src="{pi_url}/video/right.mjpg" onerror="this.alt='No signal'">
  </div>
  <div class="panel">
    <div class="panel-title">Depth Map (SGBM)</div>
    <img src="/video/depth.mjpg" onerror="this.alt='No depth signal'">
  </div>
  <div class="panel panel-wide">
    <div class="panel-title">Top-down Map (X/Z plane)</div>
    <canvas id="map"></canvas>
  </div>
  <div class="panel">
    <div class="panel-title">Detections</div>
    <div id="detections">
      <table>
        <thead><tr><th>ID</th><th>Class</th><th>Conf</th><th>X</th><th>Y</th><th>Z</th><th>Depth</th></tr></thead>
        <tbody id="tbody"></tbody>
      </table>
    </div>
  </div>
</div>
<script>
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const fpsEl = document.getElementById('fps');
const objectsEl = document.getElementById('objects');
const classesEl = document.getElementById('classes');
const camposEl = document.getElementById('campos');
let lastT = 0, frameCount = 0, fps = 0;
let objects = [];


function resizeCanvas() {{
  canvas.width = canvas.offsetWidth;
  canvas.height = canvas.offsetHeight;
}}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// World-to-canvas mapping with auto-scale
function drawMap() {{
  const w = canvas.width, h = canvas.height;
  ctx.fillStyle = '#0d0d0d';
  ctx.fillRect(0, 0, w, h);

  // Grid
  ctx.strokeStyle = '#1a1a1a';
  ctx.lineWidth = 1;
  for (let i = 0; i < w; i += 40) {{ ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, h); ctx.stroke(); }}
  for (let i = 0; i < h; i += 40) {{ ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(w, i); ctx.stroke(); }}

  // Camera icon at center-top
  const camX = w / 2, camY = 40;
  const axisLen = 28;

  // X axis (red) — points right in world space
  ctx.strokeStyle = '#f44'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(camX, camY); ctx.lineTo(camX + axisLen, camY); ctx.stroke();
  ctx.fillStyle = '#f44'; ctx.font = 'bold 10px monospace';
  ctx.fillText('X', camX + axisLen + 3, camY + 4);

  // Z axis (blue) — points forward (depth)
  ctx.strokeStyle = '#44f'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(camX, camY); ctx.lineTo(camX, camY + axisLen); ctx.stroke();
  ctx.fillStyle = '#44f';
  ctx.fillText('Z', camX - 12, camY + axisLen + 3);

  // Y axis (green) — points up, shown as dot (into screen)
  ctx.strokeStyle = '#4f4'; ctx.fillStyle = '#4f4';
  ctx.beginPath(); ctx.arc(camX, camY, 4, 0, Math.PI * 2); ctx.fill();
  ctx.font = '10px monospace';
  ctx.fillText('Y', camX - 14, camY - 6);

  // Camera body
  ctx.fillStyle = '#8af';
  ctx.beginPath(); ctx.arc(camX, camY, 6, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#8af'; ctx.font = '10px monospace';
  ctx.fillText('CAM', camX + 9, camY - 6);

  // Scale: 1m = 60px
  const scale = 60;

  for (const obj of objects) {{
    const px = camX + obj.x * scale;
    const py = camY + obj.z * scale;

    // Detection circle
    ctx.fillStyle = 'rgba(255,60,60,0.85)';
    ctx.beginPath(); ctx.arc(px, py, 10, 0, Math.PI * 2); ctx.fill();

    // Label
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(obj.class_id, px + 13, py - 4);
    ctx.fillStyle = '#aaa';
    ctx.font = '10px monospace';
    ctx.fillText(`${{obj.depth_m.toFixed(1)}}m`, px + 13, py + 8);

    // Line from camera to object
    ctx.strokeStyle = 'rgba(255,60,60,0.3)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(camX, camY); ctx.lineTo(px, py); ctx.stroke();
  }}
}}

function updateTable() {{
  const tbody = document.getElementById('tbody');
  tbody.innerHTML = objects.map(o => `
    <tr>
      <td class="id">${{o.tracking_id}}</td>
      <td>${{o.class_id}}</td>
      <td class="conf">${{(o.confidence * 100).toFixed(0)}}%</td>
      <td class="pos">${{o.x.toFixed(2)}}</td>
      <td class="pos">${{o.y.toFixed(2)}}</td>
      <td class="pos">${{o.z.toFixed(2)}}</td>
      <td>${{o.depth_m.toFixed(2)}}m</td>
    </tr>`).join('');
}}

function connect() {{
  const ws = new WebSocket(`ws://${{location.host}}/stream`);
  ws.onopen = () => {{ statusEl.textContent = 'connected'; statusEl.style.color = '#8f8'; }};
  ws.onclose = () => {{
    statusEl.textContent = 'disconnected — retrying...';
    statusEl.style.color = '#f88';
    setTimeout(connect, 2000);
  }};
  ws.onerror = () => ws.close();
  ws.onmessage = (e) => {{
    const data = JSON.parse(e.data);
    objects = data.objects || [];
    if (data.camera_pos) {{
      const p = data.camera_pos;
      camposEl.textContent = `${{p[0].toFixed(2)}}, ${{p[1].toFixed(2)}}, ${{p[2].toFixed(2)}}`;
    }}
    const now = performance.now();
    frameCount++;
    if (now - lastT > 1000) {{
      fps = (frameCount * 1000 / (now - lastT)).toFixed(1);
      fpsEl.textContent = `${{fps}} Hz`;
      frameCount = 0; lastT = now;
    }}
    objectsEl.textContent = objects.length;
    const uniqueClasses = [...new Set(objects.map(o => o.class_id))];
    classesEl.textContent = uniqueClasses.length ? uniqueClasses.join(', ') : '—';
    drawMap();
    updateTable();
  }};
}}
connect();
setInterval(() => {{ resizeCanvas(); drawMap(); }}, 100);
</script>
</body></html>"""
        return web.Response(text=html, content_type="text/html")

    async def _pose_endpoint(self, request: web.Request) -> web.Response:
        pose = getattr(self, "_current_pose", None)
        if pose is None:
            return web.json_response({"position_m": [0, 0, 0], "rpy_deg": [0, 0, 0], "mount_rpy_deg": [0, 0, 0]})
        return web.json_response({
            "position_m": pose.position_m.tolist(),
            "rpy_deg": pose.rpy_deg.tolist(),
            "mount_rpy_deg": pose.mount_rpy_deg.tolist(),
        })
