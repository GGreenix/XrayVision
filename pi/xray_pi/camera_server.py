"""Lightweight dual-camera MJPEG server — no YOLO, no SGBM, no depth.

Endpoints:
    GET /healthz              -> JSON status
    GET /video/left.mjpg      -> raw left camera MJPEG stream
    GET /video/right.mjpg     -> raw right camera MJPEG stream
    GET /                     -> browser dashboard
"""

import asyncio
import threading
import time

import cv2
import numpy as np
from aiohttp import web

STALE_FRAME_THRESHOLD_S = 2.0


class CameraServer:
    def __init__(self, left_device, right_device, width, height, fps, jpeg_quality, host, port):
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.host = host
        self.port = port
        self.video_period = 1.0 / fps

        self._left_frame = None
        self._right_frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ts = 0.0

        self._left_cap = self._open(left_device)
        self._right_cap = self._open(right_device)
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)

        self.app = web.Application()
        self.app.router.add_get("/", self._index)
        self.app.router.add_get("/healthz", self._healthz)
        self.app.router.add_get("/video/left.mjpg", self._stream_left)
        self.app.router.add_get("/video/right.mjpg", self._stream_right)

    def _open(self, device):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera {device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _capture_loop(self):
        while not self._stop.is_set():
            ok_l = self._left_cap.grab()
            ok_r = self._right_cap.grab()
            ts = time.time()
            if not (ok_l and ok_r):
                time.sleep(0.01)
                continue
            ok_l, left = self._left_cap.retrieve()
            ok_r, right = self._right_cap.retrieve()
            if not (ok_l and ok_r) or left is None or right is None:
                continue
            with self._lock:
                self._left_frame = left
                self._right_frame = right
                self._ts = ts

    async def run(self):
        self._thread.start()
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        print(f"[camera_server] listening on http://{self.host}:{self.port}", flush=True)
        try:
            while True:
                await asyncio.sleep(1)
        finally:
            self._stop.set()
            self._left_cap.release()
            self._right_cap.release()
            await runner.cleanup()

    async def _stream(self, request, side):
        boundary = "frame"
        response = web.StreamResponse(
            status=200,
            reason="OK",
            headers={
                "Content-Type": f"multipart/x-mixed-replace; boundary={boundary}",
                "Cache-Control": "no-cache, no-store, private",
                "Connection": "close",
            },
        )
        await response.prepare(request)
        loop = asyncio.get_running_loop()
        no_signal = self._make_no_signal()
        last_ts = 0.0

        try:
            while True:
                with self._lock:
                    frame = self._left_frame if side == "left" else self._right_frame
                    ts = self._ts

                now = time.time()
                if frame is None or (now - ts) > STALE_FRAME_THRESHOLD_S:
                    await response.write(_mjpeg_chunk(boundary, no_signal))
                    await asyncio.sleep(self.video_period)
                    continue

                if ts == last_ts:
                    await asyncio.sleep(self.video_period / 2)
                    continue
                last_ts = ts

                jpeg = await loop.run_in_executor(None, self._encode, frame)
                if jpeg:
                    await response.write(_mjpeg_chunk(boundary, jpeg))
                await asyncio.sleep(self.video_period)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _stream_left(self, request):
        return await self._stream(request, "left")

    async def _stream_right(self, request):
        return await self._stream(request, "right")

    def _encode(self, frame):
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return buf.tobytes() if ok else None

    def _make_no_signal(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "NO SIGNAL", (130, 240), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return buf.tobytes() if ok else b""

    async def _healthz(self, request):
        with self._lock:
            ts = self._ts
        age = time.time() - ts if ts > 0 else None
        return web.json_response({"ok": True, "live": age is not None and age < STALE_FRAME_THRESHOLD_S, "age_s": age})

    async def _index(self, request):
        html = """<!doctype html>
<html><head><title>XrayVision Camera Server</title>
<style>body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px}
h1{font-size:16px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
img{width:100%;background:#000;border:1px solid #333}</style></head>
<body><h1>XrayVision Camera Server</h1>
<div class="grid">
  <div><div>left</div><img src="/video/left.mjpg"></div>
  <div><div>right</div><img src="/video/right.mjpg"></div>
</div>
<p><a href="/healthz" style="color:#8af">/healthz</a></p>
</body></html>"""
        return web.Response(text=html, content_type="text/html")


def _mjpeg_chunk(boundary, jpeg):
    return (
        f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n"
    ).encode("ascii") + jpeg + b"\r\n"
