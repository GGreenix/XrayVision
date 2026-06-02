"""HTTP + WebSocket server exposing the Pi's perception output.

Endpoints:
    GET  /healthz        -> JSON status
    GET  /video.mjpg     -> multipart MJPEG of the rectified left feed
                            (also viewable in any browser)
    GET  /pose           -> JSON of the configured static camera pose
    GET  /apriltag       -> JSON of the latest AprilTag pose (or null)
    WS   /stream         -> JSON detections, one frame per detector cycle
                            { "t": 1700000000.123,
                              "objects": [ {tracking_id, class_id, confidence,
                                            x, y, z, depth_m, bbox}, ... ],
                              "apriltag": {id, x, y, z, distance_m, rvec} | null }
"""

import asyncio
import json
import time

import cv2
import numpy as np
from aiohttp import WSMsgType, web

# A frame older than this is treated as "no signal" — covers the case where
# capture stalls (USB unplug, driver hang) while the last frame lingers.
STALE_FRAME_THRESHOLD_S = 2.0

from xray_pi.detector import DepthDetector, StaticPose
from xray_pi.mono import MonoPipeline


class Server:
    def __init__(
        self,
        stereo: MonoPipeline,
        detector: DepthDetector,
        pose: StaticPose,
        host: str,
        port: int,
        detection_rate_hz: float,
        video_jpeg_quality: int,
        video_rate_hz: float,
        bbox_filter: dict | None = None,
        apriltag_detector=None,
        pv_reader=None,
    ) -> None:
        self.stereo = stereo
        self.detector = detector
        self.pose = pose
        self.apriltag_detector = apriltag_detector
        self.pv_reader = pv_reader
        self.host = host
        self.port = port
        self.detection_period = 1.0 / detection_rate_hz
        self.video_period = 1.0 / video_rate_hz
        self.video_jpeg_quality = video_jpeg_quality

        self._bbox_filter = bbox_filter or {"min_w": 0, "max_w": 100, "min_h": 0, "max_h": 100}
        self._latest_detections: dict = {"t": 0.0, "objects": []}
        self._latest_apriltag = None  # AprilTagPose | None
        self._detection_subscribers: set[web.WebSocketResponse] = set()
        self._lock = asyncio.Lock()
        self._no_signal_jpeg = _make_no_signal_jpeg(video_jpeg_quality)

        self.app = web.Application()
        self.app.router.add_get("/", self._index)
        self.app.router.add_get("/healthz", self._healthz)
        self.app.router.add_get("/video.mjpg", self._video_mjpg)
        self.app.router.add_get("/video/left.mjpg", self._video_left_raw)
        self.app.router.add_get("/video/right.mjpg", self._video_right_raw)
        self.app.router.add_get("/video/left_rect.mjpg", self._video_left_rect)
        self.app.router.add_get("/video/right_rect.mjpg", self._video_right_rect)
        self.app.router.add_get("/pose", self._pose)
        self.app.router.add_get("/apriltag", self._apriltag)
        self.app.router.add_get("/stream", self._stream_ws)

    async def run(self) -> None:
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        print(f"[server] listening on http://{self.host}:{self.port}", flush=True)

        try:
            await self._detection_loop()
        finally:
            await runner.cleanup()

    async def _detection_loop(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            start = time.monotonic()
            frame = self.stereo.get_latest()

            detections: list[dict] = []
            tag = None
            if frame is not None:
                # YOLO is heavy — run it off the asyncio thread.
                detections = await loop.run_in_executor(None, self.detector.process, frame)
                f = self._bbox_filter
                min_w, max_w = f["min_w"] / 100.0, f["max_w"] / 100.0
                min_h, max_h = f["min_h"] / 100.0, f["max_h"] / 100.0
                detections = [
                    d for d in detections
                    if min_w <= d["bbox"]["w_norm"] <= max_w
                    and min_h <= d["bbox"]["h_norm"] <= max_h
                ]

                if self.apriltag_detector is not None:
                    tag = await loop.run_in_executor(
                        None, self.apriltag_detector.detect, frame
                    )
                    self._latest_apriltag = tag
                    if tag is not None and self.apriltag_detector.tag_world_pos is not None:
                        pos, wfb = tag.camera_world_pose_server(
                            self.apriltag_detector.tag_world_pos,
                            self.apriltag_detector.tag_world_euler,
                        )
                        self.detector.update_camera_pose(pos, wfb)

            # Always broadcast — even with no XrayVision frame — so the last-known
            # PhotonVision camera pose keeps flowing to Unity while the camera is
            # handed to PhotonVision.
            payload = {
                "t": frame.timestamp if frame is not None else time.time(),
                "objects": detections,
                "apriltag": tag.to_dict() if tag is not None else None,
                "camera": self.pv_reader.last_unity_pose if self.pv_reader is not None else None,
            }
            async with self._lock:
                self._latest_detections = payload
            await self._broadcast(payload)

            elapsed = time.monotonic() - start
            await asyncio.sleep(max(0.0, self.detection_period - elapsed))

    async def _broadcast(self, payload: dict) -> None:
        if not self._detection_subscribers:
            return
        text = json.dumps(payload)
        dead: list[web.WebSocketResponse] = []
        for ws in list(self._detection_subscribers):
            try:
                await ws.send_str(text)
            except Exception:  # noqa: BLE001
                dead.append(ws)
        for ws in dead:
            self._detection_subscribers.discard(ws)

    def get_latest_detections(self) -> list[dict]:
        return self._latest_detections.get("objects", [])

    def get_latest_apriltag(self):
        return self._latest_apriltag

    async def _healthz(self, request: web.Request) -> web.Response:
        frame = self.stereo.get_latest()
        age = None if frame is None else time.time() - frame.timestamp
        live = frame is not None and age is not None and age <= STALE_FRAME_THRESHOLD_S
        return web.json_response({
            "ok": True,
            "have_frame": frame is not None,
            "live": live,
            "last_frame_age_s": age,
            "subscribers": len(self._detection_subscribers),
        })

    async def _pose(self, request: web.Request) -> web.Response:
        return web.json_response({
            "position_m": list(self.pose.position_m),
            "rpy_deg": list(self.pose.rpy_deg),
            "mount_rpy_deg": list(self.pose.mount_rpy_deg),
        })

    async def _apriltag(self, request: web.Request) -> web.Response:
        tag = self._latest_apriltag
        return web.json_response(tag.to_dict() if tag is not None else None)

    async def _video_mjpg(self, request: web.Request) -> web.StreamResponse:
        return await self._stream_view(request, lambda f: f.left_rect)

    async def _video_left_raw(self, request: web.Request) -> web.StreamResponse:
        return await self._stream_view(request, lambda f: f.left_raw)

    async def _video_right_raw(self, request: web.Request) -> web.StreamResponse:
        return await self._stream_view(request, lambda f: f.right_raw)

    async def _video_left_rect(self, request: web.Request) -> web.StreamResponse:
        return await self._stream_view(request, lambda f: f.left_rect)

    async def _video_right_rect(self, request: web.Request) -> web.StreamResponse:
        return await self._stream_view(request, lambda f: f.right_rect)

    async def _stream_view(self, request: web.Request, pick) -> web.StreamResponse:
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
        last_ts = 0.0
        sent_no_signal = False
        try:
            while True:
                frame = self.stereo.get_latest()
                now = time.time()
                image = pick(frame) if frame is not None else None
                is_stale = (
                    frame is None
                    or image is None
                    or (now - frame.timestamp) > STALE_FRAME_THRESHOLD_S
                )

                if is_stale:
                    if not sent_no_signal or (now - last_ts) > 0.5:
                        await response.write(_mjpeg_chunk(boundary, self._no_signal_jpeg))
                        sent_no_signal = True
                        last_ts = now
                    await asyncio.sleep(self.video_period)
                    continue

                if frame.timestamp == last_ts:
                    await asyncio.sleep(self.video_period / 2)
                    continue
                last_ts = frame.timestamp
                sent_no_signal = False

                jpeg = await loop.run_in_executor(
                    None, _encode_jpeg, image, self.video_jpeg_quality
                )
                if jpeg is None:
                    continue
                await response.write(_mjpeg_chunk(boundary, jpeg))
                await asyncio.sleep(self.video_period)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _index(self, request: web.Request) -> web.Response:
        html = """<!doctype html>
<html><head><title>XrayVision Pi</title>
<style>body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px}
h1{font-size:16px;margin:0 0 8px}h2{font-size:13px;color:#aaa;margin:8px 0 4px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
img{width:100%;background:#000;border:1px solid #333}</style></head>
<body><h1>XrayVision Pi station</h1>
<h2>Raw cameras</h2>
<div class="grid">
  <div><div>left_raw</div><img src="/video/left.mjpg"></div>
  <div><div>right_raw</div><img src="/video/right.mjpg"></div>
</div>
<h2>Rectified (post-stereo-rectify)</h2>
<div class="grid">
  <div><div>left_rect (default /video.mjpg)</div><img src="/video/left_rect.mjpg"></div>
  <div><div>right_rect</div><img src="/video/right_rect.mjpg"></div>
</div>
<p><a href="/healthz" style="color:#8af">/healthz</a> &nbsp;
   <a href="/pose" style="color:#8af">/pose</a></p>
</body></html>"""
        return web.Response(text=html, content_type="text/html")

    async def _stream_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)
        self._detection_subscribers.add(ws)

        # Send the latest frame immediately so a fresh client doesn't wait.
        async with self._lock:
            initial = json.dumps(self._latest_detections)
        await ws.send_str(initial)

        try:
            async for msg in ws:
                if msg.type == WSMsgType.ERROR:
                    break
        finally:
            self._detection_subscribers.discard(ws)
        return ws


def _encode_jpeg(image, quality: int) -> bytes | None:
    ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return buf.tobytes()


def _mjpeg_chunk(boundary: str, jpeg: bytes) -> bytes:
    return (
        f"--{boundary}\r\n"
        f"Content-Type: image/jpeg\r\n"
        f"Content-Length: {len(jpeg)}\r\n\r\n"
    ).encode("ascii") + jpeg + b"\r\n"


def _make_no_signal_jpeg(quality: int) -> bytes:
    """Pre-render the placeholder once so we don't re-encode each tick."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, "NO SIGNAL", (130, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4, cv2.LINE_AA)
    cv2.putText(img, "camera not detected", (170, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
    return _encode_jpeg(img, quality) or b""
