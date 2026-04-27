"""HTTP + WebSocket server exposing the Pi's perception output.

Endpoints:
    GET  /healthz        -> JSON status
    GET  /video.mjpg     -> multipart MJPEG of the rectified left feed
                            (also viewable in any browser)
    GET  /pose           -> JSON of the configured static camera pose
    WS   /stream         -> JSON detections, one frame per detector cycle
                            { "t": 1700000000.123,
                              "objects": [ {tracking_id, class_id, confidence,
                                            x, y, z, depth_m, bbox}, ... ] }
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
from xray_pi.stereo import StereoPipeline


class Server:
    def __init__(
        self,
        stereo: StereoPipeline,
        detector: DepthDetector,
        pose: StaticPose,
        host: str,
        port: int,
        detection_rate_hz: float,
        video_jpeg_quality: int,
        video_rate_hz: float,
    ) -> None:
        self.stereo = stereo
        self.detector = detector
        self.pose = pose
        self.host = host
        self.port = port
        self.detection_period = 1.0 / detection_rate_hz
        self.video_period = 1.0 / video_rate_hz
        self.video_jpeg_quality = video_jpeg_quality

        self._latest_detections: dict = {"t": 0.0, "objects": []}
        self._detection_subscribers: set[web.WebSocketResponse] = set()
        self._lock = asyncio.Lock()
        self._no_signal_jpeg = _make_no_signal_jpeg(video_jpeg_quality)

        self.app = web.Application()
        self.app.router.add_get("/healthz", self._healthz)
        self.app.router.add_get("/video.mjpg", self._video_mjpg)
        self.app.router.add_get("/pose", self._pose)
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
            if frame is not None:
                # YOLO is heavy — run it off the asyncio thread.
                detections = await loop.run_in_executor(None, self.detector.process, frame)
                payload = {"t": frame.timestamp, "objects": detections}
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

    async def _video_mjpg(self, request: web.Request) -> web.StreamResponse:
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
                is_stale = (
                    frame is None
                    or (now - frame.timestamp) > STALE_FRAME_THRESHOLD_S
                )

                if is_stale:
                    # Re-emit the no-signal frame at ~2 Hz so a viewer that
                    # connected mid-outage sees something instead of stalling.
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
                    None, _encode_jpeg, frame.left_rect, self.video_jpeg_quality
                )
                if jpeg is None:
                    continue
                await response.write(_mjpeg_chunk(boundary, jpeg))
                await asyncio.sleep(self.video_period)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

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
