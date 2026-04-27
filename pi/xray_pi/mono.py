"""Single-camera capture pipeline. No stereo, no depth.

Produces StereoFrame objects (compatible with the rest of the pipeline)
where `depth` is all zeros — the detector then falls back to ground-plane
projection for localization.

Intrinsics are estimated from a configured horizontal FOV since there is
no calibration in mono mode. This is good enough for "where on the ground
is that person" with a fixed camera, but absolute positions will be off
by the FOV error.
"""

import math
import threading
import time

import cv2
import numpy as np

from xray_pi.stereo import StereoFrame, _open_camera


class MonoPipeline:
    def __init__(
        self,
        device: str,
        capture_size: tuple[int, int],
        capture_fps: float,
        horizontal_fov_deg: float,
    ) -> None:
        self.capture_w, self.capture_h = capture_size
        self.cap = _open_camera(device, self.capture_w, self.capture_h, capture_fps)

        # Intrinsics from FOV: fx = (W/2) / tan(HFOV/2). Square pixels.
        hfov_rad = math.radians(horizontal_fov_deg)
        self.fx = (self.capture_w / 2.0) / math.tan(hfov_rad / 2.0)
        self.fy = self.fx
        self.cx = self.capture_w / 2.0
        self.cy = self.capture_h / 2.0
        self._zero_depth = np.zeros((self.capture_h, self.capture_w), dtype=np.float32)

        self._latest: StereoFrame | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="mono")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self.cap.release()
        except Exception:  # noqa: BLE001
            pass

    def get_latest(self) -> StereoFrame | None:
        with self._lock:
            return self._latest

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                ok = self.cap.grab()
                ts = time.time()
                if not ok:
                    time.sleep(0.01)
                    continue
                ok, img = self.cap.retrieve()
                if not ok or img is None:
                    continue
                if img.ndim == 3:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if img.shape[:2] != (self.capture_h, self.capture_w):
                    img = cv2.resize(img, (self.capture_w, self.capture_h),
                                     interpolation=cv2.INTER_AREA)

                frame = StereoFrame(
                    timestamp=ts,
                    left_rect=img,
                    depth=self._zero_depth,
                    fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                    left_raw=img,
                )
                with self._lock:
                    self._latest = frame
            except Exception as exc:  # noqa: BLE001
                print(f"[mono] iteration failed: {exc}", flush=True)
                time.sleep(0.05)
