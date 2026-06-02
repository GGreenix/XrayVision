"""Single-camera capture pipeline. No stereo, no depth.

Produces StereoFrame objects (compatible with the rest of the pipeline)
where `depth` is all zeros — the detector then falls back to ground-plane
projection for localization.

If a calibration_file is provided (mono_calibration.yaml) the real camera
matrix and distortion coefficients are used and frames are undistorted.
Otherwise intrinsics are estimated from horizontal FOV.
"""

import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from xray_pi.stereo import StereoFrame, _open_camera


class MonoPipeline:
    def __init__(
        self,
        device: str,
        capture_size: tuple[int, int],
        capture_fps: float,
        horizontal_fov_deg: float,
        calibration_file: str = "",
    ) -> None:
        self.capture_w, self.capture_h = capture_size
        self._device = device
        self._fps = capture_fps
        self.cap = _open_camera(device, self.capture_w, self.capture_h, capture_fps)
        # When set, the camera is released so another app (e.g. PhotonVision)
        # can open it. Cleared = we own the camera and capture frames.
        self._camera_off = threading.Event()

        self._undistort_map_x: np.ndarray | None = None
        self._undistort_map_y: np.ndarray | None = None

        if calibration_file and Path(calibration_file).is_file():
            with open(calibration_file, "r", encoding="utf-8") as fh:
                cal = yaml.safe_load(fh)
            K = np.array(cal["K"], dtype=np.float64).reshape(3, 3)
            D = np.array(cal["D"], dtype=np.float64)
            self.fx = float(K[0, 0])
            self.fy = float(K[1, 1])
            self.cx = float(K[0, 2])
            self.cy = float(K[1, 2])
            # Precompute undistortion maps — applied once per frame.
            K_new, _ = cv2.getOptimalNewCameraMatrix(
                K, D, (self.capture_w, self.capture_h), alpha=0.0
            )
            self._undistort_map_x, self._undistort_map_y = cv2.initUndistortRectifyMap(
                K, D, None, K_new,
                (self.capture_w, self.capture_h), cv2.CV_32FC1,
            )
            self.fx = float(K_new[0, 0])
            self.fy = float(K_new[1, 1])
            self.cx = float(K_new[0, 2])
            self.cy = float(K_new[1, 2])
            print(f"[mono] loaded calibration from {calibration_file}", flush=True)
        else:
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
            if self.cap is not None:
                self.cap.release()
        except Exception:  # noqa: BLE001
            pass

    def get_latest(self) -> StereoFrame | None:
        with self._lock:
            return self._latest

    def set_camera_enabled(self, enabled: bool) -> None:
        if enabled:
            self._camera_off.clear()
        else:
            self._camera_off.set()

    def is_camera_enabled(self) -> bool:
        return not self._camera_off.is_set()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                # Camera turned off: release it so PhotonVision can grab it.
                if self._camera_off.is_set():
                    if self.cap is not None:
                        try:
                            self.cap.release()
                        except Exception:  # noqa: BLE001
                            pass
                        self.cap = None
                        with self._lock:
                            self._latest = None
                        print("[mono] camera released (off)", flush=True)
                    time.sleep(0.1)
                    continue

                # Camera turned back on: reopen it.
                if self.cap is None:
                    try:
                        self.cap = _open_camera(
                            self._device, self.capture_w, self.capture_h, self._fps
                        )
                        print("[mono] camera reacquired (on)", flush=True)
                    except Exception as exc:  # noqa: BLE001
                        print(f"[mono] reopen failed (is PhotonVision holding it?): {exc}", flush=True)
                        time.sleep(0.5)
                        continue

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
                if self._undistort_map_x is not None:
                    img = cv2.remap(img, self._undistort_map_x, self._undistort_map_y,
                                    cv2.INTER_LINEAR)

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
