"""Stereo capture (dual USB Arducam via V4L2) + SGBM depth.

USB cameras enumerate as /dev/videoN with non-deterministic order across
reboots, so we strongly prefer stable paths under /dev/v4l/by-id/. Both
left_device and right_device should be those by-id paths.

Capture priority:  MJPG  ->  YUYV  ->  whatever the camera negotiates.
MJPG keeps USB bandwidth low enough for two 1600x1300 streams at 20+ Hz
on a single Pi 5 USB3 controller.
"""

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from xray_pi.stereo_calibration import StereoCalibration, StereoRectification


@dataclass
class StereoFrame:
    timestamp: float
    left_rect: np.ndarray         # uint8 grayscale, rectified
    depth: np.ndarray             # float32 meters, 0 = invalid
    fx: float
    fy: float
    cx: float
    cy: float


def _open_camera(device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {device}")

    # Try MJPG first (compressed on the camera, low USB bandwidth). If the
    # camera doesn't advertise MJPG at this resolution it'll silently keep
    # the default (usually YUYV) — that still works, just heavier.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    # Keep the buffer shallow so read() returns a fresh frame, not a stale one.
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:  # noqa: BLE001
        pass

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (actual_w, actual_h) != (width, height):
        print(
            f"[stereo] {device}: requested {width}x{height}, got {actual_w}x{actual_h}",
            flush=True,
        )
    return cap


class StereoPipeline:
    def __init__(
        self,
        calibration: StereoCalibration,
        capture_size: tuple[int, int],
        sgbm_downscale: int,
        left_device: str,
        right_device: str,
        capture_fps: float,
        sgbm_params: dict,
        min_depth_m: float,
        max_depth_m: float,
    ) -> None:
        self.capture_w, self.capture_h = capture_size
        self.downscale = max(1, sgbm_downscale)
        self.min_depth = min_depth_m
        self.max_depth = max_depth_m

        self.left_cap = _open_camera(left_device, self.capture_w, self.capture_h, capture_fps)
        self.right_cap = _open_camera(right_device, self.capture_w, self.capture_h, capture_fps)

        self.rectification = StereoRectification.from_calibration(calibration, downscale=self.downscale)

        self.matcher = cv2.StereoSGBM_create(
            minDisparity=int(sgbm_params.get("min_disparity", 0)),
            numDisparities=int(sgbm_params.get("num_disparities", 96)),
            blockSize=int(sgbm_params.get("block_size", 7)),
            P1=8 * int(sgbm_params.get("block_size", 7)) ** 2,
            P2=32 * int(sgbm_params.get("block_size", 7)) ** 2,
            uniquenessRatio=int(sgbm_params.get("uniqueness_ratio", 10)),
            speckleWindowSize=int(sgbm_params.get("speckle_window_size", 100)),
            speckleRange=int(sgbm_params.get("speckle_range", 2)),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self._latest: StereoFrame | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="stereo")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        for cap in (self.left_cap, self.right_cap):
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass

    def get_latest(self) -> StereoFrame | None:
        with self._lock:
            return self._latest

    def _run(self) -> None:
        rect = self.rectification
        target_size = rect.size
        while not self._stop.is_set():
            try:
                # cv2.VideoCapture.grab() pulls without decoding — issuing
                # both grabs back-to-back gets the frames as close in time
                # as USB will allow. retrieve() then decodes each.
                ok_l = self.left_cap.grab()
                ok_r = self.right_cap.grab()
                ts = time.time()
                if not (ok_l and ok_r):
                    time.sleep(0.01)
                    continue
                ok_l, left = self.left_cap.retrieve()
                ok_r, right = self.right_cap.retrieve()
                if not (ok_l and ok_r) or left is None or right is None:
                    continue

                # OV2311 USB modules typically expose YUYV/MJPG and decode
                # to BGR — convert to grayscale for stereo matching.
                if left.ndim == 3:
                    left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                if right.ndim == 3:
                    right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

                # Crop to the calibrated capture size if the camera gave us
                # something slightly different (some cameras refuse exact sizes).
                if left.shape[:2] != (self.capture_h, self.capture_w):
                    left = left[: self.capture_h, : self.capture_w]
                if right.shape[:2] != (self.capture_h, self.capture_w):
                    right = right[: self.capture_h, : self.capture_w]

                if self.downscale != 1:
                    left = cv2.resize(left, target_size, interpolation=cv2.INTER_AREA)
                    right = cv2.resize(right, target_size, interpolation=cv2.INTER_AREA)

                left_r = cv2.remap(left, rect.map_left_x, rect.map_left_y, cv2.INTER_LINEAR)
                right_r = cv2.remap(right, rect.map_right_x, rect.map_right_y, cv2.INTER_LINEAR)

                disparity_raw = self.matcher.compute(left_r, right_r)
                disparity = disparity_raw.astype(np.float32) / 16.0

                with np.errstate(divide="ignore", invalid="ignore"):
                    depth = np.where(
                        disparity > 0,
                        (rect.fx * rect.baseline_m) / disparity,
                        np.float32(0.0),
                    ).astype(np.float32)
                depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0.0

                frame = StereoFrame(
                    timestamp=ts,
                    left_rect=left_r,
                    depth=depth,
                    fx=rect.fx, fy=rect.fy, cx=rect.cx, cy=rect.cy,
                )
                with self._lock:
                    self._latest = frame
            except Exception as exc:  # noqa: BLE001
                print(f"[stereo] iteration failed: {exc}", flush=True)
                time.sleep(0.05)
