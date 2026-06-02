"""Camera frame dataclass and cross-platform camera opener."""

import sys
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class StereoFrame:
    timestamp: float
    left_rect: np.ndarray   # uint8 grayscale
    depth: np.ndarray       # float32 meters, always zero in mono mode
    fx: float
    fy: float
    cx: float
    cy: float
    left_raw: np.ndarray | None = None


def _open_camera(device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
    if sys.platform == "win32":
        cap = cv2.VideoCapture(int(device), cv2.CAP_MSMF)
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {device}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:  # noqa: BLE001
        pass

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (actual_w, actual_h) != (width, height):
        print(f"[camera] {device}: requested {width}x{height}, got {actual_w}x{actual_h}", flush=True)
    return cap
