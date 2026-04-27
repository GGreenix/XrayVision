"""Simplified pinhole camera model."""

from dataclasses import dataclass


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


# OpenCV optical (X right, Y down, Z forward) -> body (X forward, Y left, Z up).
OPTICAL_TO_BODY = [
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
]
