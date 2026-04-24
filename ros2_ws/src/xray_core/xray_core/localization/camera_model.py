"""Pinhole camera model shared by all localizer adapters.

Intrinsics use the standard OpenCV convention: fx, fy focal lengths in pixels,
(cx, cy) principal point in pixels, image size (width, height) in pixels.
The optical frame is OpenCV's: +X right, +Y down, +Z forward.
"""

from dataclasses import dataclass
from typing import Sequence

from xray_core.math_utils import normalize


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_params(
        cls,
        width: int,
        height: int,
        fx: float,
        fy: float,
        cx: float | None = None,
        cy: float | None = None,
    ) -> "CameraIntrinsics":
        return cls(
            width=int(width),
            height=int(height),
            fx=float(fx),
            fy=float(fy),
            cx=float(cx) if cx is not None else width / 2.0,
            cy=float(cy) if cy is not None else height / 2.0,
        )

    def pixel_to_ray(self, u_px: float, v_px: float) -> list[float]:
        """Unit ray in the optical (OpenCV) frame for pixel (u, v)."""
        x = (float(u_px) - self.cx) / self.fx
        y = (float(v_px) - self.cy) / self.fy
        return normalize([x, y, 1.0])

    def normalized_to_pixel(self, u_norm: float, v_norm: float) -> tuple[float, float]:
        """Detection2D uses 0..1 normalized coords; convert to pixels."""
        return (float(u_norm) * self.width, float(v_norm) * self.height)


# Rotation from OpenCV optical frame (X right, Y down, Z forward) to REP-103
# body frame (X forward, Y left, Z up). Constant for every camera mount.
# v_body = OPTICAL_TO_BODY @ v_optical
OPTICAL_TO_BODY: list[list[float]] = [
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
]


def default_intrinsics_for_fov(width: int, height: int, horizontal_fov_deg: float) -> CameraIntrinsics:
    """Fallback intrinsics when a calibration file isn't available yet.

    Assumes square pixels and principal point at the image center.
    """
    import math

    fx = (width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)
    return CameraIntrinsics(width=width, height=height, fx=fx, fy=fx, cx=width / 2.0, cy=height / 2.0)
