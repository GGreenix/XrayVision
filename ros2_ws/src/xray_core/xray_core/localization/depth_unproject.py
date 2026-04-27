"""Depth-to-3D unprojection — shared between stereo and other depth sources.

Given a pixel and a metric depth (Z, in meters along the optical axis), the
3D point in the camera's optical frame is:
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = Z

Then we rotate camera-optical → body → world and translate by the camera
origin. Same adapter pattern as ground_plane.py, just with real depth
instead of a plane intersection — so the per-platform localizer nodes
share most of their code.
"""

from dataclasses import dataclass
from typing import Sequence

from xray_core.localization.camera_model import OPTICAL_TO_BODY, CameraIntrinsics
from xray_core.math_utils import (
    add_vectors,
    matrix_multiply,
    matrix_vector_multiply,
)


@dataclass
class DepthLocalizationResult:
    world_position: list[float]
    range_m: float
    point_camera: list[float]   # in camera optical frame, useful for debug


def localize_pixel_with_depth(
    u_px: float,
    v_px: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    world_from_body: Sequence[Sequence[float]],
    body_from_camera_mount: Sequence[Sequence[float]],
    camera_origin_world: Sequence[float],
) -> DepthLocalizationResult | None:
    if depth_m <= 0.0 or not _finite(depth_m):
        return None

    x_cam = (float(u_px) - intrinsics.cx) * depth_m / intrinsics.fx
    y_cam = (float(v_px) - intrinsics.cy) * depth_m / intrinsics.fy
    z_cam = float(depth_m)
    point_camera = [x_cam, y_cam, z_cam]

    point_body = matrix_vector_multiply(
        matrix_multiply(body_from_camera_mount, OPTICAL_TO_BODY), point_camera
    )
    point_world = add_vectors(
        camera_origin_world,
        matrix_vector_multiply(world_from_body, point_body),
    )
    range_m = (x_cam * x_cam + y_cam * y_cam + z_cam * z_cam) ** 0.5
    return DepthLocalizationResult(
        world_position=point_world, range_m=range_m, point_camera=point_camera
    )


def sample_depth_patch(depth_image, u_px: float, v_px: float, half_window: int = 3) -> float:
    """Robust depth sample: median of a small window around (u, v).

    Stereo depth is noisy and has holes (NaN/0 on textureless regions). A
    median over an N×N window is dramatically more reliable than reading
    a single pixel. Returns 0.0 if no valid samples.
    """
    import numpy as np

    h, w = depth_image.shape[:2]
    u, v = int(round(u_px)), int(round(v_px))
    u0 = max(0, u - half_window)
    u1 = min(w, u + half_window + 1)
    v0 = max(0, v - half_window)
    v1 = min(h, v + half_window + 1)
    patch = depth_image[v0:v1, u0:u1]
    valid = patch[(patch > 0) & np.isfinite(patch)]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))


def _finite(value: float) -> bool:
    return value == value and value not in (float("inf"), float("-inf"))
