"""Pixel + depth -> 3D world. Vendored from xray_core/localization/depth_unproject.py."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from xray_pi.camera_model import OPTICAL_TO_BODY, CameraIntrinsics
from xray_pi.math_utils import add_vectors, matrix_multiply, matrix_vector_multiply


@dataclass
class DepthLocalizationResult:
    world_position: list[float]
    range_m: float


def localize_pixel_with_depth(
    u_px: float,
    v_px: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    world_from_body: Sequence[Sequence[float]],
    body_from_camera_mount: Sequence[Sequence[float]],
    camera_origin_world: Sequence[float],
) -> DepthLocalizationResult | None:
    if depth_m <= 0.0 or not np.isfinite(depth_m):
        return None

    x_cam = (float(u_px) - intrinsics.cx) * depth_m / intrinsics.fx
    y_cam = (float(v_px) - intrinsics.cy) * depth_m / intrinsics.fy
    z_cam = float(depth_m)

    point_body = matrix_vector_multiply(
        matrix_multiply(body_from_camera_mount, OPTICAL_TO_BODY),
        [x_cam, y_cam, z_cam],
    )
    point_world = add_vectors(
        camera_origin_world,
        matrix_vector_multiply(world_from_body, point_body),
    )
    range_m = float(np.sqrt(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam))
    return DepthLocalizationResult(world_position=point_world, range_m=range_m)


def sample_depth_patch(depth_image: np.ndarray, u_px: float, v_px: float, half_window: int = 4) -> float:
    h, w = depth_image.shape[:2]
    u, v = int(round(u_px)), int(round(v_px))
    u0, u1 = max(0, u - half_window), min(w, u + half_window + 1)
    v0, v1 = max(0, v - half_window), min(h, v + half_window + 1)
    patch = depth_image[v0:v1, u0:u1]
    valid = patch[(patch > 0) & np.isfinite(patch)]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))
