"""Ground-plane localization: unproject a pixel and intersect with z=0.

Same math works for both the fixed Pi camera and the moving drone camera;
the adapter nodes supply the camera pose in the world frame.
"""

from dataclasses import dataclass
from typing import Sequence

from xray_core.localization.camera_model import OPTICAL_TO_BODY, CameraIntrinsics
from xray_core.math_utils import (
    add_vectors,
    matrix_multiply,
    matrix_vector_multiply,
    scale_vector,
)


@dataclass
class LocalizationResult:
    world_position: list[float]          # [x, y, z] — z==ground_z
    range_m: float                        # distance from camera to hit point
    ray_world: list[float]                # unit ray in world frame (for debug)


def localize_pixel(
    u_px: float,
    v_px: float,
    intrinsics: CameraIntrinsics,
    world_from_body: Sequence[Sequence[float]],
    body_from_camera_mount: Sequence[Sequence[float]],
    camera_origin_world: Sequence[float],
    ground_z: float = 0.0,
) -> LocalizationResult | None:
    """Return the ground-plane intersection for the given image pixel.

    Returns None if the ray points away from the plane (object above horizon
    or camera looking up) — caller should drop such detections.

    world_from_body: rotation matrix, body frame expressed in world frame.
    body_from_camera_mount: mount-only rotation (tilt/roll of camera rig on
        body); the OpenCV-optical→body swap is applied internally.
    """
    ray_optical = intrinsics.pixel_to_ray(u_px, v_px)
    ray_body = matrix_vector_multiply(
        matrix_multiply(body_from_camera_mount, OPTICAL_TO_BODY), ray_optical
    )
    ray_world = matrix_vector_multiply(world_from_body, ray_body)

    # Intersect camera_origin + t * ray_world with plane z = ground_z.
    denom = ray_world[2]
    if abs(denom) < 1e-6:
        return None
    t = (ground_z - camera_origin_world[2]) / denom
    if t <= 0.0:
        return None

    world_position = add_vectors(camera_origin_world, scale_vector(ray_world, t))
    return LocalizationResult(world_position=world_position, range_m=float(t), ray_world=ray_world)


def bbox_bottom_center_px(
    center_x_norm: float,
    center_y_norm: float,
    width_norm: float,
    height_norm: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float]:
    """Bottom-center of a normalized bbox, in pixels.

    For things that sit on the ground (people, vehicles), the bbox bottom is
    the right point to unproject — the top of the head is NOT on the floor.
    """
    u_norm = center_x_norm
    v_norm = center_y_norm + height_norm / 2.0
    return intrinsics.normalized_to_pixel(u_norm, v_norm)
