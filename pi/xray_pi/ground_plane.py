"""Mono-camera fallback: project a pixel onto a flat ground plane.

Used when no stereo depth is available. Assumes the world has a flat
ground at z = ground_z_m and the camera pose is known. Ray-cast the
pixel through the camera, transform into world coords, intersect with
the ground plane.

This will be wrong if the ground isn't flat or the object isn't standing
on it (e.g. flying drones), so prefer stereo depth whenever possible.
"""

from typing import Sequence

from xray_pi.camera_model import OPTICAL_TO_BODY, CameraIntrinsics
from xray_pi.depth_unproject import DepthLocalizationResult
from xray_pi.math_utils import add_vectors, matrix_multiply, matrix_vector_multiply


def localize_pixel_on_ground(
    u_px: float,
    v_px: float,
    intrinsics: CameraIntrinsics,
    world_from_body: Sequence[Sequence[float]],
    body_from_camera_mount: Sequence[Sequence[float]],
    camera_origin_world: Sequence[float],
    ground_z_m: float = 0.0,
    max_range_m: float = 50.0,
) -> DepthLocalizationResult | None:
    # Ray in optical frame: direction proportional to ((u-cx)/fx, (v-cy)/fy, 1)
    dir_optical = [
        (float(u_px) - intrinsics.cx) / intrinsics.fx,
        (float(v_px) - intrinsics.cy) / intrinsics.fy,
        1.0,
    ]
    body_from_optical = matrix_multiply(body_from_camera_mount, OPTICAL_TO_BODY)
    dir_body = matrix_vector_multiply(body_from_optical, dir_optical)
    dir_world = matrix_vector_multiply(world_from_body, dir_body)

    # Intersect with z = ground_z_m. Origin + t*dir, solve for t at target z.
    dz = dir_world[2]
    if abs(dz) < 1e-6:
        return None  # Ray parallel to ground — no intersection.
    t = (ground_z_m - camera_origin_world[2]) / dz
    if t <= 0.0 or t > max_range_m:
        return None  # Behind camera or absurdly far.

    point_world = add_vectors(
        camera_origin_world,
        [dir_world[0] * t, dir_world[1] * t, dir_world[2] * t],
    )
    range_m = float(t * (dir_world[0] ** 2 + dir_world[1] ** 2 + dir_world[2] ** 2) ** 0.5)
    return DepthLocalizationResult(world_position=point_world, range_m=range_m)
