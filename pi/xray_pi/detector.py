"""YOLO + 3D localization, all in one pass.

Pulls the latest stereo frame, runs detection, samples depth at each
detection, unprojects to world coordinates. Result is a list of dicts
ready to be JSON-serialized over the WebSocket.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

from xray_pi.camera_model import CameraIntrinsics
from xray_pi.depth_unproject import localize_pixel_with_depth, sample_depth_patch
from xray_pi.ground_plane import localize_pixel_on_ground
from xray_pi.math_utils import rotation_matrix_from_euler
from xray_pi.stereo import StereoFrame


@dataclass
class DetectorConfig:
    model_path: str = "yolov8n.pt"
    confidence_threshold: float = 0.35
    iou_threshold: float = 0.5
    imgsz: int = 640
    device: str = ""
    allowed_classes: tuple[str, ...] = ()  # empty = all
    sample_bbox_bottom: bool = False
    depth_window: int = 4
    # Mono fallback: when stereo depth is unavailable (zero), project the
    # pixel onto a flat ground plane at world z = ground_z_m. Requires
    # sample_bbox_bottom=True to make any sense (object is touching the ground).
    ground_plane_fallback: bool = False
    ground_z_m: float = 0.0
    max_range_m: float = 50.0


@dataclass
class StaticPose:
    """Camera pose in the world frame, configured once at startup."""

    position_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    mount_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)


class DepthDetector:
    def __init__(self, config: DetectorConfig, pose: StaticPose) -> None:
        from ultralytics import YOLO

        self.config = config
        self.pose = pose
        self.model = YOLO(config.model_path)
        if config.device:
            self.model.to(config.device)

        self.world_from_body = rotation_matrix_from_euler(
            math.radians(pose.rpy_deg[0]),
            math.radians(pose.rpy_deg[1]),
            math.radians(pose.rpy_deg[2]),
        )
        self.body_from_camera_mount = rotation_matrix_from_euler(
            math.radians(pose.mount_rpy_deg[0]),
            math.radians(pose.mount_rpy_deg[1]),
            math.radians(pose.mount_rpy_deg[2]),
        )
        self.camera_origin_world = list(pose.position_m)
        self.allowed = set(c for c in config.allowed_classes if c) or None

    def process(self, frame: StereoFrame) -> list[dict]:
        # YOLO expects BGR; rectified-left is single-channel grayscale.
        bgr = cv2.cvtColor(frame.left_rect, cv2.COLOR_GRAY2BGR)
        results = self.model.predict(
            bgr,
            conf=self.config.confidence_threshold,
            iou=self.config.iou_threshold,
            imgsz=self.config.imgsz,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        names = result.names
        boxes = result.boxes
        if boxes is None:
            return []

        intrinsics = CameraIntrinsics(
            width=frame.left_rect.shape[1],
            height=frame.left_rect.shape[0],
            fx=frame.fx, fy=frame.fy, cx=frame.cx, cy=frame.cy,
        )

        xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
        confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)
        clss = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else np.asarray(boxes.cls)

        out: list[dict] = []
        for index, ((x1, y1, x2, y2), conf, cls_idx) in enumerate(zip(xyxy, confs, clss)):
            class_name = names.get(int(cls_idx), str(int(cls_idx)))
            if self.allowed is not None and class_name not in self.allowed:
                continue

            u_px = (x1 + x2) / 2.0
            v_px = (y1 + y2) / 2.0 if not self.config.sample_bbox_bottom else y2
            depth_m = sample_depth_patch(frame.depth, u_px, v_px, half_window=self.config.depth_window)

            if depth_m > 0.0:
                world = localize_pixel_with_depth(
                    u_px=u_px, v_px=v_px, depth_m=depth_m, intrinsics=intrinsics,
                    world_from_body=self.world_from_body,
                    body_from_camera_mount=self.body_from_camera_mount,
                    camera_origin_world=self.camera_origin_world,
                )
            elif self.config.ground_plane_fallback:
                world = localize_pixel_on_ground(
                    u_px=u_px, v_px=v_px, intrinsics=intrinsics,
                    world_from_body=self.world_from_body,
                    body_from_camera_mount=self.body_from_camera_mount,
                    camera_origin_world=self.camera_origin_world,
                    ground_z_m=self.config.ground_z_m,
                    max_range_m=self.config.max_range_m,
                )
                depth_m = world.range_m if world is not None else 0.0
            else:
                continue

            if world is None:
                continue

            out.append({
                "tracking_id": f"{class_name}-{index}",
                "class_id": class_name,
                "confidence": float(conf),
                "x": float(world.world_position[0]),
                "y": float(world.world_position[1]),
                "z": float(world.world_position[2]),
                "depth_m": float(depth_m),
                "bbox": {
                    "u_norm": float(((x1 + x2) / 2.0) / intrinsics.width),
                    "v_norm": float(((y1 + y2) / 2.0) / intrinsics.height),
                    "w_norm": float((x2 - x1) / intrinsics.width),
                    "h_norm": float((y2 - y1) / intrinsics.height),
                },
            })
        return out
