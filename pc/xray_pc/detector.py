"""YOLO detection + 3D localization on PC."""

from typing import List

import cv2
import numpy as np
from ultralytics import YOLO

from xray_pc.pose import Pose


class ObjectDetector:
    def __init__(self, model_path: str, confidence: float, imgsz: int,
                 device: str, allowed_classes: list, pose: Pose, depth_window: int = 4):
        self.model = YOLO(model_path)
        self.confidence = confidence
        self.imgsz = imgsz
        self.device = device or "cpu"
        self.allowed_classes = set(allowed_classes) if allowed_classes else None
        self.pose = pose
        self.depth_window = depth_window

    def process(self, frame: np.ndarray, depth: np.ndarray,
                fx: float, fy: float, cx: float, cy: float) -> List[dict]:
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        results = self.model.track(
            frame, imgsz=self.imgsz, conf=self.confidence,
            device=self.device, persist=True, verbose=False,
        )
        detections = []
        if not results or results[0].boxes is None:
            return detections

        h, w = frame.shape[:2]
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]
            if self.allowed_classes and class_name not in self.allowed_classes:
                continue

            conf = float(box.conf[0])
            track_id = int(box.id[0]) if box.id is not None else cls_id
            tid = f"{class_name}-{track_id}"

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = (x1 + x2) / 2
            v = (y1 + y2) / 2
            u_px = int(np.clip(u, 0, w - 1))
            v_px = int(np.clip(v, 0, h - 1))

            dw = self.depth_window
            patch = depth[max(0, v_px - dw):v_px + dw + 1, max(0, u_px - dw):u_px + dw + 1]
            valid = patch[patch > 0]
            if len(valid) == 0:
                continue
            depth_m = float(np.median(valid))

            xc = (u_px - cx) * depth_m / fx
            yc = (v_px - cy) * depth_m / fy
            world = self.pose.camera_to_world([xc, yc, depth_m])

            detections.append({
                "tracking_id": tid,
                "class_id": class_name,
                "confidence": round(conf, 3),
                "x": round(world[0], 3),
                "y": round(world[1], 3),
                "z": round(world[2], 3),
                "depth_m": round(depth_m, 3),
                "bbox": {
                    "u_norm": round(u / w, 4),
                    "v_norm": round(v / h, 4),
                    "w_norm": round((x2 - x1) / w, 4),
                    "h_norm": round((y2 - y1) / h, 4),
                },
            })
        return detections
