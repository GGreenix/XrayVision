"""YOLO detection + solvePnP depth estimation + 3D localization.

Depth pipeline per detection:
  1. Crop the YOLO bbox from the frame.
  2. Run MediaPipe Pose on the crop to get shoulder/hip landmarks.
  3. Feed 2D landmarks + known 3D body model into cv2.solvePnP.
  4. tvec[2] = depth in metres.
  Fallback: if MediaPipe finds no landmarks, estimate depth from bbox height
  using the known average person height (less accurate but always available).
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

from xray_pi.camera_model import CameraIntrinsics
from xray_pi.depth_unproject import localize_pixel_with_depth
from xray_pi.math_utils import rotation_matrix_from_euler
from xray_pi.stereo import StereoFrame

PERSON_HEIGHT_M = 1.75

# 3D body model in metres, origin = midpoint between hips.
# Points: left shoulder, right shoulder, left hip, right hip.
# MediaPipe landmark indices: 11, 12, 23, 24.
_BODY_3D = np.array([
    [-0.20,  0.55, 0.0],
    [ 0.20,  0.55, 0.0],
    [-0.17,  0.00, 0.0],
    [ 0.17,  0.00, 0.0],
], dtype=np.float32)

_PNP_LM_IDS = [11, 12, 23, 24]
_MIN_VISIBILITY = 0.5
_MIN_CROP_PX = 50  # skip solvePnP on tiny crops


class _PnPEstimator:
    """Single instance shared across all detections in one process call."""

    def __init__(self) -> None:
        import mediapipe as mp
        self._pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    def estimate(
        self,
        bgr: np.ndarray,
        intrinsics: CameraIntrinsics,
        x1: float, y1: float, x2: float, y2: float,
    ) -> float | None:
        """Returns depth in metres, or None if landmarks not found."""
        ih, iw = bgr.shape[:2]
        cx1, cy1 = max(0, int(x1)), max(0, int(y1))
        cx2, cy2 = min(iw, int(x2)), min(ih, int(y2))
        crop = bgr[cy1:cy2, cx1:cx2]
        if crop.shape[0] < _MIN_CROP_PX or crop.shape[1] < _MIN_CROP_PX:
            return None

        result = self._pose.process(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        if not result.pose_landmarks:
            return None

        ch, cw = crop.shape[:2]
        lms = result.pose_landmarks.landmark
        pts_2d = []
        for lm_id in _PNP_LM_IDS:
            lm = lms[lm_id]
            if lm.visibility < _MIN_VISIBILITY:
                return None
            pts_2d.append([cx1 + lm.x * cw, cy1 + lm.y * ch])

        pts_2d = np.array(pts_2d, dtype=np.float32)
        K = np.array([
            [intrinsics.fx, 0,             intrinsics.cx],
            [0,             intrinsics.fy, intrinsics.cy],
            [0,             0,             1            ],
        ], dtype=np.float64)

        # Frames are already undistorted by MonoPipeline so distCoeffs = 0.
        ok, _rvec, tvec = cv2.solvePnP(
            _BODY_3D, pts_2d, K, np.zeros(4),
            flags=cv2.SOLVEPNP_IPPE,
        )
        if not ok or float(tvec[2]) <= 0:
            return None
        return float(tvec[2])


def _depth_fallback(fy: float, y1: float, y2: float) -> float:
    bbox_h = y2 - y1
    return (PERSON_HEIGHT_M * fy) / bbox_h if bbox_h > 0 else 0.0


@dataclass
class DetectorConfig:
    model_path: str = "yolov8n.pt"
    confidence_threshold: float = 0.35
    iou_threshold: float = 0.5
    imgsz: int = 640
    device: str = ""
    allowed_classes: tuple[str, ...] = ()


@dataclass
class StaticPose:
    position_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    mount_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _build_rotations(pose: StaticPose):
    wfb = rotation_matrix_from_euler(
        math.radians(pose.rpy_deg[0]),
        math.radians(pose.rpy_deg[1]),
        math.radians(pose.rpy_deg[2]),
    )
    bfm = rotation_matrix_from_euler(
        math.radians(pose.mount_rpy_deg[0]),
        math.radians(pose.mount_rpy_deg[1]),
        math.radians(pose.mount_rpy_deg[2]),
    )
    return wfb, bfm


class DepthDetector:
    def __init__(self, config: DetectorConfig, pose: StaticPose) -> None:
        from ultralytics import YOLO
        import threading

        self.config = config
        self.model = YOLO(config.model_path)
        if config.device:
            self.model.to(config.device)

        self.world_from_body, self.body_from_camera_mount = _build_rotations(pose)
        self.camera_origin_world = list(pose.position_m)
        self.allowed = set(c for c in config.allowed_classes if c) or None
        self._pose_lock = threading.Lock()

        try:
            self._pnp = _PnPEstimator()
            print("[detector] solvePnP depth estimator ready (MediaPipe)", flush=True)
        except Exception as e:
            self._pnp = None
            print(f"[detector] MediaPipe unavailable, using bbox fallback: {e}", flush=True)

    def update_camera_pose(self, position_m: list[float], world_from_body: np.ndarray) -> None:
        with self._pose_lock:
            self.camera_origin_world = list(position_m)
            self.world_from_body = world_from_body.tolist()

    def process(self, frame: StereoFrame) -> list[dict]:
        with self._pose_lock:
            origin = list(self.camera_origin_world)
            wfb    = [list(r) for r in self.world_from_body]
            bfcm   = [list(r) for r in self.body_from_camera_mount]
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
        boxes = result.boxes
        if boxes is None:
            return []

        intrinsics = CameraIntrinsics(
            width=frame.left_rect.shape[1],
            height=frame.left_rect.shape[0],
            fx=frame.fx, fy=frame.fy, cx=frame.cx, cy=frame.cy,
        )

        xyxy  = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
        confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)
        clss  = boxes.cls.cpu().numpy()  if hasattr(boxes.cls,  "cpu") else np.asarray(boxes.cls)

        out: list[dict] = []
        for index, ((x1, y1, x2, y2), conf, cls_idx) in enumerate(zip(xyxy, confs, clss)):
            class_name = result.names.get(int(cls_idx), str(int(cls_idx)))
            if self.allowed is not None and class_name not in self.allowed:
                continue

            # PnP depth, fallback to bbox-height estimate
            if self._pnp is not None:
                depth_m = self._pnp.estimate(bgr, intrinsics, x1, y1, x2, y2)
            else:
                depth_m = None
            if depth_m is None:
                depth_m = _depth_fallback(frame.fy, y1, y2)

            u_px = (x1 + x2) / 2.0
            v_px = (y1 + y2) / 2.0

            world = localize_pixel_with_depth(
                u_px=u_px, v_px=v_px, depth_m=depth_m, intrinsics=intrinsics,
                world_from_body=wfb,
                body_from_camera_mount=bfcm,
                camera_origin_world=origin,
            )
            if world is None:
                continue

            wx, wy, wz = world.world_position
            out.append({
                "tracking_id": f"{class_name}-{index}",
                "class_id": class_name,
                "confidence": float(conf),
                # Output in Unity world frame: X=right, Y=up, Z=forward
                "x": float(-wy),
                "y": float(wz),
                "z": float(wx),
                "depth_m": float(depth_m),
                "bbox": {
                    "u_norm": float(u_px / intrinsics.width),
                    "v_norm": float(v_px / intrinsics.height),
                    "w_norm": float((x2 - x1) / intrinsics.width),
                    "h_norm": float((y2 - y1) / intrinsics.height),
                },
            })
        return out


class RoboflowDetector:
    """Cloud inference via Roboflow serverless API."""

    def __init__(
        self,
        api_key: str,
        model_id: str,
        config: DetectorConfig,
        pose: StaticPose,
        api_url: str = "https://serverless.roboflow.com",
    ) -> None:
        from inference_sdk import InferenceHTTPClient

        self.client = InferenceHTTPClient(api_url=api_url, api_key=api_key)
        self.model_id = model_id
        self.config = config
        self.world_from_body, self.body_from_camera_mount = _build_rotations(pose)
        self.camera_origin_world = list(pose.position_m)
        self.allowed = set(c for c in config.allowed_classes if c) or None

        try:
            self._pnp = _PnPEstimator()
        except Exception:
            self._pnp = None

    def process(self, frame: StereoFrame) -> list[dict]:
        bgr = cv2.cvtColor(frame.left_rect, cv2.COLOR_GRAY2BGR)
        orig_h, orig_w = bgr.shape[:2]
        scale = self.config.imgsz / max(orig_w, orig_h)
        if scale < 1.0:
            send_w, send_h = int(orig_w * scale), int(orig_h * scale)
            send_img = cv2.resize(bgr, (send_w, send_h), interpolation=cv2.INTER_AREA)
        else:
            send_img, send_w, send_h = bgr, orig_w, orig_h

        result = self.client.infer(send_img, model_id=self.model_id)
        predictions = result.get("predictions", [])
        if not predictions:
            return []

        sx, sy = orig_w / send_w, orig_h / send_h
        intrinsics = CameraIntrinsics(
            width=orig_w, height=orig_h,
            fx=frame.fx, fy=frame.fy, cx=frame.cx, cy=frame.cy,
        )

        out: list[dict] = []
        for index, pred in enumerate(predictions):
            if float(pred.get("confidence", 0)) < self.config.confidence_threshold:
                continue
            class_name = str(pred.get("class", ""))
            if self.allowed is not None and class_name not in self.allowed:
                continue

            cx_px = float(pred["x"]) * sx
            cy_px = float(pred["y"]) * sy
            w_px  = float(pred["width"]) * sx
            h_px  = float(pred["height"]) * sy
            x1, y1, x2, y2 = cx_px - w_px/2, cy_px - h_px/2, cx_px + w_px/2, cy_px + h_px/2

            if self._pnp is not None:
                depth_m = self._pnp.estimate(bgr, intrinsics, x1, y1, x2, y2)
            else:
                depth_m = None
            if depth_m is None:
                depth_m = _depth_fallback(frame.fy, y1, y2)

            world = localize_pixel_with_depth(
                u_px=cx_px, v_px=cy_px, depth_m=depth_m, intrinsics=intrinsics,
                world_from_body=self.world_from_body,
                body_from_camera_mount=self.body_from_camera_mount,
                camera_origin_world=self.camera_origin_world,
            )
            if world is None:
                continue

            wx, wy, wz = world.world_position
            out.append({
                "tracking_id": f"{class_name}-{index}",
                "class_id": class_name,
                "confidence": float(pred["confidence"]),
                "x": float(-wy),
                "y": float(wz),
                "z": float(wx),
                "depth_m": float(depth_m),
                "bbox": {
                    "u_norm": float(cx_px / intrinsics.width),
                    "v_norm": float(cy_px / intrinsics.height),
                    "w_norm": float(w_px / intrinsics.width),
                    "h_norm": float(h_px / intrinsics.height),
                },
            })
        return out
