"""Station entry point: camera capture + YOLO + HTTP/WebSocket server.

Run with:
    python -m xray_pi.station --config config/station_pc.yaml --preview
"""

import argparse
import asyncio
import signal
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import yaml

from xray_pi.detector import DepthDetector, DetectorConfig, RoboflowDetector, StaticPose
from xray_pi.mono import MonoPipeline
from xray_pi.server import Server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Path to station.yaml")
    parser.add_argument("--preview", action="store_true", help="Show live camera window")
    return parser.parse_args()


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _draw_detections(img: "cv2.Mat", detections: list[dict]) -> "cv2.Mat":
    h, w = img.shape[:2]
    for det in detections:
        bbox = det.get("bbox", {})
        u  = bbox.get("u_norm", 0.5)
        v  = bbox.get("v_norm", 0.5)
        bw = bbox.get("w_norm", 0.0)
        bh = bbox.get("h_norm", 0.0)
        x1 = int((u - bw / 2) * w)
        y1 = int((v - bh / 2) * h)
        x2 = int((u + bw / 2) * w)
        y2 = int((v + bh / 2) * h)

        label = det.get("class_id", "")
        depth = det.get("depth_m", 0.0)
        color = (0, 255, 0)

        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        text = f"{label}  {depth:.1f}m" if depth > 0 else label
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(img, text, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return img


def _draw_apriltag(img, tag, fx, fy, cx, cy) -> None:
    corners = tag.corners_px.astype(int)
    cv2.polylines(img, [corners], True, (255, 0, 255), 2)
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    try:
        cv2.drawFrameAxes(img, K, np.zeros(4), tag.rvec, tag.tvec, tag.tag_size_m * 0.5, 2)
    except Exception:  # noqa: BLE001
        pass
    c0 = corners[0]
    cv2.putText(img, f"tag{tag.tag_id}  {tag.distance_m:.2f}m",
                (int(c0[0]), int(c0[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)


def _no_camera_frame() -> "cv2.Mat":
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, "CAMERA OFF", (150, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 165, 255), 3, cv2.LINE_AA)
    cv2.putText(img, "released for PhotonVision", (150, 270),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(img, "press C to take it back", (150, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def _preview_thread(pipeline, server, stop_event: threading.Event, filters: dict) -> None:
    """Dedicated thread for all cv2 GUI calls. Shares filters dict with server."""
    win = "XrayVision - Preview  (Q quit, C camera on/off)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def _cb(key):
        return lambda v: filters.__setitem__(key, v)

    cv2.createTrackbar("Min W %", win, filters["min_w"], 100, _cb("min_w"))
    cv2.createTrackbar("Max W %", win, filters["max_w"], 100, _cb("max_w"))
    cv2.createTrackbar("Min H %", win, filters["min_h"], 100, _cb("min_h"))
    cv2.createTrackbar("Max H %", win, filters["max_h"], 100, _cb("max_h"))

    while not stop_event.is_set():
        cam_on = pipeline.is_camera_enabled()
        if cam_on:
            frame = pipeline.get_latest()
            if frame is not None:
                img = frame.left_rect
                if img.ndim == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

                detections = server.get_latest_detections() if server is not None else []
                if detections:
                    img = _draw_detections(img, detections)

                tag = server.get_latest_apriltag() if server is not None else None
                if tag is not None:
                    _draw_apriltag(img, tag, frame.fx, frame.fy, frame.cx, frame.cy)

                cv2.putText(img, "CAM: XrayVision (C to release)", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(win, img)
        else:
            cv2.imshow(win, _no_camera_frame())

        key = cv2.waitKey(33)
        if key in (ord("q"), ord("Q"), 27):
            stop_event.set()
            break
        if key in (ord("c"), ord("C")):
            pipeline.set_camera_enabled(not cam_on)
    cv2.destroyAllWindows()


async def main_async(config: dict, preview: bool) -> None:
    capture = config["capture"]
    mono_cfg = config.get("mono", {})

    pipeline = MonoPipeline(
        device=str(mono_cfg.get("device") or capture.get("device", "0")),
        capture_size=(int(capture["width"]), int(capture["height"])),
        capture_fps=float(capture.get("fps", 30.0)),
        horizontal_fov_deg=float(mono_cfg.get("horizontal_fov_deg", 89.0)),
        calibration_file=str(mono_cfg.get("calibration_file", "")),
    )
    pipeline.start()

    pose_cfg = config["pose"]
    pose = StaticPose(
        position_m=tuple(pose_cfg["position_m"]),
        rpy_deg=tuple(pose_cfg["rpy_deg"]),
        mount_rpy_deg=tuple(pose_cfg.get("mount_rpy_deg", [0.0, 0.0, 0.0])),
    )

    det_cfg = config["detector"]
    det_config = DetectorConfig(
        model_path=str(det_cfg.get("model_path", "yolov8n.pt")),
        confidence_threshold=float(det_cfg.get("confidence_threshold", 0.35)),
        iou_threshold=float(det_cfg.get("iou_threshold", 0.5)),
        imgsz=int(det_cfg.get("imgsz", 640)),
        device=str(det_cfg.get("device", "")),
        allowed_classes=tuple(det_cfg.get("allowed_classes", []) or []),
    )

    rf_cfg = config.get("roboflow")
    if rf_cfg:
        detector = RoboflowDetector(
            api_key=str(rf_cfg["api_key"]),
            model_id=str(rf_cfg["model_id"]),
            api_url=str(rf_cfg.get("api_url", "https://serverless.roboflow.com")),
            config=det_config,
            pose=pose,
        )
    else:
        detector = DepthDetector(config=det_config, pose=pose)

    pv_reader = None
    pv_cfg = config.get("photonvision", {})
    if pv_cfg.get("enabled", False):
        from xray_pi.photonvision_pose import PhotonVisionPoseReader, build_field_layout
        tag_configs = pv_cfg.get("tags", [])
        field_layout = build_field_layout(tag_configs)
        pv_reader = PhotonVisionPoseReader(
            camera_name=str(pv_cfg.get("camera_name", "photonvision")),
            field_layout=field_layout,
            on_pose=detector.update_camera_pose,
            poll_hz=float(pv_cfg.get("poll_hz", 20.0)),
        )
        pv_reader.start()
        print(f"[station] PhotonVision pose reader started: {pv_cfg.get('camera_name')}", flush=True)

    at_cfg = config.get("apriltag", {})
    apriltag_detector = None
    if at_cfg.get("enabled", False):
        from xray_pi.apriltag_pose import AprilTagDetector
        apriltag_detector = AprilTagDetector(
            tag_size_m=float(at_cfg.get("tag_size_m", 0.1)),
            tag_id=int(at_cfg.get("tag_id", 1)),
            tag_world_pos=at_cfg.get("unity_world_pos"),
            tag_world_euler=at_cfg.get("unity_world_euler"),
        )
        print(f"[station] AprilTag enabled: 36h11 id={apriltag_detector.tag_id} "
              f"size={apriltag_detector.tag_size_m}m", flush=True)

    server_cfg = config["server"]
    bbox_filter = {"min_w": 0, "max_w": 37, "min_h": 71, "max_h": 100}
    server = Server(
        stereo=pipeline,
        detector=detector,
        pose=pose,
        host=str(server_cfg.get("host", "0.0.0.0")),
        port=int(server_cfg.get("port", 8765)),
        detection_rate_hz=float(server_cfg.get("detection_rate_hz", 10.0)),
        video_jpeg_quality=int(server_cfg.get("video_jpeg_quality", 75)),
        video_rate_hz=float(server_cfg.get("video_rate_hz", 15.0)),
        bbox_filter=bbox_filter,
        apriltag_detector=apriltag_detector,
        pv_reader=pv_reader,
    )

    stop_event = asyncio.Event()
    preview_stop = threading.Event()

    def _shutdown(*_args) -> None:
        stop_event.set()
        preview_stop.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _shutdown)
        except NotImplementedError:
            pass  # Windows

    if preview:
        t = threading.Thread(
            target=_preview_thread,
            args=(pipeline, server, preview_stop, bbox_filter),
            daemon=True,
            name="preview",
        )
        t.start()

        async def _watch_preview_stop() -> None:
            while not preview_stop.is_set():
                await asyncio.sleep(0.1)
            stop_event.set()

        asyncio.create_task(_watch_preview_stop())

    server_task = asyncio.create_task(server.run())
    await stop_event.wait()
    preview_stop.set()
    server_task.cancel()
    try:
        await server_task
    except asyncio.CancelledError:
        pass
    pipeline.stop()


def main() -> int:
    args = parse_args()
    config_path = Path(args.config)
    if not config_path.is_file():
        print(f"Config not found: {config_path}", file=sys.stderr)
        return 1
    config = load_config(str(config_path))
    asyncio.run(main_async(config, preview=args.preview))
    return 0


if __name__ == "__main__":
    sys.exit(main())
