"""Pi station entry point. Single process: stereo capture + SGBM depth +
YOLO + 3D localization + HTTP/WebSocket server.

Run with:
    python3 -m xray_pi.station --config /opt/xray/config/station.yaml
"""

import argparse
import asyncio
import signal
import sys
from pathlib import Path

import yaml

from xray_pi.detector import DepthDetector, DetectorConfig, StaticPose
from xray_pi.mono import MonoPipeline
from xray_pi.server import Server
from xray_pi.stereo import StereoPipeline
from xray_pi.stereo_calibration import StereoCalibration


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Path to station.yaml")
    return parser.parse_args()


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


async def main_async(config: dict) -> None:
    mode = str(config.get("mode", "stereo")).lower()
    capture = config["capture"]

    if mode == "mono":
        mono_cfg = config.get("mono", {})
        pipeline = MonoPipeline(
            device=str(mono_cfg.get("device") or capture["left_device"]),
            capture_size=(int(capture["width"]), int(capture["height"])),
            capture_fps=float(capture.get("fps", 20.0)),
            horizontal_fov_deg=float(mono_cfg.get("horizontal_fov_deg", 89.0)),
        )
    else:
        calib = StereoCalibration.load(config["calibration_file"])
        sgbm = config["sgbm"]
        pipeline = StereoPipeline(
            calibration=calib,
            capture_size=(int(capture["width"]), int(capture["height"])),
            sgbm_downscale=int(sgbm.get("downscale", 2)),
            left_device=str(capture["left_device"]),
            right_device=str(capture["right_device"]),
            capture_fps=float(capture.get("fps", 20.0)),
            sgbm_params=sgbm,
            min_depth_m=float(sgbm.get("min_depth_m", 0.3)),
            max_depth_m=float(sgbm.get("max_depth_m", 8.0)),
        )
    pipeline.start()

    pose_cfg = config["pose"]
    pose = StaticPose(
        position_m=tuple(pose_cfg["position_m"]),
        rpy_deg=tuple(pose_cfg["rpy_deg"]),
        mount_rpy_deg=tuple(pose_cfg.get("mount_rpy_deg", [0.0, 0.0, 0.0])),
    )

    det_cfg = config["detector"]
    detector = DepthDetector(
        config=DetectorConfig(
            model_path=str(det_cfg.get("model_path", "yolov8n.pt")),
            confidence_threshold=float(det_cfg.get("confidence_threshold", 0.35)),
            iou_threshold=float(det_cfg.get("iou_threshold", 0.5)),
            imgsz=int(det_cfg.get("imgsz", 640)),
            device=str(det_cfg.get("device", "")),
            allowed_classes=tuple(det_cfg.get("allowed_classes", []) or []),
            sample_bbox_bottom=bool(det_cfg.get("sample_bbox_bottom", False)),
            depth_window=int(det_cfg.get("depth_window", 4)),
            ground_plane_fallback=bool(det_cfg.get("ground_plane_fallback", mode == "mono")),
            ground_z_m=float(det_cfg.get("ground_z_m", 0.0)),
            max_range_m=float(det_cfg.get("max_range_m", 50.0)),
        ),
        pose=pose,
    )

    server_cfg = config["server"]
    server = Server(
        stereo=pipeline,
        detector=detector,
        pose=pose,
        host=str(server_cfg.get("host", "0.0.0.0")),
        port=int(server_cfg.get("port", 8765)),
        detection_rate_hz=float(server_cfg.get("detection_rate_hz", 10.0)),
        video_jpeg_quality=int(server_cfg.get("video_jpeg_quality", 75)),
        video_rate_hz=float(server_cfg.get("video_rate_hz", 15.0)),
    )

    stop_event = asyncio.Event()

    def _shutdown(*_args) -> None:
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _shutdown)
        except NotImplementedError:
            pass  # Windows during dev

    server_task = asyncio.create_task(server.run())
    await stop_event.wait()
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
    asyncio.run(main_async(config))
    return 0


if __name__ == "__main__":
    sys.exit(main())
