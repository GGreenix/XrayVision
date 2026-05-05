"""PC-side entry point.

Pulls left+right MJPEG from Pi -> SGBM depth -> YOLO -> 3D localization
-> WebSocket to Unity. Periodically refines camera pose using ICP against
the room mesh.

Run with:
    python pc/main.py
"""

import asyncio
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from xray_pc.camera_client import MJPEGClient
from xray_pc.detector import ObjectDetector
from xray_pc.pose import Pose
from xray_pc.pose_estimator import MeshLocalizer
from xray_pc.server import PCServer
from xray_pc.stereo import SGBMProcessor, StereoCalib


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


async def detection_loop(left_client, right_client, sgbm, detector,
                         localizer, server, rate_hz, icp_interval_s):
    period = 1.0 / rate_hz
    last_icp = 0.0

    while True:
        start = time.monotonic()

        left, _ = left_client.get_frame()
        right, _ = right_client.get_frame()

        if left is None or right is None:
            await asyncio.sleep(period)
            continue

        loop = asyncio.get_running_loop()

        frame = await loop.run_in_executor(None, sgbm.process, left, right)

        now = time.time()
        if now - last_icp > icp_interval_s:
            valid_depth = int((frame.depth > 0).sum())
            print(f"[icp] firing — localizer.enabled={localizer.enabled}, valid_depth_px={valid_depth}", flush=True)
            await loop.run_in_executor(
                None, localizer.refine,
                frame.depth, frame.fx, frame.fy, frame.cx, frame.cy,
            )
            await server.publish_pose(localizer.pose)
            last_icp = now

        objects = await loop.run_in_executor(
            None, detector.process,
            frame.left_rect, frame.depth,
            frame.fx, frame.fy, frame.cx, frame.cy,
        )

        pos = localizer.pose.position_m
        print(f"[pose] x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}", flush=True)

        depth = frame.depth
        valid = depth > 0
        if valid.any():
            d_norm = np.zeros_like(depth, dtype=np.uint8)
            d_norm[valid] = cv2.normalize(
                depth[valid].reshape(-1, 1), None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U
            ).ravel()
            depth_color = cv2.applyColorMap(d_norm, cv2.COLORMAP_TURBO)
            depth_color[~valid] = 0
            _, jpg = cv2.imencode(".jpg", depth_color, [cv2.IMWRITE_JPEG_QUALITY, 70])
            await server.publish_depth(jpg.tobytes())

        await server.publish({
            "t": time.time(),
            "objects": objects,
            "camera_pos": localizer.pose.position_m.tolist(),
            "camera_rot_q": localizer.pose.rotation_quat_xyzw(),
        })

        elapsed = time.monotonic() - start
        await asyncio.sleep(max(0.0, period - elapsed))


async def main_async(config: dict):
    pi = config["pi"]
    base_url = f"http://{pi['ip']}:{pi['port']}"

    left_client = MJPEGClient(f"{base_url}/video/left.mjpg")
    right_client = MJPEGClient(f"{base_url}/video/right.mjpg")
    left_client.start()
    right_client.start()
    print(f"[main] connecting to Pi at {base_url}", flush=True)

    calib = StereoCalib.load(config["calibration_file"])
    sgbm = SGBMProcessor(calib, config.get("sgbm", {}))

    pose_cfg = config["pose"]
    pose = Pose(
        position_m=pose_cfg["position_m"],
        rpy_deg=pose_cfg["rpy_deg"],
        mount_rpy_deg=pose_cfg.get("mount_rpy_deg", [0.0, 0.0, 0.0]),
    )

    mesh_cfg = config.get("mesh", {})
    localizer = MeshLocalizer(
        mesh_path=str(mesh_cfg.get("path", "")) if mesh_cfg.get("enabled", True) else "",
        initial_pose=pose,
        icp_threshold=float(mesh_cfg.get("icp_threshold", 0.1)),
    )

    det_cfg = config["detector"]
    detector = ObjectDetector(
        model_path=str(det_cfg.get("model_path", "yolov8n.pt")),
        confidence=float(det_cfg.get("confidence_threshold", 0.35)),
        imgsz=int(det_cfg.get("imgsz", 640)),
        device=str(det_cfg.get("device", "")),
        allowed_classes=list(det_cfg.get("allowed_classes", []) or []),
        pose=pose,
        depth_window=int(det_cfg.get("depth_window", 4)),
    )

    srv_cfg = config["server"]
    server = PCServer(
        host=str(srv_cfg.get("host", "0.0.0.0")),
        port=int(srv_cfg.get("port", 8766)),
    )
    server._pi_url = base_url

    await asyncio.gather(
        server.run(),
        detection_loop(
            left_client, right_client, sgbm, detector, localizer, server,
            rate_hz=float(srv_cfg.get("detection_rate_hz", 10.0)),
            icp_interval_s=float(mesh_cfg.get("localization_interval_s", 30.0)),
        ),
    )


def main():
    config_path = Path(__file__).parent / "config" / "pc_config.yaml"
    if not config_path.exists():
        print(f"Config not found: {config_path}", file=sys.stderr)
        sys.exit(1)
    config = load_config(str(config_path))
    asyncio.run(main_async(config))


if __name__ == "__main__":
    main()
