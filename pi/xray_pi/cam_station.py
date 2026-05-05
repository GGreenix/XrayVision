"""Lightweight Pi entry point — camera server only, no YOLO/SGBM.

Run with:
    python3 -m xray_pi.cam_station --config /opt/xray/config/station.yaml
"""

import argparse
import asyncio
from pathlib import Path

import yaml

from xray_pi.camera_server import CameraServer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    capture = config["capture"]
    server_cfg = config["server"]

    server = CameraServer(
        left_device=str(capture["left_device"]),
        right_device=str(capture["right_device"]),
        width=int(capture["width"]),
        height=int(capture["height"]),
        fps=float(capture.get("fps", 30.0)),
        jpeg_quality=int(server_cfg.get("video_jpeg_quality", 75)),
        host=str(server_cfg.get("host", "0.0.0.0")),
        port=int(server_cfg.get("port", 8765)),
    )

    asyncio.run(server.run())


if __name__ == "__main__":
    main()
