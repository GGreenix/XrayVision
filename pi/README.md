# XrayVision Pi station (no-ROS edition)

Runs a fixed-pose stereo camera station on a Raspberry Pi 5. Single
Python process: dual capture → SGBM depth → YOLO → 3D world positions
→ HTTP/WebSocket server. The PC's ROS stack consumes the stream via a
small bridge node, so everything downstream (world_model, Unity bridge)
is unchanged.

## Why this isn't ROS

ROS adds ~500 MB of install, locks the Pi to Ubuntu 22.04, and brings
DDS-over-WiFi quirks — none of which the Pi side actually needs. It has
one publisher (perception output) and one consumer (the PC bridge). A
plain HTTP/WebSocket server does the job in ~700 lines and runs on
Raspberry Pi OS Bookworm out of the box.

## Hardware

- Raspberry Pi 5 (4 GB or 8 GB).
- Two **Arducam OV2311 USB** modules (1600×1300 global-shutter mono,
  UVC-class — they appear as standard webcams).
- Both connected to the Pi 5's USB-3 ports (don't share a USB-2 hub —
  bandwidth will choke at full resolution).
- Stereo rig with **8 cm** between optical centers (validated by the
  calibration tool).

No dtoverlay or libcamera config needed — UVC cameras are recognized
out of the box by the kernel's `uvcvideo` driver.

### Identify your cameras

USB enumeration is order-dependent (`/dev/video0` and `/dev/video2` may
swap on reboot), so always reference the cameras by their stable
`/dev/v4l/by-id/` symlinks:

```bash
ls -l /dev/v4l/by-id/
# usb-Arducam_OV2311_USB_Camera_SN12345-video-index0 -> ../../video0
# usb-Arducam_OV2311_USB_Camera_SN67890-video-index0 -> ../../video2
```

Pick which serial number is "left" vs "right" (mark the housings) and
paste the by-id paths into `pi/config/station.yaml`.

### Verify a camera works

```bash
v4l2-ctl --device /dev/v4l/by-id/usb-Arducam_...-video-index0 \
         --list-formats-ext
```

You should see MJPG and YUYV at 1600×1300 among the available modes.

## OS

Raspberry Pi OS Bookworm (64-bit) recommended. Ubuntu 22.04 Server
works too. Nothing in this stack is OS-locked.

## Install

```bash
git clone <repo-url> ~/XrayVision
cd ~/XrayVision/pi
sudo ./install.sh
```

Takes ~5 minutes (mostly the `ultralytics` pip install). The installer:

1. apt-installs `v4l-utils`, numpy, yaml; adds the service user to the
   `video` group.
2. pip-installs `ultralytics`, `opencv-python-headless`, `aiohttp`,
   `pyyaml` for the service user.
3. Copies `xray_pi/` and `config/` to `/opt/xray`.
4. Installs and enables `xray-station.service`.

After install, edit `/opt/xray/config/station.yaml` and replace the
placeholder `left_device` / `right_device` paths with the by-id values
from `ls /dev/v4l/by-id/`.

## Calibrate the stereo pair

The placeholder `stereo_calibration.yaml` lets the pipeline run but
positions will not be metric until you calibrate. Print a 9×6
checkerboard at 25 mm squares, then on the Pi:

```bash
sudo systemctl stop xray-station    # release the cameras

python3 ~/XrayVision/pi/tools/stereo_calibrate.py \
    --left  /dev/v4l/by-id/usb-Arducam_..._SN12345-video-index0 \
    --right /dev/v4l/by-id/usb-Arducam_..._SN67890-video-index0 \
    --output /opt/xray/config/stereo_calibration.yaml

sudo systemctl start xray-station
```

The tool warns if the measured baseline differs from 8 cm by more than
2 cm — fix the mount before trusting depth.

## Configure the camera's pose

Edit `/opt/xray/config/station.yaml`, the `pose:` section:

```yaml
pose:
  position_m: [0.0, 0.0, 1.5]   # left-camera origin in world frame
  rpy_deg:    [0.0, 0.0, 0.0]   # roll/pitch/yaw of the rig
  mount_rpy_deg: [0.0, 0.0, 0.0]  # extra mount tilt (e.g. [-10, 0, 0])
```

Restart the service after edits:

```bash
sudo systemctl restart xray-station
```

## Start / stop / logs

```bash
sudo systemctl start xray-station
sudo systemctl stop xray-station
sudo systemctl status xray-station
journalctl -u xray-station -f
```

## Verify from the Pi

```bash
curl -s http://localhost:8765/healthz
curl -s http://localhost:8765/pose
```

## Verify from the PC

Open the MJPEG in a browser:

```
http://<pi-ip>:8765/video.mjpg
```

Tail detections from the WebSocket (Python one-liner on the PC):

```bash
python3 -c "
from websockets.sync.client import connect
with connect('ws://<pi-ip>:8765/stream') as ws:
    while True:
        print(ws.recv())
"
```

## Wire it into the PC ROS stack

```bash
docker compose exec ground bash -lc "
  source /opt/ros/humble/setup.bash &&
  source /opt/xray/ros2_ws/install/setup.bash &&
  ros2 launch xray_bringup pi_bridge.launch.py
"
```

The `pi_bridge` node:
- Pulls `/video.mjpg` and republishes as `/xray/camera/image/compressed`.
- Connects to `/stream` and republishes as `/xray/perception/objects_raw`
  (`TrackedObjectArray`).
- Fetches `/pose` once and republishes as `/xray/uav/odom`.

Override the Pi address via parameter file or CLI:

```bash
ros2 launch xray_bringup pi_bridge.launch.py \
    --ros-args -p pi_host:=192.168.1.42 -p pi_port:=8765
```

## Troubleshooting

### `Failed to open /dev/video...`

Most often: the service user isn't in the `video` group. The installer
adds them, but you must log out and back in (or reboot) for it to take
effect. Check with `groups` — `video` should be in the list.

### Left/right swapped after reboot

You're using `/dev/video0` / `/dev/video2` instead of the `by-id`
paths. Switch to the symlinks under `/dev/v4l/by-id/` — they're tied to
each camera's USB serial number and never change.

### Both cameras together drop frames at full resolution

Two USB-3 OV2311 streams at 1600×1300 MJPG fit on a single Pi 5 USB
controller, but **only over USB 3**. If you plugged into a USB-2 hub
you'll see dropped grabs. Either move to a powered USB-3 hub or lower
`capture.width`/`height` in `station.yaml`.

### Bridge can't reach the Pi

```bash
curl -v http://<pi-ip>:8765/healthz
```

If that times out, check the Pi's firewall (`sudo ufw status`) and that
both machines are on the same subnet. There's no DDS multicast involved
anymore — TCP works through any normal LAN/WiFi.

### Low depth framerate

`/opt/xray/config/station.yaml` → `sgbm.downscale: 3` (third resolution).
Pi 5 with default `downscale: 2` runs SGBM at ~10–15 Hz; `3` should push
it to ~25 Hz at the cost of depth resolution.

### YOLO is slow

Pi 5 has no GPU; `yolov8n` runs at ~5–10 fps on CPU. Faster options:
drop `imgsz` (e.g., 416), or set `allowed_classes: [person]` to filter
early. For real speed, run YOLO on the **PC** instead — add a
`yolo_detector` ROS node on the ground side that consumes the bridge's
republished image and ignore the Pi's WebSocket detections.
