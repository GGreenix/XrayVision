# XrayVision Camera Station (Raspberry Pi 5)

Runs a fixed-pose camera station on a Pi 5, publishing:

- `/xray/camera/image/compressed` (`sensor_msgs/CompressedImage`, JPEG, 30 Hz)
- `/xray/uav/odom` (`nav_msgs/Odometry`, static pose, 2 Hz)
- `/tf_static` for `map` → `base_link` → `camera_link`

All three are the same canonical topics the Gazebo sim publishes, so the ground
station (YOLO + object_localizer + world_model + Unity bridge) needs no changes
to consume the Pi's data instead of sim data.

## Prerequisites

- Raspberry Pi 5 (4 GB or 8 GB).
- Pi Camera Module (v2, v3, or HQ — all work via libcamera/picamera2).
- **Ubuntu 22.04 LTS Server (64-bit)**, not Raspberry Pi OS. Flash with
  Raspberry Pi Imager → "Other general-purpose OS" → "Ubuntu Server 22.04 LTS".
  ROS 2 Humble is only officially packaged for Ubuntu 22.04.
- The Pi on the same LAN as your PC (WiFi or Ethernet).
- The repo cloned to the Pi at any path, e.g. `~/XrayVision`.

## Install

Clone the repo to the Pi and run the installer:

```bash
git clone <repo-url> ~/XrayVision
cd ~/XrayVision/pi
sudo ./install.sh
```

Takes ~15 min on first run (ROS 2 base is a few hundred MB). The installer:

1. Adds the ROS 2 apt repo and installs `ros-humble-ros-base`.
2. Installs `picamera2` and the CycloneDDS RMW.
3. Syncs the `ros2_ws/src/{xray_interfaces,xray_core,xray_bringup}` packages
   into `/opt/xray/ros2_ws` and builds them as your login user.
4. Installs a systemd service that starts the camera station on boot.

## Configure the camera's pose

The Pi is fixed at a known location. Edit
`/opt/xray/ros2_ws/src/xray_bringup/config/camera_station.yaml`:

```yaml
static_pose_publisher:
  ros__parameters:
    position_m: [0.0, 0.0, 1.5]  # [x, y, z] in meters, map frame
    rpy_deg:    [0.0, 0.0, 0.0]  # [roll, pitch, yaw] in degrees
```

Then rebuild and restart:

```bash
cd /opt/xray/ros2_ws
colcon build --packages-select xray_bringup
sudo systemctl restart xray-camera
```

## Start / stop / logs

```bash
sudo systemctl start xray-camera      # start now
sudo systemctl stop xray-camera       # stop
sudo systemctl status xray-camera     # state + recent log lines
journalctl -u xray-camera -f          # live logs
```

The service is enabled by default, so it comes back up on reboot.

## Verify from the Pi

In a shell on the Pi:

```bash
source /opt/ros/humble/setup.bash
source /opt/xray/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic hz /xray/camera/image/compressed   # expect ~30 Hz
ros2 topic hz /xray/uav/odom                  # expect 2 Hz
```

## Verify from the PC

Prerequisites: both machines must have the same `ROS_DOMAIN_ID` (default 42)
and be on the same subnet. The `.env` on the PC already sets this.

From a shell inside the PC's Docker stack:

```powershell
docker compose exec ground bash -lc "source /opt/ros/humble/setup.bash && ros2 topic list | grep xray"
```

You should see the Pi's topics. If not, see Troubleshooting below.

## Connecting the rest of the stack to the Pi

On the PC, stop using the `sim` profile and use the `real` one, with fake
sources disabled (the Pi now provides the real pose; YOLO is phase 2):

```powershell
# .env on the PC:
XRAY_MODE=real
XRAY_USE_FAKE_POSE=false
XRAY_USE_FAKE_DETECTIONS=true   # still true until YOLO is wired up (phase 2)
```

Then:

```powershell
docker compose --profile real up ground unity_bridge video_gateway
```

Unity connects to `localhost:10000` as before — nothing on the Unity side
changes when switching between sim and Pi.

## Troubleshooting

### Pi topics don't appear on the PC

DDS discovery over WiFi is sometimes blocked by access-point AP isolation or
firewalls. Quick checks:

```bash
# On the Pi:
ip addr | grep 'inet '           # confirm Pi's LAN IP
ping <PC-ip>                     # can the Pi see the PC?

# On the PC (WSL):
wsl -- ping <Pi-ip>              # can the PC see the Pi?
```

If ping works but ROS topics still don't cross, your WiFi AP is probably
filtering multicast. Workaround: add an explicit peer list to
`config/cyclonedds.xml` on both ends:

```xml
<Discovery>
  <ParticipantIndex>auto</ParticipantIndex>
  <Peers>
    <Peer address="<Pi-ip>"/>
    <Peer address="<PC-ip>"/>
  </Peers>
</Discovery>
```

### Camera publisher errors: "picamera2 is not installed"

The install script installs it, but on minimal Ubuntu images it may have been
skipped. Run:

```bash
sudo apt install -y python3-picamera2 --no-install-recommends
```

### High JPEG frame times / dropped frames

Lower resolution or JPEG quality in `camera_station.yaml`:

```yaml
camera_publisher:
  ros__parameters:
    width: 640
    height: 480
    jpeg_quality: 60
```

Pi 5 handles 1280x720 @ 30 Hz at q=80 comfortably; older Pi 4s may need a
smaller config.
