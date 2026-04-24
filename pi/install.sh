#!/usr/bin/env bash
# Idempotent installer for the XrayVision camera station on a Raspberry Pi 5.
# Target OS: Ubuntu 22.04 LTS Server (arm64). Run with sudo.
#
# What it does:
#   1. Installs ROS 2 Humble (ros-base).
#   2. Installs picamera2 + CycloneDDS RMW.
#   3. Builds the xray_core + xray_bringup packages.
#   4. Installs and enables a systemd service that auto-starts the camera station.
#
# What it does NOT do:
#   - Configure network (WiFi/Ethernet must already be up).
#   - Set up secrets or any firewall rules.

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root (sudo $0)." >&2
    exit 1
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INSTALL_DIR="/opt/xray"
WORKSPACE_DIR="$INSTALL_DIR/ros2_ws"
SERVICE_USER="${SUDO_USER:-ubuntu}"

echo "=== Step 1/5: apt update and base deps ==="
apt update
apt install -y \
    curl gnupg lsb-release software-properties-common \
    locales git

locale-gen en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository universe -y

echo "=== Step 2/5: ROS 2 Humble apt repo ==="
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    > /etc/apt/sources.list.d/ros2.list

apt update

echo "=== Step 3/5: install ROS 2 + picamera2 ==="
apt install -y \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-tf2-ros \
    python3-colcon-common-extensions \
    python3-picamera2 --no-install-recommends

# Perception deps (YOLO + OpenCV). Installed via pip because Ubuntu 22.04
# doesn't package ultralytics, and the apt opencv is too old for some ops.
apt install -y python3-pip python3-numpy
sudo -u "${SUDO_USER:-ubuntu}" pip3 install --user \
    "ultralytics>=8.1" \
    "opencv-python-headless>=4.8"

echo "=== Step 4/5: sync and build the workspace ==="
mkdir -p "$WORKSPACE_DIR/src"
rsync -a --delete \
    "$REPO_ROOT/ros2_ws/src/xray_interfaces" \
    "$REPO_ROOT/ros2_ws/src/xray_core" \
    "$REPO_ROOT/ros2_ws/src/xray_bringup" \
    "$WORKSPACE_DIR/src/"
chown -R "$SERVICE_USER":"$SERVICE_USER" "$INSTALL_DIR"

sudo -u "$SERVICE_USER" bash -lc "
    source /opt/ros/humble/setup.bash
    cd '$WORKSPACE_DIR'
    colcon build --symlink-install --packages-select xray_interfaces xray_core xray_bringup
"

echo "=== Step 5/5: install and enable systemd service ==="
install -m 644 "$REPO_ROOT/pi/systemd/xray-camera.service" \
    /etc/systemd/system/xray-camera.service
sed -i "s|__USER__|$SERVICE_USER|g" /etc/systemd/system/xray-camera.service
systemctl daemon-reload
systemctl enable xray-camera.service

echo ""
echo "=== Install complete ==="
echo "Start now:   sudo systemctl start xray-camera"
echo "Check logs:  journalctl -u xray-camera -f"
echo "Edit pose:   $WORKSPACE_DIR/src/xray_bringup/config/camera_station.yaml"
echo "            (then: cd $WORKSPACE_DIR && colcon build && sudo systemctl restart xray-camera)"
