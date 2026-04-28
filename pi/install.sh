#!/usr/bin/env bash
# Idempotent installer for the XrayVision Pi station.
# Target OS: Raspberry Pi OS Bookworm (64-bit) on a Pi 5.
# (Ubuntu 22.04 also works.)
#
# What it does:
#   1. Installs python3-picamera2 (apt; needed for the libcamera bindings).
#   2. pip-installs ultralytics + opencv + aiohttp + pyyaml.
#   3. Copies xray_pi/ + config/ to /opt/xray.
#   4. Installs and enables a systemd service that starts the station on boot.
#
# What it does NOT do:
#   - Configure WiFi / network.
#   - The OV9281 USB modules are UVC-class — no dtoverlay needed.

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root (sudo $0)." >&2
    exit 1
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INSTALL_DIR="/opt/xray"
SERVICE_USER="${SUDO_USER:-pi}"

echo "=== Step 1/4: apt deps ==="
apt update
apt install -y \
    python3-pip \
    python3-numpy \
    python3-yaml \
    v4l-utils
# Add the service user to the video group so they can open /dev/video*.
usermod -aG video "$SERVICE_USER" || true

echo "=== Step 2/4: pip deps (as $SERVICE_USER) ==="
sudo -u "$SERVICE_USER" pip3 install --user --break-system-packages \
    "ultralytics>=8.1" \
    "opencv-python-headless>=4.8" \
    "aiohttp>=3.9" \
    "pyyaml>=6.0"

echo "=== Step 3/4: copy xray_pi to $INSTALL_DIR ==="
mkdir -p "$INSTALL_DIR"
rsync -a --delete "$REPO_ROOT/pi/xray_pi" "$INSTALL_DIR/"
mkdir -p "$INSTALL_DIR/config"
# Don't clobber an existing calibration file — only copy if missing.
if [ ! -f "$INSTALL_DIR/config/stereo_calibration.yaml" ]; then
    if [ -f "$REPO_ROOT/pi/config/stereo_calibration.yaml" ]; then
        cp "$REPO_ROOT/pi/config/stereo_calibration.yaml" "$INSTALL_DIR/config/"
    fi
fi
cp "$REPO_ROOT/pi/config/station.yaml" "$INSTALL_DIR/config/"
chown -R "$SERVICE_USER":"$SERVICE_USER" "$INSTALL_DIR"

echo "=== Step 4/4: install and enable systemd service ==="
install -m 644 "$REPO_ROOT/pi/systemd/xray-station.service" \
    /etc/systemd/system/xray-station.service
sed -i "s|__USER__|$SERVICE_USER|g" /etc/systemd/system/xray-station.service
systemctl daemon-reload
systemctl enable xray-station.service

echo ""
echo "=== Install complete ==="
echo "Calibrate stereo:  python3 $REPO_ROOT/pi/tools/stereo_calibrate.py \\"
echo "                       --output $INSTALL_DIR/config/stereo_calibration.yaml"
echo "Start now:         sudo systemctl start xray-station"
echo "Check logs:        journalctl -u xray-station -f"
echo "Verify in browser: http://<pi-ip>:8765/video.mjpg"
