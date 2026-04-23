#!/usr/bin/env bash
# Run the Gazebo sim backend with a 3D GUI window via WSLg.
# Must be executed from inside a WSL2 shell (Windows 11) with Docker Desktop's
# WSL integration enabled, or with docker-ce installed directly in WSL.

set -euo pipefail

if [ ! -S /tmp/.X11-unix/X0 ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "error: no X11 socket and no WAYLAND_DISPLAY — are you inside WSLg?" >&2
  echo "       try 'xeyes' from this shell first to verify WSLg works." >&2
  exit 1
fi

export XRAY_MODE=sim
export XRAY_SIM_BACKEND=gazebo
export XRAY_USE_FAKE_POSE=false
export XRAY_USE_FAKE_DETECTIONS=true
export XRAY_USE_SIM_TIME=true
export XRAY_GAZEBO_HEADLESS=false
export DISPLAY="${DISPLAY:-:0}"

cd "$(dirname "$0")/.."

exec docker compose \
  -f docker-compose.yml \
  -f docker-compose.gui.yml \
  --profile sim up --build sim_backend ground unity_bridge
