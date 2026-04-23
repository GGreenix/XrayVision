#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /opt/xray/overlay_ws/install/setup.bash ]; then
  source /opt/xray/overlay_ws/install/setup.bash
fi

if [ -f /opt/xray/ws/install/setup.bash ]; then
  source /opt/xray/ws/install/setup.bash
fi

exec "$@"

