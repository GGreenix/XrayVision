# XrayVision

`XrayVision` is a starter codebase for a ROS 2 + Unity drone visualization stack. It gives you a canonical topic contract, Dockerized runtime, a mock simulation path, persistent 3D object tracking, and a Unity bridge surface that stays stable across `sim` and `real` modes.

The first cut is intentionally opinionated:

- ROS 2 `Humble` on Linux containers
- Unity structured-data bridge via `ROS-TCP-Endpoint`
- Low-latency video handled outside ROS via `MediaMTX`
- Canonical ROS topics under the `/xray/...` namespace
- A lightweight mock simulator that exercises the same interfaces as the real stack
- A Gazebo-backed headless simulator for world and odometry bring-up

## Quick Start

### 1. Prepare the environment

Copy `.env.example` to `.env` and adjust values if needed.

### 2. Run the mock simulation stack

```powershell
./scripts/run-sim.ps1
```

This starts:

- `sim_backend`: fake pose + fake detections + 3D localization
- `ground`: persistent world model
- `unity_bridge`: ROS TCP endpoint for Unity
- `video_gateway`: RTSP/WebRTC gateway

### 3. Run the Gazebo simulation stack

```powershell
./scripts/run-gazebo.ps1
```

This starts the same ROS-side stack, but replaces the fake pose source with a headless Gazebo world bridged through `ros_gz`.

### 4. Inspect Gazebo topics

```powershell
docker compose exec sim_backend bash -lc "source /opt/ros/humble/setup.bash && source /opt/xray/overlay_ws/install/setup.bash && source /opt/xray/ws/install/setup.bash && ign topic -l"
```

### 5. Publish a test video into the gateway

If `ffmpeg` is available on your host:

```powershell
ffmpeg -re -f lavfi -i testsrc=size=1280x720:rate=30 -c:v libx264 -f rtsp rtsp://127.0.0.1:8554/drone_cam
```

### 6. Point Unity at the bridge

- ROS TCP endpoint: `127.0.0.1:10000`
- Video gateway: `rtsp://127.0.0.1:8554/drone_cam`
- Persistent objects topic: `/xray/world/objects`
- Drone odometry topic: `/xray/uav/odom`

## Canonical ROS Topics

- `/xray/uav/odom`: `nav_msgs/msg/Odometry`
- `/xray/perception/detections_2d`: `xray_interfaces/msg/Detection2DArray`
- `/xray/perception/objects_raw`: `xray_interfaces/msg/TrackedObjectArray`
- `/xray/world/objects`: `xray_interfaces/msg/TrackedObjectArray`
- `/tf`: frame transforms for `map -> base_link -> camera_link`

Unity should bind to these topics only. Real-drone and simulator integrations should adapt themselves to this contract instead of making Unity mode-aware.

## Repository Layout

- `docker-compose.yml`: runtime entrypoint with `sim`, `real`, and `debug` profiles
- `docker/`: ROS image build and external overlays
- `config/`: DDS and media gateway runtime config
- `docs/`: architecture, runtime, Docker debugging, Unity integration
- `ros2_ws/src/xray_interfaces`: custom message definitions
- `ros2_ws/src/xray_core`: runnable ROS 2 nodes
- `ros2_ws/src/xray_bringup`: launch files and configuration
- `scripts/`: PowerShell wrappers for common flows

## Current Scope

This repo is a working baseline, not a finished flight stack.

- The `mock` simulator runs today and exercises the exact ROS interfaces Unity will use.
- The `real` and `px4_gz` paths are scaffolded around the same canonical topics.
- The Gazebo path now provides a headless world and bridged odometry under the same `/xray/...` contract.
- PX4 flight control, full Gazebo camera/video integration, and hardware camera drivers are integration targets for the next pass.

## Next Integration Steps

1. Replace `fake_pose_publisher` with PX4 or VIO output normalized into `/xray/uav/odom`.
2. Replace `fake_detector` with a real model node that publishes `Detection2DArray`.
3. Feed depth, stereo, or SLAM-backed range into `object_localizer`.
4. Connect Unity OpenXR components to the ROS TCP endpoint and video gateway.

See [docs/architecture.md](docs/architecture.md), [docs/runtime.md](docs/runtime.md), [docs/docker-debug.md](docs/docker-debug.md), and [docs/unity.md](docs/unity.md) for the operational details.
