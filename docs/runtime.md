# Runtime

## Simulation

The repo ships with a `mock` simulation mode that is lightweight enough for quick bring-up:

- `fake_pose_publisher` emits canonical odometry and TF
- `fake_detector` emits repeatable 2D detections
- `object_localizer` converts those detections to approximate map-frame 3D markers
- `world_model` makes object IDs persistent across updates

Start it with:

```powershell
./scripts/run-sim.ps1
```

## Gazebo

The repo also ships with a headless Gazebo-backed simulator for world bring-up and canonical ROS topic validation:

- Gazebo runs a simple `xray_world` scene with a drone marker and reference objects
- `ros_gz_bridge` forwards `/clock` and Gazebo odometry into ROS 2
- The canonical pose topic remains `/xray/uav/odom`
- The current perception path still uses `fake_detector` and `object_localizer`

Start it with:

```powershell
./scripts/run-gazebo.ps1
```

Useful checks:

```powershell
docker compose exec sim_backend bash -lc "source /opt/ros/humble/setup.bash && source /opt/xray/overlay_ws/install/setup.bash && source /opt/xray/ws/install/setup.bash && ign topic -l"
docker compose exec ground bash -lc "source /opt/ros/humble/setup.bash && source /opt/xray/overlay_ws/install/setup.bash && source /opt/xray/ws/install/setup.bash && ros2 topic echo /xray/uav/odom --once"
```

The Gazebo flow is deliberately headless because that is the most reliable Docker baseline. If you need a GUI client, run the same world from Linux or WSLg instead of plain Docker Desktop PowerShell.

## Real Drone

The `real` mode expects external publishers for the drone pose and, optionally, the detector.

Start it with:

```powershell
./scripts/run-real.ps1
```

If you already have a real pose source, disable the scaffolded fake publishers in `.env`:

```text
XRAY_USE_FAKE_POSE=false
XRAY_USE_FAKE_DETECTIONS=false
XRAY_MODE=real
```

## Switching Modes

Switching modes should only change environment values and launch entrypoints, not Unity logic.

- `XRAY_MODE=sim`: start `sim_backend`, `ground`, `unity_bridge`, `video_gateway`
- `XRAY_SIM_BACKEND=mock`: use the lightweight publisher-only simulator
- `XRAY_SIM_BACKEND=gazebo`: use the Gazebo-backed simulator
- `XRAY_MODE=real`: start `drone`, `ground`, `unity_bridge`, `video_gateway`

The intended next simulator backend is `px4_gz`, but this repo keeps that as an adapter target instead of hardcoding simulator-specific topics into Unity.

## Video Path

`MediaMTX` is included as the media gateway. The default RTSP path is:

```text
rtsp://127.0.0.1:8554/drone_cam
```

For a smoke test without a drone camera:

```powershell
ffmpeg -re -f lavfi -i testsrc=size=1280x720:rate=30 -c:v libx264 -f rtsp rtsp://127.0.0.1:8554/drone_cam
```
