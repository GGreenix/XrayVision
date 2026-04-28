# Architecture

## Layers

### Vehicle

- Flight controller or external estimator owns raw vehicle state.
- A localization adapter is responsible for publishing canonical odometry on `/xray/uav/odom`.
- Camera transport stays outside ROS for latency-sensitive streaming.

### Perception

- Detector nodes publish `xray_interfaces/msg/Detection2DArray` on `/xray/perception/detections_2d`.
- `object_localizer` projects detections into the `map` frame and publishes `/xray/perception/objects_raw`.
- `world_model` merges repeated observations into stable IDs on `/xray/world/objects`.

### Ground Station

- Aggregates canonical topics for logging, debugging, and Unity-facing transport.
- Runs `ROS-TCP-Endpoint` for structured topic access from Unity.
- Hosts the media gateway for RTSP/WebRTC distribution.

### Unity

- Consumes ROS topics only from the ground station.
- Consumes video only from the media gateway.
- Never branches on `sim` versus `real`; it binds to the same endpoints either way.

## Coordinate Frames

- `map`: persistent world frame used for Unity alignment
- `base_link`: drone body frame
- `camera_link`: forward-facing camera frame

The current scaffold publishes `map -> base_link` from the pose source and `base_link -> camera_link` as a static transform. Real deployments should keep this frame tree intact and replace only the publishers behind it.

## Canonical Interfaces

- Pose source contract: `nav_msgs/msg/Odometry` on `/xray/uav/odom`
- Detection contract: `xray_interfaces/msg/Detection2DArray` on `/xray/perception/detections_2d`
- Raw localized objects: `xray_interfaces/msg/TrackedObjectArray` on `/xray/perception/objects_raw`
- Persistent objects: `xray_interfaces/msg/TrackedObjectArray` on `/xray/world/objects`

## Extension Points

- Replace `fake_pose_publisher` with PX4, VIO, SLAM, RTK fusion, or simulator odometry.
- Replace `fake_detector` with YOLO, TensorRT, or ground-side inference.
- Replace range estimation in `object_localizer` with stereo, depth, or SLAM-backed triangulation.

