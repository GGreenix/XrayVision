# Unity Integration

## Data Split

Use two transport paths:

- ROS TCP for structured data
- RTSP or WebRTC for video

Do not stream high-rate camera frames through the ROS TCP endpoint.

## ROS TCP

Import `ROS-TCP-Connector` into Unity and connect it to:

```text
127.0.0.1:10000
```

Subscribe to:

- `/xray/uav/odom`
- `/xray/world/objects`

Generate C# message types from the `xray_interfaces` package so Unity can deserialize `Detection2DArray` and `TrackedObjectArray`.

## World Alignment

- Treat ROS `map` as the Unity world root.
- Convert ROS coordinates into Unity coordinates in one place only.
- Keep `base_link` and `camera_link` semantics consistent between sim and real modes.

The scaffold keeps object markers in `map` so they remain stable even if detections arrive intermittently.

