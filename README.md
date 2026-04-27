# XrayVision

A two-piece system for streaming stereo camera detections from a fixed
Raspberry Pi 5 station into Unity for VR visualization. **No ROS, no
docker, no broker** — the Pi serves HTTP/WebSocket directly and Unity
connects to it.

## Architecture

```
┌─────────────────────────┐         HTTP/WS        ┌──────────────────────┐
│  Raspberry Pi 5         │  ─────────────────►    │  Unity (Windows/PC)  │
│  - Dual OV9281 cameras  │                        │  - PiClient.cs       │
│  - SGBM depth           │   /video.mjpg          │  - VR scene render   │
│  - YOLO detection       │   /stream  (WS, JSON)  │                      │
│  - 3D world positions   │   /pose, /healthz      │                      │
└─────────────────────────┘                        └──────────────────────┘
```

## Repo layout

- [`pi/`](pi/) — everything that runs on the Pi: capture pipeline, YOLO,
  HTTP/WebSocket server, install script. See [pi/README.md](pi/README.md).
- [`unity/XrayVisionVR/`](unity/XrayVisionVR/) — Unity project. The
  `PiClient` MonoBehaviour talks to the Pi.
- [`docs/unity.md`](docs/unity.md) — Unity integration notes.

## Quick start

1. **Set up the Pi** — follow [pi/README.md](pi/README.md). Verify with
   `http://<pi-ip>:8765/` (dashboard with all four camera feeds).
2. **Open the Unity project** at `unity/XrayVisionVR/`.
3. **Add the `PiClient` component** to a GameObject in your scene.
4. Set `host` to your Pi's IP (e.g. `10.0.0.101`) — fallbacks `pi.local`
   and `pi` are tried automatically.
5. Press play. Console should log `websocket connected`.
