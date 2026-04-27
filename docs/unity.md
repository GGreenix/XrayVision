# Unity Integration

The Unity project (`unity/XrayVisionVR`) connects directly to the Pi
station's HTTP/WebSocket server. There is no ROS layer, no broker, no
docker. One MonoBehaviour does the work.

## PiClient

`Assets/Scripts/PiClient.cs` is a single-file client that:

1. **Probes** `http://{host}:{port}/healthz` to find a reachable Pi.
   The `host` field is tried first; falls back to `pi.local` and `pi`.
2. **Fetches** `/pose` once at startup (the camera's static world pose).
3. **Connects** to `ws://{host}:{port}/stream` and reads JSON detection
   payloads at the rate the Pi publishes them (~10 Hz by default).

It exposes:

- `IsConnected` (bool) — current WebSocket state.
- `ResolvedHost` (string) — which candidate host responded to healthz.
- `LastPayload` (string) — most recent JSON detection payload.
- `OnDetectionPayload` (event) — fired on every WebSocket message.
- `OnPoseReceived` (event) — fired once after `/pose` is fetched.

## Setup

1. Open `unity/XrayVisionVR/` in Unity.
2. Add the `PiClient` component to any GameObject (e.g. an empty
   `[PiClient]` object in your scene).
3. Set `host` to your Pi's IP (e.g. `10.0.0.101`).
4. Press play — watch the Console for:
   - `[PiClient] using host 10.0.0.101:8765`
   - `[PiClient] pose: pos=[0,0,1.5] rpy=[0,0,0]`
   - `[PiClient] websocket connected`

If you see `no candidate host responded`, check that the Pi station is
running (`curl http://<pi-ip>:8765/healthz` from your PC).

## Detection payload schema

Each `/stream` message is a JSON object:

```json
{
  "t": 1714253451.123,
  "objects": [
    {
      "tracking_id": "person-0",
      "class_id": "person",
      "confidence": 0.91,
      "x": 1.42, "y": 0.10, "z": 1.00,
      "depth_m": 2.81,
      "bbox": { "u_norm": 0.5, "v_norm": 0.6, "w_norm": 0.2, "h_norm": 0.4 }
    }
  ]
}
```

Coordinates `x, y, z` are in the world frame as configured in the Pi's
`station.yaml` (`pose:` section).

## Video

Currently `PiClient` does not pull the MJPEG video stream — only
detections. To preview the camera feeds, open `http://<pi-ip>:8765/` in
a browser (the Pi serves a 2×2 dashboard of all four feeds).

If you want video inside Unity later, the simplest path is a
`UnityWebRequestTexture` polling `/video/left.mjpg` boundary frames into
a `Texture2D` and assigning it to a `RawImage` or material. Not in
scope for the current step.
