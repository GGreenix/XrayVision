# Docker Debugging

## Common Commands

Start a debug shell:

```powershell
./scripts/debug-shell.ps1
```

Tail logs:

```powershell
docker compose logs -f ground
docker compose logs -f sim_backend
```

Inspect topics from a running container:

```powershell
docker compose exec ground bash
ros2 topic list
ros2 topic echo /xray/world/objects
ros2 topic hz /xray/uav/odom
ros2 node list
ros2 node info /world_model
```

Inspect Gazebo transport topics from the Gazebo-backed sim:

```powershell
docker compose exec sim_backend bash
ign topic -l
ign topic -e -t /xray/gz/odometry
```

## Networking Notes

- The Compose file uses `network_mode: host` because ROS 2 DDS is easier to reason about on Linux host networking.
- Docker Desktop on Windows does not behave like native Linux for DDS discovery. Use WSL2 or a Linux host for realistic networking tests.
- Gazebo GUI access from Docker Desktop Windows is not a supported baseline in this repo. The Gazebo flow runs headless by design.
- Keep Unity bound to the ROS TCP endpoint and media gateway instead of exposing raw DDS to the headset.

## Logging and Reproducibility

- `./bags` is reserved for bag output and replay artifacts.
- `./logs` stores Compose logs or exported diagnostics.
- `.env` is the runtime switchboard for `sim` versus `real`.
- Reproduce issues by recording `/xray/uav/odom`, `/xray/perception/objects_raw`, `/xray/world/objects`, `/tf`, and `/tf_static`.
