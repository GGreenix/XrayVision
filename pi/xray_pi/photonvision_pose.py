"""Read camera field pose from PhotonVision via photonlibpy + NT4.

PhotonVision runs its own AprilTag detection pipeline. This module connects
to it, reads the estimated camera pose in the field frame, and calls
on_pose(position_m, world_from_body) so the detector always uses the latest
camera world pose for detection localization.

Field frame == server body frame: X=forward, Y=left, Z=up.
"""

import math
import threading
import time
from typing import Callable

import numpy as np

# OPTICAL_TO_BODY (matches camera_model.py)
_OTB = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def _unity_to_wpilib_translation(unity_pos: list[float]):
    """Unity (X=right, Y=up, Z=forward) → WPILib (X=forward, Y=left, Z=up)."""
    from wpimath.geometry import Translation3d
    x, y, z = unity_pos
    return Translation3d(z, -x, y)


def _unity_euler_to_wpilib_rotation(unity_euler_deg: list[float]):
    """Unity Euler XYZ degrees → WPILib Rotation3d (RPY radians, right-handed Z-up)."""
    from wpimath.geometry import Rotation3d
    pitch, yaw, roll = unity_euler_deg
    return Rotation3d(
        math.radians(roll),
        math.radians(pitch),
        math.radians(-yaw),  # negate: Unity left-handed Y → WPILib right-handed Z
    )


def build_field_layout(tag_configs: list[dict]):
    """Build AprilTagFieldLayout from tag dicts with Unity-frame poses.

    Each dict: {"id": int, "unity_pos": [x,y,z], "unity_euler": [p,y,r]}
    """
    from wpimath.geometry import Pose3d
    from robotpy_apriltag import AprilTag, AprilTagFieldLayout

    tags = []
    for t in tag_configs:
        pose = Pose3d(
            _unity_to_wpilib_translation(t["unity_pos"]),
            _unity_euler_to_wpilib_rotation(t.get("unity_euler", [0., 0., 0.])),
        )
        tag = AprilTag()
        tag.ID = t["id"]
        tag.pose = pose
        tags.append(tag)

    return AprilTagFieldLayout(tags, 1000.0, 1000.0)


# Server/field frame (X=forward, Y=left, Z=up) → Unity (X=right, Y=up, Z=forward).
_M_S2U = np.array([[0., -1., 0.], [0., 0., 1.], [1., 0., 0.]])


def _matrix_to_quaternion(m: np.ndarray) -> list[float]:
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25*s, (m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s
    elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
        s = math.sqrt(1.0+m[0,0]-m[1,1]-m[2,2]) * 2.0
        w, x, y, z = (m[2,1]-m[1,2])/s, 0.25*s, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s
    elif m[1,1] > m[2,2]:
        s = math.sqrt(1.0+m[1,1]-m[0,0]-m[2,2]) * 2.0
        w, x, y, z = (m[0,2]-m[2,0])/s, (m[0,1]+m[1,0])/s, 0.25*s, (m[1,2]+m[2,1])/s
    else:
        s = math.sqrt(1.0+m[2,2]-m[0,0]-m[1,1]) * 2.0
        w, x, y, z = (m[1,0]-m[0,1])/s, (m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s, 0.25*s
    return [float(x), float(y), float(z), float(w)]


def _server_pose_to_unity(position_m: list[float], rotation3d) -> dict:
    """Camera world pose (field frame) → Unity world pose dict {pos, rot}."""
    pos = (_M_S2U @ np.array(position_m)).tolist()
    q = rotation3d.getQuaternion()
    w, x, y, z = q.W(), q.X(), q.Y(), q.Z()
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)  ],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)  ],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])
    R_u = _M_S2U @ R @ _M_S2U.T
    return {"pos": [float(v) for v in pos], "rot": _matrix_to_quaternion(R_u)}


def _rotation3d_to_world_from_body(rot) -> np.ndarray:
    """Convert WPILib Rotation3d → world_from_body matrix for the detector.

    WPILib pose gives camera body orientation in world frame.
    Detector needs: world_from_body such that
        p_world = cam_origin + world_from_body @ OTB @ p_optical
    So: world_from_body @ OTB = R_cam_world → world_from_body = R_cam_world @ OTB.T
    """
    q = rot.getQuaternion()
    w, x, y, z = q.W(), q.X(), q.Y(), q.Z()
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)  ],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)  ],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)],
    ])
    return R @ _OTB.T


class PhotonVisionPoseReader:
    """Background thread that reads camera pose from PhotonVision and calls on_pose."""

    def __init__(
        self,
        camera_name: str,
        field_layout,
        on_pose: Callable[[list[float], np.ndarray], None],
        poll_hz: float = 20.0,
    ) -> None:
        self._camera_name = camera_name
        self._field_layout = field_layout
        self._on_pose = on_pose
        self._poll_period = 1.0 / poll_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="photon-pose"
        )
        # Last camera pose seen (Unity world frame). Persists when PhotonVision
        # loses the camera, so the camera stays put instead of snapping away.
        self.last_unity_pose: dict | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        try:
            import ntcore
            from photonlibpy.photonCamera import PhotonCamera
            from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
            from wpimath.geometry import Transform3d

            # We act as the NT4 SERVER; PhotonVision connects to us as a client
            # (PhotonVision → Settings → Networking → server address = 127.0.0.1,
            #  and "Run NetworkTables Server (Debugging Only)" must be OFF).
            nt = ntcore.NetworkTableInstance.getDefault()
            nt.startServer()
            print("[photon] NT4 server started — waiting for PhotonVision to connect "
                  "(it must point at 127.0.0.1)", flush=True)

            deadline = time.time() + 10.0
            while time.time() < deadline and not nt.isConnected():
                time.sleep(0.2)
            if nt.isConnected():
                ips = [c.remote_ip for c in nt.getConnections()]
                print(f"[photon] PhotonVision CONNECTED from {ips}", flush=True)
            else:
                print("[photon] No client connected yet — check PhotonVision is pointed "
                      "at 127.0.0.1 and the debug NT server is OFF.", flush=True)

            # Show which camera tables PhotonVision is actually publishing.
            time.sleep(0.5)
            cams = nt.getTable("photonvision").getSubTables()
            print(f"[photon] cameras published by PhotonVision: {cams}", flush=True)
            if self._camera_name not in cams:
                print(f"[photon] WARNING: configured camera_name '{self._camera_name}' "
                      f"is not in that list — set camera_name to one of {cams}.", flush=True)

            camera = PhotonCamera(self._camera_name)
            estimator = PhotonPoseEstimator(
                self._field_layout,
                Transform3d(),  # camera IS the tracked body
            )

            last_print = 0.0
            while not self._stop.is_set():
                # When PhotonVision doesn't own the camera (your XrayVision feed
                # is using it), it stops publishing — skip quietly and keep the
                # last pose instead of spamming "not sending new data".
                if not camera.isConnected():
                    time.sleep(self._poll_period)
                    continue

                try:
                    pipeline_result = camera.getLatestResult()
                except Exception:  # noqa: BLE001
                    time.sleep(self._poll_period)
                    continue

                result = estimator.estimateCoprocMultiTagPose(pipeline_result) \
                    or estimator.estimateLowestAmbiguityPose(pipeline_result)
                if result is not None:
                    t = result.estimatedPose.translation()
                    r = result.estimatedPose.rotation()
                    position_m = [t.x, t.y, t.z]
                    world_from_body = _rotation3d_to_world_from_body(r)
                    # Save the last pose (Unity frame) for forwarding to Unity.
                    self.last_unity_pose = _server_pose_to_unity(position_m, r)
                    self._on_pose(position_m, world_from_body)
                    now = time.time()
                    if now - last_print > 0.5:
                        print(f"[photon] camera pos  x={t.x:.3f}  y={t.y:.3f}  z={t.z:.3f} m", flush=True)
                        last_print = now
                time.sleep(self._poll_period)

        except Exception as exc:
            print(f"[photon] pose reader error: {exc}", flush=True)
