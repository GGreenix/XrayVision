"""AprilTag 36h11 detection + 6-DOF pose estimation (single tag).

Detects one AprilTag (family tag36h11) per frame and estimates its pose
relative to the camera with cv2.solvePnP. Only the configured tag id is
reported (default id 1); all other tags/families are ignored.

Uses cv2.aruco — no extra dependency beyond OpenCV. Frames from MonoPipeline
are already undistorted, so solvePnP runs with zero distortion coefficients.

The tag is meant as a world anchor: it marks a known, fixed reference point,
so the camera's pose can be recovered from where the tag appears.
"""

from dataclasses import dataclass

import cv2
import numpy as np

import math

from xray_pi.stereo import StereoFrame

_FLIP_Y = np.diag([1.0, -1.0, 1.0])

# Unity frame → server body frame (X=forward, Y=left, Z=up):
#   server_x = unity_z,  server_y = -unity_x,  server_z = unity_y
_M_U2S = np.array([[0., 0., 1.], [-1., 0., 0.], [0., 1., 0.]])

# Precomputed: M_opt2unity @ OPTICAL_TO_BODY.T
# Used to build world_from_body from a Unity-frame camera rotation.
_M_R = np.array([[0., -1., 0.], [0., 0., 1.], [1., 0., 0.]])

# OPTICAL_TO_BODY (for reference, matches camera_model.py)
_OTB = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def _matrix_to_quaternion(m: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> (x, y, z, w)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def _make_aruco_detect():
    """Return a detect(gray) callable for tag36h11, spanning OpenCV API versions."""
    dict_id = cv2.aruco.DICT_APRILTAG_36h11
    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)
    # OpenCV < 4.7
    dictionary = cv2.aruco.Dictionary_get(dict_id)
    params = cv2.aruco.DetectorParameters_create()
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


@dataclass
class AprilTagPose:
    tag_id: int
    # Pose of the tag in the CAMERA OPTICAL frame (X right, Y down, Z forward).
    tvec: np.ndarray           # (3,1) metres
    rvec: np.ndarray           # (3,1) Rodrigues rotation
    corners_px: np.ndarray     # (4,2) detected corner pixels, for drawing
    tag_size_m: float

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.tvec))

    def camera_world_pose_server(
        self,
        tag_unity_pos: list[float],
        tag_unity_euler_deg: list[float],
    ) -> tuple[list[float], np.ndarray]:
        """Return (position_m, world_from_body) in server frame.

        tag_unity_pos   : [x,y,z] Unity world position of the tag anchor.
        tag_unity_euler : [pitch,yaw,roll] Unity Euler degrees of the tag anchor.
        """
        d = self.to_dict()
        cam_pos_tag = np.array(d["cam_pos"])        # camera in tag local Unity frame
        cam_quat    = d["cam_rot"]                  # camera rotation in tag local Unity frame

        # Tag world rotation matrix in Unity frame (Y-up, right-hand formula for matrices).
        pitch, yaw, roll = [math.radians(e) for e in tag_unity_euler_deg]
        # Build Ry(yaw) — note: negate yaw for Unity left-hand convention.
        yaw = -yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll),  math.sin(roll)
        R_tag_world = np.array([
            [cy*cr + sy*sp*sr,  cr*sp*sy - cy*sr, cp*sy],
            [cp*sr,             cp*cr,            -sp  ],
            [cy*sp*sr - sy*cr,  sy*sr + cy*cr*sp, cp*cy],
        ])

        # Camera orientation matrix in tag local Unity frame.
        x, y, z, w = cam_quat
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        R_cam_tag = np.array([
            [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
            [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)],
        ])

        # Camera world rotation in Unity frame.
        R_cam_world_unity = R_tag_world @ R_cam_tag

        # Camera world position in Unity frame.
        tag_pos = np.array(tag_unity_pos)
        cam_pos_world_unity = tag_pos + R_tag_world @ cam_pos_tag

        # Convert to server frame.
        position_m   = (_M_U2S @ cam_pos_world_unity).tolist()
        world_from_body = _M_U2S @ R_cam_world_unity @ _M_R

        return position_m, world_from_body

    def to_dict(self) -> dict:
        x, y, z = (float(v) for v in self.tvec.flatten())

        # Pose of the CAMERA in the TAG frame, in Unity conventions, so the tag
        # can act as a world anchor that positions the camera.
        # solvePnP gives  p_cam = R @ p_tag + t  (tag -> camera).
        R, _ = cv2.Rodrigues(self.rvec)
        t = self.tvec.reshape(3)
        R_u = _FLIP_Y @ R @ _FLIP_Y          # express rotation in Unity (Y-up) frame
        t_u = _FLIP_Y @ t
        R_tag_cam = R_u.T                    # camera orientation in tag frame
        cam_pos = (-R_tag_cam @ t_u)         # camera position in tag frame
        cam_rot = _matrix_to_quaternion(R_tag_cam)

        return {
            "id": int(self.tag_id),
            # Tag position in the camera-local Unity frame (for the cube marker).
            "x": x,
            "y": -y,
            "z": z,
            "distance_m": self.distance_m,
            "rvec": [float(v) for v in self.rvec.flatten()],
            # Camera pose in the tag/world frame (for driving the Unity camera).
            "cam_pos": [float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])],
            "cam_rot": [cam_rot[0], cam_rot[1], cam_rot[2], cam_rot[3]],
        }


class AprilTagDetector:
    def __init__(
        self,
        tag_size_m: float,
        tag_id: int = 1,
        tag_world_pos: list[float] | None = None,
        tag_world_euler: list[float] | None = None,
    ) -> None:
        self.tag_size_m = float(tag_size_m)
        self.tag_id = int(tag_id)
        self.tag_world_pos   = tag_world_pos    # Unity frame [x,y,z]
        self.tag_world_euler = tag_world_euler  # Unity Euler degrees [pitch,yaw,roll]
        self._detect = _make_aruco_detect()

        # Tag corners in the tag's own frame: centered at the tag, on the Z=0
        # plane. Order/sign matches what SOLVEPNP_IPPE_SQUARE expects and the
        # cv2.aruco corner order (top-left, top-right, bottom-right, bottom-left).
        h = self.tag_size_m / 2.0
        self._object_points = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

    def detect(self, frame: StereoFrame) -> AprilTagPose | None:
        gray = frame.left_rect
        if gray.ndim == 3:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self._detect(gray)
        if ids is None:
            return None

        ids = ids.flatten()
        match = np.where(ids == self.tag_id)[0]
        if match.size == 0:
            return None
        idx = int(match[0])

        tag_corners = corners[idx].reshape(4, 2).astype(np.float32)

        K = np.array([
            [frame.fx, 0.0,      frame.cx],
            [0.0,      frame.fy, frame.cy],
            [0.0,      0.0,      1.0     ],
        ], dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            self._object_points, tag_corners, K, np.zeros(4),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None

        return AprilTagPose(
            tag_id=self.tag_id,
            tvec=tvec,
            rvec=rvec,
            corners_px=tag_corners,
            tag_size_m=self.tag_size_m,
        )
