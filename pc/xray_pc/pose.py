"""Camera pose: position + rotation, with camera->world transform."""

import math
import numpy as np


class Pose:
    def __init__(self, position_m, rpy_deg, mount_rpy_deg=None):
        self.position_m = np.array(position_m, dtype=float)
        self.rpy_deg = np.array(rpy_deg, dtype=float)
        self.mount_rpy_deg = np.array(mount_rpy_deg or [0.0, 0.0, 0.0], dtype=float)
        self._build_matrix()

    def _build_matrix(self):
        rpy = self.rpy_deg + self.mount_rpy_deg
        r, p, y = [math.radians(x) for x in rpy]
        Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        self.R = Rz @ Ry @ Rx

    def camera_to_world(self, point_camera):
        pt = np.array(point_camera, dtype=float)
        return (self.R @ pt + self.position_m).tolist()

    def update(self, R: np.ndarray, t: np.ndarray):
        self.R = R
        self.position_m = t

    def as_matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.position_m
        return T

    def rotation_quat_xyzw(self) -> list:
        """Return rotation matrix as [x, y, z, w] quaternion."""
        R = self.R
        trace = R[0,0] + R[1,1] + R[2,2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            return [(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s]
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            return [0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s]
        elif R[1,1] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            return [(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s]
        else:
            s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            return [(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s]
