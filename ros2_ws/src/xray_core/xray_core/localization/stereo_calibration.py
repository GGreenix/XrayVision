"""Loader for stereo calibration data.

The calibration YAML is the source of truth for intrinsics, distortion,
extrinsics (R, T from left to right camera), and image size. Produced by
pi/tools/stereo_calibrate.py and consumed by the stereo depth node and
the depth-based localizer.

YAML schema:
  image_width: 1600
  image_height: 1300
  left:
    K:  [fx, 0, cx, 0, fy, cy, 0, 0, 1]   # 3x3, row-major
    D:  [k1, k2, p1, p2, k3]              # opencv distortion
  right:
    K:  [...]
    D:  [...]
  R:    [...9 values, 3x3 row-major...]   # right-from-left rotation
  T:    [tx, ty, tz]                       # right-from-left translation, meters
"""

from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class StereoCalibration:
    image_width: int
    image_height: int
    K_left: np.ndarray   # 3x3
    D_left: np.ndarray   # (5,)
    K_right: np.ndarray
    D_right: np.ndarray
    R: np.ndarray        # 3x3 right-from-left rotation
    T: np.ndarray        # (3,) right-from-left translation in meters

    @property
    def baseline_m(self) -> float:
        return float(np.linalg.norm(self.T))

    @classmethod
    def load(cls, path: str) -> "StereoCalibration":
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
        return cls(
            image_width=int(raw["image_width"]),
            image_height=int(raw["image_height"]),
            K_left=np.array(raw["left"]["K"], dtype=np.float64).reshape(3, 3),
            D_left=np.array(raw["left"]["D"], dtype=np.float64).reshape(-1),
            K_right=np.array(raw["right"]["K"], dtype=np.float64).reshape(3, 3),
            D_right=np.array(raw["right"]["D"], dtype=np.float64).reshape(-1),
            R=np.array(raw["R"], dtype=np.float64).reshape(3, 3),
            T=np.array(raw["T"], dtype=np.float64).reshape(3),
        )

    def save(self, path: str) -> None:
        data = {
            "image_width": int(self.image_width),
            "image_height": int(self.image_height),
            "left": {
                "K": self.K_left.flatten().tolist(),
                "D": self.D_left.flatten().tolist(),
            },
            "right": {
                "K": self.K_right.flatten().tolist(),
                "D": self.D_right.flatten().tolist(),
            },
            "R": self.R.flatten().tolist(),
            "T": self.T.flatten().tolist(),
        }
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(data, fh, sort_keys=False)


@dataclass
class StereoRectification:
    """Output of cv2.stereoRectify — reusable maps for fast rectification."""

    map_left_x: np.ndarray
    map_left_y: np.ndarray
    map_right_x: np.ndarray
    map_right_y: np.ndarray
    P_left: np.ndarray      # 3x4 projection matrix in rectified frame
    P_right: np.ndarray
    Q: np.ndarray           # 4x4 disparity-to-depth reprojection
    fx_rect: float
    fy_rect: float
    cx_rect: float
    cy_rect: float
    baseline_m: float

    @classmethod
    def from_calibration(cls, calib: StereoCalibration) -> "StereoRectification":
        import cv2

        size = (calib.image_width, calib.image_height)
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            cameraMatrix1=calib.K_left,
            distCoeffs1=calib.D_left,
            cameraMatrix2=calib.K_right,
            distCoeffs2=calib.D_right,
            imageSize=size,
            R=calib.R,
            T=calib.T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0.0,
        )
        map_lx, map_ly = cv2.initUndistortRectifyMap(
            calib.K_left, calib.D_left, R1, P1, size, cv2.CV_32FC1
        )
        map_rx, map_ry = cv2.initUndistortRectifyMap(
            calib.K_right, calib.D_right, R2, P2, size, cv2.CV_32FC1
        )
        # Rectified intrinsics live in P1: [[fx, 0, cx, 0],[0, fy, cy, 0],[0,0,1,0]]
        fx, fy = float(P1[0, 0]), float(P1[1, 1])
        cx, cy = float(P1[0, 2]), float(P1[1, 2])
        # Q[3,2] = -1/Tx (in pixels). Baseline = -P2[0,3] / fx.
        baseline = float(-P2[0, 3] / fx)
        return cls(
            map_left_x=map_lx,
            map_left_y=map_ly,
            map_right_x=map_rx,
            map_right_y=map_ry,
            P_left=P1,
            P_right=P2,
            Q=Q,
            fx_rect=fx,
            fy_rect=fy,
            cx_rect=cx,
            cy_rect=cy,
            baseline_m=baseline,
        )
