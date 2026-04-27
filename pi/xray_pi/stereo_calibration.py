"""Stereo calibration loader + rectification. Vendored from xray_core/localization/."""

from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class StereoCalibration:
    image_width: int
    image_height: int
    K_left: np.ndarray
    D_left: np.ndarray
    K_right: np.ndarray
    D_right: np.ndarray
    R: np.ndarray
    T: np.ndarray

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


@dataclass
class StereoRectification:
    map_left_x: np.ndarray
    map_left_y: np.ndarray
    map_right_x: np.ndarray
    map_right_y: np.ndarray
    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float
    size: tuple[int, int]   # (width, height)

    @classmethod
    def from_calibration(cls, calib: StereoCalibration, downscale: int = 1) -> "StereoRectification":
        import cv2

        if downscale == 1:
            K_l, K_r = calib.K_left.copy(), calib.K_right.copy()
            w, h = calib.image_width, calib.image_height
        else:
            scale = 1.0 / downscale
            K_l = calib.K_left.copy()
            K_l[:2, :] *= scale
            K_r = calib.K_right.copy()
            K_r[:2, :] *= scale
            w = calib.image_width // downscale
            h = calib.image_height // downscale

        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            cameraMatrix1=K_l, distCoeffs1=calib.D_left,
            cameraMatrix2=K_r, distCoeffs2=calib.D_right,
            imageSize=(w, h), R=calib.R, T=calib.T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0,
        )
        m_lx, m_ly = cv2.initUndistortRectifyMap(K_l, calib.D_left, R1, P1, (w, h), cv2.CV_32FC1)
        m_rx, m_ry = cv2.initUndistortRectifyMap(K_r, calib.D_right, R2, P2, (w, h), cv2.CV_32FC1)
        fx, fy = float(P1[0, 0]), float(P1[1, 1])
        cx, cy = float(P1[0, 2]), float(P1[1, 2])
        baseline = float(-P2[0, 3] / fx)
        return cls(
            map_left_x=m_lx, map_left_y=m_ly,
            map_right_x=m_rx, map_right_y=m_ry,
            fx=fx, fy=fy, cx=cx, cy=cy,
            baseline_m=baseline, size=(w, h),
        )
