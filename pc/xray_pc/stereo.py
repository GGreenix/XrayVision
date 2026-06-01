"""SGBM stereo depth processing on PC."""

from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np
import yaml


@dataclass
class StereoCalib:
    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float
    Left_Stereo_Map: tuple
    Right_Stereo_Map: tuple

    @classmethod
    def load(cls, path: str):
        with open(path) as f:
            d = yaml.safe_load(f)
        w, h = int(d["image_width"]), int(d["image_height"])
        Kl = np.array(d["left"]["K"], dtype=np.float64).reshape(3, 3)
        Dl = np.array(d["left"]["D"], dtype=np.float64)
        Kr = np.array(d["right"]["K"], dtype=np.float64).reshape(3, 3)
        Dr = np.array(d["right"]["D"], dtype=np.float64)
        R  = np.array(d["R"], dtype=np.float64).reshape(3, 3)
        T  = np.array(d["T"], dtype=np.float64).reshape(3, 1)

        size = (w, h)
        RL, RR, PL, PR, _, _, _ = cv2.stereoRectify(Kl, Dl, Kr, Dr, size, R, T, alpha=0)
        Left_Stereo_Map = cv2.initUndistortRectifyMap(Kl, Dl, RL, PL, size, cv2.CV_16SC2)
        Right_Stereo_Map = cv2.initUndistortRectifyMap(Kr, Dr, RR, PR, size, cv2.CV_16SC2)

        return cls(
            fx=float(PL[0, 0]), fy=float(PL[1, 1]),
            cx=float(PL[0, 2]), cy=float(PL[1, 2]),
            baseline_m=abs(float(T[0])),
            Left_Stereo_Map=Left_Stereo_Map,
            Right_Stereo_Map=Right_Stereo_Map,
        )


@dataclass
class StereoFrame:
    left_rect: np.ndarray
    right_rect: np.ndarray
    depth: np.ndarray
    left_raw: np.ndarray
    right_raw: np.ndarray
    fx: float
    fy: float
    cx: float
    cy: float


class SGBMProcessor:
    def __init__(self, calib: StereoCalib, params: dict = None):
        self.calib = calib
        p = params or {}
        self.downscale = max(1, int(p.get("downscale", 1)))
        bs = int(p.get("block_size", 7))
        mode_str = p.get("mode", "SGBM_3WAY")
        if mode_str == "HH":
            mode = cv2.STEREO_SGBM_MODE_HH
        else:
            mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=int(p.get("min_disparity", 0)),
            numDisparities=int(p.get("num_disparities", 96)),
            blockSize=bs,
            P1=8 * bs ** 2,
            P2=32 * bs ** 2,
            uniquenessRatio=int(p.get("uniqueness_ratio", 10)),
            speckleWindowSize=int(p.get("speckle_window_size", 100)),
            speckleRange=int(p.get("speckle_range", 2)),
            mode=mode,
        )
        self.min_depth = float(p.get("min_depth_m", 0.3))
        self.max_depth = float(p.get("max_depth_m", 8.0))

    def process(self, left: np.ndarray, right: np.ndarray) -> StereoFrame:
        c = self.calib
        lg = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if left.ndim == 3 else left.copy()
        rg = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if right.ndim == 3 else right.copy()

        lr = cv2.remap(lg, c.Left_Stereo_Map[0], c.Left_Stereo_Map[1], cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
        rr = cv2.remap(rg, c.Right_Stereo_Map[0], c.Right_Stereo_Map[1], cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)

        if self.downscale > 1:
            h, w = lr.shape[:2]
            small_w, small_h = w // self.downscale, h // self.downscale
            lr_s = cv2.resize(lr, (small_w, small_h), interpolation=cv2.INTER_AREA)
            rr_s = cv2.resize(rr, (small_w, small_h), interpolation=cv2.INTER_AREA)
        else:
            lr_s, rr_s = lr, rr

        disp = self.matcher.compute(lr_s, rr_s).astype(np.float32) / 16.0

        scale = 1.0 / self.downscale
        fx_s = c.fx * scale
        with np.errstate(divide="ignore", invalid="ignore"):
            depth_s = np.where(disp > 0, (fx_s * c.baseline_m) / disp, 0.0).astype(np.float32)
        depth_s[(depth_s < self.min_depth) | (depth_s > self.max_depth)] = 0.0

        # Upsample depth back to full resolution for YOLO bbox sampling
        if self.downscale > 1:
            depth = cv2.resize(depth_s, (lr.shape[1], lr.shape[0]), interpolation=cv2.INTER_NEAREST)
        else:
            depth = depth_s

        return StereoFrame(
            left_rect=lr, right_rect=rr, depth=depth,
            left_raw=left, right_raw=right,
            fx=c.fx, fy=c.fy, cx=c.cx, cy=c.cy,
        )
