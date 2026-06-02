#!/usr/bin/env python3
"""Stereo calibration for the dual USB Arducam OV9281 setup.

Workflow:
    1. Print a checkerboard (default: 9x6 inner corners, 25 mm squares).
    2. List your cameras to find their stable paths:
           ls -l /dev/v4l/by-id/
    3. Run this script on the Pi:
           sudo systemctl stop xray-station
           python3 stereo_calibrate.py \
               --left  /dev/v4l/by-id/usb-Arducam_LEFT-video-index0 \
               --right /dev/v4l/by-id/usb-Arducam_RIGHT-video-index0 \
               --output /opt/xray/config/stereo_calibration.yaml
    4. Hold the board in front of both cameras; press SPACE to capture
       a pair (only saved if both cameras see the full board).
       Capture ~25 pairs covering the whole image, near and far.
       Press 'c' to compute and save calibration. 'q' to quit.

Output is the YAML schema consumed by xray_pi.stereo_calibration.
"""

import argparse
import sys
import time
from pathlib import Path
import platform

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="Path to write calibration yaml")
    parser.add_argument("--left", required=True, help="Left camera device (index on Windows, /dev/v4l/... on Linux)")
    parser.add_argument("--right", help="Right camera device (omit for mono calibration)")
    parser.add_argument("--mono", action="store_true", help="Single-camera intrinsic calibration only")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--rows", type=int, default=6, help="Inner-corner rows on the checkerboard")
    parser.add_argument("--cols", type=int, default=9, help="Inner-corner cols on the checkerboard")
    parser.add_argument("--square-size-m", type=float, default=0.025, help="Edge length of one square (meters)")
    parser.add_argument("--min-pairs", type=int, default=15)
    parser.add_argument("--no-preview", action="store_true", help="No GUI; capture from terminal ENTER")
    return parser.parse_args()


def make_object_points(rows: int, cols: int, square: float) -> np.ndarray:
    pts = np.zeros((rows * cols, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return pts * square


def open_cap(device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
    if sys.platform == "win32":
        cap = cv2.VideoCapture(int(device), cv2.CAP_MSMF)
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open {device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


def grab_gray(cap: cv2.VideoCapture) -> np.ndarray | None:
    ok, frame = cap.read()
    if not ok or frame is None:
        return None
    if frame.ndim == 3:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame


def find_corners(image: np.ndarray, rows: int, cols: int):
    found, corners = cv2.findChessboardCorners(
        image, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK,
    )
    if not found:
        return False, None
    refined = cv2.cornerSubPix(
        image, corners, (11, 11), (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    return True, refined


def main() -> int:
    args = parse_args()
    mono = args.mono or args.right is None

    left = open_cap(args.left, args.width, args.height, args.fps)
    right = None if mono else open_cap(args.right, args.width, args.height, args.fps)
    time.sleep(0.5)

    object_pts = make_object_points(args.rows, args.cols, args.square_size_m)
    obj_points: list[np.ndarray] = []
    left_points: list[np.ndarray] = []
    right_points: list[np.ndarray] = []

    mode_str = "mono" if mono else "stereo"
    print(f"Capturing {mode_str} frames (need >= {args.min_pairs}).")
    print("  SPACE / ENTER : capture")
    print("  c             : compute and save")
    print("  q             : quit")

    show_preview = not args.no_preview
    win_title = "calibration" if mono else "stereo (L | R)"
    if show_preview:
        cv2.namedWindow(win_title, cv2.WINDOW_NORMAL)

    try:
        while True:
            l_img = grab_gray(left)
            if l_img is None:
                continue
            r_img = grab_gray(right) if right is not None else None
            if not mono and r_img is None:
                continue

            if show_preview:
                vis = cv2.cvtColor(l_img, cv2.COLOR_GRAY2BGR)
                ok_preview, corners_preview = find_corners(l_img, args.rows, args.cols)
                if ok_preview:
                    cv2.drawChessboardCorners(vis, (args.cols, args.rows), corners_preview, True)
                    status = "BOARD FOUND - press SPACE to capture"
                    color = (0, 255, 0)
                else:
                    status = "looking for board..."
                    color = (0, 0, 255)
                preview = cv2.resize(vis, (960, int(960 * args.height / args.width)))
                cv2.putText(preview, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                cv2.putText(preview, f"captured={len(obj_points)}/{args.min_pairs}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow(win_title, preview)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = ord(input("[ENTER]=capture, c=compute, q=quit: ").strip()[:1] or " ")

            if key == ord("q"):
                break
            if key == ord("c"):
                break
            if key in (ord(" "), 13):
                ok_l, c_l = find_corners(l_img, args.rows, args.cols)
                if mono:
                    if ok_l:
                        obj_points.append(object_pts)
                        left_points.append(c_l)
                        print(f"  captured #{len(obj_points)}")
                    else:
                        print("  board not visible")
                else:
                    ok_r, c_r = find_corners(r_img, args.rows, args.cols)
                    if ok_l and ok_r:
                        obj_points.append(object_pts)
                        left_points.append(c_l)
                        right_points.append(c_r)
                        print(f"  captured pair #{len(obj_points)}")
                    else:
                        print(f"  board not fully visible (left={ok_l}, right={ok_r})")
    finally:
        if show_preview:
            cv2.destroyAllWindows()
        left.release()
        if right is not None:
            right.release()

    if len(obj_points) < args.min_pairs:
        print(f"Only {len(obj_points)} frames — need >= {args.min_pairs}. Aborting.", file=sys.stderr)
        return 1

    image_size = (args.width, args.height)
    print("Calibrating camera...")
    err, K, D, _, _ = cv2.calibrateCamera(obj_points, left_points, image_size, None, None)
    print(f"  RMS reprojection error = {err:.3f} px  (good if < 1.0)")

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if mono:
        output = {
            "image_width": args.width,
            "image_height": args.height,
            "K": K.flatten().tolist(),
            "D": D.flatten().tolist(),
        }
    else:
        print("Calibrating right camera...")
        err_r, K_r, D_r, _, _ = cv2.calibrateCamera(obj_points, right_points, image_size, None, None)
        print(f"  right RMS = {err_r:.3f} px")
        print("Calibrating stereo extrinsics...")
        err_s, K, D, K_r, D_r, R, T, _, _ = cv2.stereoCalibrate(
            obj_points, left_points, right_points,
            K, D, K_r, D_r, image_size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
            flags=cv2.CALIB_FIX_INTRINSIC,
        )
        baseline = float(np.linalg.norm(T))
        print(f"  stereo RMS = {err_s:.3f} px,  baseline = {baseline * 100:.2f} cm")
        if abs(baseline - 0.08) > 0.02:
            print(f"  WARNING: measured baseline {baseline*100:.2f} cm differs from intended 8 cm.")
        output = {
            "image_width": args.width,
            "image_height": args.height,
            "left":  {"K": K.flatten().tolist(), "D": D.flatten().tolist()},
            "right": {"K": K_r.flatten().tolist(), "D": D_r.flatten().tolist()},
            "R": R.flatten().tolist(),
            "T": T.flatten().tolist(),
        }

    with out_path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(output, fh, sort_keys=False)
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
