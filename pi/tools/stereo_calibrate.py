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

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="Path to write stereo_calibration.yaml")
    parser.add_argument("--left", required=True, help="Left camera device path (prefer /dev/v4l/by-id/...)")
    parser.add_argument("--right", required=True, help="Right camera device path")
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

    left = open_cap(args.left, args.width, args.height, args.fps)
    right = open_cap(args.right, args.width, args.height, args.fps)
    time.sleep(0.5)

    object_pts = make_object_points(args.rows, args.cols, args.square_size_m)
    obj_points: list[np.ndarray] = []
    left_points: list[np.ndarray] = []
    right_points: list[np.ndarray] = []

    print(f"Capturing pairs (need >= {args.min_pairs}).")
    print("  SPACE / ENTER : capture a pair")
    print("  c             : compute and save")
    print("  q             : quit")

    show_preview = not args.no_preview
    if show_preview:
        cv2.namedWindow("stereo (L | R)", cv2.WINDOW_NORMAL)

    try:
        while True:
            l_img = grab_gray(left)
            r_img = grab_gray(right)
            if l_img is None or r_img is None:
                continue

            if show_preview:
                preview = np.hstack(
                    [cv2.resize(l_img, (640, int(640 * args.height / args.width))),
                     cv2.resize(r_img, (640, int(640 * args.height / args.width)))]
                )
                cv2.putText(preview, f"pairs={len(obj_points)}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, 255, 2)
                cv2.imshow("stereo (L | R)", preview)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = ord(input("[ENTER]=capture, c=compute, q=quit: ").strip()[:1] or " ")

            if key == ord("q"):
                break
            if key == ord("c"):
                break
            if key in (ord(" "), 13):
                ok_l, c_l = find_corners(l_img, args.rows, args.cols)
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
        right.release()

    if len(obj_points) < args.min_pairs:
        print(f"Only {len(obj_points)} pairs — need >= {args.min_pairs}. Aborting.", file=sys.stderr)
        return 1

    image_size = (args.width, args.height)
    print("Calibrating left camera...")
    err_l, K_l, D_l, _, _ = cv2.calibrateCamera(obj_points, left_points, image_size, None, None)
    print(f"  left RMS reprojection error = {err_l:.3f} px")
    print("Calibrating right camera...")
    err_r, K_r, D_r, _, _ = cv2.calibrateCamera(obj_points, right_points, image_size, None, None)
    print(f"  right RMS reprojection error = {err_r:.3f} px")

    print("Calibrating stereo extrinsics...")
    err_s, K_l, D_l, K_r, D_r, R, T, _, _ = cv2.stereoCalibrate(
        obj_points, left_points, right_points,
        K_l, D_l, K_r, D_r, image_size,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    baseline = float(np.linalg.norm(T))
    print(f"  stereo RMS error = {err_s:.3f} px,  baseline = {baseline * 100:.2f} cm")

    if abs(baseline - 0.08) > 0.02:
        print(f"  WARNING: measured baseline {baseline*100:.2f} cm differs from "
              "the intended 8 cm. Check your mount.")

    output = {
        "image_width": args.width,
        "image_height": args.height,
        "left":  {"K": K_l.flatten().tolist(), "D": D_l.flatten().tolist()},
        "right": {"K": K_r.flatten().tolist(), "D": D_r.flatten().tolist()},
        "R": R.flatten().tolist(),
        "T": T.flatten().tolist(),
    }
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(output, fh, sort_keys=False)
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
