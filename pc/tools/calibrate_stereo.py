"""Stereo calibration tool — captures chessboard pairs from the Pi and calibrates.

Based on Stephane Vujasinovic / Frederic Uhrweiller (2017).

Usage:
    python pc/tools/calibrate_stereo.py [--square 0.025] [--rows 6] [--cols 9]
    python pc/tools/calibrate_stereo.py --gen-board [--out chessboard.png]

Capture phase (live MJPEG from Pi):
    s   save current pair (only when chessboard detected in both views)
    c   skip current frame
    q   stop capturing and run calibration
    SPACE  same as q

Output: writes pi/config/stereo_calibration.yaml (overwrites). Saved chessboard
images go in pc/tools/calib_images/.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pc"))

from xray_pc.camera_client import MJPEGClient


def load_pi_config():
    cfg_path = ROOT / "pc" / "config" / "pc_config.yaml"
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    return cfg["pi"]["ip"], int(cfg["pi"]["port"])


def capture_pairs(left_url, right_url, save_dir, pattern_size):
    save_dir.mkdir(parents=True, exist_ok=True)

    print(f"[capture] left:  {left_url}")
    print(f"[capture] right: {right_url}")
    print("Push (s) to save the image you want and push (c) to see next frame without saving.")
    print("Push (q) or SPACE to stop capturing and run calibration.\n")

    CamL = MJPEGClient(left_url)
    CamR = MJPEGClient(right_url)
    CamL.start()
    CamR.start()

    # wait for first frame
    deadline = time.time() + 10.0
    while time.time() < deadline:
        l, _ = CamL.get_frame()
        r, _ = CamR.get_frame()
        if l is not None and r is not None:
            break
        time.sleep(0.1)
    else:
        CamL.stop(); CamR.stop()
        raise RuntimeError("Timed out waiting for MJPEG frames from Pi")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    id_image = 0

    try:
        while True:
            frameL, _ = CamL.get_frame()
            frameR, _ = CamR.get_frame()
            if frameL is None or frameR is None:
                time.sleep(0.05)
                continue

            grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
            grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

            retL, cornersL = cv2.findChessboardCorners(grayL, pattern_size, None)
            retR, cornersR = cv2.findChessboardCorners(grayR, pattern_size, None)

            visL = frameL.copy()
            visR = frameR.copy()
            both_found = retL and retR

            if both_found:
                corners2L = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
                corners2R = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(visL, pattern_size, corners2L, retL)
                cv2.drawChessboardCorners(visR, pattern_size, corners2R, retR)

            status = f"saved={id_image}  found={'YES' if both_found else 'no'}"
            cv2.putText(visL, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Left", visL)
            cv2.imshow("Right", visR)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and both_found:
                str_id = str(id_image)
                cv2.imwrite(str(save_dir / f"chessboard-L{str_id}.png"), frameL)
                cv2.imwrite(str(save_dir / f"chessboard-R{str_id}.png"), frameR)
                print(f"[capture] saved pair {str_id}")
                id_image += 1
            elif key == ord('s') and not both_found:
                print("[capture] chessboard not detected in both views — not saving")
            elif key == ord('c'):
                pass
            elif key == ord('q') or key == ord(' '):
                break
    finally:
        CamL.stop()
        CamR.stop()
        cv2.destroyAllWindows()

    return id_image


def calibrate(save_dir, num_pairs, pattern_size, square_size_m):
    rows, cols = pattern_size[1], pattern_size[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m  # scale to metric so baseline T comes out in meters

    objpoints = []
    imgpointsL = []
    imgpointsR = []

    image_size = None
    print(f"\n[calibrate] processing {num_pairs} pairs...")
    for i in range(num_pairs):
        t = str(i)
        ImaL = cv2.imread(str(save_dir / f"chessboard-L{t}.png"), 0)
        ImaR = cv2.imread(str(save_dir / f"chessboard-R{t}.png"), 0)
        if ImaL is None or ImaR is None:
            print(f"  [skip] pair {t}: missing file")
            continue

        if image_size is None:
            image_size = ImaL.shape[::-1]  # (w, h)

        retL, cornersL = cv2.findChessboardCorners(ImaL, pattern_size, None)
        retR, cornersR = cv2.findChessboardCorners(ImaR, pattern_size, None)

        if retL and retR:
            objpoints.append(objp)
            cv2.cornerSubPix(ImaL, cornersL, (11, 11), (-1, -1), criteria)
            cv2.cornerSubPix(ImaR, cornersR, (11, 11), (-1, -1), criteria)
            imgpointsL.append(cornersL)
            imgpointsR.append(cornersR)
        else:
            print(f"  [skip] pair {t}: corners not found on reload")

    if len(objpoints) < 6:
        raise RuntimeError(f"Need at least 6 valid pairs, got {len(objpoints)}")

    print(f"[calibrate] {len(objpoints)} usable pairs, image size {image_size}")

    # Per-camera calibration
    retL, mtxL, distL, _, _ = cv2.calibrateCamera(objpoints, imgpointsL, image_size, None, None)
    retR, mtxR, distR, _, _ = cv2.calibrateCamera(objpoints, imgpointsR, image_size, None, None)
    print(f"[calibrate] left RMS={retL:.4f}  right RMS={retR:.4f}")

    # Stereo calibration
    criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    retS, MLS, dLS, MRS, dRS, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpointsL, imgpointsR,
        mtxL, distL, mtxR, distR,
        image_size,
        criteria=criteria_stereo,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    print(f"[calibrate] stereo RMS={retS:.4f}  baseline={abs(T[0, 0]):.4f}m")

    # Rectify + maps (sanity check; runtime recomputes from YAML)
    rectify_scale = 0
    RL, RR, PL, PR, Q, _, _ = cv2.stereoRectify(
        MLS, dLS, MRS, dRS, image_size, R, T, rectify_scale, (0, 0)
    )
    Left_Stereo_Map = cv2.initUndistortRectifyMap(MLS, dLS, RL, PL, image_size, cv2.CV_16SC2)
    Right_Stereo_Map = cv2.initUndistortRectifyMap(MRS, dRS, RR, PR, image_size, cv2.CV_16SC2)
    print(f"[calibrate] maps built: left={Left_Stereo_Map[0].shape} right={Right_Stereo_Map[0].shape}")

    return {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "left":  {"K": MLS.flatten().tolist(), "D": dLS.flatten().tolist()},
        "right": {"K": MRS.flatten().tolist(), "D": dRS.flatten().tolist()},
        "R": R.flatten().tolist(),
        "T": T.flatten().tolist(),
    }


def write_yaml(data, out_path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
    print(f"[write] wrote {out_path}")


def generate_chessboard(rows=6, cols=9, square_px=50, out_path="chessboard.png"):
    """Generate a printable chessboard pattern."""
    h = rows * square_px
    w = cols * square_px
    board = np.zeros((h, w), dtype=np.uint8)

    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 1:
                y_start = r * square_px
                y_end = (r + 1) * square_px
                x_start = c * square_px
                x_end = (c + 1) * square_px
                board[y_start:y_end, x_start:x_end] = 255

    cv2.imwrite(out_path, board)
    print(f"[gen-board] wrote {out_path} ({w}×{h}px, {rows}×{cols} inner corners)")
    print(f"[gen-board] print at ~{w/100:.1f}cm × {h/100:.1f}cm for 50px/cm resolution")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--square", type=float, default=0.025,
                    help="chessboard square size in meters (default 0.025)")
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row (default 9)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column (default 6)")
    ap.add_argument("--skip-capture", action="store_true",
                    help="skip capture, just calibrate from existing images")
    ap.add_argument("--num-pairs", type=int, default=None,
                    help="when --skip-capture, how many pairs to load (default: auto-detect)")
    ap.add_argument("--gen-board", action="store_true",
                    help="generate a chessboard PNG and exit")
    ap.add_argument("--out", type=str, default="chessboard.png",
                    help="output path for --gen-board (default chessboard.png)")
    args = ap.parse_args()

    if args.gen_board:
        generate_chessboard(rows=args.rows, cols=args.cols, out_path=args.out)
        return

    pattern_size = (args.cols, args.rows)
    save_dir = Path(__file__).parent / "calib_images"
    out_yaml = ROOT / "pi" / "config" / "stereo_calibration.yaml"

    if args.skip_capture:
        if args.num_pairs is not None:
            num_pairs = args.num_pairs
        else:
            existing = sorted(save_dir.glob("chessboard-L*.png"))
            num_pairs = len(existing)
            print(f"[main] auto-detected {num_pairs} existing pairs in {save_dir}")
    else:
        ip, port = load_pi_config()
        base = f"http://{ip}:{port}"
        num_pairs = capture_pairs(
            f"{base}/video/right.mjpg",
            f"{base}/video/left.mjpg",
            save_dir, pattern_size,
        )
        if num_pairs == 0:
            print("[main] no pairs captured — aborting")
            sys.exit(1)

    data = calibrate(save_dir, num_pairs, pattern_size, args.square)
    write_yaml(data, out_yaml)
    print("\n[done] calibration complete. Restart pc/main.py to pick up new YAML.")


if __name__ == "__main__":
    main()
