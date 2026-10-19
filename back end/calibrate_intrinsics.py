"""One-time camera intrinsics calibration for the workspace webcam.

Removes lens distortion from the calibration pipeline (calibration.py picks the
result up automatically on the next homography fit).

Usage (close the web app tab first — the browser holds the camera):

    python calibrate_intrinsics.py [--camera 0] [--cols 9] [--rows 6] [--square-mm 24]

1. Print a checkerboard (e.g. a standard 9x6 INNER-corner board; any calib.io
   or OpenCV pattern works). Tape it to something flat.
2. Run this script. A preview window opens. Move the board around the view:
   near/far, all four corners, tilted. The script auto-captures a frame each
   time the board is found in a sufficiently new pose, until it has 20.
3. It calibrates, prints the reprojection RMS (aim for < 0.5 px), and writes
   camera_intrinsics.json next to this script.

Then re-run the homography fit in the web app (Calibrate camera -> Fit) so the
new intrinsics are baked into the pixel mapping.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

BACKEND_ROOT = Path(__file__).resolve().parent
OUTPUT_PATH = BACKEND_ROOT / "camera_intrinsics.json"

TARGET_VIEWS = 20
MIN_POSE_SPREAD_PX = 35  # a new view must move the board center this far


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0, help="camera index (default 0)")
    parser.add_argument("--cols", type=int, default=9, help="inner corners per row (default 9)")
    parser.add_argument("--rows", type=int, default=6, help="inner corners per column (default 6)")
    parser.add_argument("--square-mm", type=float, default=24.0, help="square size in mm (scale only)")
    args = parser.parse_args()

    pattern = (args.cols, args.rows)
    capture = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not capture.isOpened():
        print("ERROR: could not open the camera. Close the web app tab (the browser")
        print("holds the camera exclusively) and try again.")
        return 1

    # Board model: (0,0,0), (1,0,0)... scaled by the square size.
    object_template = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    object_template[:, :2] = np.mgrid[0 : pattern[0], 0 : pattern[1]].T.reshape(-1, 2)
    object_template *= args.square_mm

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    captured_centers: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    last_capture_at = 0.0

    print(f"Show the {pattern[0]}x{pattern[1]} checkerboard. Capturing {TARGET_VIEWS} varied views...")
    print("Keys: q = quit early (calibrates with what it has, min 10), ESC = abort.")

    while len(object_points) < TARGET_VIEWS:
        ok, frame = capture.read()
        if not ok:
            print("ERROR: camera stopped delivering frames.")
            return 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        display = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            cv2.drawChessboardCorners(display, pattern, corners, found)
            center = corners.mean(axis=0).ravel()
            fresh_pose = all(
                np.linalg.norm(center - previous) > MIN_POSE_SPREAD_PX
                for previous in captured_centers
            )
            settled = time.monotonic() - last_capture_at > 0.6
            if fresh_pose and settled:
                object_points.append(object_template.copy())
                image_points.append(corners)
                captured_centers.append(center)
                last_capture_at = time.monotonic()
                print(f"  captured view {len(object_points)}/{TARGET_VIEWS}")

        cv2.putText(
            display,
            f"views: {len(object_points)}/{TARGET_VIEWS}  (move the board to a new spot/tilt)",
            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 220, 60), 2,
        )
        cv2.imshow("Intrinsics calibration", display)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Aborted.")
            return 1
        if key == ord("q"):
            break

    capture.release()
    cv2.destroyAllWindows()

    if len(object_points) < 10:
        print(f"ERROR: only {len(object_points)} views captured; need at least 10.")
        return 1

    print(f"Calibrating from {len(object_points)} views...")
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    print(f"Reprojection RMS: {rms:.3f} px  (under 0.5 px is good, under 1.0 px acceptable)")

    OUTPUT_PATH.write_text(
        json.dumps(
            {
                "camera_matrix": camera_matrix.tolist(),
                "dist_coeffs": dist_coeffs.ravel().tolist(),
                "image_size": list(image_size),
                "reprojection_rms_px": rms,
                "views": len(object_points),
                "pattern": list(pattern),
                "square_mm": args.square_mm,
                "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"Saved {OUTPUT_PATH}")
    print("Now re-run the homography fit in the web app so the intrinsics take effect.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
