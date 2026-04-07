#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import threading
import time
import argparse
from datetime import date

PATTERN_SIZE = (9, 9)
SQUARE_SIZE = 0.070  # 70mm


def make_object_points():
    pts = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 1, 3), dtype=np.float64)
    pts[:, 0, :2] = (
        np.mgrid[0 : PATTERN_SIZE[0], 0 : PATTERN_SIZE[1]]
        .T.reshape(-1, 2)
        .astype(np.float64)
        * SQUARE_SIZE
    )
    return pts


def save_yaml(path, K, D, rms, width, height):
    cam_rows = []
    for i in range(3):
        cam_rows.append([float(K[i, j]) for j in range(3)])

    dist_data = [float(D[i, 0]) for i in range(4)]

    result = {
        "calibration_date": str(date.today()),
        "camera_matrix": {
            "columns": 3,
            "data": cam_rows,
            "rows": 3,
        },
        "chessboard": {
            "pattern_size": {
                "columns": PATTERN_SIZE[0],
                "rows": PATTERN_SIZE[1],
            },
            "square_size_meters": SQUARE_SIZE,
        },
        "distortion_coefficients": {
            "columns": 4,
            "data": dist_data,
            "rows": 1,
        },
        "image_size": {
            "height": height,
            "width": width,
        },
        "rms_reprojection_error": float(rms),
    }

    with open(path, "w") as f:
        yaml.dump(result, f, default_flow_style=False, sort_keys=True)


def main():
    parser = argparse.ArgumentParser(description="Fisheye camera calibration")
    parser.add_argument("--device", default="/dev/video0", help="V4L2 device path")
    parser.add_argument(
        "--output", default="cam_intrinsic.yaml", help="Output YAML path"
    )
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 60)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera: {args.device}")
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Camera opened: {w}x{h} @ {fps:.0f}fps")

    objp = make_object_points()
    objpoints = []
    imgpoints = []

    calibrating = False
    calib_done = False
    calib_rms = 0.0
    capture_flash_until = 0.0

    subpix_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_FAST_CHECK
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    def run_calibration():
        nonlocal calibrating, calib_done, calib_rms

        n = len(objpoints)
        print(f"\n[INFO] Starting calibration with {n} images...")
        print("[INFO] Preparing object/image points...")

        K = np.zeros((3, 3), dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]

        calib_flags = (
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            | cv2.fisheye.CALIB_CHECK_COND
            | cv2.fisheye.CALIB_FIX_SKEW
        )
        calib_criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            100,
            1e-6,
        )

        print(
            "[INFO] Running cv2.fisheye.calibrate()... (this may take a few seconds)"
        )
        t0 = time.time()

        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                (w, h),
                K,
                D,
                rvecs,
                tvecs,
                flags=calib_flags,
                criteria=calib_criteria,
            )
        except cv2.error as e:
            print(f"[ERROR] Calibration failed: {e}")
            print("[INFO] Try removing some problematic captures and retry.")
            calibrating = False
            return

        elapsed = time.time() - t0
        print(f"[INFO] Calibration complete! RMS reprojection error: {rms:.10f}")
        print(f"[INFO] Elapsed: {elapsed:.2f}s")

        print(f"[INFO] Saving to {args.output}...")
        save_yaml(args.output, K, D, rms, w, h)
        print(f"[INFO] Done. Saved to {args.output}")

        print(f"\n[INFO] Camera matrix:\n{K}")
        print(f"[INFO] Distortion coefficients:\n{D.T}")

        calib_rms = rms
        calib_done = True
        calibrating = False

    window_name = "Fisheye Calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    last_corners = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame")
            break

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, find_flags)

        if found:
            refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), subpix_criteria)
            last_corners = refined
            cv2.drawChessboardCorners(display, PATTERN_SIZE, refined, found)
            cv2.putText(
                display,
                "Chessboard Detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            last_corners = None

        cv2.putText(
            display,
            f"Captured: {len(objpoints)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        now = time.time()
        if now < capture_flash_until:
            cv2.putText(
                display,
                "Captured!",
                (w // 2 - 80, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 0),
                3,
            )

        if calibrating:
            cv2.putText(
                display,
                "Calibrating...",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 165, 255),
                2,
            )
        elif calib_done:
            cv2.putText(
                display,
                f"RMS: {calib_rms:.6f}  (saved)",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        cv2.putText(
            display,
            "SPACE: capture | C: calibrate | Q: quit",
            (10, h - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

        cv2.imshow(window_name, display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(" ") and last_corners is not None and not calibrating:
            objpoints.append(objp)
            imgpoints.append(last_corners)
            capture_flash_until = time.time() + 0.5
            print(f"[INFO] Captured frame #{len(objpoints)}")

        elif key == ord("c") and not calibrating:
            if len(objpoints) < 3:
                print("[WARN] Need at least 3 captures to calibrate")
            else:
                calibrating = True
                calib_done = False
                threading.Thread(target=run_calibration, daemon=True).start()

        elif key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
