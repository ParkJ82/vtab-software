import cv2
import numpy as np
import subprocess
import sys
import time
from typing import Optional, Tuple
import math

import config
from gyro_reader import GyroReader

EXECUTABLE_PATH = r"C:\Users\chrsf\OneDrive\Desktop\sldptest\testvhid.exe"


Point = Tuple[int, int]


def clamp(value: float, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, int(value)))


def map_range(value: float, in_min: float, in_max: float, out_min: int, out_max: int) -> int:
    if in_max == in_min:
        return out_min
    ratio = (value - in_min) / (in_max - in_min)
    mapped = out_min + ratio * (out_max - out_min)
    return clamp(mapped, out_min, out_max)

def _tip_from_sharpest_hull_vertex(
    hull: np.ndarray,
    apex_max_rad: float,
    prev_local: Optional[np.ndarray],
) -> Optional[Tuple[int, int]]:
    """
    On a triangular (or kite-like) blue blob, the geometric tip is the sharpest convex-hull vertex
    (smallest apex angle), not the centroid or the PCA segment midpoint.
    Returns ROI-local (x, y) or None if corners are all blunt (e.g. square ~90°).
    """
    pts = hull.reshape(-1, 2).astype(np.float64)
    n = pts.shape[0]
    if n < 3:
        return None

    angles: list[float] = []
    for i in range(n):
        p = pts[i]
        v1 = pts[(i - 1) % n] - p
        v2 = pts[(i + 1) % n] - p
        n1 = float(np.linalg.norm(v1))
        n2 = float(np.linalg.norm(v2))
        if n1 < 1e-6 or n2 < 1e-6:
            angles.append(math.pi)
            continue
        c = float(np.dot(v1, v2) / (n1 * n2))
        c = max(-1.0, min(1.0, c))
        angles.append(float(np.arccos(c)))

    min_ang = min(angles)
    if min_ang >= apex_max_rad:
        return None

    eps = 0.03
    cand_idx = [i for i, a in enumerate(angles) if a <= min_ang + eps]
    if not cand_idx:
        cand_idx = [int(np.argmin(angles))]

    if prev_local is not None:
        best_i = min(
            cand_idx,
            key=lambda i: float(np.linalg.norm(pts[i] - prev_local)),
        )
    else:
        best_i = max(cand_idx, key=lambda i: float(pts[i, 1]))

    return int(round(pts[best_i, 0])), int(round(pts[best_i, 1]))

def apply_ema(previous: Optional[Point], current: Point, alpha: float) -> Point:
    if previous is None:
        return current

    px, py = previous
    cx, cy = current

    smoothed_x = alpha * cx + (1.0 - alpha) * px
    smoothed_y = alpha * cy + (1.0 - alpha) * py

    return int(smoothed_x), int(smoothed_y)


def detect_pen_tip(
    frame: np.ndarray,
    previous_tip: Optional[Point] = None,
    previous_previous_tip: Optional[Point] = None,
) -> Tuple[Optional[Point], np.ndarray]:
    """
    Detect pen tip using HSV thresholding and contour endpoint selection.
    Returns:
        detected_point in full-frame coordinates, or None
        mask used for detection
    """
    x1, y1, x2, y2 = config.ROI
    roi_frame = frame[y1:y2, x1:x2]

    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

    lower = np.array(config.LOWER_HSV, dtype=np.uint8)
    upper = np.array(config.UPPER_HSV, dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)
    if hasattr(config, "LOWER_HSV_2") and hasattr(config, "UPPER_HSV_2"):
        lower2 = np.array(config.LOWER_HSV_2, dtype=np.uint8)
        upper2 = np.array(config.UPPER_HSV_2, dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))

    # Clean the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    candidate_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 1.0:
            continue

        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        if w <= 1 or h <= 1:
            continue

        aspect_ratio = max(w, h) / min(w, h)

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        candidate_contours.append((contour, np.array([cx, cy], dtype=np.float32), area, aspect_ratio))

    if not candidate_contours:
        return None, mask

    # Choose a seed contour, then merge nearby candidate contours so split pen
    # blobs (from finger occlusion) are treated as a single pen.
    if previous_tip is not None:
        prev_local = np.array([previous_tip[0] - x1, previous_tip[1] - y1], dtype=np.float32)
        seed_idx = int(np.argmin([float(np.linalg.norm(c[1] - prev_local)) for c in candidate_contours]))
    else:
        seed_idx = int(np.argmax([c[3] + 0.001 * c[2] for c in candidate_contours]))

    seed_center = candidate_contours[seed_idx][1]
    merged_points = []
    for contour, center, _, _ in candidate_contours:
        if float(np.linalg.norm(center - seed_center)) <= config.MERGE_CONTOUR_DISTANCE:
            merged_points.append(contour.reshape(-1, 2))

    if not merged_points:
        return None, mask

    merged_pts = np.vstack(merged_points).astype(np.float32)
    if merged_pts.shape[0] < 5:
        return None, mask

    best_contour = cv2.convexHull(merged_pts.astype(np.int32))

    apex_max = float(getattr(config, "TIP_APEX_MAX_RAD", 1.40))
    prev_local_for_tip: Optional[np.ndarray] = None
    if previous_tip is not None:
        prev_local_for_tip = np.array(
            [previous_tip[0] - x1, previous_tip[1] - y1], dtype=np.float32
        )

    sharp = _tip_from_sharpest_hull_vertex(best_contour, apex_max, prev_local_for_tip)
    if sharp is not None:
        return (x1 + sharp[0], y1 + sharp[1]), mask

    # If the selected blob is roughly round (typical green tip marker),
    # use centroid directly; endpoint logic is better for elongated pen-body blobs.
    marker_rect = cv2.minAreaRect(best_contour)
    mw, mh = marker_rect[1]
    if mw > 1 and mh > 1:
        marker_aspect_ratio = max(mw, mh) / min(mw, mh)
        if marker_aspect_ratio < 1.6:
            moments = cv2.moments(best_contour)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                return (x1 + cx, y1 + cy), mask

    # Get pen axis via PCA and use contour extremes on that axis
    # as pen-end candidates; choose one consistently over time.
    pts = best_contour.reshape(-1, 2).astype(np.float32)
    if pts.shape[0] < 5:
        return None, mask

    mean, eigenvectors = cv2.PCACompute(pts, mean=None)
    if mean is None or eigenvectors is None or len(eigenvectors) == 0:
        return None, mask

    axis = eigenvectors[0]
    norm = float(np.linalg.norm(axis))
    if norm == 0:
        return None, mask
    axis = axis / norm

    projections = pts @ axis
    min_idx = int(np.argmin(projections))
    max_idx = int(np.argmax(projections))
    end_a = pts[min_idx]
    end_b = pts[max_idx]

    if previous_tip is None:
        # First frame: bias toward lower part of image (writing side).
        tip_local = end_a if end_a[1] >= end_b[1] else end_b
    else:
        prev_local = np.array([previous_tip[0] - x1, previous_tip[1] - y1], dtype=np.float32)
        dist_a = float(np.linalg.norm(end_a - prev_local))
        dist_b = float(np.linalg.norm(end_b - prev_local))

        if previous_previous_tip is None:
            tip_local = end_a if dist_a <= dist_b else end_b
        else:
            prev_prev_local = np.array(
                [previous_previous_tip[0] - x1, previous_previous_tip[1] - y1], dtype=np.float32
            )
            motion = prev_local - prev_prev_local
            motion_norm = float(np.linalg.norm(motion))

            if motion_norm < 1.0:
                # If movement is tiny, continuity is more reliable than direction.
                tip_local = end_a if dist_a <= dist_b else end_b
            else:
                motion_unit = motion / motion_norm
                delta_a = end_a - prev_local
                delta_b = end_b - prev_local
                align_a = float(np.dot(delta_a, motion_unit))
                align_b = float(np.dot(delta_b, motion_unit))

                # Motion-aware score:
                # - prefer continuity (small distance)
                # - prefer endpoint that follows recent writing direction
                score_a = dist_a - 0.4 * align_a
                score_b = dist_b - 0.4 * align_b
                tip_local = end_a if score_a <= score_b else end_b

    # Convert ROI-local coords back to full-frame coords
    full_x = x1 + int(tip_local[0])
    full_y = y1 + int(tip_local[1])

    return (full_x, full_y), mask


def frame_point_to_absolute(point: Point) -> Point:
    """
    Convert frame pixel coordinates inside ROI to absolute 0..32767 coordinates.
    """
    px, py = point
    x1, y1, x2, y2 = config.ROI

    abs_x = map_range(px, x1, x2, config.ABSOLUTE_MIN, config.ABSOLUTE_MAX)
    abs_y = map_range(py, y1, y2, config.ABSOLUTE_MIN, config.ABSOLUTE_MAX)

    return abs_x, abs_y


def draw_debug(
    frame: np.ndarray,
    writing_active: bool,
    raw_point: Optional[Point],
    smooth_point: Optional[Point],
    abs_point: Optional[Point],
    gyro_sample: Optional[Tuple[float, float, float]] = None,
) -> np.ndarray:
    output = frame.copy()

    # Draw ROI
    x1, y1, x2, y2 = config.ROI
    cv2.rectangle(output, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(output, "ROI", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    dev = "DEVICE (send strokes): ON" if writing_active else "DEVICE (send strokes): OFF"
    color = (0, 255, 0) if writing_active else (0, 165, 255)
    cv2.putText(output, dev, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    cv2.putText(
        output,
        "Tip detection: always on",
        (20, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (200, 200, 200),
        1,
    )

    if not writing_active:
        cv2.putText(
            output,
            "Press SPACE to start pen tracking",
            (20, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            output,
            "Pen + gyro not sent; camera overlay hidden",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            2,
        )
    if raw_point is not None:
        cv2.circle(output, raw_point, 7, (0, 0, 255), -1)
        cv2.putText(
            output,
            f"raw=({raw_point[0]}, {raw_point[1]})",
            (20, 118),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )

    if smooth_point is not None:
        cv2.circle(output, smooth_point, 8, (0, 255, 0), 2)
        cv2.putText(
            output,
            f"smooth=({smooth_point[0]}, {smooth_point[1]})",
            (20, 148),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )

    if abs_point is not None:
        cv2.putText(
            output,
            f"absolute=({abs_point[0]}, {abs_point[1]})",
            (20, 178),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
    else:
        cv2.putText(
            output,
            "TIP NOT DETECTED",
            (20, 178),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 0, 255),
            2,
        )

    if writing_active and gyro_sample is not None:
        gx, gy, gz = gyro_sample
        cv2.putText(
            output,
            f"gyro (rad/s): ({gx:.4f}, {gy:.4f}, {gz:.4f})",
            (20, 208),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 255),
            2,
        )
    elif writing_active and config.GYRO_MODE == "none":
        cv2.putText(
            output,
            "Gyro: not used (GYRO_MODE=none)",
            (20, 208),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (140, 140, 140),
            2,
        )

    cv2.putText(
        output,
        "Space: toggle devicewriting | q: quit",
        (20, output.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )

    return output


def setup_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(config.CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera. Check CAMERA_INDEX in config.py and make sure your iPhone webcam is connected."
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

    return cap


def main() -> None:
    proc: Optional[subprocess.Popen] = None
    if config.DRIVER_ENABLED:
        print("Starting C HID interface...")
        try:
            proc = subprocess.Popen(
                [EXECUTABLE_PATH],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except FileNotFoundError:
            print(f"Error: Could not find executable at {EXECUTABLE_PATH}")
            sys.exit(1)

    time.sleep(0.5)

    cap: Optional[cv2.VideoCapture] = None
    if config.CAMERA_ENABLED:
        try:
            cap = setup_camera()
        except RuntimeError as e:
            print(f"[camera] {e}")
            print("[camera] Continuing in IMU-only mode.")
            if proc is not None and proc.stdin is not None:
                proc.stdin.close()
                proc.wait()
            cap = None

    gyro = GyroReader()
    writing_active = False
    smoothed_point: Optional[Point] = None
    previous_raw_tip: Optional[Point] = None
    previous_previous_raw_tip: Optional[Point] = None
    last_terminal_print_t = 0.0

    try:
        if cap is None:
            gyro.start()
            print("[imu] IMU-only mode (camera disabled/unavailable). Press Ctrl+C to stop.")
            while True:
                imu_sample = gyro.read_imu()
                now = time.perf_counter()
                if now - last_terminal_print_t >= 0.1:
                    last_terminal_print_t = now
                    if imu_sample is None:
                        imu_s = "imu=NA"
                    else:
                        ax, ay, az, gx, gy, gz = imu_sample
                        imu_s = f"imu=ax,ay,az,gx,gy,gz=({ax},{ay},{az},{gx},{gy},{gz})"
                    print(f"abs=NA tip=NA {imu_s}", flush=True)
                time.sleep(0.01)

        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            raw_point: Optional[Point] = None
            abs_point: Optional[Point] = None
            x1, y1, x2, y2 = config.ROI
            mask_h, mask_w = y2 - y1, x2 - x1
            mask = np.zeros((mask_h, mask_w), dtype=np.uint8)

            raw_point, mask = detect_pen_tip(frame, previous_raw_tip, previous_previous_raw_tip)

            if raw_point is not None:
                previous_previous_raw_tip = previous_raw_tip
                previous_raw_tip = raw_point
                # Use only raw tip coordinates (no EMA smoothing).
                smoothed_point = None
                abs_point = frame_point_to_absolute(raw_point)
                if writing_active and proc is not None and proc.stdin is not None:
                    try:
                        proc.stdin.write(f"{abs_point[0]} {abs_point[1]}\n")
                        proc.stdin.flush()
                    except BrokenPipeError:
                        print("Error: The C program terminated unexpectedly.")
                        proc = None
            else:
                smoothed_point = None
                previous_raw_tip = None
                previous_previous_raw_tip = None
                abs_point = None

            gyro_sample: Optional[Tuple[float, float, float]] = None
            imu_sample = None
            if writing_active:
                imu_sample = gyro.read_imu()
                gyro_sample = gyro.read()

            now = time.perf_counter()
            if now - last_terminal_print_t >= 0.1:
                last_terminal_print_t = now
                if abs_point is None:
                    tip_s = "tip=NOT_DETECTED"
                    abs_s = "abs=NA"
                else:
                    tip_s = "tip=OK"
                    abs_s = f"abs=({abs_point[0]},{abs_point[1]})"

                if writing_active:
                    if imu_sample is None:
                        imu_s = "imu=NA"
                    else:
                        ax, ay, az, gx, gy, gz = imu_sample
                        imu_s = f"imu=ax,ay,az,gx,gy,gz=({ax},{ay},{az},{gx},{gy},{gz})"
                    print(f"{abs_s} {tip_s} {imu_s} write=ON", flush=True)
                else:
                    print(f"{abs_s} {tip_s} imu=— write=OFF", flush=True)

            if config.SHOW_DEBUG:
                debug_frame = draw_debug(
                    frame, writing_active, raw_point, smoothed_point, abs_point, gyro_sample
                )
                cv2.imshow("Pen Tip Detection", debug_frame)

            if config.SHOW_MASK:
                cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                writing_active = not writing_active
                if writing_active:
                    gyro.start()
                else:
                    gyro.stop()
            elif key == ord("q"):
                break

    finally:
        if cap is not None:
            cap.release()
        gyro.stop()
        if proc is not None and proc.poll() is None and proc.stdin is not None:
            proc.stdin.close()
            proc.wait()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()