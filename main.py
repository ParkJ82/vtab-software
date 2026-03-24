# main.py

import cv2
import numpy as np
from typing import Optional, Tuple

import config
from driver_client import DriverClient


Point = Tuple[int, int]


def clamp(value: float, min_value: int, max_value: int) -> int:
    return max(min_value, min(max_value, int(value)))


def map_range(value: float, in_min: float, in_max: float, out_min: int, out_max: int) -> int:
    if in_max == in_min:
        return out_min
    ratio = (value - in_min) / (in_max - in_min)
    mapped = out_min + ratio * (out_max - out_min)
    return clamp(mapped, out_min, out_max)


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
        if area < config.MIN_CONTOUR_AREA or area > config.MAX_CONTOUR_AREA:
            continue

        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        if w <= 1 or h <= 1:
            continue

        aspect_ratio = max(w, h) / min(w, h)
        if aspect_ratio < config.MIN_PEN_ASPECT_RATIO:
            continue

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


def draw_debug(frame: np.ndarray, raw_point: Optional[Point], smooth_point: Optional[Point], abs_point: Optional[Point]) -> np.ndarray:
    output = frame.copy()

    # Draw ROI
    x1, y1, x2, y2 = config.ROI
    cv2.rectangle(output, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(output, "ROI", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    if raw_point is not None:
        cv2.circle(output, raw_point, 7, (0, 0, 255), -1)
        cv2.putText(
            output,
            f"raw=({raw_point[0]}, {raw_point[1]})",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    if smooth_point is not None:
        cv2.circle(output, smooth_point, 8, (0, 255, 0), 2)
        cv2.putText(
            output,
            f"smooth=({smooth_point[0]}, {smooth_point[1]})",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    if abs_point is not None:
        cv2.putText(
            output,
            f"absolute=({abs_point[0]}, {abs_point[1]})",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
    else:
        cv2.putText(
            output,
            "TIP NOT DETECTED",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            3,
        )

    cv2.putText(
        output,
        "Press q to quit",
        (20, output.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
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
    cap = setup_camera()

    driver = DriverClient(config.DRIVER_HOST, config.DRIVER_PORT)
    driver.connect()

    smoothed_point: Optional[Point] = None
    previous_raw_tip: Optional[Point] = None
    previous_previous_raw_tip: Optional[Point] = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            raw_point, mask = detect_pen_tip(frame, previous_raw_tip, previous_previous_raw_tip)

            abs_point: Optional[Point] = None

            if raw_point is not None:
                previous_previous_raw_tip = previous_raw_tip
                previous_raw_tip = raw_point
                smoothed_point = apply_ema(smoothed_point, raw_point, config.SMOOTHING_ALPHA)
                abs_point = frame_point_to_absolute(smoothed_point)

                # Send immediately to driver
                driver.send_coordinates(abs_point[0], abs_point[1])
            else:
                # Optional behavior:
                # keep previous smooth point, or clear it
                smoothed_point = None
                previous_raw_tip = None
                previous_previous_raw_tip = None

            if config.SHOW_DEBUG:
                debug_frame = draw_debug(frame, raw_point, smoothed_point, abs_point)
                cv2.imshow("Pen Tip Detection", debug_frame)

            if config.SHOW_MASK:
                cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        cap.release()
        driver.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()