# Pen tip camera → drawing on a “device”

This project tracks a **neon green marker** on a pen tip from a webcam (OpenCV + NumPy), maps the tip position into **absolute coordinates** `0…32767`, and sends them over TCP to a second program that **draws the user’s strokes** on a canvas.

## How to run

1. **Terminal A — device display** (must be running first so the client can connect):

   ```bash
   python test_server.py
   ```

2. **Terminal B — camera + tracking**:

   ```bash
   python main.py
   ```

3. **Controls (camera / computer window)**

   - **Space** — toggles **pen position tracking** on/off.
     - **Off:** the camera still runs, but **no pen detection**, **no overlays** for raw/smooth/absolute position, and **no coordinates** are sent to the device.
     - **On:** detection runs, overlays show positions, coordinates stream to the device, and the device **accumulates a line drawing**.
   - **q** — quit the camera app (sends a “tracking stop” to the device if tracking was on).

4. **Controls (device window)** — **q** closes the device viewer.

## How pen position is tracked (camera)

The implementation lives in **`main.py`**, function **`detect_pen_tip()`**.

1. **ROI** — Only a rectangle of the frame (`config.ROI`) is processed so the “paper” area matches your setup.
2. **Color segmentation** — The ROI is converted to HSV; **`cv2.inRange`** keeps pixels in the tuned green band (`LOWER_HSV` / `UPPER_HSV` in `config.py`).
3. **Mask cleanup** — Morphological **open/close** reduces noise.
4. **Contours** — **`cv2.findContours`** finds blobs; area and aspect ratio filters reject non-pen regions.
5. **Merge split blobs** — Nearby contours (e.g. partial occlusion) are merged before analysis.
6. **Tip point** — For a round marker, the **centroid** is used; for elongated blobs, **PCA** (`cv2.PCACompute`) finds the pen axis and the tip is chosen between **endpoints**, using **previous frames** for stable continuity.
7. **Smoothing** — **`apply_ema()`** applies exponential moving average on pixel coordinates before mapping.

Mapping to the wire format uses **`frame_point_to_absolute()`**: ROI pixel \((x, y)\) is linearly mapped to **uint16** `0…32767` for both axes.

## How position data is collected and sent

- While **tracking is on** (`main.py` main loop), each frame may produce a smoothed point; **`frame_point_to_absolute()`** converts it to `(abs_x, abs_y)`.
- **`driver_client.py`** sends **5-byte** TCP packets: `struct.pack("!BHH", cmd, x, y)`:
  - **`cmd == 0`** (`CMD_MOVE`) — pen moved to `(x, y)` in `0…32767`.
  - **`cmd == 2`** (`CMD_TRACKING_START`) — new drawing session (device clears canvas).
  - **`cmd == 1`** (`CMD_TRACKING_STOP`) — user stopped tracking; device **lifts the pen** (next move won’t connect with a line from the old point).

Connection settings: **`config.DRIVER_HOST`**, **`config.DRIVER_PORT`**.

## How the user sees the drawing (device)

**`test_server.py`** accepts the same TCP stream and builds a **white canvas** (`DEVICE_CANVAS_WIDTH` × `DEVICE_CANVAS_HEIGHT` in `config.py`).

- On **`CMD_TRACKING_START`**, the canvas is cleared.
- On each **`CMD_MOVE`**, \((x, y)\) in `0…32767` is scaled to pixel coordinates on the canvas; **`cv2.line`** connects consecutive points so the path matches the pen motion.
- On **`CMD_TRACKING_STOP`**, the last point is cleared so the next stroke does not connect across a gap.

## Files

| File | Role |
|------|------|
| `main.py` | Camera capture, pen detection, smoothing, optional debug windows, **Space** toggles tracking |
| `config.py` | Camera index, ROI, HSV thresholds, smoothing, TCP host/port, device canvas size |
| `driver_client.py` | TCP client: move + start/stop commands |
| `test_server.py` | TCP server + OpenCV window: **live drawing** |
| `run.py` | Helper to discover which camera index works (macOS AVFoundation) |

## Dependencies

See **`requirements.txt`** (`opencv-python`, `numpy`).
