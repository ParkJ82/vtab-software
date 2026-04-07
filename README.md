# Pen tip camera → drawing on a “device”

This project tracks a **green pen tip** (black pen body, brown background in frame) using HSV color segmentation from a webcam (OpenCV + NumPy), maps the tip position into **absolute coordinates** `0…32767`, and sends them over TCP to a second program that **draws** on a **brown canvas with black strokes** (device window).

The **same Space bar** also starts and stops **gyroscope sampling** (see `gyro_reader.py` and `config.GYRO_MODE`). When tracking is off, **pen overlays and gyro readouts on the computer** stop; the device does not receive pen coordinates until you turn tracking on again.

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

   - **Space** — toggles **pen position tracking** and **gyroscope reading** together.
     - **Off:** the camera still runs, but **no pen detection**, **no pen overlays** (raw / smooth / absolute), **no gyro on-screen**, and **no pen coordinates** are sent to the device.
     - **On:** camera runs `detect_pen_tip`, **gyro** is sampled per `GYRO_MODE` in `config.py`, overlays show pen + gyro on the computer, and pen coordinates stream to the device so it **draws the stroke**.
   - **q** — quit the camera app (sends a “tracking stop” to the device if tracking was on).

4. **Controls (device window)** — **q** closes the device viewer.

## Gyroscope (starts/stops with Space)

- **Code:** `gyro_reader.py` — `GyroReader.start()` / `stop()` are called when you toggle tracking; `read()` returns angular rates when tracking is on.
- **Modes** (`config.py`):
  - **`none`** — no gyro samples (no line on the debug view except a note).
  - **`mock`** — synthetic sine/cosine rates (no hardware) so you can demo the flow.
  - **`serial`** — read lines like `gx gy gz` or `gx,gy,gz` from `GYRO_SERIAL_PORT` (requires `pyserial`).
- Gyro is shown on the **computer** debug window only while tracking is on. **Pen drawing on the device** still uses **only** the camera-derived `(x, y)` stream.

## How pen position is tracked (camera)

The implementation lives in **`main.py`**, function **`detect_pen_tip()`**.

1. **ROI** — Only a rectangle of the frame (`config.ROI`) is processed so the “paper” area matches your setup.
2. **Color segmentation** — The ROI is converted to HSV; **`cv2.inRange`** keeps pixels in the tuned **green tip** band (`LOWER_HSV` / `UPPER_HSV` in `config.py`). Adjust if your lighting or green differs.
3. **Mask cleanup** — Morphological **open/close** reduces noise.
4. **Contours** — **`cv2.findContours`** finds blobs; area and aspect ratio filters reject non-pen regions.
5. **Merge split blobs** — Nearby contours (e.g. partial occlusion) are merged before analysis.
6. **Tip point** — For a round marker, the **centroid** is used; for elongated blobs, **PCA** (`cv2.PCACompute`) finds the pen axis and the tip is chosen between **endpoints**, using **previous frames** for stable continuity.
7. **Smoothing** — **`apply_ema()`** applies exponential moving average on pixel coordinates before mapping.

Mapping to the wire format uses **`frame_point_to_absolute()`**: ROI pixel \((x, y)\) is linearly mapped to **uint16** `0…32767` for both axes.

## How position data is collected and sent

- While **tracking is on** (`main.py` main loop), each frame may produce a smoothed point; **`frame_point_to_absolute()`** converts it to `(abs_x, abs_y)`.
- Each frame while tracking also calls **`gyro.read()`** when `GYRO_MODE` is not `none` (see `gyro_reader.py`); values are **displayed locally** on the OpenCV window, not sent over the same pen packet (the assignment’s “drawing” is pen position on the device).
- **`driver_client.py`** sends **5-byte** TCP packets: `struct.pack("!BHH", cmd, x, y)`:
  - **`cmd == 0`** (`CMD_MOVE`) — pen moved to `(x, y)` in `0…32767`.
  - **`cmd == 2`** (`CMD_TRACKING_START`) — resume drawing (device does **not** clear; it just lifts the pen).
  - **`cmd == 1`** (`CMD_TRACKING_STOP`) — user stopped tracking; device **lifts the pen** (next move won’t connect with a line from the old point).

Connection settings: **`config.DRIVER_HOST`**, **`config.DRIVER_PORT`**.

## How the user sees the drawing (device)

**`test_server.py`** accepts the same TCP stream and builds a **brown “paper” canvas** with **black stroke** (`DEVICE_CANVAS_BG_BGR` / `DEVICE_STROKE_BGR`, size `DEVICE_CANVAS_WIDTH` × `DEVICE_CANVAS_HEIGHT` in `config.py`).

- On **`CMD_TRACKING_START`**, the device **does not reset** the canvas; it only “pen-ups” so a new stroke starts cleanly.
- On each **`CMD_MOVE`**, \((x, y)\) in `0…32767` is scaled to pixel coordinates on the canvas; **`cv2.line`** connects consecutive points so the path matches the pen motion.
- On **`CMD_TRACKING_STOP`**, the last point is cleared so the next stroke does not connect across a gap.

## Files

| File | Role |
|------|------|
| `main.py` | Camera capture, pen detection, smoothing, gyro sampling while tracking, **Space** toggles pen + gyro |
| `config.py` | Camera index, ROI, HSV thresholds, smoothing, TCP host/port, device canvas size, gyro mode |
| `gyro_reader.py` | Gyroscope sampling (mock / serial / none) |
| `driver_client.py` | TCP client: move + start/stop commands |
| `test_server.py` | TCP server + OpenCV window: **live drawing** |
| `run.py` | Helper to discover which camera index works (macOS AVFoundation) |

## Dependencies

See **`requirements.txt`** (`opencv-python`, `numpy`, `pyserial` for serial gyro).
