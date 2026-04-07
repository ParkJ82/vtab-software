# config.py

# Camera
CAMERA_INDEX = 1
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# If False, run in IMU-only mode (no camera, no OpenCV windows).
CAMERA_ENABLED = True

# Green tip on a brown surface (desk/paper), black pen body.
# Tune LOWER_HSV / UPPER_HSV for your lighting; green in OpenCV H is ~35–95 (0–179 scale).
LOWER_HSV = (35, 70, 70)
UPPER_HSV = (95, 255, 255)

# Minimum contour area to count as a valid pen tip blob.
MIN_CONTOUR_AREA = 80
MAX_CONTOUR_AREA = 3000

# Pen-like contour filtering (to reject hand blobs):
# elongated objects have larger major/minor axis ratio.
MIN_PEN_ASPECT_RATIO = 1.0

# If pen is split into multiple close blobs (e.g., hand occlusion),
# merge nearby contours before endpoint/tip extraction.
MERGE_CONTOUR_DISTANCE = 120

# Exponential moving average smoothing
SMOOTHING_ALPHA = 0.35

# Region of interest in the frame that corresponds to your drawing area.
# Set these once you know your camera framing.
# Format: x1, y1, x2, y2
# Shrunk ROI to make border margins bigger (ignore more of the edges).
ROI = (140, 110, 1140, 650)

# Map ROI coordinates to driver absolute coordinates
ABSOLUTE_MIN = 0
ABSOLUTE_MAX = 32767

# Driver connection
DRIVER_HOST = "127.0.0.1"
DRIVER_PORT = 9999

# If False, do not connect/send pen coordinates to the driver.
DRIVER_ENABLED = True

# Debug windows (when tracking is off, pen overlays are hidden but camera preview can stay)
SHOW_MASK = True
SHOW_DEBUG = True

# Device canvas (test_server maps 0..32767 pen coords to this size)
DEVICE_CANVAS_WIDTH = 640
DEVICE_CANVAS_HEIGHT = 480
# BGR colors for device window: brown “paper”, black “ink” (matches black pen on brown background)
DEVICE_CANVAS_BG_BGR = (45, 72, 105)
DEVICE_STROKE_BGR = (0, 0, 0)

# Gyroscope — started/stopped together with pen tracking (Space). Same button.
# "none" = no gyro sampling (readings not shown).
# "mock" = synthetic rates for demo without hardware.
# "serial" = read text lines "gx gy gz" or "gx,gy,gz" from GYRO_SERIAL_PORT (needs pyserial).
GYRO_MODE = "serial"
GYRO_SERIAL_PORT = None
GYRO_SERIAL_BAUD = 9600