# config.py

# Camera
CAMERA_INDEX = 1
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Green tip tracking profile.
# Use a bright/neon green marker on the pen tip and tune for your lighting.
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
ROI = (100, 80, 1180, 680)

# Map ROI coordinates to driver absolute coordinates
ABSOLUTE_MIN = 0
ABSOLUTE_MAX = 32767

# Driver connection
DRIVER_HOST = "127.0.0.1"
DRIVER_PORT = 9999

# Debug windows (when tracking is off, pen overlays are hidden but camera preview can stay)
SHOW_MASK = True
SHOW_DEBUG = True

# Device canvas (test_server maps 0..32767 pen coords to this size)
DEVICE_CANVAS_WIDTH = 640
DEVICE_CANVAS_HEIGHT = 480