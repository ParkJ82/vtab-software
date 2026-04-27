# test_server.py — "device" display: shows the drawing from pen coordinates (0..32767).

import socket
import struct
from typing import Optional, Tuple

import cv2
import numpy as np

import config
from driver_client import CMD_CLEAR, CMD_MOVE, CMD_POINTER_HIDE, CMD_TRACKING_START, CMD_TRACKING_STOP


def recv_exact(conn: socket.socket, n: int) -> Optional[bytes]:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def abs_to_canvas(x: int, y: int, cw: int, ch: int) -> Tuple[int, int]:
    px = int(x * (cw - 1) / 32767)
    py = int(y * (ch - 1) / 32767)
    return px, py


def main() -> None:
    host = config.DRIVER_HOST
    port = config.DRIVER_PORT
    cw = config.DEVICE_CANVAS_WIDTH
    ch = config.DEVICE_CANVAS_HEIGHT

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)

    print(f"Device display listening on {host}:{port} (5-byte packets: move / start / stop / clear)")
    print("OpenCV window shows the drawing. Press c to clear, q to exit.")

    canvas = np.zeros((ch, cw, 3), dtype=np.uint8)
    canvas[:] = config.DEVICE_CANVAS_BG_BGR
    last_pt: Optional[Tuple[int, int]] = None
    cursor_marker: Optional[Tuple[int, int]] = None
    tracking_active = False
    line_color = config.DEVICE_STROKE_BGR

    conn, addr = server.accept()
    print(f"Connected by {addr}")

    try:
        while True:
            data = recv_exact(conn, 5)
            if data is None:
                print("Connection closed.")
                break

            cmd, x, y = struct.unpack("!BHH", data)

            if cmd == CMD_TRACKING_START:
                # Resume drawing; do not clear existing strokes.
                tracking_active = True
                last_pt = None
            elif cmd == CMD_TRACKING_STOP:
                tracking_active = False
                last_pt = None
            elif cmd == CMD_CLEAR:
                canvas[:] = config.DEVICE_CANVAS_BG_BGR
                last_pt = None
            elif cmd == CMD_POINTER_HIDE:
                cursor_marker = None
            elif cmd == CMD_MOVE:
                x = min(32767, max(0, int(x)))
                y = min(32767, max(0, int(y)))
                pt = abs_to_canvas(x, y, cw, ch)
                cursor_marker = pt
                if tracking_active:
                    if last_pt is not None:
                        cv2.line(canvas, last_pt, pt, line_color, 2, lineType=cv2.LINE_AA)
                    last_pt = pt

            # Draw a temporary pointer overlay so users always know location.
            display = canvas.copy()
            if cursor_marker is not None:
                px, py = cursor_marker
                cv2.circle(display, (px, py), 5, (0, 0, 255), -1)
                cv2.line(display, (px - 10, py), (px + 10, py), (0, 0, 255), 1, lineType=cv2.LINE_AA)
                cv2.line(display, (px, py - 10), (px, py + 10), (0, 0, 255), 1, lineType=cv2.LINE_AA)
            cv2.imshow("Pen drawing (device)", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c"):
                canvas[:] = config.DEVICE_CANVAS_BG_BGR
                last_pt = None
            elif key == ord("q"):
                break
    finally:
        conn.close()
        server.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
