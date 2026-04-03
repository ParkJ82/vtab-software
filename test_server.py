# test_server.py — "device" display: shows the drawing from pen coordinates (0..32767).

import socket
import struct
from typing import Optional, Tuple

import cv2
import numpy as np

import config
from driver_client import CMD_MOVE, CMD_TRACKING_START, CMD_TRACKING_STOP


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

    print(f"Device display listening on {host}:{port} (5-byte packets: move / start / stop)")
    print("OpenCV window shows the drawing. Press q in that window to exit.")

    conn, addr = server.accept()
    print(f"Connected by {addr}")

    canvas = np.ones((ch, cw, 3), dtype=np.uint8) * 255
    last_pt: Optional[Tuple[int, int]] = None
    line_color = (40, 120, 255)

    try:
        while True:
            data = recv_exact(conn, 5)
            if data is None:
                print("Connection closed.")
                break

            cmd, x, y = struct.unpack("!BHH", data)

            if cmd == CMD_TRACKING_START:
                canvas = np.ones((ch, cw, 3), dtype=np.uint8) * 255
                last_pt = None
            elif cmd == CMD_TRACKING_STOP:
                last_pt = None
            elif cmd == CMD_MOVE:
                x = min(32767, max(0, int(x)))
                y = min(32767, max(0, int(y)))
                pt = abs_to_canvas(x, y, cw, ch)
                if last_pt is not None:
                    cv2.line(canvas, last_pt, pt, line_color, 2, lineType=cv2.LINE_AA)
                last_pt = pt

            cv2.imshow("Pen drawing (device)", canvas)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        conn.close()
        server.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
