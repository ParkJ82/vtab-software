# driver_client.py

import socket
import struct
from typing import Optional


class DriverClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

    def send_coordinates(self, x: int, y: int) -> None:
        if self.sock is None:
            raise RuntimeError("Driver socket is not connected.")

        # Clamp again for safety
        x = max(0, min(32767, int(x)))
        y = max(0, min(32767, int(y)))

        packet = struct.pack("!HH", x, y)
        self.sock.sendall(packet)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None