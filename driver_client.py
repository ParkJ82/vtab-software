# driver_client.py

import socket
import struct
from typing import Optional

# Packet: command byte + uint16 x + uint16 y (5 bytes, big-endian).
# Coordinates are only meaningful for CMD_MOVE.
CMD_MOVE = 0
CMD_TRACKING_STOP = 1
CMD_TRACKING_START = 2


class DriverClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

    def _send_packet(self, cmd: int, x: int = 0, y: int = 0) -> None:
        if self.sock is None:
            raise RuntimeError("Driver socket is not connected.")
        x = max(0, min(65535, int(x)))
        y = max(0, min(65535, int(y)))
        packet = struct.pack("!BHH", cmd & 0xFF, x, y)
        self.sock.sendall(packet)

    def send_coordinates(self, x: int, y: int) -> None:
        """Send pen position (0..32767) while tracking is active."""
        x = max(0, min(32767, int(x)))
        y = max(0, min(32767, int(y)))
        self._send_packet(CMD_MOVE, x, y)

    def send_tracking_start(self) -> None:
        """Notify device: new drawing session; clear canvas and lift pen until next move."""
        self._send_packet(CMD_TRACKING_START, 0, 0)

    def send_tracking_stop(self) -> None:
        """Notify device: stop drawing; lift pen (no line to next position)."""
        self._send_packet(CMD_TRACKING_STOP, 0, 0)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None
