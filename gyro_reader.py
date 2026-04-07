# gyro_reader.py — gyroscope samples; lifecycle tied to pen tracking (see main.py).

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import config

GyroSample = Tuple[float, float, float]
ImuSample = Tuple[int, int, int, int, int, int]


class GyroReader:
    """
    Call start() when tracking turns on and stop() when it turns off.
    read() returns None if GYRO_MODE is "none" or the reader is not started.
    """

    def __init__(self) -> None:
        self._started = False
        self._serial = None
        self._t0 = 0.0
        self._last: GyroSample = (0.0, 0.0, 0.0)
        self._last_imu: Optional[ImuSample] = None

    def start(self) -> None:
        self.stop()
        if config.GYRO_MODE == "none":
            self._started = True
            return
        if config.GYRO_MODE == "mock":
            self._t0 = time.perf_counter()
            self._started = True
            return
        if config.GYRO_MODE == "serial":
            import serial
            from serial.tools import list_ports

            port = config.GYRO_SERIAL_PORT
            if not port:
                # Best-effort auto-detect for Arduino-style ports.
                candidates = []
                for p in list_ports.comports():
                    dev = (p.device or "").lower()
                    desc = (p.description or "").lower()
                    if "usbmodem" in dev or "usbserial" in dev or "arduino" in desc:
                        candidates.append(p.device)
                if not candidates:
                    raise RuntimeError(
                        "No serial ports found for GYRO_MODE='serial'. "
                        "Set config.GYRO_SERIAL_PORT (e.g. /dev/cu.usbmodemXXXX)."
                    )
                port = candidates[0]

            self._serial = serial.Serial(
                port,
                config.GYRO_SERIAL_BAUD,
                timeout=0.02,
            )
            self._started = True
            return
        raise ValueError(f"Unknown GYRO_MODE: {config.GYRO_MODE!r}")

    def stop(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except OSError:
                pass
            self._serial = None
        self._started = False

    def read_imu(self) -> Optional[ImuSample]:
        if not self._started:
            return None
        if config.GYRO_MODE == "none":
            return None
        if config.GYRO_MODE == "mock":
            t = time.perf_counter() - self._t0
            gx = 0.12 * math.sin(t * 2.0)
            gy = 0.08 * math.cos(t * 1.7)
            gz = 0.05 * math.sin(t * 2.3)
            self._last = (gx, gy, gz)
            return None
        assert self._serial is not None
        raw = self._serial.readline()
        if not raw:
            return self._last_imu
        try:
            s = raw.decode("utf-8", errors="ignore").strip()
            parts = s.replace(",", " ").split()
            if len(parts) >= 6:
                ax, ay, az, gx, gy, gz = (int(float(p)) for p in parts[:6])
                self._last_imu = (ax, ay, az, gx, gy, gz)
                self._last = (float(gx), float(gy), float(gz))
            elif len(parts) >= 3:
                # Back-compat: if only gyro is provided.
                gx, gy, gz = float(parts[0]), float(parts[1]), float(parts[2])
                self._last = (gx, gy, gz)
        except ValueError:
            pass
        return self._last_imu

    def read(self) -> Optional[GyroSample]:
        # Keep old API for overlay: returns latest gx,gy,gz (floats).
        if not self._started:
            return None
        if config.GYRO_MODE == "none":
            return None
        # Pull one serial line (if any) and update caches.
        _ = self.read_imu()
        return self._last
