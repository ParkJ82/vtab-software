# gyro_reader.py — gyroscope samples; lifecycle tied to pen tracking (see main.py).

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import config

GyroSample = Tuple[float, float, float]


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
            if not config.GYRO_SERIAL_PORT:
                raise RuntimeError("Set config.GYRO_SERIAL_PORT for GYRO_MODE='serial'")
            import serial

            self._serial = serial.Serial(
                config.GYRO_SERIAL_PORT,
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

    def read(self) -> Optional[GyroSample]:
        if not self._started:
            return None
        if config.GYRO_MODE == "none":
            return None
        if config.GYRO_MODE == "mock":
            t = time.perf_counter() - self._t0
            return (
                0.12 * math.sin(t * 2.0),
                0.08 * math.cos(t * 1.7),
                0.05 * math.sin(t * 2.3),
            )
        assert self._serial is not None
        raw = self._serial.readline()
        if not raw:
            return self._last
        try:
            s = raw.decode("utf-8", errors="ignore").strip()
            parts = s.replace(",", " ").split()
            if len(parts) >= 3:
                gx, gy, gz = float(parts[0]), float(parts[1]), float(parts[2])
                self._last = (gx, gy, gz)
        except ValueError:
            pass
        return self._last
