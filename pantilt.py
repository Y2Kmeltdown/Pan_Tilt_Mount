"""
pantilt - small control library for an RS485 pan/tilt head that speaks Pelco-D.

Chain: PC -> USB-serial adapter -> TTL-to-RS485 converter -> head (RS485+ / RS485-)

The head cannot report its position and this library does not estimate it: it only
sends commands and times moves. Use external sensors (compass / IMU) for angles.

Requires:  pip install pyserial      (not needed for dry-run mode)
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional

try:
    import serial  # pyserial
except ImportError:  # dry-run mode works without pyserial
    serial = None

__all__ = [
    "HeadConfig", "PanTilt", "PanTiltError", "build_frame", "RESERVED_PRESETS",
    "DryRunSerial", "FakeClock", "MAX_SPEED", "DEFAULT_SPEED",
]

# --- Pelco-D constants -------------------------------------------------------
SYNC = 0xFF
CMD_RIGHT, CMD_LEFT, CMD_UP, CMD_DOWN = 0x02, 0x04, 0x08, 0x10
CMD_SET_PRESET, CMD_CLEAR_PRESET, CMD_GOTO_PRESET = 0x03, 0x05, 0x07
MAX_SPEED = 0x3F
DEFAULT_SPEED = 0x20

# Preset numbers this head uses as *commands* (see manual section 5). Calling or
# setting them changes head settings instead of just moving it.
RESERVED_PRESETS = frozenset({17, 18, 19, 22, 70, 71, 72, 83, 84, 89,
                              105, 110, 111, 120, 125})


class PanTiltError(Exception):
    """Base class for library errors."""


def build_frame(address: int, cmd1: int, cmd2: int, data1: int = 0, data2: int = 0) -> bytes:
    """Build one 7-byte Pelco-D frame. Checksum excludes the 0xFF sync byte."""
    body = [address, cmd1, cmd2, data1, data2]
    for v in body:
        if not 0 <= v <= 0xFF:
            raise ValueError(f"frame byte out of range: {v}")
    return bytes([SYNC] + body + [sum(body) & 0xFF])


@dataclass
class HeadConfig:
    """Timing used by the limit/sweep helpers (no angles).

    Defaults come from the model 303 manual: pan 350 deg at 14 deg/s = 25 s,
    tilt 55 deg (-32..+23) at 2 deg/s = 27.5 s. Time a full sweep on your head
    and adjust (see README, "Calibration").
    """
    pan_sweep_s: float = 25.0      # time for a full pan sweep
    tilt_sweep_s: float = 27.5     # time for a full tilt sweep
    limit_margin_s: float = 2.0    # extra drive time when running into a limit


# --- Dry-run helpers (no hardware) -------------------------------------------
class FakeClock:
    """A clock whose sleep() advances time instantly - for dry runs and tests."""

    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += max(0.0, seconds)


class DryRunSerial:
    """Stand-in for a serial port that just records what would be sent."""

    def __init__(self):
        self.writes: List[bytes] = []

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def _sign(v: int, name: str) -> int:
    if v not in (-1, 0, 1):
        raise ValueError(f"{name} must be -1, 0 or +1, got {v!r}")
    return int(v)


class PanTilt:
    """Control one Pelco-D pan/tilt head.

    Directions are -1, 0, +1: pan +1 = right, tilt +1 = up.

        with PanTilt("COM6", address=57) as head:
            head.move_for(pan=1, tilt=0, seconds=1.0)
    """

    def __init__(self, port: Optional[str] = None, address: int = 1, baud: int = 9600, *,
                 config: Optional[HeadConfig] = None, serial_port=None,
                 sleep: Optional[Callable[[float], None]] = None, verbose: bool = False):
        if not 0 <= address <= 255:
            raise ValueError("address must be 0-255 (this head: 0-63, set by DIP 1-6)")
        self.port, self.address, self.baud = port, address, baud
        self.config = config or HeadConfig()
        self.verbose = verbose
        self._ser = serial_port
        self._sleep = sleep or time.sleep

    @classmethod
    def dry_run(cls, address: int = 1, config: Optional[HeadConfig] = None,
                verbose: bool = False) -> "PanTilt":
        """A PanTilt wired to a fake port and clock: nothing is sent, time is instant."""
        return cls(port=None, address=address, config=config, serial_port=DryRunSerial(),
                   sleep=FakeClock().sleep, verbose=verbose)

    # -- connection ---------------------------------------------------------
    def open(self) -> "PanTilt":
        if self._ser is None:
            if serial is None:
                raise PanTiltError("pyserial is not installed: pip install pyserial")
            if not self.port:
                raise PanTiltError("no port given")
            self._ser = serial.Serial(self.port, self.baud, bytesize=8,
                                      parity=serial.PARITY_NONE, stopbits=1, timeout=1)
        return self

    def close(self) -> None:
        if self._ser is not None:
            try:
                self.stop()
            finally:
                self._ser.close()
                self._ser = None

    def __enter__(self) -> "PanTilt":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- low level ----------------------------------------------------------
    def _send(self, frame: bytes) -> None:
        if self._ser is None:
            raise PanTiltError("port is not open")
        if self.verbose:
            print("TX:", frame.hex(" ").upper())
        self._ser.write(frame)
        self._ser.flush()

    def send_raw(self, cmd1: int, cmd2: int, data1: int = 0, data2: int = 0) -> None:
        """Send any Pelco-D command using this head's address."""
        self._send(build_frame(self.address, cmd1, cmd2, data1, data2))

    # -- motion -------------------------------------------------------------
    def move(self, pan: int = 0, tilt: int = 0, speed: int = DEFAULT_SPEED,
             pan_speed: Optional[int] = None, tilt_speed: Optional[int] = None) -> None:
        """Start moving (non-blocking). The head keeps moving until stop().

        Speed bytes are 0x00-0x3F. This head appears to use fixed speeds, so they
        may have no effect.
        """
        pan, tilt = _sign(pan, "pan"), _sign(tilt, "tilt")
        ps = speed if pan_speed is None else pan_speed
        ts = speed if tilt_speed is None else tilt_speed
        if not (0 <= ps <= MAX_SPEED and 0 <= ts <= MAX_SPEED):
            raise ValueError(f"speed must be 0-{MAX_SPEED}")
        cmd2 = 0
        if pan > 0:
            cmd2 |= CMD_RIGHT
        elif pan < 0:
            cmd2 |= CMD_LEFT
        if tilt > 0:
            cmd2 |= CMD_UP
        elif tilt < 0:
            cmd2 |= CMD_DOWN
        self.send_raw(0x00, cmd2, ps, ts)

    def stop(self) -> None:
        """Stop all motion."""
        self.send_raw(0x00, 0x00, 0x00, 0x00)

    def move_for(self, pan: int = 0, tilt: int = 0, seconds: float = 1.0,
                 speed: int = DEFAULT_SPEED, refresh_s: Optional[float] = None) -> None:
        """Move for a fixed time (blocking), then stop - even on error or Ctrl+C.

        refresh_s: if set, re-send the move command at this interval. Useful if a
        head stops on its own after a timeout or a frame may have been lost.
        """
        if seconds <= 0:
            return
        try:
            self.move(pan, tilt, speed)
            remaining = seconds
            while remaining > 0:
                step = remaining if not refresh_s else min(refresh_s, remaining)
                self._sleep(step)
                remaining -= step
                if refresh_s and remaining > 0:
                    self.move(pan, tilt, speed)
        finally:
            self.stop()

    def pause(self, seconds: float) -> None:
        self._sleep(seconds)

    def drive_to_limit(self, axis: str, direction: int, speed: int = DEFAULT_SPEED) -> None:
        """Run an axis into its end stop (full sweep time + margin).

        axis: 'pan' or 'tilt'. direction: +1 (right/up) or -1 (left/down).
        """
        direction = _sign(direction, "direction")
        if direction == 0 or axis not in ("pan", "tilt"):
            raise ValueError("axis must be 'pan' or 'tilt', direction +1 or -1")
        c = self.config
        if axis == "pan":
            self.move_for(direction, 0, c.pan_sweep_s + c.limit_margin_s, speed)
        else:
            self.move_for(0, direction, c.tilt_sweep_s + c.limit_margin_s, speed)

    def zigzag(self, pan_dir: int = 1, legs: int = 4, first_tilt_dir: int = 1,
               speed: int = DEFAULT_SPEED) -> None:
        """Sweep the full pan range while the tilt direction alternates each leg.

        Each leg moves pan and tilt together (a diagonal) for pan_sweep_s / legs.
        Pan is much faster than tilt on this head, so the diagonals are shallow.
        Make sure the head has tilt room for one leg from where it starts.
        """
        pan_dir, tilt_dir = _sign(pan_dir, "pan_dir"), _sign(first_tilt_dir, "first_tilt_dir")
        if pan_dir == 0 or tilt_dir == 0 or legs < 1:
            raise ValueError("pan_dir and first_tilt_dir must be +/-1 and legs >= 1")
        leg_time = self.config.pan_sweep_s / legs
        for _ in range(legs):
            self.move_for(pan_dir, tilt_dir, leg_time, speed)
            tilt_dir = -tilt_dir

    # -- presets ------------------------------------------------------------
    def goto_preset(self, n: int, allow_reserved: bool = False) -> None:
        self._check_preset(n, allow_reserved)
        self.send_raw(0x00, CMD_GOTO_PRESET, 0x00, n)

    def set_preset(self, n: int, allow_reserved: bool = False) -> None:
        self._check_preset(n, allow_reserved)
        self.send_raw(0x00, CMD_SET_PRESET, 0x00, n)

    @staticmethod
    def _check_preset(n: int, allow_reserved: bool) -> None:
        if not 1 <= n <= 255:
            raise ValueError("preset must be 1-255")
        if n in RESERVED_PRESETS and not allow_reserved:
            raise ValueError(f"preset {n} is a head command (limit scan, cruise, reset...); "
                             "pass allow_reserved=True if you really mean it")
