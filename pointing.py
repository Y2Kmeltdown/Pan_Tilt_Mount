"""
pointing.py - pure geometry for the pan/tilt pointing module (no I/O, no hardware).

Everything here is deterministic and unit-testable:

  * angle helpers (wrap, signed difference)
  * WGS-84 geodetic -> local East/North/Up, azimuth / elevation / range
  * dead-reckoning of a moving target between position fixes
  * pan-axis error with an optional travel window (head cannot cross its end stop)

Conventions
-----------
* Azimuth ("yaw") is a compass bearing in degrees clockwise from north, 0-360.
* Elevation ("pitch") is degrees above the horizon, positive up.
* Positions are WGS-84 latitude/longitude in degrees and altitude in metres.
* Speeds are metres/second, vertical rate is metres/second (positive up).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# WGS-84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


# --------------------------------------------------------------------------- angles
def wrap360(a: float) -> float:
    """Wrap to [0, 360)."""
    return a % 360.0


def wrap180(a: float) -> float:
    """Wrap to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# --------------------------------------------------------------------------- geodesy
def geodetic_to_ecef(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    la, lo = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(la) ** 2)
    x = (n + alt) * math.cos(la) * math.cos(lo)
    y = (n + alt) * math.cos(la) * math.sin(lo)
    z = (n * (1.0 - _E2) + alt) * math.sin(la)
    return x, y, z


def enu_offset(obs: Tuple[float, float, float],
               tgt: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """East/North/Up (metres) of `tgt` relative to `obs`, both (lat, lon, alt)."""
    ox, oy, oz = geodetic_to_ecef(*obs)
    tx, ty, tz = geodetic_to_ecef(*tgt)
    dx, dy, dz = tx - ox, ty - oy, tz - oz
    la, lo = math.radians(obs[0]), math.radians(obs[1])
    sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    e = -so * dx + co * dy
    n = -sl * co * dx - sl * so * dy + cl * dz
    u = cl * co * dx + cl * so * dy + sl * dz
    return e, n, u


def look_angles(obs: Tuple[float, float, float],
                tgt: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """(azimuth_deg 0-360, elevation_deg, slant_range_m) from `obs` to `tgt`.

    Uses a full ECEF -> ENU transform, so earth curvature is accounted for
    (a flat-earth approximation is wrong by degrees for aircraft at 100 km).
    """
    e, n, u = enu_offset(obs, tgt)
    az = wrap360(math.degrees(math.atan2(e, n)))
    horiz = math.hypot(e, n)
    el = math.degrees(math.atan2(u, horiz))
    rng = math.sqrt(e * e + n * n + u * u)
    return az, el, rng


def move_latlon(lat: float, lon: float, speed_mps: float, track_deg: float,
                dt: float) -> Tuple[float, float]:
    """Advance a position along a ground track for `dt` seconds."""
    dist = speed_mps * dt
    dn = dist * math.cos(math.radians(track_deg))
    de = dist * math.sin(math.radians(track_deg))
    la = math.radians(lat)
    sin2 = math.sin(la) ** 2
    rn = _A / math.sqrt(1.0 - _E2 * sin2)                       # prime vertical radius
    rm = _A * (1.0 - _E2) / (1.0 - _E2 * sin2) ** 1.5           # meridional radius
    new_lat = lat + math.degrees(dn / rm)
    new_lon = lon + math.degrees(de / (rn * max(math.cos(la), 1e-9)))
    return new_lat, ((new_lon + 180.0) % 360.0) - 180.0


# --------------------------------------------------------------------------- dead reckoning
@dataclass
class Fix:
    """One position report from a tracked target."""
    lat: float
    lon: float
    alt: float                       # metres
    t: float                         # wall-clock receive time, seconds
    speed: Optional[float] = None    # m/s
    track: Optional[float] = None    # deg true
    vrate: Optional[float] = None    # m/s


class DeadReckoner:
    """Extrapolate the last fix forward to `now + lead_s`.

    Mirrors tripwire-gimbal-point's DeadReckonTracker: the reference fix is
    replaced whenever a newer one arrives, and extrapolation is capped at
    `max_extrap_s` so a dropped link freezes the aim point instead of letting
    it run away. Without speed/track the target is simply held at its last fix.
    """

    def __init__(self, lead_s: float = 1.0, max_extrap_s: float = 20.0):
        self.lead_s = lead_s
        self.max_extrap_s = max_extrap_s
        self.fix: Optional[Fix] = None

    def update(self, fix: Fix) -> None:
        self.fix = fix

    def age(self, now: float) -> Optional[float]:
        return None if self.fix is None else now - self.fix.t

    def predict(self, now: float) -> Optional[Tuple[float, float, float]]:
        f = self.fix
        if f is None:
            return None
        dt = min(max(now - f.t, 0.0), self.max_extrap_s) + self.lead_s
        lat, lon = f.lat, f.lon
        if f.speed is not None and f.track is not None:
            lat, lon = move_latlon(f.lat, f.lon, f.speed, f.track, dt)
        alt = f.alt + (f.vrate * dt if f.vrate is not None else 0.0)
        return lat, lon, alt


class Smoother:
    """Exponential smoothing of an (lat, lon, alt) observer position."""

    def __init__(self, alpha: float):
        self.alpha = clamp(alpha, 0.0, 1.0)
        self.value: Optional[Tuple[float, float, float]] = None

    def update(self, pos: Tuple[float, float, float]) -> Tuple[float, float, float]:
        if self.value is None:
            self.value = pos
        else:
            a = self.alpha
            self.value = tuple(v + a * (p - v) for v, p in zip(self.value, pos))  # type: ignore
        return self.value


# --------------------------------------------------------------------------- pan travel
def pan_error(target_az: float, head_az: float, center: Optional[float] = None,
              half_range: float = 175.0) -> Tuple[float, bool]:
    """Signed pan error in degrees (positive = pan right / clockwise) and a
    flag saying the target had to be clamped to the travel window.

    Without `center` the shortest path is taken (the head can spin freely).
    With `center` (compass bearing of the head's mid-travel position) the head
    has a physical end stop `half_range` either side of it and must never be
    told to cross the dead gap behind it, so the error is computed in the
    base-relative frame with no wrap-around.
    """
    if center is None:
        return wrap180(target_az - head_az), False
    t = wrap180(target_az - center)
    h = wrap180(head_az - center)
    clamped = abs(t) > half_range
    t = clamp(t, -half_range, half_range)
    return t - h, clamped
