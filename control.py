"""
control.py - closed-loop pointing logic for an open-loop Pelco-D pan/tilt head.

The head only understands "move left/right/up/down until told to stop" at
(effectively) fixed speeds and reports nothing back. Position feedback comes
from a tilt-compensated compass mounted on the moving head, so the loop is
closed in the *earth frame*: the compass bearing is the head's azimuth and its
pitch is the head's elevation.

`Pointer` is a pure state machine (no threads, no sockets, no serial): feed it
sensor readings and a clock, call `step(now)`, and it returns the direction
(-1/0/+1) for each axis plus a status dictionary. The service module does all
the I/O around it, which keeps this file unit-testable with a simulated head.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from pointing import (DeadReckoner, Fix, Smoother, clamp, look_angles, pan_error,
                      wrap180, wrap360)

MODES = ("idle", "jog", "angle", "latlon", "track")


# --------------------------------------------------------------------------- per-axis control
class AxisController:
    """Bang-bang controller with hysteresis, predictive stop and a safety trip.

    * start moving when |error| > start_deg (and the axis has settled);
    * stop when the error is within what the head will still travel during the
      sensor/serial latency (rate_dps * latency_s) - i.e. stop *early* so the
      coast lands on the target rather than past it;
    * trip a fault if, after `check_s` of driving, the head has not actually
      moved in the commanded direction. That catches inverted sensor sign,
      a stalled/unpowered head, a dead compass and an end stop all at once -
      without it a wrong sign would drive the head into its limit forever.
    """

    def __init__(self, start_deg: float, rate_dps: float, latency_s: float,
                 settle_s: float = 0.4, check_s: float = 2.5,
                 min_motion_deg: float = 0.3, wrap: bool = False, name: str = "axis"):
        self.name = name
        self.start_deg = start_deg
        self.rate_dps = rate_dps
        self.latency_s = latency_s
        self.settle_s = settle_s
        self.check_s = check_s
        self.min_motion_deg = min_motion_deg
        self.wrap = wrap
        self.dir = 0
        self.fault: Optional[str] = None
        self._t0 = 0.0
        self._pos0 = 0.0
        self._stopped_at = -1e9

    def reset(self) -> None:
        self.dir = 0
        self.fault = None
        self._stopped_at = -1e9

    def halt(self, now: float) -> None:
        if self.dir != 0:
            self._stopped_at = now
        self.dir = 0

    def update(self, err: float, pos: float, now: float) -> int:
        """err: signed error (+ = move in the positive direction).
        pos: current axis position in the same positive sense (any origin)."""
        if self.fault:
            self.dir = 0
            return 0
        if self.dir == 0:
            if abs(err) > self.start_deg and now - self._stopped_at >= self.settle_s:
                self.dir = 1 if err > 0 else -1
                self._t0, self._pos0 = now, pos
        else:
            coast = self.rate_dps * self.latency_s
            if err * self.dir <= coast:
                self.halt(now)
            elif now - self._t0 >= self.check_s:
                moved = pos - self._pos0
                if self.wrap:
                    moved = wrap180(moved)
                if moved * self.dir < self.min_motion_deg:
                    self.fault = f"{self.name}_not_moving"
                    self.halt(now)
                else:
                    self._t0, self._pos0 = now, pos      # re-arm the check
        return self.dir


# --------------------------------------------------------------------------- configuration
@dataclass
class PointerConfig:
    # sensor calibration
    declination_deg: float = 0.0       # added to compass bearing (magnetic -> true)
    yaw_offset_deg: float = 0.0        # compass mounting offset about vertical
    pitch_offset_deg: float = 0.0      # compass mounting offset in pitch
    pitch_sign: int = 1                # +1 / -1: flip if compass pitch is nose-down positive
    # head geometry
    # Travel limits ("dead zones"). The head has no position feedback and the
    # compass is the only sensor, so the physical end stops are described in
    # compass terms and set at startup (re-set them if the platform is moved):
    #   pan_center_heading - true bearing the head faces at mid-travel, i.e. the
    #                        dead gap (360 - pan_range) is centred on this + 180.
    #                        None = unknown: pan is unconstrained (unsafe).
    pan_center_heading: Optional[float] = None
    pan_range_deg: float = 350.0
    tilt_min_deg: float = -35.0        # compass pitch at the bottom end stop
    tilt_max_deg: float = 35.0         # compass pitch at the top end stop
    limit_margin_deg: float = 3.0      # stay this far short of every end stop

    @property
    def pan_half_range_deg(self) -> float:
        return max(self.pan_range_deg / 2.0 - self.limit_margin_deg, 0.0)

    @property
    def tilt_lo(self) -> float:
        return self.tilt_min_deg + self.limit_margin_deg

    @property
    def tilt_hi(self) -> float:
        return self.tilt_max_deg - self.limit_margin_deg

    # loop tuning
    pan_rate_dps: float = 14.0
    tilt_rate_dps: float = 2.0
    pan_start_deg: float = 1.5
    tilt_start_deg: float = 1.0
    latency_s: float = 0.25
    on_target_deg: float = 2.0
    # staleness / smoothing
    compass_max_age_s: float = 1.0
    gps_max_age_s: float = 10.0
    smooth: float = 0.1                # GPS observer smoothing 0..1
    jog_timeout_s: float = 0.6
    # target tracking
    lead_time_s: float = 1.0
    max_extrap_s: float = 20.0
    alt_offset_m: float = 0.0
    # fixed fallback observer position (lat, lon, alt) used when GPS has no fix
    fallback: Optional[Tuple[float, float, float]] = None


# --------------------------------------------------------------------------- state machine
class Pointer:
    """Mode + sensor state -> axis directions. Thread-safe (one lock)."""

    def __init__(self, cfg: PointerConfig):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.pan_ctl = AxisController(cfg.pan_start_deg, cfg.pan_rate_dps, cfg.latency_s,
                                      wrap=cfg.pan_center_heading is None, name="pan")
        self.tilt_ctl = AxisController(cfg.tilt_start_deg, cfg.tilt_rate_dps, cfg.latency_s,
                                       name="tilt")

        self.mode = "idle"
        # sensors
        self._compass: Optional[Dict[str, float]] = None     # raw: yaw, pitch, roll, t
        self._gps: Optional[Dict[str, float]] = None         # lat, lon, alt, t, lock
        self._obs_smooth = Smoother(cfg.smooth)
        self._obs_override: Optional[Tuple[float, float, float]] = None
        self.manual_last: Optional[Tuple[float, float, float]] = None   # remembered when disabled
        # command state per mode
        self.target_fixed: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # latlon mode
        self.angle_cmd: Tuple[float, float] = (0.0, 0.0)                  # earth yaw, pitch
        self.jog_cmd: Tuple[int, int] = (0, 0)
        self._jog_t = 0.0
        self.dr = DeadReckoner(cfg.lead_time_s, cfg.max_extrap_s)
        self.track_label: Optional[str] = None
        # user nudges (applied to computed aim in latlon/track modes)
        self.yaw_nudge = 0.0
        self.pitch_nudge = 0.0
        # last computed values, for telemetry
        self.last: Dict[str, object] = {}

    # ---- inputs ---------------------------------------------------------------
    def set_compass(self, yaw: float, pitch: float, roll: float, t: float) -> None:
        with self.lock:
            self._compass = {"yaw": yaw, "pitch": pitch, "roll": roll, "t": t}

    def set_gps(self, lat: float, lon: float, alt: float, has_lock: bool, t: float) -> None:
        with self.lock:
            self._gps = {"lat": lat, "lon": lon, "alt": alt, "t": t, "lock": has_lock}

    def set_observer_override(self, pos: Optional[Tuple[float, float, float]]) -> None:
        """Manual observer position; overrides GPS while set. None = back to GPS."""
        with self.lock:
            self._obs_override = pos
            if pos is not None:
                self.manual_last = pos

    def set_track_fix(self, fix: Fix) -> None:
        with self.lock:
            self.dr.update(fix)

    def set_limits(self, center: Optional[float] = ..., pan_range: Optional[float] = None,
                   tilt_min: Optional[float] = None, tilt_max: Optional[float] = None,
                   margin: Optional[float] = None) -> None:
        """Update travel limits at runtime. center=None clears the pan window."""
        with self.lock:
            c = self.cfg
            if center is not ...:
                c.pan_center_heading = None if center is None else wrap360(center)
            if pan_range is not None:
                c.pan_range_deg = clamp(pan_range, 10.0, 360.0)
            if tilt_min is not None:
                c.tilt_min_deg = tilt_min
            if tilt_max is not None:
                c.tilt_max_deg = tilt_max
            if margin is not None:
                c.limit_margin_deg = max(margin, 0.0)
            if c.tilt_hi <= c.tilt_lo:
                raise ValueError("tilt limits leave no usable range")

    def set_center_here(self, now: float) -> float:
        """Declare the head's current heading to be mid-travel (calibration helper)."""
        with self.lock:
            att = self.head_attitude(now)
            if att is None:
                raise ValueError("no compass reading yet")
            self.cfg.pan_center_heading = att[0]
            return att[0]

    def set_lead_time(self, s: float) -> None:
        with self.lock:
            self.cfg.lead_time_s = s
            self.dr.lead_s = s

    # ---- mode commands --------------------------------------------------------
    def _enter(self, mode: str) -> None:
        self.mode = mode
        self.pan_ctl.reset()
        self.tilt_ctl.reset()

    def stop(self) -> None:
        with self.lock:
            self._enter("idle")

    def point_latlon(self, lat: float, lon: float, alt: float) -> None:
        with self.lock:
            self.target_fixed = (lat, lon, alt)
            self._enter("latlon")

    def point_angle(self, yaw: float, pitch: float, frame: str = "earth") -> None:
        """frame 'earth': yaw is a true bearing. 'body': yaw is relative to the
        base (needs pan_center_heading)."""
        with self.lock:
            if frame == "body":
                if self.cfg.pan_center_heading is None:
                    raise ValueError("body frame needs pan_center_heading to be configured")
                yaw = wrap360(self.cfg.pan_center_heading + yaw)
            elif frame != "earth":
                raise ValueError("frame must be 'earth' or 'body'")
            self.angle_cmd = (wrap360(yaw), pitch)
            self._enter("angle")

    def follow_track(self, label: Optional[str] = None) -> None:
        with self.lock:
            self.dr = DeadReckoner(self.cfg.lead_time_s, self.cfg.max_extrap_s)
            self.track_label = label
            self._enter("track")

    def jog(self, pan: int, tilt: int, now: float) -> None:
        with self.lock:
            if self.mode != "jog":
                self._enter("jog")
            self.jog_cmd = (int(clamp(pan, -1, 1)), int(clamp(tilt, -1, 1)))
            self._jog_t = now

    # ---- derived sensor values ------------------------------------------------
    def head_attitude(self, now: float) -> Optional[Tuple[float, float, float, float]]:
        """(yaw_true, pitch, roll, age_s) of the head, or None before first reading."""
        c = self._compass
        if c is None:
            return None
        cfg = self.cfg
        yaw = wrap360(c["yaw"] + cfg.declination_deg + cfg.yaw_offset_deg)
        pitch = cfg.pitch_sign * c["pitch"] + cfg.pitch_offset_deg
        return yaw, pitch, c["roll"], now - c["t"]

    def observer(self, now: float) -> Tuple[Optional[Tuple[float, float, float]], str]:
        cfg = self.cfg
        if self._obs_override is not None:
            return self._obs_override, "override"
        g = self._gps
        if g is not None and g["lock"] and now - g["t"] <= cfg.gps_max_age_s:
            return self._obs_smooth.update((g["lat"], g["lon"], g["alt"])), "gps"
        if self._obs_smooth.value is not None:
            return self._obs_smooth.value, "gps_held"
        if cfg.fallback is not None:
            return cfg.fallback, "fallback"
        return None, "none"

    # ---- the control step -----------------------------------------------------
    def step(self, now: float) -> Tuple[int, int, Dict[str, object]]:
        """Compute (pan_dir, tilt_dir, status). Call at a steady rate (~10 Hz)."""
        with self.lock:
            return self._step(now)

    def _step(self, now: float):
        cfg = self.cfg
        self.pan_ctl.wrap = cfg.pan_center_heading is None
        att = self.head_attitude(now)
        obs, obs_src = self.observer(now)
        info: Dict[str, object] = {
            "mode": self.mode, "observer": obs, "observer_source": obs_src,
            "head_yaw": att[0] if att else None, "head_pitch": att[1] if att else None,
            "head_roll": att[2] if att else None, "compass_age": att[3] if att else None,
            "aim_yaw": None, "aim_pitch": None, "range_m": None, "target": None,
            "err_pan": None, "err_tilt": None, "pan_clamped": False,
            "limits_set": cfg.pan_center_heading is not None,
        }

        def done(pan: int, tilt: int, status: str):
            info["status"] = status
            info["pan_dir"], info["tilt_dir"] = pan, tilt
            info["fault"] = self.pan_ctl.fault or self.tilt_ctl.fault
            self.last = info
            return pan, tilt, info

        if self.mode == "idle":
            self.pan_ctl.halt(now)
            self.tilt_ctl.halt(now)
            return done(0, 0, "idle")

        faulted = self.pan_ctl.fault or self.tilt_ctl.fault
        if faulted:
            return done(0, 0, f"fault:{faulted}")

        # ---- jog (open loop; compass optional, used only for soft limits) ----
        if self.mode == "jog":
            if now - self._jog_t > cfg.jog_timeout_s:
                self.mode = "idle"
                return done(0, 0, "idle")
            pan, tilt = self.jog_cmd
            if att:
                if tilt > 0 and att[1] >= cfg.tilt_hi:
                    tilt = 0
                if tilt < 0 and att[1] <= cfg.tilt_lo:
                    tilt = 0
                if cfg.pan_center_heading is not None:
                    rel = wrap180(att[0] - cfg.pan_center_heading)
                    if (pan > 0 and rel >= cfg.pan_half_range_deg) or \
                       (pan < 0 and rel <= -cfg.pan_half_range_deg):
                        pan = 0
            return done(pan, tilt, "jog")

        # ---- closed-loop modes need a fresh compass reading ----
        if att is None:
            return done(0, 0, "no_compass")
        if att[3] > cfg.compass_max_age_s:
            self.pan_ctl.halt(now)
            self.tilt_ctl.halt(now)
            return done(0, 0, "compass_stale")
        head_yaw, head_pitch = att[0], att[1]

        # ---- resolve the aim point ----
        status_ok = "tracking"
        if self.mode == "angle":
            aim_yaw, aim_pitch = self.angle_cmd
        else:
            if obs is None:
                return done(0, 0, "no_observer")
            if self.mode == "latlon":
                tgt = self.target_fixed
            else:  # track
                pred = self.dr.predict(now)
                if pred is None:
                    return done(0, 0, "no_track_fix")
                tgt = (pred[0], pred[1], pred[2] + cfg.alt_offset_m)
                age = self.dr.age(now)
                if age is not None and age > cfg.max_extrap_s:
                    status_ok = "track_stale"
            info["target"] = {"lat": tgt[0], "lon": tgt[1], "alt": tgt[2]}
            aim_yaw, aim_pitch, rng = look_angles(obs, tgt)
            info["range_m"] = rng
            aim_yaw = wrap360(aim_yaw + self.yaw_nudge)
            aim_pitch += self.pitch_nudge

        aim_pitch = clamp(aim_pitch, cfg.tilt_lo, cfg.tilt_hi)
        info["aim_yaw"], info["aim_pitch"] = aim_yaw, aim_pitch

        err_pan, clamped = pan_error(aim_yaw, head_yaw, cfg.pan_center_heading,
                                     cfg.pan_half_range_deg)
        err_tilt = aim_pitch - head_pitch
        info["err_pan"], info["err_tilt"], info["pan_clamped"] = err_pan, err_tilt, clamped

        pan_pos = head_yaw if cfg.pan_center_heading is None \
            else wrap180(head_yaw - cfg.pan_center_heading)
        pan = self.pan_ctl.update(err_pan, pan_pos, now)
        tilt = self.tilt_ctl.update(err_tilt, head_pitch, now)

        if self.pan_ctl.fault or self.tilt_ctl.fault:
            return done(0, 0, f"fault:{self.pan_ctl.fault or self.tilt_ctl.fault}")
        on = abs(err_pan) <= cfg.on_target_deg and abs(err_tilt) <= cfg.on_target_deg
        return done(pan, tilt, "on_target" if on else status_ok)
