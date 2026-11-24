"""
calibration.py - end-stop travel measurement for the pan/tilt head.

The head has limit switches, so driving an axis into its end stop makes it
stall, and the head-mounted compass simply stops changing. Per axis:

  1. drive to the LOW end (left / down) until the compass has been still for
     `cal_stall_s` - that is the low stop (already being there is fine);
  2. drive to the HIGH end (right / up), integrating the unwrapped compass
     travel, until it stalls again. The integrated travel is the usable range.

From the stop readings we get the pan centre (the bearing the head faces at
mid-travel, i.e. the base's orientation) and the tilt min/max pitch.

Pure state machine (no I/O): call step(now, yaw, pitch) -> (pan_dir, tilt_dir).
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

from pointing import wrap180, wrap360


class Calibrator:
    def __init__(self, cfg, axes: str, now: float):
        if axes not in ("pan", "tilt", "both"):
            raise ValueError("axes must be 'pan', 'tilt' or 'both'")
        self.cfg = cfg
        self.axes = axes
        self.state = "running"             # running | done | error
        self.error: Optional[str] = None
        self.hint: Optional[str] = None
        self.result: Dict[str, Dict[str, float]] = {}
        names = ["pan"] if axes == "pan" else ["tilt"] if axes == "tilt" else ["pan", "tilt"]
        self._phases = [(a, k) for a in names for k in ("low", "high")]
        self._i = 0
        self._init_phase = True
        self._settle_until = 0.0
        self._low_pos = 0.0
        self._t0 = self._last_change = now
        self._ref = self._prev = 0.0
        self.axis, self.kind = self._phases[0]
        self.started = now
        self.travel = 0.0

    # -- helpers
    @staticmethod
    def _diff(axis: str, a: float, b: float) -> float:
        return wrap180(a - b) if axis == "pan" else a - b

    @property
    def phase(self) -> str:
        return "finished" if self.state != "running" else f"{self.axis}_{self.kind}"

    def _fail(self, msg: str, hint: Optional[str] = None) -> Tuple[int, int]:
        self.state, self.error, self.hint = "error", msg, hint
        return 0, 0

    def progress(self) -> Dict[str, object]:
        return {"state": self.state, "phase": self.phase, "axes": self.axes,
                "error": self.error, "hint": self.hint, "result": self.result,
                "travel": self.travel}

    # -- stepping
    def step(self, now: float, yaw: float, pitch: float) -> Tuple[int, int]:
        if self.state != "running":
            return 0, 0
        cfg = self.cfg
        if now < self._settle_until:
            return 0, 0
        axis, kind = self._phases[self._i]
        self.axis, self.kind = axis, kind
        pos = yaw if axis == "pan" else pitch
        if self._init_phase:
            self._init_phase = False
            self._t0 = self._last_change = now
            self._ref = self._prev = pos
            self.travel = 0.0

        # unwrapped travel since the phase began, and stall bookkeeping
        self.travel += self._diff(axis, pos, self._prev)
        self._prev = pos
        if abs(self._diff(axis, pos, self._ref)) > cfg.cal_eps_deg:
            self._ref, self._last_change = pos, now

        rate = cfg.pan_rate_dps if axis == "pan" else cfg.tilt_rate_dps
        full = 360.0 if axis == "pan" else 90.0
        if axis == "pan" and abs(self.travel) > full + 5.0:
            return self._fail("no end stop found: pan travelled more than a full turn",
                              "are the head's limit switches working?")
        if now - self._t0 > 1.6 * full / rate + cfg.cal_stall_s:
            return self._fail(f"{axis} {kind}: timed out before the head stalled")

        sign = -1 if kind == "low" else 1
        if now - self._last_change < cfg.cal_stall_s:
            return (sign, 0) if axis == "pan" else (0, sign)

        # ---- stalled: the compass stopped changing => end stop reached
        if kind == "low":
            self._low_pos = pos
        else:
            rng, lo = self.travel, self._low_pos
            min_range = 20.0 if axis == "pan" else 5.0
            if rng < -min_range:
                return self._fail(
                    f"{axis} compass moved the wrong way ({rng:.1f} deg while driving positive)",
                    "set pitch_sign to -1" if axis == "tilt"
                    else "the compass turns against the pan direction - check its mounting")
            if rng < min_range:
                return self._fail(
                    f"{axis} did not move ({rng:.1f} deg): head unpowered or wired wrong, "
                    "or the compass is not updating")
            if axis == "pan":
                self.result["pan"] = {"left_heading": lo, "right_heading": pos, "range": rng,
                                      "center_heading": wrap360(lo + rng / 2.0)}
            else:
                self.result["tilt"] = {"min": lo, "max": pos, "range": rng}
        self._i += 1
        self._init_phase = True
        self._settle_until = now + cfg.cal_settle_s
        if self._i >= len(self._phases):
            self.state = "done"
        return 0, 0
