#!/usr/bin/env python3
"""
pantilt_service.py - Eventide service that points a Pelco-D pan/tilt head.

Inputs
  * compass endpoint  - the tilt-compensated-compass module (mounted on the head)
  * GPS endpoint      - the same module's GPS service (platform position)
  * track endpoint    - any JSON URL giving lat/lon/alt of a target (optional)

Pointing modes
  idle     stop
  jog      open-loop manual direction
  angle    point at a yaw (true bearing) / pitch (elevation)
  latlon   point at a fixed latitude / longitude / altitude
  track    follow a target polled from an arbitrary JSON endpoint

The head has no position feedback; the loop is closed on the head-mounted
compass (see control.py). Travel limits are startup parameters (see README).

    python3 pantilt_service.py --http-port 8090 --dry-run
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import signal
import time
from typing import Any, Dict, Optional

from aiohttp import ClientSession, ClientTimeout, web

from control import MODES, Pointer, PointerConfig
from pointing import Fix, wrap360
from pantilt import PanTilt, PanTiltError

# ---------------------------------------------------------------------------- helpers
SPEED_ALIASES = ("speed", "groundspeed", "ground_speed", "gs")
TRACK_ALIASES = ("track", "heading", "hdg", "course")
VRATE_ALIASES = ("vertical_rate", "vrate", "climb", "climb_rate")


def dig(data: Any, path: Optional[str]) -> Any:
    """Follow a dot-path into nested dicts; None if anything is missing."""
    if not path:
        return None
    for part in path.split("."):
        if not isinstance(data, dict):
            return None
        data = data.get(part)
    return data


def num(v: Any) -> Optional[float]:
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def probe(data: Any, explicit: Optional[str], aliases) -> Optional[float]:
    for key in ([explicit] if explicit else aliases):
        v = num(dig(data, key))
        if v is not None:
            return v
    return None


def first_object(data: Any) -> Optional[dict]:
    if isinstance(data, list):
        data = data[0] if data else None
    return data if isinstance(data, dict) else None


async def fetch_json(session: ClientSession, url: str) -> Any:
    async with session.get(url) as resp:
        resp.raise_for_status()
        return json.loads(await resp.text())


def bad(msg: str, status: int = 400) -> web.Response:
    return web.json_response({"error": msg}, status=status)


async def body(request: web.Request) -> dict:
    try:
        data = await request.json()
    except Exception:
        raise web.HTTPBadRequest(text=json.dumps({"error": "invalid JSON body"}),
                                 content_type="application/json")
    if not isinstance(data, dict):
        raise web.HTTPBadRequest(text=json.dumps({"error": "JSON object expected"}),
                                 content_type="application/json")
    return data


def need_num(data: dict, key: str, default: Optional[float] = None) -> float:
    v = data.get(key, default)
    f = num(v)
    if f is None:
        raise web.HTTPBadRequest(text=json.dumps({"error": f"'{key}' must be a number"}),
                                 content_type="application/json")
    return f


# ---------------------------------------------------------------------------- service
class PanTiltService:
    def __init__(self, args: argparse.Namespace, pointer: Pointer, head: Optional[PanTilt]):
        self.args = args
        self.pointer = pointer
        self.head = head
        self.head_ok = head is not None
        self.head_error: Optional[str] = None
        self.sent = (0, 0)
        self._sent_t = 0.0
        self.status: Dict[str, Any] = {}
        self.sources: Dict[str, Dict[str, Any]] = {
            "compass": {"url": args.compass_url, "ok": False, "error": None, "t": None},
            "gps": {"url": args.gps_url, "ok": False, "error": None, "t": None},
        }
        # endpoint track config (set via PUT /api/mode/track)
        self.track_cfg: Dict[str, Any] = {"url": "", "interval": 1.0, "label": "",
                                          "lat_key": "lat", "lon_key": "lon",
                                          "alt_key": "alt", "speed_key": "",
                                          "track_key": "", "vrate_key": ""}
        self.track_state: Dict[str, Any] = {"active": False, "ok": False, "error": None}
        self._track_task: Optional[asyncio.Task] = None
        self._tasks: list = []
        self._shutdown = asyncio.Event()

    # ---- pollers ------------------------------------------------------------
    async def _poll_compass(self, session: ClientSession) -> None:
        a, src = self.args, self.sources["compass"]
        while not self._shutdown.is_set():
            try:
                d = await fetch_json(session, a.compass_url)
                yaw, pitch = num(dig(d, a.compass_yaw_key)), num(dig(d, a.compass_pitch_key))
                if yaw is None or pitch is None:
                    raise ValueError("heading/pitch missing from compass reply")
                roll = num(dig(d, "roll")) or 0.0
                self.pointer.set_compass(yaw, pitch, roll, time.time())
                src.update(ok=True, error=None, t=time.time())
            except Exception as exc:
                src.update(ok=False, error=str(exc) or type(exc).__name__)
            await asyncio.sleep(a.compass_interval)

    async def _poll_gps(self, session: ClientSession) -> None:
        a, src = self.args, self.sources["gps"]
        while not self._shutdown.is_set():
            try:
                d = await fetch_json(session, a.gps_url)
                lat, lon = num(d.get("latitude")), num(d.get("longitude"))
                alt = num(d.get("altitude_m")) or 0.0
                if lat is None or lon is None:
                    raise ValueError("no position in GPS reply")
                self.pointer.set_gps(lat, lon, alt, bool(d.get("has_lock")), time.time())
                src.update(ok=True, error=None, t=time.time())
            except Exception as exc:
                src.update(ok=False, error=str(exc) or type(exc).__name__)
            await asyncio.sleep(a.gps_interval)

    async def _poll_track(self, session: ClientSession, cfg: Dict[str, Any]) -> None:
        st = self.track_state
        while True:
            try:
                d = first_object(await fetch_json(session, cfg["url"]))
                lat, lon, alt = (num(dig(d, cfg["lat_key"])), num(dig(d, cfg["lon_key"])),
                                 num(dig(d, cfg["alt_key"])))
                if lat is None or lon is None or alt is None:
                    raise ValueError("lat/lon/alt not found in endpoint reply")
                self.pointer.set_track_fix(Fix(
                    lat, lon, alt, time.time(),
                    speed=probe(d, cfg["speed_key"] or None, SPEED_ALIASES),
                    track=probe(d, cfg["track_key"] or None, TRACK_ALIASES),
                    vrate=probe(d, cfg["vrate_key"] or None, VRATE_ALIASES)))
                st.update(ok=True, error=None, t=time.time(), lat=lat, lon=lon, alt_m=alt)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                st.update(ok=False, error=str(exc) or type(exc).__name__)
            await asyncio.sleep(cfg["interval"])

    def begin_track(self, cfg: Dict[str, Any]) -> None:
        """Switch to track mode and start polling `cfg` (retries forever if the
        endpoint is unreachable, so the target can come online later)."""
        self.stop_track_task()
        self.track_cfg = cfg
        self.track_state.update(active=True, ok=False, error=None)
        self.pointer.follow_track(cfg["label"] or cfg["url"])
        self._track_task = asyncio.create_task(self._poll_track(self._session, cfg))

    def stop_track_task(self) -> None:
        if self._track_task:
            self._track_task.cancel()
            self._track_task = None
        self.track_state["active"] = False

    # ---- control loop ---------------------------------------------------------
    def _send(self, pan: int, tilt: int, now: float) -> None:
        if self.head is None:
            return
        # Re-send periodically even when unchanged: a lost frame (especially a
        # lost Stop) must never leave the head running.
        if (pan, tilt) == self.sent and now - self._sent_t < self.args.refresh_s:
            return
        try:
            if pan == 0 and tilt == 0:
                self.head.stop()
            else:
                self.head.move(pan, tilt)
            self.sent, self._sent_t = (pan, tilt), now
            self.head_ok, self.head_error = True, None
        except Exception as exc:
            self.head_ok, self.head_error = False, str(exc)

    async def _control_loop(self) -> None:
        period = self.args.period
        while not self._shutdown.is_set():
            now = time.time()
            pan, tilt, info = self.pointer.step(now)
            # Never drive the head unless the *mode* wants motion; a stale
            # status already yields (0, 0) from the pointer.
            self._send(pan, tilt, now)
            info["head_connected"] = self.head_ok
            info["head_error"] = self.head_error
            if info.get("status") in ("calibration_done", "calibration_error"):
                cal = self.pointer.calibration_status()     # log once, with the values to
                print(f"[pantilt] calibration {cal['state']}: {cal['result']} "     # keep as args
                      f"error={cal['error']} applied={cal['applied']}", flush=True)
            self.status = info
            await asyncio.sleep(period)

    # ---- lifecycle ------------------------------------------------------------
    async def start(self) -> None:
        timeout = ClientTimeout(total=self.args.http_timeout)
        self._session = ClientSession(timeout=timeout)
        if self.args.compass_url:
            self._tasks.append(asyncio.create_task(self._poll_compass(self._session)))
        if self.args.gps_url:
            self._tasks.append(asyncio.create_task(self._poll_gps(self._session)))
        self._tasks.append(asyncio.create_task(self._control_loop()))

    async def stop(self) -> None:
        self._shutdown.set()
        self.stop_track_task()
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        await self._session.close()
        if self.head is not None:
            try:
                self.head.close()          # sends Stop
            except Exception:
                pass

    # ---- API ------------------------------------------------------------------
    def telemetry(self) -> Dict[str, Any]:
        p, c, now = self.pointer, self.pointer.cfg, time.time()
        s = dict(self.status)
        s.update({
            "t": now,
            "sources": self.sources,
            "track": dict(self.track_state, label=p.track_label,
                          age=p.dr.age(now), lead_time_s=c.lead_time_s),
            "nudge": {"yaw": p.yaw_nudge, "pitch": p.pitch_nudge},
            "limits": self.limits(),
            "cmd_yaw": p.angle_cmd[0], "cmd_pitch": p.angle_cmd[1],
        })
        return s

    def limits(self) -> Dict[str, Any]:
        c = self.pointer.cfg
        return {"pan_center_heading": c.pan_center_heading, "pan_range_deg": c.pan_range_deg,
                "tilt_min_deg": c.tilt_min_deg, "tilt_max_deg": c.tilt_max_deg,
                "limit_margin_deg": c.limit_margin_deg,
                "pan_usable_half_range_deg": c.pan_half_range_deg,
                "tilt_usable_min_deg": c.tilt_lo, "tilt_usable_max_deg": c.tilt_hi,
                "limits_set": c.pan_center_heading is not None}


def build_app(svc: PanTiltService) -> web.Application:
    p = svc.pointer
    r = web.RouteTableDef()

    @r.get("/api/health")
    async def health(_):
        return web.json_response({"ok": svc.head_ok and svc.sources["compass"]["ok"]})

    @r.get("/api/telemetry")
    async def telemetry(_):
        return web.json_response(svc.telemetry())

    @r.get("/api/track")          # for the dashboard map widget
    async def map_track(_):
        s = svc.status
        obs = s.get("observer")
        out = {"lat": obs[0] if obs else None, "lon": obs[1] if obs else None,
               "heading": p.cfg.pan_center_heading, "gimbal": s.get("head_yaw"), "frame": 0}
        if s.get("target"):
            out["target"] = {"lat": s["target"]["lat"], "lon": s["target"]["lon"]}
        return web.json_response(out)

    @r.post("/api/stop")
    async def stop(_):
        svc.stop_track_task()
        p.stop()
        return web.json_response({"status": "ok", "mode": "idle"})

    # -- calibration: run the head into its end stops to measure travel
    def calibration_view() -> Dict[str, Any]:
        st = p.calibration_status()
        opts = st.pop("options")
        return {**opts, **st}

    @r.get("/api/calibrate")
    async def get_calibrate(_):
        return web.json_response(calibration_view())

    @r.put("/api/calibrate")
    async def put_calibrate(req):
        """{"axes": "pan"|"tilt"|"both"|"cancel", "apply": bool, "center": bool}"""
        d = await body(req)
        axes = str(d.get("axes", "both"))
        if axes == "cancel":
            svc.stop_track_task()
            if p.mode == "calibrate":
                p.stop()
            return web.json_response(calibration_view())
        if axes not in ("pan", "tilt", "both"):
            return bad("axes must be pan, tilt, both or cancel")
        if not svc.head_ok or svc.head is None:
            return bad("head is not connected", 409)
        if not svc.sources["compass"]["ok"] or p.head_attitude(time.time()) is None:
            return bad("compass is not available - calibration needs it", 409)
        svc.stop_track_task()
        p.start_calibration(axes, bool(d.get("apply", True)), bool(d.get("center", True)),
                            time.time())
        print(f"[pantilt] calibration started: axes={axes} apply={d.get('apply', True)}",
              flush=True)
        return web.json_response(calibration_view())

    # -- modes
    @r.get("/api/mode/latlon")
    async def get_latlon(_):
        lat, lon, alt = p.target_fixed
        return web.json_response({"lat": lat, "lon": lon, "alt": alt})

    @r.put("/api/mode/latlon")
    async def put_latlon(req):
        d = await body(req)
        lat, lon = need_num(d, "lat"), need_num(d, "lon")
        alt = need_num(d, "alt", 0.0)
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return bad("lat/lon out of range")
        svc.stop_track_task()
        p.point_latlon(lat, lon, alt)
        return web.json_response({"status": "ok", "mode": "latlon"})

    @r.get("/api/mode/angle")
    async def get_angle(_):
        return web.json_response({"yaw": p.angle_cmd[0], "pitch": p.angle_cmd[1],
                                  "frame": "earth"})

    @r.put("/api/mode/angle")
    async def put_angle(req):
        d = await body(req)
        yaw, pitch = need_num(d, "yaw", d.get("pan")), need_num(d, "pitch", d.get("tilt"))
        frame = d.get("frame", "earth")
        if isinstance(frame, bool):          # dashboard joystick: true = base ("body") frame
            frame = "body" if frame else "earth"
        svc.stop_track_task()
        try:
            p.point_angle(yaw, pitch, str(frame))
        except ValueError as exc:
            return bad(str(exc))
        return web.json_response({"status": "ok", "mode": "angle"})

    @r.put("/api/mode/jog")
    async def put_jog(req):
        d = await body(req)
        svc.stop_track_task()
        p.jog(int(need_num(d, "pan", 0)), int(need_num(d, "tilt", 0)), time.time())
        return web.json_response({"status": "ok", "mode": "jog"})

    @r.get("/api/mode/track")
    async def get_track(_):
        return web.json_response(dict(svc.track_cfg, **svc.track_state,
                                      label=p.track_label or svc.track_cfg["label"]))

    @r.put("/api/mode/track")
    async def put_track(req):
        d = await body(req)
        url = str(d.get("url", "")).strip()
        if not url.startswith(("http://", "https://")):
            return bad("url must start with http:// or https://")
        interval = num(d.get("interval")) or 1.0
        if interval <= 0:
            return bad("interval must be positive")
        cfg = dict(svc.track_cfg)
        cfg.update(url=url, interval=interval, label=str(d.get("label") or "").strip())
        for k in ("lat_key", "lon_key", "alt_key", "speed_key", "track_key", "vrate_key"):
            if k in d:
                if not isinstance(d[k], str):
                    return bad(f"'{k}' must be a string")
                if d[k].strip():
                    cfg[k] = d[k].strip()
        svc.begin_track(cfg)
        return web.json_response({"status": "ok", "mode": "track", "url": url})

    @r.delete("/api/mode/track")
    async def del_track(_):
        svc.stop_track_task()
        p.stop()
        return web.json_response({"status": "ok", "mode": "idle"})

    # -- manual observer position (overrides GPS; for stationary use without GPS)
    def manual_state():
        pos = p._obs_override
        last = pos or p.manual_last or (0.0, 0.0, 0.0)
        return {"enabled": pos is not None, "lat": last[0], "lon": last[1], "alt": last[2]}

    @r.get("/api/observer")
    async def get_observer(_):
        return web.json_response(manual_state())

    @r.put("/api/observer")
    async def put_observer(req):
        d = await body(req)
        if d.get("enabled") is False:
            p.set_observer_override(None)
            return web.json_response(manual_state())
        lat, lon = need_num(d, "lat"), need_num(d, "lon")
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return bad("lat/lon out of range")
        p.set_observer_override((lat, lon, need_num(d, "alt", 0.0)))
        return web.json_response(manual_state())

    @r.delete("/api/observer")
    async def del_observer(_):
        p.set_observer_override(None)
        return web.json_response(manual_state())

    # -- nudges / tracking offsets (one endpoint each so `nudge` form fields work)
    def scalar(path: str, key: str, getter, setter):
        async def get(_):
            return web.json_response({key: getter()})

        async def put(req):
            d = await body(req)
            setter(need_num(d, key))
            return web.json_response({"status": "ok", key: getter()})
        r.get(path)(get)
        r.put(path)(put)

    def set_attr(obj, name):
        return lambda v: setattr(obj, name, v)
    scalar("/api/offset/yaw", "yaw_offset_deg", lambda: p.yaw_nudge, set_attr(p, "yaw_nudge"))
    scalar("/api/offset/pitch", "pitch_offset_deg", lambda: p.pitch_nudge,
           set_attr(p, "pitch_nudge"))
    scalar("/api/offset/lead_time", "lead_time_s", lambda: p.cfg.lead_time_s, p.set_lead_time)
    scalar("/api/offset/alt", "alt_offset_m", lambda: p.cfg.alt_offset_m,
           set_attr(p.cfg, "alt_offset_m"))

    # -- travel limits ("dead zones")
    @r.get("/api/limits")
    async def get_limits(_):
        return web.json_response(svc.limits())

    @r.put("/api/limits")
    async def put_limits(req):
        d = await body(req)
        kw: Dict[str, Any] = {}
        if "pan_center_heading" in d:
            kw["center"] = None if d["pan_center_heading"] in (None, "") \
                else need_num(d, "pan_center_heading")
        for src, dst in (("pan_range_deg", "pan_range"), ("tilt_min_deg", "tilt_min"),
                         ("tilt_max_deg", "tilt_max"), ("limit_margin_deg", "margin")):
            if src in d:
                kw[dst] = need_num(d, src)
        try:
            p.set_limits(**kw)
        except ValueError as exc:
            return bad(str(exc))
        return web.json_response(svc.limits())

    @r.post("/api/limits/center_here")
    async def center_here(_):
        """Calibration: the head has been moved to mid-travel; record its heading."""
        try:
            hdg = p.set_center_here(time.time())
        except ValueError as exc:
            return bad(str(exc), 409)
        return web.json_response({"status": "ok", "pan_center_heading": hdg})

    app = web.Application()
    app.add_routes(r)
    return app


# ---------------------------------------------------------------------------- main
# Endpoint presets for --track-host: URL path on the other payload + JSON keys.
TRACK_PRESETS = {
    # another payload running tripwire-gimbal-point (platform position from MAVLink)
    "tripwire": {"path": "/proxy/tripwire-gimbal-point/api/telemetry",
                 "lat_key": "lat", "lon_key": "lon", "alt_key": "alt"},
    # another payload running the tilt-compensated-compass GPS service
    "gps": {"path": "/proxy/tilt-compensated-compass/gps_http/api/telemetry",
            "lat_key": "latitude", "lon_key": "longitude", "alt_key": "altitude_m"},
}


def default_track_config(a: argparse.Namespace) -> Optional[Dict[str, Any]]:
    """Track config built from startup args, or None when no default target is set.

    --track-url (full URL) wins; otherwise --track-host (IP / hostname, optional
    :port) is expanded with the --track-preset endpoint. Key args left blank
    take the preset's keys (or lat/lon/alt for a bare --track-url).
    """
    host, url = (a.track_host or "").strip(), (a.track_url or "").strip()
    if not host and not url:
        return None
    preset = TRACK_PRESETS[a.track_preset]
    if url:
        if not url.startswith(("http://", "https://")):
            raise SystemExit("--track-url must start with http:// or https://")
        keys = {"lat_key": "lat", "lon_key": "lon", "alt_key": "alt"}
    else:
        host = host.removeprefix("http://").removeprefix("https://").rstrip("/")
        url = f"http://{host}{preset['path']}"
        keys = {k: preset[k] for k in ("lat_key", "lon_key", "alt_key")}
    cfg = {"url": url, "interval": a.track_interval,
           "label": (a.track_label or host or url).strip(),
           "speed_key": "", "track_key": "", "vrate_key": ""}
    for k in ("lat_key", "lon_key", "alt_key"):
        cfg[k] = (getattr(a, "track_" + k) or "").strip() or keys[k]
    return cfg


def parse_fallback(s: str):
    s = (s or "").strip()
    if not s:
        return None
    try:
        la, lo, al = (float(x) for x in s.split(","))
        return la, lo, al
    except ValueError:
        raise SystemExit("--fallback must be 'lat,lon,alt'")


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Eventide pan/tilt pointing service")
    ap.add_argument("--http-port", type=int, required=True)
    ap.add_argument("--dry-run", action="store_true", help="no serial port; log only")
    # head
    ap.add_argument("--serial-port", default="/dev/ttyUSB0")
    ap.add_argument("--address", type=int, default=57)
    ap.add_argument("--baud", type=int, default=9600)
    # inputs
    base = "http://127.0.0.1:5000/proxy/tilt-compensated-compass"
    ap.add_argument("--compass-url", default=f"{base}/http/api/telemetry")
    ap.add_argument("--gps-url", default=f"{base}/gps_http/api/telemetry")
    ap.add_argument("--compass-yaw-key", default="bearing")
    ap.add_argument("--compass-pitch-key", default="pitch")
    ap.add_argument("--compass-interval", type=float, default=0.1)
    ap.add_argument("--gps-interval", type=float, default=1.0)
    ap.add_argument("--http-timeout", type=float, default=3.0)
    ap.add_argument("--track-host", default="",
                    help="IP/hostname of another eventide payload to track automatically at startup")
    ap.add_argument("--track-preset", default="tripwire", choices=sorted(TRACK_PRESETS),
                    help="what the --track-host payload runs: tripwire-gimbal-point or the TCC GPS")
    ap.add_argument("--track-url", default="",
                    help="full endpoint URL to track at startup (overrides --track-host)")
    ap.add_argument("--track-label", default="")
    ap.add_argument("--track-interval", type=float, default=1.0)
    ap.add_argument("--track-lat-key", default="", help="blank = preset default")
    ap.add_argument("--track-lon-key", default="")
    ap.add_argument("--track-alt-key", default="")
    ap.add_argument("--fallback", default="", help="lat,lon,alt used until GPS has a fix")
    ap.add_argument("--manual-position", default="",
                    help="lat,lon,alt observer position that OVERRIDES GPS (no-GPS / stationary use)")
    # calibration
    ap.add_argument("--declination", type=float, default=0.0)
    ap.add_argument("--yaw-offset", type=float, default=0.0)
    ap.add_argument("--pitch-offset", type=float, default=0.0)
    ap.add_argument("--pitch-sign", type=int, default=1, choices=(-1, 1))
    # travel limits
    ap.add_argument("--pan-center-heading", type=float, default=None,
                    help="true bearing of the head at mid-travel (omit = pan unconstrained)")
    ap.add_argument("--pan-range", type=float, default=350.0)
    ap.add_argument("--tilt-min", type=float, default=-35.0)
    ap.add_argument("--tilt-max", type=float, default=35.0)
    ap.add_argument("--limit-margin", type=float, default=3.0)
    # loop tuning
    ap.add_argument("--period", type=float, default=0.1)
    ap.add_argument("--refresh-s", type=float, default=0.5)
    ap.add_argument("--pan-rate", type=float, default=14.0)
    ap.add_argument("--tilt-rate", type=float, default=2.0)
    ap.add_argument("--latency", type=float, default=0.25)
    ap.add_argument("--smooth", type=float, default=0.1)
    ap.add_argument("--lead-time", type=float, default=1.0)
    return ap.parse_args(argv)


def make_pointer(a: argparse.Namespace) -> Pointer:
    ptr = Pointer(PointerConfig(
        declination_deg=a.declination, yaw_offset_deg=a.yaw_offset,
        pitch_offset_deg=a.pitch_offset, pitch_sign=a.pitch_sign,
        pan_center_heading=None if a.pan_center_heading is None else wrap360(a.pan_center_heading),
        pan_range_deg=a.pan_range, tilt_min_deg=a.tilt_min, tilt_max_deg=a.tilt_max,
        limit_margin_deg=a.limit_margin, pan_rate_dps=a.pan_rate, tilt_rate_dps=a.tilt_rate,
        latency_s=a.latency, smooth=a.smooth, lead_time_s=a.lead_time,
        fallback=parse_fallback(a.fallback)))
    manual = parse_fallback(a.manual_position)
    if manual is not None:
        ptr.set_observer_override(manual)
    return ptr


async def open_head(a: argparse.Namespace) -> Optional[PanTilt]:
    if a.dry_run:
        return PanTilt.dry_run(address=a.address)
    try:
        return PanTilt(a.serial_port, address=a.address, baud=a.baud).open()
    except Exception as exc:       # keep the API up so the problem is visible
        print(f"[pantilt] cannot open {a.serial_port}: {exc}", flush=True)
        return None


async def main() -> None:
    a = parse_args()
    pointer = make_pointer(a)
    head = await open_head(a)
    svc = PanTiltService(a, pointer, head)
    if head is None:
        svc.head_error = f"cannot open {a.serial_port}"
    await svc.start()
    track = default_track_config(a)
    if track is not None:
        svc.begin_track(track)
        print(f"[pantilt] auto-tracking {track['url']} ({track['label']})", flush=True)

    runner = web.AppRunner(build_app(svc))
    await runner.setup()
    await web.TCPSite(runner, "0.0.0.0", a.http_port).start()
    lim = svc.limits()
    print(f"[pantilt] HTTP API on :{a.http_port}; head={'dry-run' if a.dry_run else a.serial_port}"
          f" addr {a.address}; limits_set={lim['limits_set']} centre={lim['pan_center_heading']}",
          flush=True)
    if not lim["limits_set"]:
        print("[pantilt] WARNING: --pan-center-heading not set; pan travel is NOT limited",
              flush=True)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            loop.add_signal_handler(sig, svc._shutdown.set)
        except NotImplementedError:    # Windows
            pass
    try:
        await svc._shutdown.wait()
    finally:
        await runner.cleanup()
        await svc.stop()


if __name__ == "__main__":
    asyncio.run(main())
