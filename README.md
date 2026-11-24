# pantilt - Pelco-D control for an RS485 pan/tilt head

> This repository is also an **Eventide module** (`pan-tilt-pointing`): see [Eventide module](#eventide-module-pan-tilt-pointing) below.

A small Python library (`pantilt.py`) and testbench (`testbench.py`) for the RS485
pan/tilt head (model 303 manual: Pelco-D/Pelco-P adaptive, 1200-9600 baud, addresses 0-63).

```
PC --USB--> USB-serial adapter --TTL--> TTL-to-RS485 converter --A/B--> head (RS485+ / RS485-)
```

## Install

```
pip install pyserial
```

Dry-run mode (no hardware) works without pyserial.

## Head setup (DIP switches on the base)

Power-cycle the head after changing any switch.

**Address** - DIP 1-6, the sum of the ON switches: 1=1, 2=2, 3=4, 4=8, 5=16, 6=32.
Example: address 57 = 1+8+16+32, so switches 1, 4, 5, 6 ON.

**Baud rate** - DIP 7-8:

| Switch 7 | Switch 8 | Baud |
|---|---|---|
| ON | OFF | 1200 |
| ON | ON | 4800 |
| OFF | ON | 2400 |
| OFF | OFF | 9600 |

Wiring: orange = RS485+, yellow = RS485- (control plug pins 3 and 4). If the head does not
respond, swap the two data wires; wrong polarity gives no error, just no movement.

## Quick start

```python
from pantilt import PanTilt

with PanTilt("COM6", address=57, baud=9600) as head:
    head.move_for(pan=1, tilt=0, seconds=1.0)   # right for 1 s, then stop
    head.drive_to_limit("pan", -1)              # run into the left end stop
    head.drive_to_limit("tilt", -1)             # run into the bottom end stop
    head.move(pan=1)                            # start moving right...
    head.stop()                                 # ...until you stop it (e.g. on an IMU reading)
```

Leaving the `with` block (or Ctrl+C / any exception inside `move_for`) sends Stop.

## Testbench

```
python testbench.py                      # COM6, address 57, 9600 baud
python testbench.py --port COM3 --address 1 --baud 4800
python testbench.py --dry-run            # no hardware, instant
python testbench.py --skip rotation      # skip: axis | rotation | zigzag
python testbench.py --verbose            # print every frame
```

1. **Axis test** - right, left, up, down, 1 s each.
2. **Rotation test** - full pan sweep right then left; then, separately, tilt to the bottom limit,
   to the top limit, then half a sweep back down (roughly the middle, timed). About 2 minutes.
3. **Zig-zag test** - two passes across the pan range (left to right, then right to left). Each
   pass is 4 diagonal legs with tilt alternating up/down, so the two passes cover both halves of
   the tilt range.

The real run asks for Enter before moving. Ctrl+C stops the head.

## API

### `PanTilt(port=None, address=1, baud=9600, *, config=None, verbose=False)`
Context manager. `PanTilt.dry_run(address=1)` returns an instance that sends nothing and
sleeps instantly.

| Method | Description |
|---|---|
| `move(pan=0, tilt=0, speed=0x20)` | Start moving (non-blocking). `pan`/`tilt` are -1, 0, +1 (right/up positive). Runs until `stop()`. |
| `stop()` | Stop all motion. |
| `move_for(pan, tilt, seconds, speed=0x20, refresh_s=None)` | Move, wait, stop (stop is guaranteed). `refresh_s` re-sends the command periodically. |
| `drive_to_limit(axis, direction)` | Run `'pan'`/`'tilt'` into its end stop (full sweep time + margin). |
| `zigzag(pan_dir=1, legs=4, first_tilt_dir=1)` | Sweep the pan range while tilt alternates each leg. |
| `set_preset(n)` / `goto_preset(n)` | Head-stored presets. Reserved numbers are refused unless `allow_reserved=True`. |
| `send_raw(cmd1, cmd2, data1, data2)` | Send any Pelco-D command to this address. |
| `pause(seconds)` | Sleep (uses the dry-run clock when dry running). |

`build_frame(address, cmd1, cmd2, data1, data2)` builds a raw 7-byte frame.

### `HeadConfig`
Timings for the limit/sweep helpers: `pan_sweep_s=25.0`, `tilt_sweep_s=27.5`,
`limit_margin_s=2.0` (model 303 manual figures; see Calibration).

## Protocol

Pelco-D, 8N1:

```
0xFF | address | cmd1 | cmd2 | data1 (pan speed) | data2 (tilt speed) | checksum
checksum = (address + cmd1 + cmd2 + data1 + data2) & 0xFF
cmd2: 0x02 right, 0x04 left, 0x08 up, 0x10 down (OR together for diagonals)
Stop = cmd2 0x00.  Presets: 0x03 set, 0x05 clear, 0x07 go to (number in data2).
```

The head keeps moving until it receives Stop. Speed bytes are 0x00-0x3F; this head appears to
have fixed speeds, so they may have no effect.

**Reserved presets** act as commands on this head and are blocked by default: 17-19 (limit scan),
22, 70-72 (cruise dwell), 83-84 (start cruise), 89, 105 (self-test), 110-111 (guard position),
120 (erase all presets), 125 (restore defaults).

## No position tracking

The head cannot report its position (no feedback command is documented) and this library does
not estimate angles. It only sends commands and times moves. Use external sensors (compass,
IMU) on the base and head for orientation, and close the loop in your own code: read the
sensors, then call `move()` / `stop()`.

- Diagonal moves send both direction bits together. Because pan is much faster than tilt
  (about 7x), a "diagonal" is shallow: it is not 45 degrees.
- Presets are stored by the head itself and may be a repeatable way to return to a position,
  but they are untested on this head.

### Calibration
`HeadConfig` holds only timings, used by `drive_to_limit()` and `zigzag()`. The defaults come
from the model 303 manual (350 deg at 14 deg/s = 25 s pan; 55 deg at 2 deg/s = 27.5 s tilt). The
AliExpress listing quotes different speeds, so time a full sweep with a stopwatch (or an IMU) and
adjust:

```python
from pantilt import PanTilt, HeadConfig
cfg = HeadConfig(pan_sweep_s=28.0, tilt_sweep_s=20.0)   # your measured full-sweep times
head = PanTilt("COM6", 57, config=cfg)
```

## Safety

- Keep the area clear and cables free; stay near the supply during the first run.
- Do not rotate pan past 360 degrees (internal wiring); the limit post prevents this.
- `drive_to_limit` deliberately runs into the end stops for a short margin (default 2 s).
- Do not leave the head in automatic mode for long periods.
- DIP changes need a power cycle.

## Troubleshooting

| Symptom | Check |
|---|---|
| Nothing moves | Baud DIP 7-8 vs `baud`; address DIP 1-6 vs `address` (some hosts count from 1, some from 0); swap A/B wires; correct COM port; head power |
| Moves a bit then stops | Supply voltage too low or wiring too long |
| Head hits a stop early/late in the sweep tests | Calibrate `HeadConfig` sweep times |


---

## Eventide module: `pan-tilt-pointing`

Points the head using the **tilt-compensated-compass** module's compass and GPS, with four pointing
methods. Install it from the dashboard MODULES tab (repo URL or zip). It is a separate module from the
compass; it only *reads* the compass module's HTTP endpoints (through the eventide proxy by default).

### Assumptions
* The compass is mounted **on the moving head**, so its bearing/pitch *are* the head's azimuth/elevation.
  There is no base sensor and the head reports nothing, so the loop is closed in the **earth frame** on the
  compass; head angles relative to the base are only known through the travel limits below.
* The head is bang-bang (fixed speed): the controller starts moving when the error exceeds a start band,
  stops *early* by `rate x latency` so the coast lands on target, and re-sends the command every 0.5 s.

### Inputs
| Input | Default | Notes |
|---|---|---|
| Compass | `.../tilt-compensated-compass/http/api/telemetry` | `bearing`, `pitch`. Stale > 1 s stops the head. |
| GPS | `.../tilt-compensated-compass/gps_http/api/telemetry` | `latitude`, `longitude`, `altitude_m`, `has_lock`. Smoothed. |
| Manual position | off | `--manual-position lat,lon,alt` or the **MANUAL POSITION** widget / `PUT /api/observer`. **Overrides GPS** while enabled - for a stationary platform with no GPS. Values are remembered when toggled off. |
| Target endpoint | - | Any JSON URL with lat/lon/alt (dot-path keys), optional speed/track/vrate (m/s, deg). |

Observer priority: manual override > live GPS > last good GPS (held) > `--fallback`.

### Default target (auto-track at startup)
Set `track_host` to the IP of another Eventide payload and the module enters track mode on that payload as soon
as it starts, retrying every `track_interval` seconds until the target answers (nothing moves until a position
arrives and the compass is live). `track_preset` says what the other payload runs:

| `track_preset` | Endpoint used | Keys |
|---|---|---|
| `tripwire` (default) | `http://<host>/proxy/tripwire-gimbal-point/api/telemetry` | `lat`, `lon`, `alt` |
| `gps` | `http://<host>/proxy/tilt-compensated-compass/gps_http/api/telemetry` | `latitude`, `longitude`, `altitude_m` |

`track_host` may include a port (`10.0.0.2:5000`). `track_url` supplies a full URL instead (overrides the host),
and `track_lat_key` / `track_lon_key` / `track_alt_key` override the keys (blank = preset). The ENDPOINT TRACK widget
can still replace the target at any time; a restart returns to the default. **Note:** with a default target set the
module no longer starts idle - it begins pointing once the target and compass are available.

### Pointing modes (`/api/...`)
| Mode | Request | Behaviour |
|---|---|---|
| angle | `PUT /api/mode/angle {yaw, pitch, frame}` | `earth`: yaw = true bearing. `body`: yaw relative to the base (needs `pan_center_heading`). |
| latlon | `PUT /api/mode/latlon {lat, lon, alt}` | Fixed target; aim recomputed every tick from the observer (ECEF/ENU, so curvature is handled). |
| track | `PUT /api/mode/track {url, interval, lat_key, lon_key, alt_key, ...}` | Polls the endpoint, dead-reckons between fixes (lead time, 20 s extrapolation cap, alt offset) like tripwire-gimbal-point. |
| jog | `PUT /api/mode/jog {pan, tilt}` | Open-loop -1/0/+1; expires after 0.6 s unless re-sent. |
| idle | `POST /api/stop` | Stops the head. Module starts idle - it never moves until commanded. |

Also: `GET /api/telemetry`, `GET/PUT /api/offset/{yaw,pitch,lead_time,alt}` (aim nudges),
`GET/PUT /api/limits`, `POST /api/limits/center_here`, `GET/PUT/DELETE /api/observer`.

### Travel limits ("dead zones") - set these at startup
The head has ~350 deg of pan and +-35 deg of tilt but cannot report where it is, so the limits are described in
compass terms and are module arguments (re-set them if the platform is moved):

| Argument | Default | Meaning |
|---|---|---|
| `pan_center_heading` | 0 | **True bearing the head faces at mid-travel.** The dead gap sits directly behind it. Easiest way to set: jog the head to mid-travel and `POST /api/limits/center_here`, or edit it in the TRAVEL LIMITS widget. |
| `pan_range` | 350 | Total pan travel. |
| `tilt_min` / `tilt_max` | -35 / +35 | Compass pitch at the end stops. |
| `limit_margin` | 3 | Stay this far short of every end stop (usable pan = +-172 deg, tilt = +-32 deg). |

Aim points inside the dead gap are clamped to the window edge, and the pan never takes the shortest path if
that would cross the end stop. If `pan_center_heading` is wrong the head can still hit a stop - which the
safety trip below catches.

### Calibration widget (measures the travel for you)
**CALIBRATE TRAVEL** (+ **CALIBRATION RESULT**) runs the head into its end stops and uses the compass to find
where they are. Because the head has limit switches, an end stop just makes the compass stop changing:

1. Per axis, drive to the low end (left / down) until the compass has been still for 2 s - that is the low stop.
   (Starting on the stop is fine.)
2. Drive to the high end while integrating the unwrapped compass travel until it stalls again.

Results: **pan range**, the compass bearing of each pan stop and the **pan centre** (the bearing the head faces
at mid-travel, i.e. the base's orientation - this is what `pan_center_heading` needs), and the **tilt min / max /
range** as compass pitch. With *Apply* on (default) these become the live travel limits; with *Centre* on the head
then drives to mid-travel using them. Axes: `pan`, `tilt`, `both`; `cancel` aborts a run (so does Stop).
API: `PUT /api/calibrate {"axes", "apply", "center"}`, `GET /api/calibrate`. Results are also logged to
supervisor (`[pantilt] calibration done: ...`) - copy them into the module arguments to keep them across a restart.

It aborts with an explanation, rather than leaving the head running, when the compass goes stale, a phase times
out (`cal_pan_timeout` 90 s / `cal_tilt_timeout` 120 s per sweep - raise them for a slow head), the pan turns more than a full circle (no limit switch), the head does not move at all (power / wiring /
compass not updating), or the compass moves against the commanded direction - for tilt it tells you to set
`pitch_sign` to -1. The head moves at its full speed to both extremes (~1-2 minutes for tilt), so clear the area
and make sure cables have slack. Needs a connected head and a live compass.

### Safety
* Starts **idle**; stops on stale compass, no observer, no track fix, or lost serial link; Stop is sent on exit.
* If an axis is driven for 2.5 s without actually moving in the commanded direction (wrong `pitch_sign`,
  dead compass, stalled head, end stop) it trips `fault: pan_not_moving` / `tilt_not_moving` and stops.
  A new mode command clears the fault.

### Calibration checklist
1. Check compass is on the head and reads sensible bearings; set `declination` (and `yaw_offset` if the compass
   is rotated on the head).
2. Jog **up**; if the telemetry `head_pitch` falls, set `pitch_sign` to -1.
3. Move the head to mid-travel, `POST /api/limits/center_here`.
4. Point at a known landmark with `angle` mode and use the AIM NUDGE widget to trim.

### Tests
```
python -m unittest -v test_pointing test_service     # simulated head + fake endpoints, no hardware
```
