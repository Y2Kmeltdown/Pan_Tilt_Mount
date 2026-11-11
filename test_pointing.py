"""Unit tests for pointing.py / control.py. Run:  python -m unittest -v test_pointing"""
import unittest

from control import AxisController, Pointer, PointerConfig
from pointing import (DeadReckoner, Fix, look_angles, move_latlon, pan_error,
                      wrap180, wrap360)


class Geometry(unittest.TestCase):
    def test_wrap(self):
        self.assertEqual(wrap360(-10), 350)
        self.assertEqual(wrap180(190), -170)
        self.assertEqual(wrap180(-180), -180)

    def test_cardinal_directions(self):
        obs = (-33.0, 150.0, 10.0)
        az, el, _ = look_angles(obs, (-32.99, 150.0, 10.0))        # north
        self.assertAlmostEqual(az, 0.0, delta=0.01)
        self.assertAlmostEqual(el, 0.0, delta=0.05)
        az, _, _ = look_angles(obs, (-33.0, 150.01, 10.0))         # east
        self.assertAlmostEqual(az, 90.0, delta=0.01)
        az, _, _ = look_angles(obs, (-33.01, 150.0, 10.0))         # south
        self.assertAlmostEqual(az, 180.0, delta=0.01)
        az, _, _ = look_angles(obs, (-33.0, 149.99, 10.0))         # west
        self.assertAlmostEqual(az, 270.0, delta=0.01)

    def test_elevation_45(self):
        obs = (0.0, 0.0, 0.0)
        e, n, u = 0.0, 1000.0, 1000.0
        tgt_lat, tgt_lon = move_latlon(0.0, 0.0, 1000.0, 0.0, 1.0)  # 1 km north
        az, el, rng = look_angles(obs, (tgt_lat, tgt_lon, 1000.0))
        self.assertAlmostEqual(el, 45.0, delta=0.1)
        self.assertAlmostEqual(rng, 1414.2, delta=2.0)

    def test_earth_curvature_drop(self):
        # A target at the same altitude 100 km away is below the local horizontal.
        la, lo = move_latlon(0.0, 0.0, 100_000.0, 90.0, 1.0)
        _, el, _ = look_angles((0.0, 0.0, 0.0), (la, lo, 0.0))
        self.assertAlmostEqual(el, -0.45, delta=0.05)               # ~ -d/(2R) rad

    def test_overhead(self):
        _, el, _ = look_angles((10.0, 20.0, 0.0), (10.0, 20.0, 500.0))
        self.assertAlmostEqual(el, 90.0, delta=0.01)

    def test_move_latlon_roundtrip(self):
        la, lo = move_latlon(-33.0, 150.0, 100.0, 45.0, 10.0)       # 1 km NE
        az, _, rng = look_angles((-33.0, 150.0, 0.0), (la, lo, 0.0))
        self.assertAlmostEqual(az, 45.0, delta=0.05)
        self.assertAlmostEqual(rng, 1000.0, delta=1.0)


class DeadReckoning(unittest.TestCase):
    def test_extrapolates_and_caps(self):
        dr = DeadReckoner(lead_s=0.0, max_extrap_s=10.0)
        dr.update(Fix(0.0, 0.0, 100.0, t=0.0, speed=100.0, track=90.0, vrate=1.0))
        la, lo, alt = dr.predict(5.0)
        self.assertAlmostEqual(alt, 105.0, places=3)
        _, _, rng5 = look_angles((0, 0, 100), (la, lo, 100))
        self.assertAlmostEqual(rng5, 500.0, delta=1.0)
        la2, lo2, alt2 = dr.predict(60.0)                            # capped at 10 s
        self.assertAlmostEqual(alt2, 110.0, places=3)
        _, _, rng10 = look_angles((0, 0, 100), (la2, lo2, 100))
        self.assertAlmostEqual(rng10, 1000.0, delta=2.0)

    def test_lead_and_hold_without_velocity(self):
        dr = DeadReckoner(lead_s=2.0)
        dr.update(Fix(1.0, 2.0, 3.0, t=0.0))
        self.assertEqual(dr.predict(5.0), (1.0, 2.0, 3.0))
        self.assertIsNone(DeadReckoner().predict(0.0))


class PanWindow(unittest.TestCase):
    def test_free_spin_takes_short_way(self):
        self.assertAlmostEqual(pan_error(10, 350)[0], 20)
        self.assertAlmostEqual(pan_error(350, 10)[0], -20)

    def test_window_forbids_crossing_the_gap(self):
        # centre 0, +-175: head at 170, target at 190 (= -170) is 20 deg away the
        # short way, but that crosses the stop - it must go 340 deg the long way.
        err, clamped = pan_error(190, 170, center=0.0)
        self.assertAlmostEqual(err, -340.0)
        self.assertFalse(clamped)

    def test_target_in_dead_gap_is_clamped(self):
        err, clamped = pan_error(180, 100, center=0.0)               # gap is 175..185
        self.assertTrue(clamped)
        self.assertAlmostEqual(err, -175 - 100)     # clamped to the window edge


class SimHead:
    """Open-loop head simulator: fixed speeds, compass reading delayed by latency."""

    def __init__(self, yaw=0.0, pitch=0.0, pan_rate=14.0, tilt_rate=2.0, pan_sign=1):
        self.yaw, self.pitch = yaw, pitch
        self.pan_rate, self.tilt_rate = pan_rate, tilt_rate
        self.pan_dir = self.tilt_dir = 0
        self.pan_sign = pan_sign

    def advance(self, dt):
        self.yaw = wrap360(self.yaw + self.pan_sign * self.pan_dir * self.pan_rate * dt)
        self.pitch = max(-35.0, min(35.0, self.pitch + self.tilt_dir * self.tilt_rate * dt))


def run_sim(pointer, head, seconds, dt=0.1, t0=1000.0, on_tick=None):
    t = t0
    for i in range(int(seconds / dt)):
        pointer.set_compass(head.yaw, head.pitch, 0.0, t)
        pan, tilt, info = pointer.step(t)
        head.pan_dir, head.tilt_dir = pan, tilt
        head.advance(dt)
        t += dt
        if on_tick:
            on_tick(t, info)
    return t, info


class ClosedLoop(unittest.TestCase):
    def make(self, **kw):
        cfg = PointerConfig(**kw)
        return Pointer(cfg)

    def test_angle_mode_converges(self):
        p = self.make()
        h = SimHead(yaw=10, pitch=0)
        p.point_angle(200, 15)
        _, info = run_sim(p, h, 40)
        self.assertLess(abs(wrap180(h.yaw - 200)), 3.0)
        self.assertLess(abs(h.pitch - 15), 1.5)
        self.assertEqual(info["status"], "on_target")
        self.assertEqual((info["pan_dir"], info["tilt_dir"]), (0, 0))

    def test_pan_takes_short_way_when_free(self):
        p = self.make()
        h = SimHead(yaw=350)
        p.point_angle(10, 0)
        seen = []
        run_sim(p, h, 5, on_tick=lambda t, i: seen.append(h.yaw))
        # never swings through 180
        self.assertTrue(all(y > 300 or y < 60 for y in seen))

    def test_pan_never_crosses_end_stop_with_window(self):
        p = self.make(pan_center_heading=0.0)
        h = SimHead(yaw=170)
        p.point_angle(190, 0)                  # 20 deg away through the stop
        ys = []
        run_sim(p, h, 40, on_tick=lambda t, i: ys.append(wrap180(h.yaw)))
        # moved the long way: must pass through 0 and end near -170
        self.assertTrue(any(abs(y) < 20 for y in ys))
        self.assertLess(abs(wrap180(h.yaw - 190)), 3.0)

    def test_latlon_points_at_target(self):
        p = self.make()
        p.set_gps(-33.0, 150.0, 20.0, True, 1000.0)
        la, lo = move_latlon(-33.0, 150.0, 500.0, 120.0, 1.0)      # 500 m ESE
        p.point_latlon(la, lo, 20.0 + 100.0)
        h = SimHead(yaw=0, pitch=0)
        _, info = run_sim(p, h, 40)
        az, el, _ = look_angles((-33.0, 150.0, 20.0), (la, lo, 120.0))
        self.assertLess(abs(wrap180(h.yaw - az)), 3.0)
        self.assertLess(abs(h.pitch - min(el, 32.0)), 1.5)

    def test_track_follows_moving_target(self):
        p = self.make(lead_time_s=0.0)
        p.set_gps(-33.0, 150.0, 0.0, True, 1000.0)
        p.follow_track("test")
        h = SimHead(yaw=90, pitch=0)
        start = (-33.0, 150.0)
        speed, trk = 60.0, 0.0           # ~54 deg/h at 1 km: slow enough for a 14 dps pan

        def feed(t, info):
            if int(t * 10) % 5 == 0:     # 2 Hz endpoint
                la, lo = move_latlon(-33.0, 150.01, speed, trk, t - 1000.0)
                p.set_track_fix(Fix(la, lo, 300.0, t, speed, trk))

        # start with a first fix so the pointer has something to look at
        p.set_track_fix(Fix(-33.0, 150.01, 300.0, 1000.0, speed, trk))
        run_sim(p, h, 60, on_tick=feed)
        la, lo = move_latlon(-33.0, 150.01, speed, trk, 60.0)
        az, el, _ = look_angles((-33.0, 150.0, 0.0), (la, lo, 300.0))
        self.assertLess(abs(wrap180(h.yaw - az)), 4.0)

    def test_wrong_compass_sign_trips_fault(self):
        p = self.make()
        h = SimHead(yaw=0, pitch=0, pan_sign=-1)        # head turns the "wrong" way
        p.point_angle(90, 0)
        _, info = run_sim(p, h, 10)
        self.assertEqual(info["fault"], "pan_not_moving")
        self.assertEqual(info["pan_dir"], 0)
        # a new command clears the fault
        p.point_angle(10, 0)
        _, _, info = p.step(2000.0)
        self.assertIsNone(info["fault"])

    def test_stale_compass_stops_head(self):
        p = self.make()
        p.point_angle(90, 0)
        p.set_compass(0, 0, 0, 1000.0)
        pan, tilt, info = p.step(1000.0)
        self.assertEqual(pan, 1)
        pan, tilt, info = p.step(1005.0)                 # 5 s without a reading
        self.assertEqual((pan, tilt), (0, 0))
        self.assertEqual(info["status"], "compass_stale")

    def test_no_compass_no_motion(self):
        p = self.make()
        p.point_angle(90, 0)
        self.assertEqual(p.step(1.0)[:2], (0, 0))

    def test_latlon_without_observer(self):
        p = self.make()
        p.set_compass(0, 0, 0, 1.0)
        p.point_latlon(1, 1, 1)
        pan, tilt, info = p.step(1.0)
        self.assertEqual(info["status"], "no_observer")
        self.assertEqual((pan, tilt), (0, 0))

    def test_fallback_observer(self):
        p = self.make(fallback=(0.0, 0.0, 0.0))
        p.set_compass(0, 0, 0, 1.0)
        p.point_latlon(0.0, 0.01, 0.0)                   # due east
        pan, _, info = p.step(1.0)
        self.assertEqual(info["observer_source"], "fallback")
        self.assertEqual(pan, 1)

    def test_jog_expires(self):
        p = self.make()
        p.jog(1, -1, 10.0)
        self.assertEqual(p.step(10.2)[:2], (1, -1))
        self.assertEqual(p.step(11.0)[:2], (0, 0))

    def test_tilt_clamped_to_limits(self):
        p = self.make()
        p.set_compass(0, 0, 0, 1.0)
        p.point_angle(0, 80)
        _, _, info = p.step(1.0)
        self.assertEqual(info["aim_pitch"], 32.0)   # 35 - 3 margin

    def test_body_frame_needs_center(self):
        p = self.make()
        with self.assertRaises(ValueError):
            p.point_angle(10, 0, "body")
        p2 = self.make(pan_center_heading=90.0)
        p2.point_angle(10, 0, "body")
        self.assertAlmostEqual(p2.angle_cmd[0], 100.0)

    def test_pitch_sign_and_declination(self):
        p = self.make(pitch_sign=-1, declination_deg=12.0)
        p.set_compass(350.0, 5.0, 0.0, 1.0)
        yaw, pitch, _, _ = p.head_attitude(1.0)
        self.assertAlmostEqual(yaw, 2.0)
        self.assertAlmostEqual(pitch, -5.0)


class Limits(unittest.TestCase):
    def test_set_limits_and_center_here(self):
        p = Pointer(PointerConfig())
        p.set_compass(123.0, 0, 0, 1.0)
        self.assertEqual(p.set_center_here(1.0), 123.0)
        p.set_limits(tilt_min=-30, tilt_max=20, margin=2)
        self.assertEqual((p.cfg.tilt_lo, p.cfg.tilt_hi), (-28, 18))
        p.set_limits(center=None)
        self.assertIsNone(p.cfg.pan_center_heading)
        with self.assertRaises(ValueError):
            p.set_limits(tilt_min=10, tilt_max=11, margin=3)

    def test_pan_stops_short_of_end_stop(self):
        p = Pointer(PointerConfig(pan_center_heading=0.0))   # window +-172
        h = SimHead(yaw=100)
        p.point_angle(179, 0)                                  # inside the 10 deg gap
        run_sim(p, h, 20)
        self.assertLess(abs(wrap180(h.yaw)), 172.0 + 3.0)
        self.assertGreater(abs(wrap180(h.yaw)), 165.0)


class AxisUnit(unittest.TestCase):
    def test_hysteresis_no_chatter(self):
        a = AxisController(start_deg=1.5, rate_dps=14, latency_s=0.25)
        self.assertEqual(a.update(1.0, 0, 0.0), 0)       # inside start band
        self.assertEqual(a.update(5.0, 0, 1.0), 1)       # start
        self.assertEqual(a.update(6.0, 0, 1.1), 1)       # keep going
        self.assertEqual(a.update(3.4, 0, 1.2), 0)       # predictive stop (<= 3.5)


if __name__ == "__main__":
    unittest.main()
