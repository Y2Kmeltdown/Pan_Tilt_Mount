"""Calibration tests with a simulated head that has real end stops.
Run:  python -m unittest -v test_calibration"""
import unittest

from control import Pointer, PointerConfig
from pointing import wrap180, wrap360


class StopHead:
    """Head with limit switches: pan +-pan_half about `base_heading`, tilt +-tilt_half."""

    def __init__(self, base_heading=200.0, pan_half=175.0, tilt_half=35.0, start_pan=40.0,
                 start_tilt=-10.0, pan_rate=14.0, tilt_rate=2.0, pitch_sign=1, pan_sign=1,
                 dead=False):
        self.base, self.pan_half, self.tilt_half = base_heading, pan_half, tilt_half
        self.rel, self.tilt = start_pan, start_tilt
        self.pan_rate, self.tilt_rate = pan_rate, tilt_rate
        self.pitch_sign, self.pan_sign, self.dead = pitch_sign, pan_sign, dead
        self.pan_dir = self.tilt_dir = 0

    def advance(self, dt):
        if self.dead:
            return
        self.rel = max(-self.pan_half, min(self.pan_half, self.rel + self.pan_dir * self.pan_rate * dt))
        self.tilt = max(-self.tilt_half, min(self.tilt_half, self.tilt + self.tilt_dir * self.tilt_rate * dt))

    @property
    def yaw(self):
        return wrap360(self.base + self.pan_sign * self.rel)

    @property
    def pitch(self):
        return self.pitch_sign * self.tilt


def run(ptr, head, seconds, dt=0.1, t0=5000.0):
    t, info = t0, {}
    for _ in range(int(seconds / dt)):
        ptr.set_compass(head.yaw, head.pitch, 0.0, t)
        pan, tilt, info = ptr.step(t)
        head.pan_dir, head.tilt_dir = pan, tilt
        head.advance(dt)
        t += dt
        if ptr.mode not in ("calibrate",) and info.get("status", "").startswith("calibration"):
            pass
    return t, info


class Calibration(unittest.TestCase):
    def test_both_axes_measure_and_apply(self):
        p = Pointer(PointerConfig())                       # defaults: centre unset, +-35 tilt
        h = StopHead(base_heading=200.0)
        p.start_calibration("both", apply=True, center=True, now=5000.0)
        run(p, h, 400)
        st = p.calibration_status()
        self.assertEqual(st["state"], "done", st)
        pan, tilt = st["result"]["pan"], st["result"]["tilt"]
        self.assertAlmostEqual(pan["range"], 350.0, delta=3.0)
        self.assertLess(abs(wrap180(pan["center_heading"] - 200.0)), 3.0)
        self.assertAlmostEqual(tilt["min"], -35.0, delta=1.0)
        self.assertAlmostEqual(tilt["max"], 35.0, delta=1.0)
        self.assertTrue(st["applied"])
        self.assertAlmostEqual(p.cfg.pan_range_deg, pan["range"], places=3)
        self.assertLess(abs(wrap180(p.cfg.pan_center_heading - 200.0)), 3.0)
        # after calibration the head is driven back to centre using the new limits
        self.assertLess(abs(h.rel), 6.0)
        self.assertLess(abs(h.tilt), 3.0)

    def test_starting_on_the_low_stop_is_fine(self):
        p = Pointer(PointerConfig())
        h = StopHead(start_pan=-175.0, start_tilt=-35.0)
        p.start_calibration("both", True, False, 5000.0)
        run(p, h, 400)
        st = p.calibration_status()
        self.assertEqual(st["state"], "done", st)
        self.assertAlmostEqual(st["result"]["pan"]["range"], 350.0, delta=3.0)
        self.assertEqual(p.mode, "idle")                    # center=False

    def test_pan_only_leaves_tilt_limits(self):
        p = Pointer(PointerConfig())
        h = StopHead()
        p.start_calibration("pan", True, False, 5000.0)
        run(p, h, 150)
        st = p.calibration_status()
        self.assertEqual(st["state"], "done")
        self.assertNotIn("tilt", st["result"])
        self.assertEqual(p.cfg.tilt_min_deg, -35.0)

    def test_measures_without_applying(self):
        p = Pointer(PointerConfig(pan_range_deg=300.0))
        h = StopHead()
        p.start_calibration("pan", apply=False, center=False, now=5000.0)
        run(p, h, 150)
        st = p.calibration_status()
        self.assertEqual(st["state"], "done")
        self.assertFalse(st["applied"])
        self.assertEqual(p.cfg.pan_range_deg, 300.0)

    def test_narrower_head_is_measured(self):
        p = Pointer(PointerConfig())
        h = StopHead(pan_half=120.0, tilt_half=20.0)
        p.start_calibration("both", True, False, 5000.0)
        run(p, h, 400)
        st = p.calibration_status()
        self.assertAlmostEqual(st["result"]["pan"]["range"], 240.0, delta=3.0)
        self.assertAlmostEqual(st["result"]["tilt"]["range"], 40.0, delta=2.0)

    def test_inverted_pitch_sign_is_reported(self):
        p = Pointer(PointerConfig())
        h = StopHead(pitch_sign=-1)
        p.start_calibration("tilt", True, False, 5000.0)
        run(p, h, 200)
        st = p.calibration_status()
        self.assertEqual(st["state"], "error")
        self.assertIn("pitch_sign", st["hint"])
        self.assertEqual(p.mode, "idle")

    def test_dead_head_reports_error(self):
        p = Pointer(PointerConfig())
        h = StopHead(dead=True)
        p.start_calibration("pan", True, False, 5000.0)
        run(p, h, 60)
        st = p.calibration_status()
        self.assertEqual(st["state"], "error")
        self.assertIn("did not move", st["error"])

    def test_free_spinning_pan_aborts(self):
        p = Pointer(PointerConfig())
        h = StopHead(pan_half=100000.0)                     # no limit switch
        p.start_calibration("pan", True, False, 5000.0)
        run(p, h, 200)
        st = p.calibration_status()
        self.assertEqual(st["state"], "error")
        self.assertIn("no end stop", st["error"])

    def test_stale_compass_aborts_and_stops(self):
        p = Pointer(PointerConfig())
        p.set_compass(0, 0, 0, 5000.0)
        p.start_calibration("pan", True, False, 5000.0)
        pan, tilt, info = p.step(5000.0)
        self.assertEqual(pan, -1)                           # driving to the low stop
        pan, tilt, info = p.step(5003.0)                    # compass went silent
        self.assertEqual((pan, tilt), (0, 0))
        self.assertEqual(p.calibration_status()["state"], "error")

    def test_cancel_with_stop(self):
        p = Pointer(PointerConfig())
        p.set_compass(0, 0, 0, 5000.0)
        p.start_calibration("both", True, True, 5000.0)
        p.step(5000.0)
        p.stop()
        self.assertEqual(p.step(5000.1)[:2], (0, 0))
        self.assertEqual(p.mode, "idle")


if __name__ == "__main__":
    unittest.main()
