"""End-to-end smoke test: fake compass/GPS/track endpoints -> service -> recorded Pelco-D frames.
Run:  python -m unittest -v test_service"""
import asyncio
import time
import unittest

from aiohttp import ClientSession, web
from aiohttp.test_utils import TestServer

import pantilt_service as ps
from pantilt import PanTilt


class Service(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.compass = {"bearing": 0.0, "pitch": 0.0, "roll": 0.0}
        self.gps = {"latitude": -33.0, "longitude": 150.0, "altitude_m": 10.0, "has_lock": True}
        self.tgt = {"position": {"lat": -33.0, "lon": 150.01}, "alt": 10.0, "gs": 0}
        app = web.Application()
        app.router.add_get("/compass", lambda r: web.json_response(self.compass))
        app.router.add_get("/gps", lambda r: web.json_response(self.gps))
        app.router.add_get("/tgt", lambda r: web.json_response([self.tgt]))
        self.fake = TestServer(app)
        await self.fake.start_server()
        u = f"http://127.0.0.1:{self.fake.port}"
        a = ps.parse_args(["--http-port", "0", "--dry-run", "--compass-url", u + "/compass",
                           "--gps-url", u + "/gps", "--compass-interval", "0.05",
                           "--gps-interval", "0.05", "--period", "0.05",
                           "--pan-center-heading", "0"])
        self.ptr = ps.make_pointer(a)
        self.head = PanTilt.dry_run(address=a.address)
        self.svc = ps.PanTiltService(a, self.ptr, self.head)
        await self.svc.start()
        self.api = TestServer(ps.build_app(self.svc))
        await self.api.start_server()
        self.http = ClientSession(f"http://127.0.0.1:{self.api.port}")
        self.u = u

    async def asyncTearDown(self):
        await self.http.close()
        await self.api.close()
        await self.svc.stop()
        await self.fake.close()

    def writes(self):
        return self.head._ser.writes

    async def test_angle_then_stop(self):
        r = await self.http.put("/api/mode/angle", json={"yaw": 90, "pitch": 0})
        self.assertEqual(r.status, 200)
        await asyncio.sleep(0.4)
        # compass says 0, target 90 -> pan right (cmd2 0x02)
        self.assertTrue(any(w[3] == 0x02 for w in self.writes()))
        t = await (await self.http.get("/api/telemetry")).json()
        self.assertEqual(t["mode"], "angle")
        self.assertTrue(t["sources"]["compass"]["ok"])
        await self.http.post("/api/stop")
        await asyncio.sleep(0.2)
        self.assertEqual(self.writes()[-1][3], 0x00)

    async def test_latlon_east_uses_gps(self):
        r = await self.http.put("/api/mode/latlon", json={"lat": -33.0, "lon": 150.01, "alt": 10})
        self.assertEqual(r.status, 200)
        await asyncio.sleep(0.5)
        t = await (await self.http.get("/api/telemetry")).json()
        self.assertEqual(t["observer_source"], "gps")
        self.assertAlmostEqual(t["aim_yaw"], 90.0, delta=0.5)

    async def test_endpoint_track(self):
        r = await self.http.put("/api/mode/track", json={
            "url": self.u + "/tgt", "interval": 0.1, "lat_key": "position.lat",
            "lon_key": "position.lon", "alt_key": "alt"})
        self.assertEqual(r.status, 200)
        await asyncio.sleep(0.6)
        t = await (await self.http.get("/api/telemetry")).json()
        self.assertEqual(t["mode"], "track")
        self.assertTrue(t["track"]["ok"], t["track"])
        self.assertAlmostEqual(t["aim_yaw"], 90.0, delta=0.5)

    async def test_manual_position_overrides_gps(self):
        r = await self.http.put("/api/observer", json={"lat": -33.0, "lon": 150.02, "alt": 10})
        self.assertTrue((await r.json())["enabled"])
        await self.http.put("/api/mode/latlon", json={"lat": -33.0, "lon": 150.01, "alt": 10})
        await asyncio.sleep(0.4)
        t = await (await self.http.get("/api/telemetry")).json()
        self.assertEqual(t["observer_source"], "override")      # GPS says 150.0, manual wins
        self.assertAlmostEqual(t["aim_yaw"], 270.0, delta=0.5)   # target is now WEST of us
        r = await self.http.put("/api/observer", json={"enabled": False})
        j = await r.json()
        self.assertFalse(j["enabled"])
        self.assertEqual(j["lon"], 150.02)                       # values remembered
        await asyncio.sleep(0.4)
        t = await (await self.http.get("/api/telemetry")).json()
        self.assertEqual(t["observer_source"], "gps")

    async def test_bad_requests_and_limits(self):
        self.assertEqual((await self.http.put("/api/mode/latlon", json={"lat": "x"})).status, 400)
        self.assertEqual((await self.http.put("/api/mode/track", json={"url": "ftp://x"})).status, 400)
        self.assertEqual((await self.http.put("/api/mode/angle",
                          json={"yaw": 1, "pitch": 1, "frame": "nope"})).status, 400)
        r = await self.http.put("/api/limits", json={"tilt_min_deg": -20, "tilt_max_deg": 20})
        self.assertEqual((await r.json())["tilt_usable_max_deg"], 17)
        await asyncio.sleep(0.2)
        r = await self.http.post("/api/limits/center_here")
        self.assertEqual(r.status, 200)


if __name__ == "__main__":
    unittest.main()
