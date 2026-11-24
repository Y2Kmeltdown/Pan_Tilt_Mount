"""API-level calibration test: service + simulated stop-aware head over fake compass."""
import asyncio
import unittest

from aiohttp import ClientSession, web
from aiohttp.test_utils import TestServer

import pantilt_service as ps
from test_calibration import StopHead


class FakeSerial:
    """Records Pelco-D frames and moves a StopHead accordingly."""
    def __init__(self, head):
        self.head, self.writes = head, []

    def write(self, data):
        self.writes.append(bytes(data))
        cmd2 = data[3]
        self.head.pan_dir = 1 if cmd2 & 0x02 else -1 if cmd2 & 0x04 else 0
        self.head.tilt_dir = 1 if cmd2 & 0x08 else -1 if cmd2 & 0x10 else 0
        return len(data)

    def flush(self): pass
    def close(self): pass


class ServiceCal(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.head = StopHead(base_heading=90.0, pan_rate=140.0, tilt_rate=20.0)   # 10x faster
        app = web.Application()
        app.router.add_get("/c", lambda r: web.json_response(
            {"bearing": self.head.yaw, "pitch": self.head.pitch, "roll": 0}))
        self.fake = TestServer(app)
        await self.fake.start_server()
        a = ps.parse_args(["--http-port", "0", "--dry-run", "--compass-url",
                           f"http://127.0.0.1:{self.fake.port}/c", "--gps-url", "",
                           "--compass-interval", "0.02", "--period", "0.02",
                           "--pan-rate", "140", "--tilt-rate", "20"])
        self.ptr = ps.make_pointer(a)
        self.ptr.cfg.cal_stall_s, self.ptr.cfg.cal_settle_s = 0.3, 0.1
        from pantilt import PanTilt
        self.ser = FakeSerial(self.head)
        self.svc = ps.PanTiltService(a, self.ptr, PanTilt(None, 57, serial_port=self.ser))
        self.svc.args.refresh_s = 0.05
        await self.svc.start()
        self.api = TestServer(ps.build_app(self.svc))
        await self.api.start_server()
        self.http = ClientSession(f"http://127.0.0.1:{self.api.port}")

        async def sim():
            import time
            last = time.monotonic()
            while True:
                now = time.monotonic()
                self.head.advance(now - last)          # real elapsed time, not nominal
                last = now
                await asyncio.sleep(0.005)
        self.sim = asyncio.create_task(sim())

    async def asyncTearDown(self):
        self.sim.cancel()
        await self.http.close(); await self.api.close()
        await self.svc.stop(); await self.fake.close()

    async def test_calibrate_via_api(self):
        await asyncio.sleep(0.3)                                   # let the compass arrive
        r = await self.http.put("/api/calibrate", json={"axes": "both", "apply": True, "center": False})
        self.assertEqual(r.status, 200, await r.text())
        for _ in range(400):
            await asyncio.sleep(0.05)
            j = await (await self.http.get("/api/calibrate")).json()
            if j["state"] in ("done", "error"):
                break
        self.assertEqual(j["state"], "done", j)
        self.assertAlmostEqual(j["result"]["pan"]["range"], 350.0, delta=4.0)
        lim = await (await self.http.get("/api/limits")).json()
        self.assertTrue(lim["limits_set"])
        self.assertLess(abs(((lim["pan_center_heading"] - 90.0 + 180) % 360) - 180), 4.0)

    async def test_rejects_without_compass_or_bad_axes(self):
        self.assertEqual((await self.http.put("/api/calibrate", json={"axes": "up"})).status, 400)
        r = await self.http.put("/api/calibrate", json={"axes": "cancel"})
        self.assertEqual(r.status, 200)


if __name__ == "__main__":
    unittest.main()
