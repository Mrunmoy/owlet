"""On-hardware checks: what only a real USB link and a real head can show.

Opt-in, because it MOVES THE MOTORS (within ±20° of centre, ≤ 30 °/s):

    OWLET_HW=1 python -m unittest test_hardware -v          # from tools/python/tests
    OWLET_HW=1 OWLET_PORT=/dev/ttyACM0 python -m unittest test_hardware

The head is homed and left limp after every motion test. Thresholds were set on
a head with a 10 ms control loop (2026-10-02); the Pico's is 2 ms.
"""

import math
import os
import random
import statistics
import time
import unittest

from owlet import Owlet
from owlet.tracking import DeviceClock, Tracker

ENABLED = os.environ.get("OWLET_HW") == "1"


@unittest.skipUnless(ENABLED, "set OWLET_HW=1 to run on a connected head (moves the motors)")
class HardwareTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.head = Owlet(os.environ.get("OWLET_PORT"))

    @classmethod
    def tearDownClass(cls):
        cls.head.close()

    def _limp_at_home(self):
        try:
            self.head.home(wait=True, timeout=5)
        finally:
            self.head.disable()

    def _enable_at_home(self):
        self.head.speed(30)
        self.head.enable()
        self.addCleanup(self._limp_at_home)
        self.head.home(wait=True, timeout=10)

    # ── link and clock (no motion) ────────────────────────────────────────

    def test_request_round_trip_is_bounded_by_the_device_loop(self):
        rtts = []
        for _ in range(30):
            time.sleep(random.uniform(0, 0.012))
            t0 = time.monotonic()
            self.head.clock()
            rtts.append(time.monotonic() - t0)
        # Uniform over one 10 ms loop: median ~6 ms. 50 ms means the client
        # is waiting on its serial read timeout.
        self.assertLess(statistics.median(rtts), 0.012)

    def test_clock_offset_is_precise_and_stable(self):
        def jitter():
            time.sleep(random.uniform(0, 0.012))

        offsets = []
        for _ in range(3):
            c = DeviceClock(window=32)
            best = c.sync(self.head.clock, 32, before_each=jitter)
            self.assertLess(best, 0.003, "no short round trip: samples phase-locked?")
            offsets.append(c.offset_s)
            time.sleep(1.0)
        # ~13 ppm drift adds well under 0.1 ms over these few seconds.
        self.assertLess(max(offsets) - min(offsets), 0.001)

    # ── motion ────────────────────────────────────────────────────────────

    def test_stream_is_on_time_and_fresh(self):
        self._enable_at_home()
        with Tracker(self.head, stream_hz=50, watchdog_ms=300) as tr:
            time.sleep(0.3)
            ages = []
            for i in range(60):
                tr.set_rates(0, 0)
                ages.append(time.monotonic() - tr.history.latest_time)
                time.sleep(0.011 + 0.003 * (i % 3))
            ts = [t for t, _, _ in list(tr.history._samples)[-60:]]
            self.assertFalse(tr.failed, tr.last_error)
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        self.assertAlmostEqual(statistics.median(gaps), 0.02, delta=0.004)
        self.assertLess(max(gaps), 0.06)
        self.assertLess(statistics.median(ages), 0.02)

    def test_velocity_runs_at_the_commanded_rate_and_the_watchdog_stops_it(self):
        self._enable_at_home()
        rate, watchdog_ms = 15.0, 300
        with Tracker(self.head, stream_hz=50, watchdog_ms=watchdog_ms) as tr:
            time.sleep(0.2)
            start = tr.latest.pan
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                tr.set_rates(rate, 0.0)
                time.sleep(1 / 30)
            t_silent = time.monotonic()
            pan_at_silence = tr.latest.pan
            time.sleep(watchdog_ms / 1000.0 + 0.7)
            stopped = tr.latest
            timeouts = tr.timeouts
            # When did it stop? The first moment the pose reaches its final value.
            t = t_silent
            while abs(tr.pose_at(t).pan - stopped.pan) > 0.05 and t < time.monotonic():
                t += 0.005
            stop_delay = t - t_silent
        self.assertAlmostEqual(pan_at_silence - start, rate, delta=0.25 * rate)
        self.assertEqual(timeouts, 1)
        self.assertEqual(stopped.state, "H")
        self.assertLessEqual(stop_delay, watchdog_ms / 1000.0 + 0.1)

    def test_tracker_follows_continuous_rates_without_errors(self):
        self._enable_at_home()
        with Tracker(self.head, stream_hz=50, watchdog_ms=300) as tr:
            time.sleep(0.2)
            t0 = time.monotonic()
            w = 2 * math.pi / 3.0  # 3 s period: ±9.5° pan, ±4.8° tilt
            while (t := time.monotonic() - t0) < 6.0:
                tr.set_rates(20 * math.cos(w * t), 10 * math.sin(w * t))
                time.sleep(1 / 30)
            tr.set_rates(0, 0)
            self.assertFalse(tr.failed, tr.last_error)
            self.assertEqual(tr.timeouts, 0)
            self.assertEqual(tr.clock_misses, 0)
            self.assertIsNone(tr.last_error)
            self.assertLess(abs(tr.latest.pan), 3.0)   # a full period returns near start
            self.assertLess(abs(tr.latest.tilt), 3.0)


if __name__ == "__main__":
    unittest.main()
