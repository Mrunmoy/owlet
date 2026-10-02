"""End-to-end: Python client ↔ pty ↔ owlet_sim (the real C++ core).

Needs the host build: ./build.py --host. Skips when owlet_sim is missing.
Override the binary path with OWLET_SIM=/path/to/owlet_sim.
"""

import os
import statistics
import subprocess
import time
import sys
import tempfile
import unittest
from pathlib import Path

from owlet import Owlet, OwletError
from owlet.tracking import Tracker

REPO = Path(__file__).resolve().parents[3]
SIM = Path(os.environ.get("OWLET_SIM", REPO / "build/host/apps/host-sim/owlet_sim"))


@unittest.skipUnless(SIM.exists(), f"owlet_sim not built ({SIM})")
class EndToEndTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.link = os.path.join(self.tmp.name, "owlet")
        self.cal_file = os.path.join(self.tmp.name, "cal.bin")
        self.start_sim()

    def start_sim(self):
        self.sim = subprocess.Popen([str(SIM), "--link", self.link, "--cal-file", self.cal_file],
                                    stdout=subprocess.PIPE, text=True)
        # Registered before anything that can fail, so a failed connect can't
        # leave an orphaned simulator behind.
        self.addCleanup(self._kill_sim, self.sim)
        self.assertEqual(self.sim.stdout.readline().strip(), self.link)
        self.head = Owlet(self.link, timeout=1.0)
        self.addCleanup(self.head.close)

    @staticmethod
    def _kill_sim(sim):
        if sim.poll() is None:
            sim.terminate()
            sim.wait(timeout=5)
        if not sim.stdout.closed:
            sim.stdout.close()

    def stop_sim(self):
        self.head.close()
        self._kill_sim(self.sim)

    def test_handshake(self):
        self.assertEqual(self.head.protocol_version, 3)
        self.assertEqual(self.head.firmware_version, "0.3.0")

    def test_boots_disabled_and_refuses_motion(self):
        self.assertFalse(self.head.position().enabled)
        with self.assertRaises(OwletError) as cm:
            self.head.move(10, 10)
        self.assertEqual(cm.exception.code, "DIS")

    def test_enable_move_wait(self):
        self.head.enable()
        self.head.speed(400)
        final = self.head.move(30, -15, wait=True, timeout=3)
        self.assertEqual((final.pan, final.tilt, final.state), (30.0, -15.0, "H"))
        self.assertIsInstance(final.t_ms, int)

    def test_range_is_enforced(self):
        self.head.enable()
        with self.assertRaises(OwletError) as cm:
            self.head.move(95, 0)
        self.assertEqual(cm.exception.code, "RNG")
        self.assertEqual(self.head.limits(), (-90.0, 90.0, -90.0, 90.0))

    def test_watch_streams_positions(self):
        self.head.enable()
        self.head.speed(100)
        self.head.move(20, 0)
        seen = []
        for p in self.head.watch(hz=20):
            seen.append(p)
            if len(seen) == 5:
                break
        self.assertTrue(any(p.moving for p in seen))
        # Streaming is switched off on exit; a normal request must still work.
        self.assertEqual(self.head.speed(), 100.0)

    def test_velocity_and_watchdog_timeout(self):
        events = []
        self.head.on_event = events.append
        self.head.enable()
        self.assertEqual(self.head.watchdog(), 500)
        self.head.watchdog(200)
        self.head.velocity(30, 0)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not any(e.name == "TIMEOUT" for e in events):
            s = self.head.read_sentence(timeout=0.1)
            if s is not None:
                events.append(s)
        self.assertTrue(any(e.name == "TIMEOUT" and e.fields == ("VEL",) for e in events))
        p = self.head.position()
        self.assertEqual(p.state, "H")
        self.assertGreater(p.pan, 3.0)   # moved for ~200 ms at 30°/s
        self.assertLess(p.pan, 12.0)

    def test_velocity_refused_while_disabled(self):
        with self.assertRaises(OwletError) as cm:
            self.head.velocity(10, 0)
        self.assertEqual(cm.exception.code, "DIS")

    def test_clock_tracks_host_monotonic_time(self):
        a = self.head.clock()
        time.sleep(0.2)
        b = self.head.clock()
        self.assertGreaterEqual((b - a) % 2**32, 150)
        self.assertLessEqual((b - a) % 2**32, 400)

    def test_tracker_follows_rates_and_stops_cleanly(self):
        self.head.enable()
        with Tracker(self.head, stream_hz=50, watchdog_ms=300) as tr:
            start = time.monotonic()
            while time.monotonic() - start < 0.6:
                tr.set_rates(40, -20)
                time.sleep(0.05)
            latest = tr.latest
            self.assertIsNotNone(latest)
            self.assertGreater(latest.pan, 10.0)
            self.assertLess(latest.tilt, -5.0)
            self.assertGreater(len(tr.history), 15)
            # Device timestamps map onto the host clock.
            self.assertLess(abs(tr.history.latest_time - time.monotonic()), 0.25)
            mid = tr.pose_at(time.monotonic() - 0.3)
            self.assertLess(mid.pan, latest.pan)
            self.assertIsNone(tr.last_error)
        # Leaving the context stops velocity mode and streaming.
        p = self.head.position()
        self.assertEqual(p.state, "H")
        time.sleep(0.3)
        self.assertEqual(self.head.position().pan, p.pan)

    def test_tracker_records_refusals_without_raising(self):
        with Tracker(self.head, stream_hz=20) as tr:  # head is still disabled
            tr.set_rates(10, 0)
            deadline = time.monotonic() + 1.0
            while tr.last_error is None and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertEqual(tr.last_error.code, "DIS")

    def test_tracker_reports_watchdog_trip_when_rates_stop_coming(self):
        self.head.enable()
        with Tracker(self.head, watchdog_ms=150) as tr:
            tr.set_rates(30, 0)
            time.sleep(0.5)
            self.assertEqual(tr.timeouts, 1)
            self.assertEqual(tr.latest.state, "H")

    # ── Timing and accuracy (P13: found on hardware, invisible to loose bounds) ──
    # The sim serves input every 10 ms, like the firmware, and stamps with
    # CLOCK_MONOTONIC, the clock behind time.monotonic(). So latency has a
    # known floor and the clock offset has a known true value.

    @staticmethod
    def _true_clock_offset_s() -> float:
        # Device ms = host monotonic ms truncated to 32 bits, so the offset is
        # the wrapped-away part plus, on average, half a millisecond.
        now_ms = time.monotonic() * 1000.0
        return (now_ms - now_ms % 2**32) / 1000.0 + 0.0005

    def test_request_round_trip_is_bounded_by_the_device_loop(self):
        rtts = []
        for _ in range(20):
            t0 = time.monotonic()
            self.head.clock()
            rtts.append(time.monotonic() - t0)
        # One 10 ms loop at most, plus scheduling. A client that waits out its
        # 50 ms serial read timeout fails this.
        self.assertLess(statistics.median(rtts), 0.015)

    def test_streamed_positions_arrive_fresh_and_on_time(self):
        self.head.enable()
        with Tracker(self.head, stream_hz=50, watchdog_ms=300) as tr:
            time.sleep(0.3)
            ages = []
            true_offset = self._true_clock_offset_s()
            for i in range(40):
                tr.set_rates(0, 0)
                # Age against the sim's true clock, not the estimated one: an
                # offset error would otherwise hide (or fake) late delivery.
                ages.append(time.monotonic() - (tr.latest.t_ms / 1000.0 + true_offset))
                time.sleep(0.011 + 0.003 * (i % 3))  # don't phase-lock to the stream
            ts = [t for t, _, _ in list(tr.history._samples)[-40:]]
        # 50 Hz stream: the newest sample is at most one interval (20 ms) old.
        self.assertLess(statistics.median(ages), 0.02)
        interval = statistics.median(b - a for a, b in zip(ts, ts[1:]))
        self.assertAlmostEqual(interval, 0.02, delta=0.004)

    def test_tracker_clock_offset_is_accurate_from_the_start(self):
        with Tracker(self.head, stream_hz=50) as tr:
            err = tr.clock.offset_s - self._true_clock_offset_s()
        # Back-to-back samples phase-lock to the 10 ms loop and bias this by ~4.5 ms.
        self.assertLess(abs(err), 0.0015, f"offset error {err * 1000:+.2f} ms")

    def test_tracker_clock_resync_stays_accurate(self):
        # One initial sample, then enough resyncs to fill the window: the
        # result comes from resync samples alone.
        with Tracker(self.head, stream_hz=50, clock_samples=1, resync_s=0.05) as tr:
            time.sleep(3.0)  # > 32 resyncs (the default window) at ~70 ms each
            err = tr.clock.offset_s - self._true_clock_offset_s()
            misses = tr.clock_misses
        self.assertEqual(misses, 0)
        self.assertLess(abs(err), 0.0015, f"offset error {err * 1000:+.2f} ms")

    def test_capabilities_are_reported(self):
        c = self.head.caps()
        self.assertEqual((c.speed_max, c.velocity_max, c.stream_max_hz), (500.0, 500.0, 50))

    def test_calibration_persists_across_a_restart(self):
        self.head.set_reversed("tilt", True)
        self.head.set_limits("pan", -85, 90)
        self.head.save_calibration()
        self.stop_sim()
        self.start_sim()  # "power cycle": a fresh process loading the same store
        tilt, pan = self.head.calibration("tilt"), self.head.calibration("pan")
        self.assertEqual((tilt.reversed, tilt.source), (True, "saved"))
        self.assertEqual(pan.min_deg, -85.0)
        self.assertEqual(self.head.limits()[0], -85.0)
        # and it really drives the head: +45° tilt on a reversed axis is still "up"
        self.head.enable()
        self.assertEqual(self.head.move(0, 45, wait=True, timeout=3).tilt, 45.0)

    def test_reset_restores_factory_and_forgets_the_saved_copy(self):
        self.head.set_reversed("tilt", True)
        self.head.save_calibration()
        self.head.reset_calibration()
        self.stop_sim()
        self.start_sim()
        t = self.head.calibration("tilt")
        self.assertEqual((t.reversed, t.source), (False, "factory"))

    def test_cli_mirrors_limits_only_when_the_direction_really_changes(self):  # review N7
        def cli(*args):
            return subprocess.run([sys.executable, "-m", "owlet", "-p", self.link, *args],
                                  capture_output=True, text=True, timeout=10,
                                  cwd=REPO / "tools/python")
        self.assertEqual(cli("cal", "tilt", "--reverse", "--limits", "-80", "30").returncode, 0)
        # Already reversed: new pulses, still reversed -> limits untouched.
        self.assertEqual(cli("cal", "tilt", "--pulses", "600", "1500", "2400", "--reverse").returncode, 0)
        c = self.head.calibration("tilt")
        self.assertEqual((c.min_us, c.max_us, c.reversed), (2400, 600, True))
        self.assertEqual((c.min_deg, c.max_deg), (-80.0, 30.0))
        # Normal axis given reversed pulses with --no-reverse: un-reversed, limits untouched.
        self.assertEqual(cli("cal", "pan", "--limits", "-30", "80").returncode, 0)
        self.assertEqual(cli("cal", "pan", "--pulses", "2400", "1500", "600", "--no-reverse").returncode, 0)
        p = self.head.calibration("pan")
        self.assertEqual((p.min_us, p.max_us, p.reversed), (600, 2400, False))
        self.assertEqual((p.min_deg, p.max_deg), (-30.0, 80.0))

    def test_cli(self):
        def cli(*args):
            return subprocess.run([sys.executable, "-m", "owlet", "-p", self.link, *args],
                                  capture_output=True, text=True, timeout=10,
                                  cwd=REPO / "tools/python")
        self.assertIn("protocol v3", cli("ping").stdout)
        self.assertIn("velocity max 500.0", cli("caps").stdout)
        self.assertEqual(cli("cal", "tilt", "--reverse", "--limits", "-80", "85").returncode, 0)
        out = cli("cal").stdout
        self.assertIn("tilt", out)
        self.assertIn("reversed", out)
        self.assertIn("-80.0..85.0", out)  # explicit --limits are the final limits, after reversing
        self.assertEqual(cli("cal", "save", "--limits", "1", "2").returncode, 2)  # flags + save: refused
        self.assertEqual(cli("cal", "save").returncode, 0)
        self.assertIn("saved", cli("cal", "tilt").stdout)
        self.assertEqual(cli("enable").returncode, 0)
        out = cli("move", "10", "-20", "--wait").stdout
        self.assertIn("pan   10.0  tilt  -20.0", out)
        bad = cli("move", "100", "0")
        self.assertEqual(bad.returncode, 1)
        self.assertIn("RNG", bad.stderr)
        self.assertEqual(cli("watchdog", "250").returncode, 0)
        self.assertIn("250 ms", cli("watchdog").stdout)
        self.assertEqual(cli("vel", "5", "0").returncode, 0)
        self.assertEqual(cli("stop").returncode, 0)


if __name__ == "__main__":
    unittest.main()
