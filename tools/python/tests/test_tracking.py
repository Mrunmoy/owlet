import math
import time
import unittest

from serial import SerialException

from fakes import FakeSerial, default_responder
from owlet import Owlet, OwletError

from owlet.tracking import DeviceClock, PixelController, Pose, PoseHistory, Tracker


class DeviceClockTest(unittest.TestCase):
    def test_unwrap_is_monotonic_across_the_32_bit_wrap(self):
        c = DeviceClock()
        self.assertEqual(c.unwrap(4_294_967_000), 4_294_967_000)
        self.assertEqual(c.unwrap(100), 4_294_967_396)  # 2**32 + 100
        self.assertEqual(c.unwrap(200), 4_294_967_496)

    def test_small_reordering_is_not_mistaken_for_a_wrap(self):
        c = DeviceClock()
        c.unwrap(10_000)
        self.assertEqual(c.unwrap(9_990), 9_990)

    def test_sync_uses_the_lowest_round_trip_sample(self):
        # (host t0, device ms, host t1): the 2nd sample has the tightest RTT.
        samples = iter([(10.000, 5_000, 10.020), (11.000, 6_002, 11.002), (12.000, 7_000, 12.030)])
        now = [0.0]
        pending = []

        def fake_monotonic():
            return now[0]

        def query():
            t0, ms, t1 = pending.pop()
            now[0] = t1
            return ms

        def next_sample():
            t0, ms, t1 = next(samples)
            now[0] = t0
            pending.append((t0, ms, t1))

        c = DeviceClock(monotonic=fake_monotonic)
        rtt = c.sync(query, samples=3, before_each=next_sample)
        self.assertAlmostEqual(rtt, 0.002)
        # Midpoint 11.001 s ↔ device 6.002 s → offset 4.999 s
        self.assertAlmostEqual(c.offset_s, 11.001 - 6.002)
        self.assertAlmostEqual(c.to_host(6_002), 11.001)

    def test_late_pre_wrap_sample_is_not_pushed_49_days_ahead(self):  # review L3
        c = DeviceClock()
        c.unwrap(4_294_967_000)
        c.unwrap(100)                                    # wrapped
        self.assertEqual(c.unwrap(4_294_966_900), 4_294_966_900)  # late, pre-wrap
        self.assertEqual(c.unwrap(300), 4_294_967_596)

    def test_offset_follows_the_best_recent_sample(self):  # review M4 (drift)
        c = DeviceClock(window=3)
        c.add_sample(10.0, 10_000, 10.002)   # rtt 2 ms, offset ~0.001
        self.assertAlmostEqual(c.offset_s, 0.001)
        for k in range(3):                   # later, slightly worse samples that
            t = 20.0 + k                     # reflect 0.05 s of drift
            c.add_sample(t, int((t - 0.05) * 1000), t + 0.004)
        # the old 2 ms sample has left the window, so drift is picked up
        self.assertAlmostEqual(c.offset_s, 0.052, places=3)

    def test_to_host_requires_sync(self):
        with self.assertRaises(RuntimeError):
            DeviceClock().to_host(0)


class PoseHistoryTest(unittest.TestCase):
    def test_empty(self):
        self.assertIsNone(PoseHistory().at(1.0))
        self.assertIsNone(PoseHistory().latest)

    def test_interpolates_between_samples(self):
        h = PoseHistory()
        h.add(1.0, 0.0, 10.0)
        h.add(2.0, 10.0, 20.0)
        p = h.at(1.25)
        self.assertAlmostEqual(p.pan, 2.5)
        self.assertAlmostEqual(p.tilt, 12.5)

    def test_clamps_outside_the_recorded_window(self):
        h = PoseHistory()
        h.add(1.0, 0.0, 0.0)
        h.add(2.0, 10.0, 0.0)
        self.assertEqual(h.at(0.5), Pose(0.0, 0.0))
        self.assertEqual(h.at(9.0), Pose(10.0, 0.0))

    def test_is_bounded(self):
        h = PoseHistory(maxlen=4)
        for i in range(10):
            h.add(float(i), float(i), 0.0)
        self.assertEqual(len(h), 4)
        self.assertEqual(h.at(0.0), Pose(6.0, 0.0))

    def test_ignores_out_of_order_samples(self):
        h = PoseHistory()
        h.add(2.0, 20.0, 0.0)
        h.add(1.0, 10.0, 0.0)
        self.assertEqual(len(h), 1)


class PixelControllerTest(unittest.TestCase):
    def setUp(self):
        self.c = PixelController(fov_h_deg=60, fov_v_deg=40, gain=2.0,
                                 deadband_deg=0.0, max_rate=1000)

    def test_centered_object_needs_no_motion(self):
        self.assertEqual(self.c.update(320, 240, 640, 480), (0.0, 0.0))

    def test_frame_edge_is_half_the_field_of_view(self):
        dpan, dtilt = self.c.angle_offset(640, 240, 640, 480)
        self.assertAlmostEqual(dpan, 30.0, places=4)
        self.assertAlmostEqual(dtilt, 0.0)

    def test_signs_follow_owlet_convention(self):
        pan, tilt = self.c.update(480, 360, 640, 480)  # right of and below center
        self.assertGreater(pan, 0)   # pan + = right
        self.assertLess(tilt, 0)     # tilt + = up, image y grows downward

    def test_uses_pinhole_geometry_not_linear(self):
        dpan, _ = self.c.angle_offset(480, 240, 640, 480)  # halfway to the edge
        expected = math.degrees(math.atan(0.5 * math.tan(math.radians(30))))
        self.assertAlmostEqual(dpan, expected, places=4)
        self.assertNotAlmostEqual(dpan, 15.0, places=2)

    def test_rate_is_gain_times_error(self):
        pan, _ = self.c.update(640, 240, 640, 480)
        self.assertAlmostEqual(pan, 60.0, places=3)

    def test_deadband(self):
        c = PixelController(fov_h_deg=60, fov_v_deg=40, gain=2.0, deadband_deg=1.0)
        self.assertEqual(c.update(322, 241, 640, 480), (0.0, 0.0))

    def test_rate_is_clamped(self):
        c = PixelController(fov_h_deg=60, fov_v_deg=40, gain=100.0, max_rate=90.0)
        pan, tilt = c.update(0, 0, 640, 480)
        self.assertEqual(pan, -90.0)
        self.assertEqual(tilt, 90.0)

    def test_compensates_motion_since_the_frame_was_captured(self):
        # Object appeared 10° right; the head has since turned 6° right,
        # so only 4° of error remain.
        pan, _ = self.c.update(*self._pixel_for_pan(10.0), 640, 480,
                               pose_at_capture=Pose(0.0, 0.0), pose_now=Pose(6.0, 0.0))
        self.assertAlmostEqual(pan, 2.0 * 4.0, places=3)

    def test_rejects_bad_geometry(self):
        with self.assertRaises(ValueError):
            PixelController(fov_h_deg=0, fov_v_deg=40)
        with self.assertRaises(ValueError):
            PixelController(fov_h_deg=60, fov_v_deg=180)
        with self.assertRaises(ValueError):
            self.c.update(1, 1, 0, 480)

    def _pixel_for_pan(self, deg):
        f = 320 / math.tan(math.radians(30))
        return 320 + f * math.tan(math.radians(deg)), 240


class TrackerRobustnessTest(unittest.TestCase):  # review M3, M4, L6, H2
    def make(self, **kw):
        fake = FakeSerial()
        head = Owlet(transport=fake)
        return fake, head, Tracker(head, **kw)

    def wait_for(self, cond, timeout=1.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.01)
        return False

    def test_reader_failure_is_reported_not_silent(self):
        fake, head, tr = self.make()
        tr.start()
        fake.read_error = SerialException("device disconnected")
        self.assertTrue(self.wait_for(lambda: tr.failed))
        self.assertIn("disconnected", str(tr.last_error))
        with self.assertRaises(OwletError):
            tr.set_rates(1, 0)
        tr.stop()  # must not hang or raise

    def test_device_reboot_is_detected(self):
        fake, head, tr = self.make()
        tr.start()
        fake.inject("READY,2,test")
        self.assertTrue(self.wait_for(lambda: tr.failed))
        self.assertEqual(tr.last_error.code, "REBOOT")
        with self.assertRaises(OwletError):
            tr.set_rates(1, 0)
        tr.stop()

    def test_device_clock_going_backwards_is_treated_as_reboot(self):
        fake, head, tr = self.make()
        tr.start()
        fake.inject("POS,1.0,0.0,1.0,0.0,M,3600020")
        self.assertTrue(self.wait_for(lambda: tr.latest is not None))
        fake.inject("POS,0.0,0.0,0.0,0.0,D,500")
        self.assertTrue(self.wait_for(lambda: tr.failed))
        self.assertEqual(tr.last_error.code, "REBOOT")
        tr.stop()

    def test_err_without_fields_does_not_kill_the_reader(self):
        fake, head, tr = self.make()
        tr.start()
        fake.rx += b"$ERR*0B\r\n"  # ERR with no fields
        fake.inject("POS,2.0,0.0,2.0,0.0,M,99999")
        self.assertTrue(self.wait_for(lambda: tr.latest is not None and tr.latest.pan == 2.0))
        self.assertFalse(tr.failed)
        tr.stop()

    def test_clock_is_resynced_periodically(self):
        fake, head, tr = self.make(resync_s=0.05)
        tr.start()
        initial = len(fake.sent("CLOCK"))
        self.assertTrue(self.wait_for(lambda: len(fake.sent("CLOCK")) >= initial + 3))
        tr.stop()

    def test_lost_clock_reply_does_not_stop_resyncing(self):  # N3
        answered = [0]

        def respond(s):
            if s.name == "CLOCK":
                answered[0] += 1
                if answered[0] > 8:      # initial sync succeeds, later replies are lost
                    return []
            return default_responder(s)
        fake = FakeSerial(respond)
        head = Owlet(transport=fake)
        tr = Tracker(head, resync_s=0.05)
        tr.start()
        self.assertTrue(self.wait_for(lambda: len(fake.sent("CLOCK")) >= 8 + 3, timeout=3.0))
        tr.stop()

    def test_late_clock_reply_does_not_steal_another_requests_count(self):  # R1
        import threading
        late = []

        def respond(s):
            if s.name == "CLOCK" and len(late) >= 8:   # after the initial sync...
                threading.Timer(0.7, lambda: fake.inject(f"CLOCK,{len(late)}")).start()
                late.append(s)
                return []                              # ...answer only after the 0.5 s timeout
            if s.name == "CLOCK":
                late.append(s)
            if s.name == "VEL":
                return []                              # this VEL's reply is still due
            return default_responder(s)
        fake = FakeSerial(respond)
        head = Owlet(transport=fake)
        tr = Tracker(head, resync_s=0.05)
        tr.start()
        self.assertTrue(self.wait_for(lambda: tr.clock_misses >= 1, timeout=2.0))
        tr.set_rates(1, 0)
        self.assertTrue(self.wait_for(lambda: len(late) >= 10, timeout=3.0))
        time.sleep(0.9)  # let the late CLOCK replies land
        self.assertGreaterEqual(head._outstanding, 1)  # the VEL is still owed a reply
        tr.stop()

    def test_one_lost_reply_does_not_make_stop_fail(self):  # R2
        lost = [False]

        def respond(s):
            if s.name == "VEL" and not lost[0]:
                lost[0] = True
                return []                              # one reply lost in transit
            return default_responder(s)
        fake = FakeSerial(respond)
        head = Owlet(transport=fake, timeout=1.0)
        tr = Tracker(head)
        tr.start()
        tr.set_rates(1, 0)
        time.sleep(0.05)
        t0 = time.monotonic()
        tr.stop()
        self.assertLess(time.monotonic() - t0, 0.5)    # no timeout wait
        self.assertIsNone(tr.last_error)
        self.assertEqual(head._outstanding, 0)

    def test_reader_keeps_outstanding_count_bounded(self):  # N5
        fake, head, tr = self.make()
        tr.start()
        for _ in range(50):
            tr.set_rates(1, 0)
        self.assertTrue(self.wait_for(lambda: head._outstanding == 0))
        tr.stop()

    def test_restart_resets_session_state(self):  # N3
        fake, head, tr = self.make()
        tr.start()
        fake.inject("POS,1.0,0.0,1.0,0.0,M,99999")
        self.assertTrue(self.wait_for(lambda: tr.latest is not None))
        tr.stop()
        tr.start()
        self.assertIsNone(tr.latest)
        self.assertEqual(len(tr.history), 0)
        tr.stop()

    def test_rates_are_clamped_to_the_device_limit(self):
        fake, head, tr = self.make()
        tr.start()
        tr.set_rates(900, -900)
        self.assertIn("$VEL,500,-500", fake.sent("VEL")[-1])
        tr.stop()

    def test_set_rates_after_stop_raises(self):
        fake, head, tr = self.make()
        tr.start()
        tr.stop()
        with self.assertRaises(OwletError):
            tr.set_rates(1, 0)

    def test_watchdog_cannot_be_disabled_for_tracking(self):
        with self.assertRaises(ValueError):
            Tracker(Owlet(transport=FakeSerial()), watchdog_ms=0)

    def test_stop_is_best_effort_when_the_link_misbehaves(self):
        def respond(s):
            if s.name == "STREAM" and s.fields == ("0",):
                return []  # never answered
            return default_responder(s)
        fake = FakeSerial(respond)
        head = Owlet(transport=fake, timeout=0.2)
        tr = Tracker(head)
        tr.start()
        tr.stop()  # must not raise out of __exit__ paths
        self.assertIsNotNone(tr.last_error)
        self.assertTrue(fake.sent("VEL"))  # the stop command was still sent


if __name__ == "__main__":
    unittest.main()
