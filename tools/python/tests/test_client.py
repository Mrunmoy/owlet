"""Client reply matching under unsolicited and late lines (review H2)."""

import unittest

import time

from fakes import FakeSerial, NoInWaiting, default_responder
from owlet import Owlet, OwletError


def responder_with(overrides):
    def respond(s):
        if s.name in overrides:
            return overrides[s.name](s)
        return default_responder(s)
    return respond


class ReplyMatchingTest(unittest.TestCase):
    def test_unsolicited_err_bus_is_an_event_not_a_failure(self):
        fake = FakeSerial(responder_with({"STOP": lambda s: ["ERR,BUS", "ACK,STOP"]}))
        events = []
        head = Owlet(transport=fake)
        head.on_event = events.append
        head.stop()  # must not raise "STOP rejected: BUS"
        self.assertTrue(any(e.name == "ERR" and e.fields == ("BUS",) for e in events))
        self.assertEqual(len(fake.rx), 0)  # the real ACK was consumed, not left behind

    def test_replies_stay_aligned_after_an_unsolicited_err(self):
        fake = FakeSerial(responder_with({"STOP": lambda s: ["ERR,BUS", "ACK,STOP"]}))
        head = Owlet(transport=fake)
        head.stop()
        self.assertEqual(head.watchdog(), 500)  # gets its own reply, not a stale ACK

    def test_frame_error_for_a_fire_and_forget_request_is_not_blamed_later(self):
        fake = FakeSerial(responder_with({"VEL": lambda s: ["ERR,CHK"]}))
        head = Owlet(transport=fake)
        head.send("VEL", 10, 0)
        head.stop()  # the ERR,CHK belongs to the VEL, not to STOP

    def test_late_nak_for_another_command_is_not_ours(self):
        fake = FakeSerial(responder_with({"VEL": lambda s: ["NAK,VEL,DIS"]}))
        head = Owlet(transport=fake)
        head.send("VEL", 10, 0)
        head.stop()

    def test_stale_reply_with_the_same_name_is_not_ours(self):  # N2
        replies = iter([["NAK,VEL,DIS"], ["ACK,VEL"]])
        fake = FakeSerial(responder_with({"VEL": lambda s: next(replies)}))
        head = Owlet(transport=fake)
        head.send("VEL", 10, 0)   # answered NAK,VEL,DIS (stale by the time we ask again)
        head.velocity(5, 0)       # must succeed: its own reply is the ACK

    def test_stale_ack_cannot_hide_our_refusal(self):  # N2
        replies = iter([["ACK,VEL"], ["NAK,VEL,RNG"]])
        fake = FakeSerial(responder_with({"VEL": lambda s: next(replies)}))
        head = Owlet(transport=fake)
        head.send("VEL", 10, 0)
        with self.assertRaises(OwletError) as cm:
            head.velocity(5000, 0)
        self.assertEqual(cm.exception.code, "RNG")

    def test_a_timed_out_request_resets_outstanding_bookkeeping(self):  # N5
        fake = FakeSerial(responder_with({"VEL": lambda s: [], "FLY": lambda s: []}))
        head = Owlet(transport=fake, timeout=0.1)
        head.send("VEL", 1, 0)            # reply lost
        with self.assertRaises(OwletError):
            head.request("FLY")           # times out
        fake.responder = default_responder
        with self.assertRaises(OwletError) as cm:
            head.request("FLY")           # genuine ERR,UNK must surface, not be absorbed
        self.assertEqual(cm.exception.code, "UNK")

    def test_own_frame_error_still_raises(self):
        head = Owlet(transport=FakeSerial())
        with self.assertRaises(OwletError) as cm:
            head.request("FLY")
        self.assertEqual(cm.exception.code, "UNK")


class CapabilitiesAndCalibrationTest(unittest.TestCase):
    def setUp(self):
        # A stepper head: the drive numbers are always 0, only the limits change.
        self.cal = {"PAN": [0, 0, 0, -90.0, 90.0], "TILT": [0, 0, 0, -60.0, 60.0]}
        self.src = "F"
        self.enabled = False

        def respond(s):
            if s.name == "CAL" and s.fields and s.fields[0] in self.cal:
                if len(s.fields) == 1:
                    mn, ct, mx, lo, hi = self.cal[s.fields[0]]
                    rev = "R" if mn > mx else "N"
                    return [f"CAL,{s.fields[0]},{mn},{ct},{mx},{lo:.1f},{hi:.1f},{rev},{self.src}"]
                new = [int(s.fields[1]), int(s.fields[2]), int(s.fields[3]),
                       float(s.fields[4]), float(s.fields[5])]
                if new[:3] != [0, 0, 0]:
                    return ["NAK,CAL,RNG"]
                self.cal[s.fields[0]] = new
                self.src = "M"
                return ["ACK,CAL"]
            if s.name == "CAL" and s.fields == ("SAVE",):
                self.src = "S"
                return ["ACK,CAL"]
            if s.name == "CAL" and s.fields == ("RESET",):
                self.cal["TILT"] = [0, 0, 0, -60.0, 60.0]
                self.src = "F"
                return ["ACK,CAL"]
            return default_responder(s)
        self.head = Owlet(transport=FakeSerial(respond))

    def test_caps(self):
        c = self.head.caps()
        self.assertEqual((c.speed_min, c.speed_max, c.velocity_max), (1.0, 500.0, 500.0))
        self.assertEqual((c.watchdog_min_ms, c.watchdog_max_ms, c.stream_max_hz), (50, 5000, 50))

    def test_diag(self):
        d = self.head.diag("pan")
        self.assertTrue(d.available)
        self.assertEqual((d.axis, d.faults, d.stalled, d.load, d.current_ma), ("pan", 6, True, 37, 459))
        self.assertEqual(d.fault_names, ["over-temperature shutdown", "short on coil A"])
        t = self.head.diag("tilt")
        self.assertFalse(t.available)
        self.assertEqual(t.fault_names, [])
        with self.assertRaises(ValueError):
            self.head.diag("roll")

    def test_read_calibration(self):
        c = self.head.calibration("tilt")
        self.assertEqual((c.axis, c.min_deg, c.max_deg, c.source), ("tilt", -60.0, 60.0, "factory"))

    def test_set_limits_changes_only_the_limits(self):
        self.head.set_limits("pan", -85, 90)
        c = self.head.calibration("pan")
        self.assertEqual((c.min_deg, c.max_deg, c.source), (-85.0, 90.0, "modified"))

    def test_save_and_reset(self):
        self.head.set_limits("tilt", -30, 80)
        self.head.save_calibration()
        self.assertEqual(self.head.calibration("tilt").source, "saved")
        self.head.reset_calibration()
        c = self.head.calibration("tilt")
        self.assertEqual((c.min_deg, c.max_deg, c.source), (-60.0, 60.0, "factory"))

    def test_unknown_axis_is_a_value_error(self):
        with self.assertRaises(ValueError):
            self.head.calibration("roll")


class ReadLatencyTest(unittest.TestCase):
    """A reply must be handed over as soon as its line is complete, not when
    the serial read times out (found on hardware: every reply took 50 ms)."""

    def test_short_reply_does_not_wait_for_the_read_timeout(self):
        head = Owlet(transport=FakeSerial(timeout=0.05))
        rtts = []
        for _ in range(5):
            t0 = time.monotonic()
            head.clock()
            rtts.append(time.monotonic() - t0)
        self.assertLess(min(rtts), 0.01)

    def test_transport_without_in_waiting_still_works(self):
        head = Owlet(transport=NoInWaiting(FakeSerial(timeout=0.05)))
        t0 = time.monotonic()
        self.assertIsInstance(head.clock(), int)
        self.assertLess(time.monotonic() - t0, 0.01)  # read(1) returns on the first byte


if __name__ == "__main__":
    unittest.main()
