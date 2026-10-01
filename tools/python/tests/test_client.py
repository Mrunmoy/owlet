"""Client reply matching under unsolicited and late lines (review H2)."""

import unittest

from fakes import FakeSerial, default_responder
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
        self.cal = {"PAN": [500, 1500, 2500, -90.0, 90.0], "TILT": [500, 1500, 2500, -90.0, 90.0]}
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
                if self.enabled and new[:3] != self.cal[s.fields[0]][:3]:
                    return ["NAK,CAL,ENA"]
                self.cal[s.fields[0]] = new
                self.src = "M"
                return ["ACK,CAL"]
            if s.name == "CAL" and s.fields == ("SAVE",):
                self.src = "S"
                return ["ACK,CAL"]
            if s.name == "CAL" and s.fields == ("RESET",):
                self.cal["TILT"] = [500, 1500, 2500, -90.0, 90.0]
                self.src = "F"
                return ["ACK,CAL"]
            return default_responder(s)
        self.head = Owlet(transport=FakeSerial(respond))

    def test_caps(self):
        c = self.head.caps()
        self.assertEqual((c.speed_min, c.speed_max, c.velocity_max), (1.0, 500.0, 500.0))
        self.assertEqual((c.watchdog_min_ms, c.watchdog_max_ms, c.stream_max_hz), (50, 5000, 50))

    def test_read_calibration(self):
        c = self.head.calibration("pan")
        self.assertEqual((c.axis, c.min_us, c.center_us, c.max_us), ("pan", 500, 1500, 2500))
        self.assertEqual((c.min_deg, c.max_deg, c.reversed, c.source), (-90.0, 90.0, False, "factory"))

    def test_set_limits_keeps_pulses(self):
        self.head.set_limits("pan", -85, 90)
        c = self.head.calibration("pan")
        self.assertEqual((c.min_deg, c.max_deg, c.min_us, c.max_us), (-85.0, 90.0, 500, 2500))
        self.assertEqual(c.source, "modified")

    def test_set_reversed_swaps_pulse_endpoints(self):
        self.head.set_reversed("tilt", True)
        c = self.head.calibration("tilt")
        self.assertEqual((c.min_us, c.max_us, c.reversed), (2500, 500, True))
        self.head.set_reversed("tilt", True)  # idempotent
        self.assertEqual(self.head.calibration("tilt").min_us, 2500)
        self.head.set_reversed("tilt", False)
        self.assertFalse(self.head.calibration("tilt").reversed)

    def test_reversing_mirrors_the_limits(self):  # review C3
        self.head.set_limits("tilt", -30, 80)
        self.head.set_reversed("tilt", True)
        c = self.head.calibration("tilt")
        # the same physical end stops, seen from the other direction
        self.assertEqual((c.min_deg, c.max_deg, c.reversed), (-80.0, 30.0, True))

    def test_pulse_change_while_enabled_is_refused(self):
        self.enabled = True
        with self.assertRaises(OwletError) as cm:
            self.head.set_reversed("tilt", True)
        self.assertEqual(cm.exception.code, "ENA")

    def test_save_and_reset(self):
        self.head.set_reversed("tilt", True)
        self.head.save_calibration()
        self.assertEqual(self.head.calibration("tilt").source, "saved")
        self.head.reset_calibration()
        c = self.head.calibration("tilt")
        self.assertEqual((c.reversed, c.source), (False, "factory"))

    def test_unknown_axis_is_a_value_error(self):
        with self.assertRaises(ValueError):
            self.head.calibration("roll")


if __name__ == "__main__":
    unittest.main()
