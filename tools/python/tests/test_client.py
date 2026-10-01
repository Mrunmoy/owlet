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


if __name__ == "__main__":
    unittest.main()
