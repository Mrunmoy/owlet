"""What the head reports by itself reaches the functions connected to it."""

import threading
import time
import unittest

from fakes import FakeSerial, default_responder
from owlet import HeadEvent, Owlet, OwletError


def responder_with(overrides):
    def respond(s):
        if s.name in overrides:
            return overrides[s.name](s)
        return default_responder(s)
    return respond


def wait_for(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return predicate()


class ManualDispatchTest(unittest.TestCase):
    """dispatch="manual": no thread; handlers run when the program reads."""

    def setUp(self):
        self.fake = FakeSerial()
        self.head = Owlet(transport=self.fake, dispatch="manual")
        self.addCleanup(self.head.close)
        self.heard = []

    def record(self, name):
        return lambda *args: self.heard.append((name,) + args)

    def test_each_kind_of_line_reaches_its_signal(self):
        h = self.head
        for name in ("arrived", "watchdog_expired", "bus_fault", "bus_ok", "limit_hit", "stalled",
                     "fault_changed", "driver_online", "driver_offline", "position_lost", "homed",
                     "homing_failed", "events_lost"):
            getattr(h, name).connect(self.record(name))
        for body in ("ARRIVED,12.5,-3.0", "TIMEOUT,VEL", "ERR,BUS", "EVT,BUSOK,-,0",
                     "EVT,LIMIT,PAN,MAX", "EVT,STALL,TILT,0", "EVT,FAULT,PAN,6", "EVT,ONLINE,PAN,0",
                     "EVT,OFFLINE,TILT,0", "EVT,LOST,PAN,0", "EVT,HOMED,-,0", "EVT,HOMEFAIL,TILT,2",
                     "EVT,OVERFLOW,-,3"):
            self.fake.inject(body)
        self.assertEqual(h.process_events(), 13)
        self.assertEqual(self.heard, [
            ("arrived", 12.5, -3.0), ("watchdog_expired",), ("bus_fault",), ("bus_ok",),
            ("limit_hit", "pan", "max"), ("stalled", "tilt"), ("fault_changed", "pan", 6),
            ("driver_online", "pan"), ("driver_offline", "tilt"), ("position_lost", "pan"),
            ("homed",), ("homing_failed", "tilt", "2"), ("events_lost", 3),
        ])

    def test_no_thread_is_started(self):
        self.head.stalled.connect(self.record("stalled"))
        self.assertIsNone(self.head._listener)
        self.fake.inject("EVT,STALL,PAN,0")
        time.sleep(0.05)
        self.assertEqual(self.heard, [], "nothing runs until the program reads")
        self.head.process_events()
        self.assertEqual(self.heard, [("stalled", "pan")])

    def test_process_events_returns_at_once_when_there_is_nothing(self):
        t = time.monotonic()
        self.assertEqual(self.head.process_events(), 0)
        self.assertLess(time.monotonic() - t, 0.2)

    def test_every_evt_line_also_reaches_the_general_signal(self):
        self.head.event.connect(self.heard.append)
        self.fake.inject("EVT,LIMIT,TILT,MIN")
        self.fake.inject("EVT,SOMETHINGNEW,-,7")  # from newer firmware than this client
        self.head.process_events()
        self.assertEqual(self.heard, [HeadEvent("LIMIT", "tilt", "MIN"),
                                      HeadEvent("SOMETHINGNEW", None, "7")])

    def test_a_line_that_arrives_during_a_request_is_heard_and_the_request_succeeds(self):
        self.fake.responder = responder_with({"STOP": lambda s: ["EVT,STALL,TILT,0", "ACK,STOP"]})
        self.head.stalled.connect(self.record("stalled"))
        self.head.stop()
        self.assertEqual(self.heard, [("stalled", "tilt")])

    def test_a_handler_that_raises_stops_neither_the_others_nor_the_link(self):
        def bad(axis):
            raise RuntimeError("boom")
        self.head.stalled.connect(bad)
        self.head.stalled.connect(self.record("stalled"))
        self.fake.inject("EVT,STALL,PAN,0")
        with self.assertLogs("owlet", level="ERROR"):
            self.head.process_events()
        self.assertEqual(self.heard, [("stalled", "pan")])
        self.assertEqual(self.head.watchdog(), 500)

    def test_a_malformed_event_line_is_ignored(self):
        self.head.event.connect(self.heard.append)
        self.fake.inject("EVT,LIMIT")
        self.fake.inject("ARRIVED,x,y")
        self.head.process_events()
        self.assertEqual(self.heard, [])

    def test_the_older_hook_still_sees_every_line(self):
        lines = []
        self.head.on_event = lambda s: lines.append(s.name)
        self.fake.inject("EVT,STALL,PAN,0")
        self.fake.inject("TIMEOUT,VEL")
        self.head.process_events()
        self.assertEqual(lines, ["EVT", "TIMEOUT"])

    def test_trouble_codes_are_fetched_and_explained(self):
        records = ["DTC,0,21,TILT,2,6,1000,1510", "DTC,1,30,PAN,1,1,1520,1520", "DTC,2,99,-,1,0,5,5"]
        self.fake.responder = responder_with({"DTC": lambda s: (
            ["DTC,3"] if not s.fields else ["ACK,DTC"] if s.fields[0] == "CLEAR"
            else [records[int(s.fields[0])]])})
        t = self.head.trouble_codes()
        self.assertEqual([(x.code, x.axis, x.count) for x in t], [(21, "tilt", 2), (30, "pan", 1), (99, None, 1)])
        self.assertEqual(t[0].name, "DRIVER-FAULT")
        self.assertEqual(t[0].detail_text, "over-temperature shutdown, short on coil A")
        self.assertEqual(t[1].detail_text, "max end")
        self.assertEqual(t[2].name, "CODE-99", "a code from newer firmware is still shown")
        self.assertIn("newer firmware", t[2].what)
        self.head.clear_trouble_codes()
        self.assertEqual(self.fake.sent("DTC")[-1][:10], "$DTC,CLEAR")

    def test_seek_home_waits_for_the_outcome(self):
        self.fake.responder = responder_with({"SEEK": lambda s: ["ACK,SEEK", "EVT,HOMED,-,0"]})
        self.head.homed.connect(self.record("homed"))
        self.head.seek_home(wait=True, timeout=1.0)
        self.assertEqual(self.heard, [("homed",)])

    def test_seek_home_raises_when_homing_fails(self):
        self.fake.responder = responder_with({"SEEK": lambda s: ["ACK,SEEK", "EVT,HOMEFAIL,TILT,1"]})
        with self.assertRaises(OwletError) as cm:
            self.head.seek_home(wait=True, timeout=1.0)
        self.assertIn("tilt", str(cm.exception))

    def test_seek_home_on_a_head_without_switches_is_refused(self):
        self.fake.responder = responder_with({"SEEK": lambda s: ["NAK,SEEK,UNS"]})
        with self.assertRaises(OwletError) as cm:
            self.head.seek_home()
        self.assertEqual(cm.exception.code, "UNS")

    def test_events_and_stats_commands(self):
        self.fake.responder = responder_with({"EVENTS": lambda s: ["ACK,EVENTS"],
                                              "STAT": lambda s: ["STAT,12,1,5"]})
        self.head.events(False)
        self.assertEqual(self.fake.sent("EVENTS")[-1][:9], "$EVENTS,0")
        self.assertEqual(self.head.stats(),
                         {"posted": 12, "dropped": 1, "high_water": 5, "late": 0, "worst_ms": 0})


class BackgroundReaderTest(unittest.TestCase):
    """The default: connecting a handler is all a program has to do."""

    def setUp(self):
        self.fake = FakeSerial()
        self.head = Owlet(transport=self.fake)
        self.addCleanup(self.head.close)

    def test_a_program_with_no_handlers_gets_no_thread(self):
        self.head.stop()
        self.assertIsNone(self.head._listener)

    def test_a_handler_runs_without_the_program_asking(self):
        heard = []
        done = threading.Event()

        def on_limit(axis, end):
            heard.append((axis, end, threading.current_thread().name))
            done.set()
        self.head.limit_hit.connect(on_limit)
        self.fake.inject("EVT,LIMIT,PAN,MIN")
        self.assertTrue(done.wait(1.0))
        self.assertEqual(heard, [("pan", "min", "owlet-events")])

    def test_requests_keep_working_while_events_arrive(self):
        heard = []
        self.head.stalled.connect(heard.append)
        stop = threading.Event()

        def flood():
            while not stop.is_set():
                self.fake.inject("EVT,STALL,PAN,0")
                time.sleep(0.001)
        t = threading.Thread(target=flood)
        t.start()
        try:
            requests = 0
            until = time.monotonic() + 0.2
            while time.monotonic() < until:
                self.assertEqual(self.head.watchdog(), 500)
                requests += 1
        finally:
            stop.set()
            t.join()
        self.assertGreater(requests, 10)
        self.assertTrue(wait_for(lambda: len(heard) > 10), f"heard {len(heard)}")
        self.assertTrue(all(axis == "pan" for axis in heard))

    def test_a_waited_move_returns_even_though_the_reader_is_running(self):
        self.fake.responder = responder_with(
            {"MOVE": lambda s: ["ACK,MOVE", "ARRIVED,5.0,0.0"]})
        arrivals = []
        self.head.arrived.connect(lambda pan, tilt: arrivals.append((pan, tilt)))
        for _ in range(20):  # the arrival comes with the reply: easy to lose to the reader
            self.assertIsNotNone(self.head.move(5, 0, wait=True, timeout=1.0))
        self.assertEqual(arrivals, [(5.0, 0.0)] * 20)

    def test_a_handler_may_ask_the_head_something(self):
        answers = []
        done = threading.Event()

        def on_stall(axis):
            answers.append(self.head.watchdog())
            done.set()
        self.head.stalled.connect(on_stall)
        self.fake.inject("EVT,STALL,TILT,0")
        self.assertTrue(done.wait(1.0))
        self.assertEqual(answers, [500])

    def test_close_stops_the_reader(self):
        self.head.stalled.connect(lambda axis: None)
        reader = self.head._listener
        self.assertTrue(reader.is_alive())
        self.head.close()
        self.assertFalse(reader.is_alive())

    def test_a_held_reader_leaves_the_lines_for_whoever_holds_it(self):
        heard = []
        self.head.stalled.connect(heard.append)
        self.head.hold_listener()
        self.fake.inject("EVT,STALL,PAN,0")
        time.sleep(0.05)
        self.assertEqual(heard, [])
        self.assertEqual(self.head.read_sentence(0.2).name, "EVT")
        self.assertEqual(heard, ["pan"], "the handler still runs, from the holder's read")
        self.head.release_listener()


if __name__ == "__main__":
    unittest.main()
