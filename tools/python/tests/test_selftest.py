"""The motor self-test, against a scripted fake and against owlet_sim."""

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from owlet import Owlet, OwletError
from owlet.protocol import Capabilities, Diagnostics, Position
from owlet.selftest import FAIL, PASS, SKIP, SelfTest, summary

REPO = Path(__file__).resolve().parents[3]
SIM = Path(os.environ.get("OWLET_SIM", REPO / "build/host/apps/host-sim/owlet_sim"))


class FakeHead:
    """A head on simulated time: a move takes distance / speed on its own clock,
    so the tests run instantly. Faults are switched on per test."""

    port = "<fake>"

    def __init__(self):
        self.pos = [0.0, 0.0]
        self.now = 0.0
        self.enabled = False
        self.log = []
        self.fail_move = False
        self.accept_out_of_range = False
        self.has_diag = True
        self.faults = 0
        self._speed = 90.0
        self._watchdog = 500

    def ping(self):
        return 3, "test"

    def caps(self):
        return Capabilities(1.0, 240.0, 240.0, 50, 5000, 50)

    def limits(self):
        return (-90.0, 90.0, -60.0, 60.0)

    def diag(self, axis):
        if not self.has_diag:
            raise OwletError("ERR,UNK", code="UNK")
        return Diagnostics(axis, True, self.faults, False, 150, 458)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
        self.log.append("disable")

    def stop(self):
        self.log.append("stop")

    def speed(self, v=None):
        if v is not None:
            self._speed = v
        return self._speed

    def watchdog(self, ms=None):
        if ms is not None:
            self._watchdog = ms
        return self._watchdog

    def velocity(self, pan, tilt):
        self.pos[0] += pan * self._watchdog / 1000.0   # ran for one watchdog period, then stopped
        self.pos[1] += tilt * self._watchdog / 1000.0

    def move(self, pan, tilt, wait=False, timeout=None):
        if self.fail_move:
            raise OwletError("NAK,MOVE,BUS", code="BUS")
        if abs(pan) > 90 and not self.accept_out_of_range:
            raise OwletError("NAK,MOVE,RNG", code="RNG")
        self.now += max(abs(pan - self.pos[0]), abs(tilt - self.pos[1])) / self._speed
        self.pos = [float(pan), float(tilt)]
        return self.position()

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def position(self):
        return Position(self.pos[0], self.pos[1], self.pos[0], self.pos[1],
                        "H" if self.enabled else "D", 0)


def status_of(checks, name):
    return next(c.status for c in checks if c.name.startswith(name))


class SelfTestOnAFake(unittest.TestCase):
    def run_test(self, head, **kw):
        test = SelfTest(head, clock=head.clock, sleep=head.sleep, **kw)
        return test, test.run()

    def test_a_healthy_head_passes_and_is_left_disabled(self):
        head = FakeHead()
        test, checks = self.run_test(head)
        self.assertTrue(test.passed, "\n".join(f"{c.status} {c.name}: {c.detail}" for c in checks))
        self.assertEqual(status_of(checks, "link"), PASS)
        self.assertEqual(status_of(checks, "pan sweep"), PASS)
        self.assertEqual(status_of(checks, "both axes"), PASS)
        self.assertEqual(status_of(checks, "soft limit"), PASS)
        self.assertEqual(status_of(checks, "velocity mode"), PASS)
        self.assertEqual(status_of(checks, "home switches"), SKIP)
        self.assertEqual(checks[-1].name, "disable")
        self.assertFalse(head.enabled)

    def test_a_failing_move_fails_the_test_and_still_disables(self):
        head = FakeHead()
        head.fail_move = True
        test, checks = self.run_test(head)
        self.assertFalse(test.passed)
        self.assertIn("BUS", next(c.detail for c in checks if c.status == FAIL))
        self.assertFalse(head.enabled)
        self.assertEqual(head.log[-1], "disable")
        self.assertIn("FAILED", summary(checks))

    def test_an_unenforced_limit_is_a_failure(self):
        head = FakeHead()
        head.accept_out_of_range = True
        test, checks = self.run_test(head)
        self.assertEqual(status_of(checks, "soft limit"), FAIL)

    def test_a_shorted_coil_fails_but_an_open_coil_flag_alone_does_not(self):
        head = FakeHead()
        head.faults = 4
        self.assertEqual(status_of(self.run_test(head)[1], "drivers at rest"), FAIL)
        head = FakeHead()
        head.faults = 16
        self.assertEqual(status_of(self.run_test(head)[1], "drivers at rest"), PASS)

    def test_firmware_without_diag_is_skipped_not_failed(self):
        head = FakeHead()
        head.has_diag = False
        test, checks = self.run_test(head)
        self.assertEqual(status_of(checks, "drivers at rest"), SKIP)
        self.assertTrue(test.passed)

    def test_declining_the_question_moves_nothing(self):
        head = FakeHead()
        test = SelfTest(head, ask=lambda q: False)
        checks = test.run()
        self.assertEqual(status_of(checks, "motion"), SKIP)
        self.assertEqual(head.pos, [0.0, 0.0])
        self.assertFalse(head.enabled)

    def test_a_person_saying_it_is_off_its_marks_is_a_failure(self):
        head = FakeHead()
        answers = iter([True, False])   # yes, move; no, it did not come back
        test, checks = self.run_test(head, ask=lambda q: next(answers))
        self.assertEqual(status_of(checks, "return home"), FAIL)

    def test_a_head_that_arrives_too_soon_is_caught(self):
        head = FakeHead()
        test = SelfTest(head, clock=lambda: 0.0, sleep=lambda s: None)   # time never passes
        checks = test.run()
        self.assertEqual(status_of(checks, "pan sweep"), FAIL)
        self.assertIn("faster than the set speed", next(c.detail for c in checks if c.status == FAIL))

    def test_bad_arguments_are_refused(self):
        for kw in ({"span": 0}, {"span": 1.5}, {"speeds": ()}, {"speeds": (2.0,)}):
            with self.assertRaises(ValueError):
                SelfTest(FakeHead(), **kw)


@unittest.skipUnless(SIM.exists(), f"owlet_sim not built ({SIM})")
class SelfTestOnTheSimulator(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.link = os.path.join(self.tmp.name, "owlet")
        self.sim = subprocess.Popen([str(SIM), "--link", self.link], stdout=subprocess.PIPE, text=True)
        self.addCleanup(self._kill)
        self.assertEqual(self.sim.stdout.readline().strip(), self.link)

    def _kill(self):
        self.sim.kill()
        self.sim.wait()
        self.sim.stdout.close()

    def test_the_real_core_passes_and_the_timing_check_is_live(self):
        with Owlet(self.link) as head:
            test = SelfTest(head, span=0.3, speeds=(0.5, 1.0))
            checks = test.run()
            self.assertTrue(test.passed, "\n".join(f"{c.status} {c.name}: {c.detail}" for c in checks))
            self.assertEqual(status_of(checks, "pan sweep"), PASS)
            self.assertEqual(status_of(checks, "both axes"), PASS)
            self.assertEqual(status_of(checks, "velocity mode"), PASS)
            self.assertEqual(head.position().state, "D")

    def test_the_command_line_runs_it(self):
        r = subprocess.run([sys.executable, "-m", "owlet", "-p", self.link, "selftest", "--yes",
                            "--span", "0.2", "--speeds", "1.0"],
                           capture_output=True, text=True, timeout=60, cwd=REPO / "tools/python")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("self-test passed", r.stdout)
