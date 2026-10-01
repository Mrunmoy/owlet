"""End-to-end: Python client ↔ pty ↔ owlet_sim (the real C++ core).

Needs the host build: ./build.py --host. Skips when owlet_sim is missing.
Override the binary path with OWLET_SIM=/path/to/owlet_sim.
"""

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from owlet import Owlet, OwletError

REPO = Path(__file__).resolve().parents[3]
SIM = Path(os.environ.get("OWLET_SIM", REPO / "build/host/apps/host-sim/owlet_sim"))


@unittest.skipUnless(SIM.exists(), f"owlet_sim not built ({SIM})")
class EndToEndTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.link = os.path.join(self.tmp.name, "owlet")
        self.sim = subprocess.Popen([str(SIM), "--link", self.link],
                                    stdout=subprocess.PIPE, text=True)
        self.assertEqual(self.sim.stdout.readline().strip(), self.link)
        self.head = Owlet(self.link, timeout=1.0)

    def tearDown(self):
        self.head.close()
        self.sim.terminate()
        self.sim.wait(timeout=5)
        self.sim.stdout.close()
        self.tmp.cleanup()

    def test_handshake(self):
        self.assertEqual(self.head.protocol_version, 1)
        self.assertEqual(self.head.firmware_version, "0.1.0")

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

    def test_cli(self):
        def cli(*args):
            return subprocess.run([sys.executable, "-m", "owlet", "-p", self.link, *args],
                                  capture_output=True, text=True, timeout=10,
                                  cwd=REPO / "tools/python")
        self.assertIn("protocol v1", cli("ping").stdout)
        self.assertEqual(cli("enable").returncode, 0)
        out = cli("move", "10", "-20", "--wait").stdout
        self.assertIn("pan   10.0  tilt  -20.0", out)
        bad = cli("move", "100", "0")
        self.assertEqual(bad.returncode, 1)
        self.assertIn("RNG", bad.stderr)


if __name__ == "__main__":
    unittest.main()
