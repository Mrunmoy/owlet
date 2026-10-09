"""Opt-in calibration acceptance check for the wider Pico mechanism.

Does not enable or move the head. Temporarily changes RAM limits and restores
them; never saves flash. Run only with camtrack stopped and motors disabled:
OWLET_WIDE_CAL_HW=1 python -m unittest discover -s tools/python/tests -p test_wide_calibration_hardware.py -v
"""
import os
import unittest
from owlet import Owlet, OwletError


@unittest.skipUnless(os.environ.get("OWLET_WIDE_CAL_HW") == "1",
                     "set OWLET_WIDE_CAL_HW=1 for the disabled Pico calibration check")
class WideCalibrationHardwareTest(unittest.TestCase):
    def test_supported_endpoints_and_rejection_outside_them(self):
        with Owlet(os.environ.get("OWLET_PORT")) as head:
            before = head.position()
            self.assertFalse(before.enabled, "disable the head before this check")
            original = {axis: head.calibration(axis) for axis in ("pan", "tilt")}
            self.assertTrue(all((c.min_us, c.center_us, c.max_us) == (0, 0, 0)
                                for c in original.values()), "requires stepper head")
            try:
                for axis, extent in (("pan", 180.0), ("tilt", 135.0)):
                    with self.subTest(axis=axis):
                        head.set_limits(axis, -extent, extent)
                        accepted = head.calibration(axis)
                        self.assertEqual((accepted.min_deg, accepted.max_deg),
                                         (-extent, extent))
                        for lo, hi in ((-extent - 0.1, extent),
                                       (-extent, extent + 0.1)):
                            with self.assertRaises(OwletError) as error:
                                head.set_limits(axis, lo, hi)
                            self.assertEqual(error.exception.code, "RNG")
                            c = head.calibration(axis)
                            self.assertEqual((c.min_deg, c.max_deg), (-extent, extent))
            finally:
                for axis, c in original.items():
                    head.set_calibration(axis, c.min_us, c.center_us, c.max_us,
                                         c.min_deg, c.max_deg)
                after = head.position()
                self.assertFalse(after.enabled)
                self.assertEqual((after.pan, after.tilt), (before.pan, before.tilt))
