#!/usr/bin/env python3
"""Offline safety diagnostics tests; no CAN or motors."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / "tools" / "analyze_mimo_overspeed.py"
spec = importlib.util.spec_from_file_location("analyze_mimo_overspeed", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def row(time_s, ref_l, ref_r, vel_l, vel_r, cmd_l, cmd_r):
    radius = 0.064
    track = 0.354
    return {
        "time_s": str(time_s),
        "ref_left_rad_s": str(ref_l), "ref_right_rad_s": str(ref_r),
        "vel_left_rad_s": str(vel_l), "vel_right_rad_s": str(vel_r),
        "cmd_current_left_A": str(cmd_l), "cmd_current_right_A": str(cmd_r),
        "actual_current_left_A": "0.3", "actual_current_right_A": "0.3",
        "tracking_left": "1", "tracking_right": "1",
        "ref_v_m_s": str(radius*(ref_l+ref_r)/2),
        "ref_omega_rad_s": str(radius*(ref_r-ref_l)/track),
        "v_m_s": str(radius*(vel_l+vel_r)/2),
        "omega_rad_s": str(radius*(vel_r-vel_l)/track),
    }


class MimoAuditTests(unittest.TestCase):
    def test_saturation_and_wrong_direction(self):
        # Left-wheel target 2 rad/s but actual 5; +2A accelerates wrong way.
        r = row(1.0, 2, 6, 5, 7, 2.0, -0.5)
        x = mod.classify(r, 2.0, .5, .1, .3, .05, .1)
        self.assertTrue(x["composite"])
        self.assertTrue(x["sat_left"])
        self.assertTrue(x["wrong_direction_left"])
        self.assertTrue(x["overspeed_left"])
        self.assertFalse(x["sat_right"])

    def test_corrective_negative_current(self):
        r = row(1.0, 2, 6, 5, 7, -0.5, -0.5)
        x = mod.classify(r, 2.0, .5, .1, .3, .05, .1)
        self.assertFalse(x["wrong_direction_left"])

    def test_count_segments(self):
        rows = [
            row(0, 2, 6, 5, 7, 2, -.3),
            row(.005, 2, 6, 5, 7, 2, -.3),
            row(.01, 2, 6, 5, 7, 0, -.3),
            row(.02, 2, 6, 5, 7, 2, -.3),
        ]
        out = mod.summarize(rows, 2, .5, .1, .3, .05, .1)
        self.assertEqual(len(out["composite"]), 4)
        self.assertAlmostEqual(out["saturation_pct"], 75)
        self.assertEqual(len(out["segments"]), 2)

    def test_not_composite(self):
        rows = [row(0, 3, 3, 3, 3, 0, 0)]
        out = mod.summarize(rows, 2, .5, .1, .3, .05, .1)
        self.assertEqual(len(out["composite"]), 0)

if __name__ == "__main__":
    unittest.main()
