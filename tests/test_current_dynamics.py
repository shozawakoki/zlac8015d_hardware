#!/usr/bin/env python3
"""Offline tests for current-step dynamics analysis; no hardware or ROS."""
import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "analyze_current_dynamics.py"
spec = importlib.util.spec_from_file_location("current_dynamics", SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class TestDynamics(unittest.TestCase):
    @staticmethod
    def sample_rows():
        rows = [
            {"elapsed_s":"0.0", "step_index":"0", "event":"step_change",
             "command_left_A":"1.0"}]
        for i in range(1, 11):
            t = i * 0.1
            rows.append({"elapsed_s":str(t), "step_index":"0", "event":"current_rx",
                         "actual_left_A":str(0.1*i)})
            rows.append({"elapsed_s":str(t), "step_index":"0", "event":"velocity_rx",
                         "left_rpm":str(10*i)})
        rows.append({"elapsed_s":"1.1", "step_index":"0", "event":"complete_zero"})
        for t in (1.2, 1.3):
            rows.append({"elapsed_s":str(t), "step_index":"0", "event":"current_rx",
                         "actual_left_A":"0"})
        return rows

    def test_regression_slope(self):
        self.assertAlmostEqual(mod.linear_slope([(0, 0), (.1, .1), (.2, .2)]), 1)
        self.assertIsNone(mod.linear_slope([(0, 0)]))

    def test_exclude_zero_current_coast(self):
        report = mod.compute(self.sample_rows(), 1, "left", .1)
        self.assertEqual(report["n_current"], 10)
        self.assertEqual(report["n_velocity"], 10)
        self.assertAlmostEqual(report["mean_current_A"], .55)
        self.assertAlmostEqual(report["command_A"], 1)
        self.assertGreater(report["wheel_accel_rad_s2"], 10)

    def test_command_vs_measured_error(self):
        result = mod.make_bins([(0.01, .2), (.03, .4)],
                               [(0.01, 5)], 1.0, .1, .1)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]["actual_mean_A"], .3)
        self.assertAlmostEqual(result[0]["error_mean_A"], .7)

    def test_no_stage_is_error(self):
        with self.assertRaisesRegex(ValueError, "No step_change"):
            mod.compute(self.sample_rows(), 3)


if __name__ == "__main__":
    unittest.main()
