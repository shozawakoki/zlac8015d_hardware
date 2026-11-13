#!/usr/bin/env python3
"""Pure offline tests for current-reversal report; does not open CAN."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import analyze_current_reversal as mod


class AnalyzeReversalTests(unittest.TestCase):
    def test_reversal_metrics(self):
        rows = [
            {"event": "reverse_command", "elapsed_s": ".1", "left_rpm": "15"},
            {"event": "velocity_rx", "elapsed_s": ".11", "left_rpm": "16"},
            {"event": "current_rx", "elapsed_s": ".11", "actual_left_A": ".4"},
            {"event": "velocity_rx", "elapsed_s": ".12", "left_rpm": "14.4"},
            {"event": "current_rx", "elapsed_s": ".12", "actual_left_A": "-.1"},
            {"event": "velocity_rx", "elapsed_s": ".14", "left_rpm": "-.2"},
            {"event": "velocity_rx", "elapsed_s": ".15", "left_rpm": "-3.1"},
            {"event": "reverse_confirmed", "elapsed_s": ".155"},
        ]
        r = mod.inspect(rows)
        self.assertAlmostEqual(r["switch_rpm"], 15.0)
        self.assertAlmostEqual(r["first_speed_drop_0p5rpm_lag_s"], .02)
        self.assertAlmostEqual(r["first_current_negative_lag_s"], .02)
        self.assertAlmostEqual(r["first_zero_speed_lag_s"], .04)
        self.assertAlmostEqual(r["confirmed_reverse_lag_s"], .055)
        self.assertAlmostEqual(r["max_rpm_after_reverse"], 16)
        self.assertTrue(r["confirmed"])

    def test_aborted_case_does_not_claim_reverse(self):
        rows = [
            {"event": "reverse_command", "elapsed_s": ".1", "left_rpm": "15"},
            {"event": "velocity_rx", "elapsed_s": ".11", "left_rpm": "17"},
            {"event": "current_rx", "elapsed_s": ".11", "actual_left_A": "0.2"},
        ]
        r = mod.inspect(rows)
        self.assertFalse(r["confirmed"])
        self.assertIsNone(r["first_current_negative_lag_s"])
        self.assertIsNone(r["first_zero_speed_lag_s"])

    def test_no_switch_marker(self):
        with self.assertRaisesRegex(ValueError, "No reverse_command"):
            mod.inspect([{"event": "forward_start"}])


if __name__ == "__main__":
    unittest.main()
