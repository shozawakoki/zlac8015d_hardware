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

    def test_overspeed_abort_without_reverse_event(self):
        rows = [
            {"event": "forward_start", "elapsed_s": "0.0", "left_rpm": "0"},
            {"event": "velocity_rx", "elapsed_s": "0.10", "left_rpm": "65"},
            {"event": "current_rx", "elapsed_s": "0.10", "actual_left_A": "0.6"},
            {"event": "velocity_rx", "elapsed_s": "0.90", "left_rpm": "300"},
            {"event": "current_rx", "elapsed_s": "0.90", "actual_left_A": "0.4"},
            {"event": "velocity_rx", "elapsed_s": "1.20", "left_rpm": "352"},
            {"event": "velocity_rx", "elapsed_s": "1.30", "left_rpm": "372.3"}
        ]
        result = mod.inspect_before_reversal(rows)
        self.assertEqual(result["max_speed_rpm"], 372.3)
        self.assertAlmostEqual(result["duration_s"], 1.3)
        self.assertEqual(result["thresholds"][300], 0.9)
        self.assertEqual(result["thresholds"][370], 1.3)
        self.assertEqual(result["n_speed"], 4)
        self.assertEqual(result["n_current"], 2)
        self.assertAlmostEqual(result["current_mean_A"], 0.5)

    def test_no_reverse_event_does_not_invent_results(self):
        rows = [
            {"event": "forward_start", "elapsed_s": "0", "left_rpm": "0"},
            {"event": "velocity_rx", "elapsed_s": "0.50", "left_rpm": "10"},
        ]
        result = mod.inspect_before_reversal(rows)
        self.assertIsNone(result["thresholds"][370])
        self.assertIsNone(result["current_mean_A"])
        self.assertEqual(result["n_current"], 0)

    def test_no_reverse_missing_start_rejected(self):
        with self.assertRaisesRegex(ValueError, "No forward_start"):
            mod.inspect_before_reversal([{"event": "velocity_rx", "elapsed_s": "1",
                                          "left_rpm": "372.3"}])


if __name__ == "__main__":
    unittest.main()
