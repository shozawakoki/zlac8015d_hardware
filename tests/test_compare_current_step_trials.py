#!/usr/bin/env python3
"""Offline tests of historical current-step trial comparison, no CAN I/O."""
import csv
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "compare_current_step_trials.py"
spec = importlib.util.spec_from_file_location("compare_current_step_trials", SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

FIELDS = ("event", "elapsed_s", "step_index", "command_left_A",
          "driver_left_mA", "actual_left_A", "left_rpm")


def fake_rows(*, preflight=False, first_current=0.2, second_current=0.6):
    rows = []
    if preflight:
        rows.append(dict(event="preflight_verified", elapsed_s="", step_index="",
                         command_left_A=0, driver_left_mA=0, actual_left_A=0,
                         left_rpm=0))
    rows.append(dict(event="start", elapsed_s="10.0", step_index="-1",
                     command_left_A="0", driver_left_mA=0,
                     actual_left_A="0", left_rpm="0"))
    rows.append(dict(event="step_change", elapsed_s="10.0", step_index="0",
                     command_left_A="1.0", driver_left_mA="-1000",
                     actual_left_A="0", left_rpm="0"))
    for k in range(7):
        t = 10.01 + k*0.02
        val = first_current if k < 3 else second_current
        rows.append(dict(event="current_rx", elapsed_s=f"{t:.4f}", step_index="0",
                         command_left_A="1.0", driver_left_mA="-1000",
                         actual_left_A=str(val), left_rpm="999"))
        rows.append(dict(event="velocity_rx", elapsed_s=f"{t+.001:.4f}",
                         step_index="0", command_left_A="1.0",
                         driver_left_mA="-1000", actual_left_A="999",
                         left_rpm=str(5*k)))
    rows.append(dict(event="complete_zero", elapsed_s="10.145", step_index="0",
                     command_left_A=0, driver_left_mA=0,
                     actual_left_A=0, left_rpm=100))
    rows.append(dict(event="current_rx", elapsed_s="10.150", step_index="0",
                     command_left_A=0, driver_left_mA=0,
                     actual_left_A=55, left_rpm=150))
    return rows


class TrialComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.csv_path = Path(self.temp.name) / "steps.csv"

    def save(self, rows):
        with self.csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def test_fresh_rows_and_common_window(self):
        self.save(fake_rows())
        r = mod.extract(self.csv_path, window=.14, bin_s=.02)
        self.assertEqual(r["sent_mA"], (-1000,))
        self.assertEqual(r["n_current"], 7)
        self.assertEqual(r["n_speed"], 7)
        self.assertAlmostEqual(r["mean_A"], (3*.2+4*.6)/7)
        self.assertEqual(r["max_rpm"], 30)
        self.assertTrue(r["accel_rad_s2"] > 0)
        self.assertFalse(r["preflight"])
        self.assertEqual(len(r["bins"]), 7)
        self.assertEqual(r["start_held_rpm"], 0)

    def test_exact_multiple_window_has_no_phantom_bin(self):
        self.save(fake_rows())
        r = mod.extract(self.csv_path, window=0.14, bin_s=0.02)
        self.assertEqual(len(r["bins"]), 7)
        self.assertTrue(all(hi > lo for lo, hi, *_ in r["bins"]))
        self.assertAlmostEqual(r["bins"][-1][1], 0.14)
        self.assertEqual(mod.fmt_time(None), "n/a")

    def test_non_multiple_window_has_short_final_bin(self):
        self.save(fake_rows())
        r = mod.extract(self.csv_path, window=0.135, bin_s=0.02)
        self.assertEqual(len(r["bins"]), 7)
        self.assertAlmostEqual(r["bins"][-1][0], 0.12)
        self.assertAlmostEqual(r["bins"][-1][1], 0.135)

    def test_preflight_event_detected(self):
        self.save(fake_rows(preflight=True))
        r = mod.extract(self.csv_path)
        self.assertTrue(r["preflight"])

    def test_reject_no_fresh_measurements(self):
        rows = fake_rows()
        rows = [r for r in rows if r["event"] != "velocity_rx"]
        self.save(rows)
        with self.assertRaisesRegex(ValueError, "no fresh"):
            mod.extract(self.csv_path)

    def test_reject_short_recording(self):
        rows = fake_rows()
        rows = [r for r in rows if r["event"] not in ("current_rx", "velocity_rx")
                or float(r["elapsed_s"]) < 10.05]
        self.save(rows)
        with self.assertRaisesRegex(ValueError, "recording too short"):
            mod.extract(self.csv_path)

    def test_reject_missing_fields(self):
        with self.csv_path.open("w") as f:
            f.write("event,elapsed_s\nstep_change,10.0\n")
        with self.assertRaisesRegex(ValueError, "missing columns"):
            mod.extract(self.csv_path)

    def test_threshold_crossing(self):
        self.assertEqual(mod.first_crossing([(0, 0), (.1, -11)], 10), .1)
        self.assertIsNone(mod.first_crossing([(0, 0)], 10))


if __name__ == "__main__":
    unittest.main()
