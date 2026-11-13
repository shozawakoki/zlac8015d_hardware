#!/usr/bin/env python3
"""No-hardware unit tests for MIMO yaw ESO/current decomposition."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT=Path(__file__).resolve().parents[1]/"tools"/"analyze_mimo_eso_balance.py"
spec=importlib.util.spec_from_file_location("mimo_eso_balance",SCRIPT)
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def sample(t=23.9, tracking_left=1, d=4.67):
    return {
        "time_s":str(t),
        "ref_omega_rad_s":"-0.60",
        "omega_rad_s":"-0.81",
        "hat_omega_rad_s":"-0.78",
        "hat_d_omega_rad_s2":str(d),
        "cmd_current_left_A":"2.49",
        "cmd_current_right_A":"1.32",
        "actual_current_left_A":"2.52",
        "actual_current_right_A":"1.09",
        "tracking_left":str(tracking_left),
        "tracking_right":"1",
        "ref_v_m_s":"0.70",
        "ref_left_rad_s":"12.6",
        "ref_right_rad_s":"9.1",
        "stall_lockout_left":"0",
        "stall_lockout_right":"0",
    }


class MimoEsoBalanceTests(unittest.TestCase):
    def test_reconstruct_239s_current_difference(self):
        k=18*.064/.354
        row=mod.analyze_row(sample(),5,k,.05)
        self.assertIsNotNone(row)
        self.assertAlmostEqual(row["feedback_accel_rad_s2"],.9)
        self.assertAlmostEqual(row["disturbance_accel_rad_s2"],-4.67)
        self.assertAlmostEqual(row["predicted_diff_A"],-1.15858,places=3)
        self.assertAlmostEqual(row["commanded_diff_A"],-1.17)
        self.assertAlmostEqual(row["command_residual_A"],-0.01142,places=3)
        self.assertAlmostEqual(row["measured_diff_A"],-1.43)
        self.assertTrue(row["disturbance_dominates"])
        self.assertTrue(row["opposite_error_sign"])

    def test_excludes_non_track(self):
        self.assertIsNone(mod.analyze_row(sample(tracking_left=0),5,3.254,.05))

    def test_excludes_nonmotion_or_stall_lockout(self):
        r=sample()
        r["ref_v_m_s"]="0"
        self.assertIsNone(mod.analyze_row(r,5,3.254,.05))
        r=sample()
        r["stall_lockout_left"]="1"
        self.assertIsNone(mod.analyze_row(r,5,3.254,.05))

    def test_summary_binned(self):
        k=18*.064/.354
        rows=[mod.analyze_row(sample(t=23.91),5,k,.05),
              mod.analyze_row(sample(t=23.92),5,k,.05)]
        subset,bins=mod.summarize(rows,23.9,24.0,.1)
        self.assertEqual(len(subset),2)
        self.assertEqual(len(bins),1)
        self.assertAlmostEqual(bins[0]["predicted_diff_A"],-1.15858,places=3)
        self.assertEqual(bins[0]["disturbance_dominates_pct"],100.0)


if __name__=="__main__":
    unittest.main()
