#!/usr/bin/env python3
"""Offline regression checks for the focused MIMO-LADRC log diagnostics."""
import importlib.util
from pathlib import Path
import unittest

SRC=Path(__file__).resolve().parents[1]/"tools"/"analyze_mimo_focus.py"
spec=importlib.util.spec_from_file_location("mimo_focus",SRC)
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def make(t,track_l=1,track_r=1,cmd_l=2,cmd_r=-.3,
         ref_l=2,ref_r=6,vel_l=5,vel_r=7):
    radius, track=.064, .354
    return dict(
        time_s=str(t),tracking_left=str(track_l),tracking_right=str(track_r),
        ref_left_rad_s=str(ref_l),ref_right_rad_s=str(ref_r),
        vel_left_rad_s=str(vel_l),vel_right_rad_s=str(vel_r),
        cmd_current_left_A=str(cmd_l),cmd_current_right_A=str(cmd_r),
        actual_current_left_A=".3",actual_current_right_A="-.1",
        ref_v_m_s=str(radius*(ref_l+ref_r)/2),
        ref_omega_rad_s=str(radius*(ref_r-ref_l)/track),
        v_m_s=str(radius*(vel_l+vel_r)/2),
        omega_rad_s=str(radius*(vel_r-vel_l)/track),
    )


class FocusTests(unittest.TestCase):
    def sample(self):
        return [mod.classify(make(.1),2,.05,.1,.5),
                mod.classify(make(.2,track_l=0,cmd_l=-.5),2,.05,.1,.5),
                mod.classify(make(.3,ref_l=3,ref_r=3),2,.05,.1,.5)]

    def test_saturation_overspeed_sign(self):
        a=self.sample()[0]
        self.assertTrue(a["composite"])
        self.assertTrue(a["saturated"])
        self.assertTrue(a["both_track"])
        self.assertTrue(a["overspeed"])
        self.assertTrue(a["driving_during_overspeed"])
        self.assertTrue(a["braking_during_overspeed"])
        # Left +2A accelerates during overspeed; right -0.3A brakes.

    def test_not_track_and_not_composite(self):
        a,b,c=self.sample()
        self.assertFalse(b["both_track"])
        self.assertTrue(b["braking_during_overspeed"])
        self.assertFalse(c["composite"])

    def test_yaw_differential_current_opposes_recovery(self):
        # Omega error positive: turning too fast clockwise, needs CCW correction.
        # Right-minus-left current negative instead drives clockwise.
        x=make(1.0,cmd_l=3.0,cmd_r=1.0,
               ref_l=12.5,ref_r=9.0,
               vel_l=14.0,vel_r=9.0)
        o=mod.classify(x,8.0,.05,.1,.5)
        self.assertTrue(o["composite"])
        self.assertAlmostEqual(o["diff_cmd_a"],-2.0)
        self.assertGreater(o["yaw_error_rad_s"],0)
        self.assertTrue(o["yaw_wrong_sign"])
        self.assertFalse(o["saturated"])

    def test_group_summary(self):
        groups=mod.summarize(self.sample())
        self.assertEqual(groups[0][1],2)
        self.assertEqual(groups[1][1],1)
        self.assertEqual(groups[2][1],1)
        self.assertEqual(groups[3][1],1)

    def test_focus_bins(self):
        b=list(mod.bins(self.sample(),0,.4,.2))
        self.assertEqual(len(b),2)
        self.assertAlmostEqual(b[0]["sat_pct"],100)
        self.assertAlmostEqual(b[1]["track_pct"],50)


if __name__=="__main__":
    unittest.main()
