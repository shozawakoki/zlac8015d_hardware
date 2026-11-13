#!/usr/bin/env python3
"""No-hardware regression coverage of focused worst-speed-error diagnosis."""
import importlib.util
from pathlib import Path
import unittest

p = Path(__file__).resolve().parents[1] / "tools" / "analyze_mimo_extremes.py"
spec = importlib.util.spec_from_file_location("analyze_mimo_extremes", p)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def mk(t, ref_v=.7, v=.5, ref_o=-.6, o=-.4, track_l=1, track_r=1, ref_l=12):
    return {
        "time_s":str(t),
        "ref_left_rad_s":str(ref_l),
        "ref_right_rad_s":"8",
        "vel_left_rad_s":"10",
        "vel_right_rad_s":"7",
        "cmd_current_left_A":"1",
        "cmd_current_right_A":"2",
        "actual_current_left_A":".9",
        "actual_current_right_A":"1.7",
        "tracking_left":str(track_l),
        "tracking_right":str(track_r),
        "ref_v_m_s":str(ref_v),
        "ref_omega_rad_s":str(ref_o),
        "v_m_s":str(v),
        "omega_rad_s":str(o),
        "breakaway_target_left_A":".65",
        "breakaway_target_right_A":".65",
        "stall_lockout_left":"0",
        "stall_lockout_right":"0",
    }


class AnalyzeMimoExtremesTests(unittest.TestCase):
    def rows(self):
        return [
            mod.row_numbers(mk(0)),
            mod.row_numbers(mk(.1,v=-.9,track_l=0,ref_l=-12)),
            mod.row_numbers(mk(.2,o=2.5,track_l=0,track_r=0)),
            mod.row_numbers(mk(.3,v=0,ref_v=0,ref_o=0)),
        ]

    def test_exact_maxima(self):
        worst_v,worst_o,n = mod.find_extrema(self.rows())
        self.assertEqual(n,3)
        self.assertAlmostEqual(worst_v["time_s"],.1)
        self.assertAlmostEqual(worst_o["time_s"],.2)

    def test_sign_and_tracking_transitions(self):
        changed=mod.transitions(self.rows(),0,.2)
        self.assertTrue(any("left TRACK 1->0" in msg for _,msg in changed))
        self.assertTrue(any("left reference-sign 1->-1" in msg for _,msg in changed))

    def test_time_windows(self):
        selected,summary=mod.summarize_window(self.rows(),.1,.1,.1,.1)
        self.assertTrue(selected)
        self.assertEqual(len(summary),2)
        self.assertEqual(summary[1]["track_pct"],0)

    def test_optional_lockout_columns(self):
        r=mk(.1)
        del r["stall_lockout_left"]
        parsed=mod.row_numbers(r)
        self.assertIsNone(parsed["stall_lockout_left"])


if __name__=="__main__":
    unittest.main()
