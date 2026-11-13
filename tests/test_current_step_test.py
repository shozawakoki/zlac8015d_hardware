#!/usr/bin/env python3
"""Offline unit checks for tools/current_step_test.py; no CAN hardware needed."""

import csv
import io
import importlib.util
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock

SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "current_step_test.py"
spec = importlib.util.spec_from_file_location("zlac_current_step_test", SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class CurrentStepTest(unittest.TestCase):
    def setUp(self):
        self.rows = io.StringIO()
        self.writer = csv.DictWriter(self.rows, fieldnames=mod.CSV_FIELDS)
        self.writer.writeheader()
        self.args = mod.parser_args([])
        self.tester = mod.ZlacCurrentTester(self.args, self.writer)
        self.tester.send = Mock()

    def test_profile_exact_sequence(self):
        self.assertEqual(mod.parse_profile(self.args.profile), (2, 4, -4, -2, 6, -2, 4))
        self.assertEqual(self.args.step_seconds, 3.0)
        self.assertFalse(self.args.execute)

    def test_default_safety_denies_full_profile(self):
        self.args.execute = True
        with self.assertRaisesRegex(ValueError, "needs 6 A"):
            mod.preflight(self.args, mod.PROFILE_A)

    def test_left_sign_and_target_encoding(self):
        self.tester.command(2.0)
        self.tester.send.assert_called_once_with(
            0x201, struct.pack("<hh", -2000, 0)
        )
        self.assertEqual(self.tester.raw_l, -2000)
        self.assertEqual(self.tester.raw_r, 0)

    def test_right_sign(self):
        self.args.wheel = "right"
        self.tester.command(-2.0)
        self.tester.send.assert_called_once_with(
            0x201, struct.pack("<hh", 0, -2000)
        )

    def test_both_wheels(self):
        self.args.wheel = "both"
        self.tester.command(2.0)
        self.tester.send.assert_called_once_with(
            0x201, struct.pack("<hh", -2000, 2000)
        )

    def test_current_above_software_limit_rejected(self):
        with self.assertRaisesRegex(ValueError, "hard limit"):
            self.tester.command(4.0)

    def test_can_feedback_conversion_and_csv(self):
        self.tester.t0 = 1.0
        self.tester.process_feedback((0x181, struct.pack("<ii", -1250, 2500), 1.010))
        self.tester.process_feedback((0x281, struct.pack("<hh", -17, 25), 1.015))
        self.assertAlmostEqual(self.tester.rpm_l, 125.0)
        self.assertAlmostEqual(self.tester.rpm_r, 250.0)
        self.assertAlmostEqual(self.tester.measured_l, 1.7)
        self.assertAlmostEqual(self.tester.measured_r, 2.5)
        self.assertEqual(self.tester.velocity_rx, 1)
        self.assertEqual(self.tester.current_rx, 1)
        self.rows.seek(0)
        logged = list(csv.DictReader(self.rows))
        self.assertEqual(len(logged), 2)
        self.assertEqual(logged[0]["event"], "velocity_rx")
        self.assertEqual(logged[1]["event"], "current_rx")
        self.assertAlmostEqual(float(logged[1]["velocity_rx_age_ms"]), 5.0)

    def test_speed_trip(self):
        self.tester.process_feedback((0x181, struct.pack("<ii", 0, 1250), 1.0),
                                     log=False)  # right wheel 125 rpm
        self.tester.process_feedback((0x281, struct.pack("<hh", 0, 0), 1.0),
                                     log=False)
        with self.assertRaisesRegex(RuntimeError, "Overspeed"):
            self.tester.check_safety(1.0)

    def test_statusword_sdo_read(self):
        self.tester.receive = Mock(return_value=(
            0x581, bytes([0x4B, 0x41, 0x60, 0x00, 0x27, 0, 0, 0]), 1.0))
        self.assertEqual(self.tester.sdo_read(0x6041, 0), 0x0027)
        self.tester.send.assert_called_with(
            0x601, bytes([0x40, 0x41, 0x60, 0, 0, 0, 0, 0]))

    def test_reject_drive_not_enabled(self):
        self.tester.configure_feedback = Mock()
        self.tester.sdo = Mock()
        self.tester.command = Mock()
        self.tester.sdo_read = Mock(side_effect=[0x0023, 4])
        with self.assertRaisesRegex(RuntimeError, "Both motors must be Operation Enabled"):
            self.tester.setup()
        self.tester.command.assert_called_with(0.0)

    def test_reject_one_motor_not_enabled(self):
        self.tester.configure_feedback = Mock()
        self.tester.sdo = Mock()
        self.tester.command = Mock()
        # Lower16 enabled, upper16 merely ready to switch on.
        self.tester.sdo_read = Mock(side_effect=[0x00210027, 4])
        with self.assertRaisesRegex(RuntimeError, "Both motors"):
            self.tester.setup()

    def test_reject_wrong_mode_display(self):
        self.tester.configure_feedback = Mock()
        self.tester.sdo = Mock()
        self.tester.command = Mock()
        self.tester.sdo_read = Mock(side_effect=[0x00270027, 3])
        with self.assertRaisesRegex(RuntimeError, "not Profile Torque"):
            self.tester.setup()

    def test_active_target_readback_decodes_signed_motor_currents(self):
        self.tester.t0 = 1.0
        self.tester.command_l = 1.0
        self.tester.command_r = 0.0
        self.tester.sdo_read = Mock(side_effect=[
            0x90279527,  # both motors operation enabled
            0xFC18,      # left target: -1000 mA, driver's sign convention
            0x0000,      # right target: 0 mA
        ])
        self.tester.read_target_during_motion()
        self.assertAlmostEqual(self.tester.readback_left, 1.0)
        self.assertAlmostEqual(self.tester.readback_right, 0.0)
        self.assertEqual(self.tester.readback_status_l, 0x9527)
        self.assertEqual(self.tester.readback_status_r, 0x9027)
        self.rows.seek(0)
        saved = list(csv.DictReader(self.rows))
        self.assertEqual(saved[-1]["event"], "active_target_readback")
        self.assertAlmostEqual(float(saved[-1]["readback_left_A"]), 1.0)

    def test_probe_requires_single_step(self):
        self.args.probe_target = True
        with self.assertRaisesRegex(ValueError, "exactly one step"):
            mod.preflight(self.args, mod.PROFILE_A)

    def test_probe_rejects_non_enabled_status(self):
        self.tester.t0 = 1.0
        self.tester.sdo_read = Mock(side_effect=[
            0x00219527, 0xFC18, 0x0000
        ])
        with self.assertRaisesRegex(RuntimeError, "changed motor state"):
            self.tester.read_target_during_motion()

    def test_reject_no_current_no_motion(self):
        self.tester.process_feedback((0x181, struct.pack("<ii", 0, 0), 1.79),
                                     log=False)
        self.tester.process_feedback((0x281, struct.pack("<hh", 0, 0), 1.79),
                                     log=False)
        self.tester.command_l = 2.0
        self.tester.step_started = 1.0
        with self.assertRaisesRegex(RuntimeError, "No actuator response"):
            self.tester.check_safety(1.8)

    def test_stale_feedback_trip(self):
        self.tester.process_feedback((0x181, struct.pack("<ii", 0, 0), 1.0),
                                     log=False)
        self.tester.process_feedback((0x281, struct.pack("<hh", 0, 0), 1.0),
                                     log=False)
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.tester.check_safety(1.2)


if __name__ == "__main__":
    unittest.main()
