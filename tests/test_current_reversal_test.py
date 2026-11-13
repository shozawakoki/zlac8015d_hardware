#!/usr/bin/env python3
"""Offline-only unit tests for speed-triggered torque reversal. No CAN hardware."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch, Mock

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import current_reversal_test as rev


class Clock:
    def __init__(self):
        self.t = 10.0

    def now(self):
        return self.t


class FakeTester:
    node = 1
    def __init__(self, clock, rpm_samples, *, max_rpm=45.0):
        self.clock = clock
        self.samples = iter(rpm_samples)
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.max_rpm = max_rpm
        self.commands = []
        self.events = []
        self.step_current = 0.0

    def wait_feedback(self):
        pass

    def wait_for_stable_zero(self):
        pass

    def read_preflight_snapshot(self):
        pass

    def sdo_read(self, index, subindex, timeout_s=0.15):
        if (index, subindex) == (0x2008, 0):
            return 370
        raise AssertionError("Unexpected SDO read")

    def check_safety(self, now):
        if abs(self.rpm_l) > self.max_rpm or abs(self.rpm_r) > self.max_rpm:
            raise RuntimeError("Overspeed")

    def command(self, amp):
        self.commands.append(amp)

    def log(self, event, when=None):
        self.events.append(event)

    def receive(self, timeout):
        self.clock.t += 0.005
        try:
            value = next(self.samples)
        except StopIteration:
            value = self.rpm_l
        return (0x181, b"\0" * 8, self.clock.now())

    def process_feedback(self, item):
        try:
            self.rpm_l = self._next_rpm
        except AttributeError:
            self.rpm_l = 0.0


class FakeSequenceTester(FakeTester):
    def receive(self, timeout):
        self.clock.t += 0.005
        try:
            value = next(self.samples)
        except StopIteration:
            value = self.rpm_l
        self.rpm_l = value
        return (0x181, b"\0" * 8, self.clock.now())

    def process_feedback(self, item):
        pass


class ReversalTests(unittest.TestCase):
    def parse(self, *extra):
        return rev.parse_args(list(extra))

    def test_default_preview_is_no_can(self):
        args = self.parse()
        self.assertFalse(rev.preflight(args))
        self.assertEqual(args.switch_rpm, 15.0)
        self.assertEqual(args.max_speed_rpm, 45.0)
        self.assertEqual(args.reverse_confirm_rpm, 3.0)

    def test_reject_too_high_current(self):
        args = self.parse("--current-a", "2.0")
        with self.assertRaisesRegex(ValueError, "current magnitude"):
            rev.preflight(args)

    def test_reject_unbounded_speed(self):
        args = self.parse("--max-speed-rpm", "120")
        with self.assertRaisesRegex(ValueError, "verified-safe-rpm"):
            rev.preflight(args)

    def test_above_60_requires_actual_verified_rating(self):
        args = self.parse("--max-speed-rpm", "120", "--verified-safe-rpm", "100")
        with self.assertRaisesRegex(ValueError, "verified-safe-rpm"):
            rev.preflight(args)
        args = self.parse("--max-speed-rpm", "120", "--verified-safe-rpm", "150")
        self.assertFalse(rev.preflight(args))

    def test_370_rpm_limit_requires_matching_verified_safe_rating(self):
        args = self.parse("--switch-after-s", "5.0", "--max-speed-rpm", "370")
        with self.assertRaisesRegex(ValueError, "verified-safe-rpm"):
            rev.preflight(args)

        args = self.parse(
            "--switch-after-s", "5.0",
            "--max-speed-rpm", "370",
            "--verified-safe-rpm", "370"
        )
        self.assertFalse(rev.preflight(args))

        args = self.parse(
            "--switch-after-s", "5.0",
            "--max-speed-rpm", "371",
            "--verified-safe-rpm", "400"
        )
        with self.assertRaisesRegex(ValueError, "370 rpm"):
            rev.preflight(args)

    def test_disable_speed_abort_only_for_five_second_hold(self):
        args = self.parse("--disable-speed-abort")
        with self.assertRaisesRegex(ValueError, "only supported"):
            rev.preflight(args)
        args = self.parse("--disable-speed-abort", "--switch-after-s", "5.0")
        self.assertFalse(rev.preflight(args))

    def test_disable_speed_abort_requires_explicit_live_confirmation(self):
        args = self.parse(
            "--execute", "--switch-after-s", "5.0", "--disable-speed-abort",
            "--confirm-rated-current", "--confirm-wheels-raised",
            "--confirm-e-stop", "--confirm-exclusive-can",
            "--confirm-regeneration-safe",
        )
        with self.assertRaisesRegex(ValueError, "confirm-hardware-speed-limiting"):
            rev.preflight(args)

    def test_2008h_still_checked_with_speed_abort_disabled(self):
        args = self.parse("--switch-after-s", "5.0", "--disable-speed-abort")
        tester = FakeSequenceTester(Clock(), [])
        self.assertEqual(rev.check_configured_drive_speed(tester, args), 370)
        tester.sdo_read = lambda *_args, **_kw: 360
        with self.assertRaisesRegex(RuntimeError, "does not match"):
            rev.check_configured_drive_speed(tester, args)

    def test_base_checker_can_ignore_speed_only_not_other_guards(self):
        import csv
        import io
        args = self.parse("--switch-after-s", "5.0", "--disable-speed-abort")
        args.max_current_a = args.current_a
        writer = csv.DictWriter(io.StringIO(), fieldnames=rev.base.CSV_FIELDS)
        tester = rev.base.ZlacCurrentTester(args, writer)
        now = 200.0
        tester.last_vel_time = now
        tester.last_cur_time = now
        tester.rpm_l = 372.3
        tester.rpm_r = 0.0
        tester.measured_l = 0.5
        tester.measured_r = 0.0
        tester.step_started = now
        tester.check_safety(now)
        args.disable_speed_abort = False
        with self.assertRaisesRegex(RuntimeError, "Overspeed"):
            tester.check_safety(now)
        args.disable_speed_abort = True
        tester.measured_l = 3.0
        with self.assertRaisesRegex(RuntimeError, "Measured motor current"):
            tester.check_safety(now)
        tester.measured_l = 0.5
        with self.assertRaisesRegex(RuntimeError, "feedback timeout"):
            tester.check_safety(now + 0.2)

    def test_live_requires_regeneration_approval(self):
        args = self.parse("--execute")
        with self.assertRaisesRegex(ValueError, "confirm-rated-current"):
            rev.preflight(args)
        args.confirm_rated_current = True
        args.confirm_wheels_raised = True
        args.confirm_e_stop = True
        args.confirm_exclusive_can = True
        with self.assertRaisesRegex(ValueError, "confirm-regeneration-safe"):
            rev.preflight(args)

    def test_forward_reverse_stop_sequence(self):
        args = self.parse()
        clock = Clock()
        tester = FakeSequenceTester(
            clock, [0.0, 5.0, 12.0, 15.5, 16.0, 12.0,
                    3.0, 0.0, -1.0, -2.5, -3.1, -3.2])
        with patch.object(rev.time, "monotonic", clock.now):
            rev.run_reversal(tester, args)
        self.assertIn("forward_start", tester.events)
        self.assertIn("reverse_command", tester.events)
        self.assertIn("zero_speed_crossing", tester.events)
        self.assertIn("reverse_confirmed", tester.events)
        self.assertEqual(tester.commands[0], 1.0)
        self.assertIn(-1.0, tester.commands)
        self.assertEqual(tester.commands[-1], 0.0)
        self.assertLess(tester.events.index("reverse_command"),
                        tester.events.index("zero_speed_crossing"))

    def test_forward_timeout_if_never_accelerates(self):
        args = self.parse("--max-forward-s", "0.05")
        clock = Clock()
        tester = FakeSequenceTester(clock, [0.1] * 50)
        with patch.object(rev.time, "monotonic", clock.now):
            with self.assertRaisesRegex(RuntimeError, "forward RPM trigger"):
                rev.run_reversal(tester, args)
        self.assertNotIn(-1.0, tester.commands)

    def test_reverse_timeout_if_negative_current_cannot_brake(self):
        args = self.parse("--max-reverse-s", "0.1")
        clock = Clock()
        tester = FakeSequenceTester(clock, [0, 15.0] + [20.0]*100)
        with patch.object(rev.time, "monotonic", clock.now):
            with self.assertRaisesRegex(RuntimeError, "Reverse not confirmed"):
                rev.run_reversal(tester, args)
        self.assertIn(-1.0, tester.commands)
        self.assertNotIn("reverse_confirmed", tester.events)

    def test_overspeed_in_reversal_aborts(self):
        args = self.parse()
        clock = Clock()
        tester = FakeSequenceTester(clock, [0, 15.0, 48.0])
        with patch.object(rev.time, "monotonic", clock.now):
            with self.assertRaisesRegex(RuntimeError, "Overspeed"):
                rev.run_reversal(tester, args)
        self.assertNotIn("reverse_confirmed", tester.events)

    def test_wrong_direction_aborts(self):
        args = self.parse()
        clock = Clock()
        tester = FakeSequenceTester(clock, [-1.0])
        with patch.object(rev.time, "monotonic", clock.now):
            with self.assertRaisesRegex(RuntimeError, "opposite"):
                rev.run_reversal(tester, args)


    def test_drive_speed_setting_check_rejects_mismatch(self):
        args = self.parse("--max-speed-rpm", "45")
        tester = FakeSequenceTester(Clock(), [])
        self.assertEqual(rev.check_configured_drive_speed(tester, args), 370)
        tester.sdo_read = lambda *_args, **_kw: 30
        with self.assertRaisesRegex(RuntimeError, "exceeds drive"):
            rev.check_configured_drive_speed(tester, args)

    def test_drive_speed_setting_readback_invalid(self):
        args = self.parse()
        tester = FakeSequenceTester(Clock(), [])
        tester.sdo_read = lambda *_args, **_kw: 0
        with self.assertRaisesRegex(RuntimeError, "Invalid 2008h"):
            rev.check_configured_drive_speed(tester, args)

    def test_timed_five_second_hold_then_reverse(self):
        args = self.parse("--switch-after-s", "5.0")
        self.assertEqual(args.max_reverse_s, 2.0)
        clock = Clock()

        class TimedTester(FakeSequenceTester):
            def __init__(self, *v):
                super().__init__(*v)
                self.reverse_time = None
                self.braking_samples = iter([12.0, 7.0, 3.0, 0.0,
                                             -1.0, -3.1, -3.2])

            def command(self, amp):
                super().command(amp)
                if amp < 0 and self.reverse_time is None:
                    self.reverse_time = self.clock.now()

            def receive(self, timeout):
                self.clock.t += 0.005
                if self.reverse_time is None:
                    self.rpm_l = 12.0
                else:
                    self.rpm_l = next(self.braking_samples, -3.2)
                return (0x181, b"\0" * 8, self.clock.now())

        tester = TimedTester(clock, [])
        with patch.object(rev.time, "monotonic", clock.now):
            rev.run_reversal(tester, args)
        self.assertIsNotNone(tester.reverse_time)
        self.assertGreaterEqual(tester.reverse_time - 10.0, 5.0)
        self.assertLess(tester.reverse_time - 10.0, 5.02)
        self.assertIn("reverse_command", tester.events)
        self.assertIn("reverse_confirmed", tester.events)
        self.assertEqual(tester.commands[-1], 0.0)

    def test_timed_switch_requires_forward_rotation(self):
        args = self.parse("--switch-after-s", "0.05")
        clock = Clock()
        tester = FakeSequenceTester(clock, [1.0] * 100)
        with patch.object(rev.time, "monotonic", clock.now):
            with self.assertRaisesRegex(RuntimeError, "Refusing timed reversal"):
                rev.run_reversal(tester, args)
        self.assertNotIn(-1.0, tester.commands)

    def test_timed_switch_is_still_overspeed_guarded(self):
        args = self.parse("--switch-after-s", "5.0")
        clock = Clock()
        tester = FakeSequenceTester(clock, [0.0, 12.0, 45.1])
        with patch.object(rev.time, "monotonic", clock.now):
            with self.assertRaisesRegex(RuntimeError, "Overspeed"):
                rev.run_reversal(tester, args)
        self.assertNotIn(-1.0, tester.commands)

    def test_reject_invalid_time_hold(self):
        for value in ("0", "5.1", "nan", "inf"):
            args = self.parse("--switch-after-s", value)
            with self.assertRaises(ValueError):
                rev.preflight(args)



if __name__ == "__main__":
    unittest.main()
