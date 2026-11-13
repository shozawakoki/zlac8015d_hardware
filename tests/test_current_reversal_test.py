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
        with self.assertRaisesRegex(ValueError, "speed cutoff"):
            rev.preflight(args)

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
