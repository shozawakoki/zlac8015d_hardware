#!/usr/bin/env python3
"""Current sign reversal on ZLAC8015D (speed-triggered OR timed), with guards.

Purpose: +I until a measured forward RPM threshold or a fixed time, then -I;
observe braking, zero-speed crossing, and confirmed negative RPM. Stop at -RPM.
PREVIEW ONLY by default. Live use requires explicit --execute and safety flags.

This is a TORQUE/CURRENT reversal, not an instantaneous reversal of wheel RPM.
This script reuses the tested current_step_test driver implementation (PDO
mapping, units, signs, zero-current baseline, SDO preflight, CAN safety).
The motor may still accelerate after negative current is requested. Physical
e-stop, a restrained raised robot, and a verified regenerative supply are
essential. Software stop does NOT guarantee electrical safety.
"""
import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import signal
import subprocess
import os
import sys
import time

import current_step_test as base


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--interface", default="can0")
    p.add_argument("--node-id", type=int, default=1)
    p.add_argument("--wheel", choices=("left", "right"), default="left")
    p.add_argument("--current-a", type=float, default=1.0,
                   help="magnitude of both +I and -I (default 1.0 A)")
    p.add_argument("--switch-rpm", type=float, default=15.0,
                   help="speed-trigger mode: reverse at measured forward RPM >= threshold")
    p.add_argument("--switch-after-s", type=float, default=None,
                   help="time-trigger mode: reverse current at this many seconds (max 5.0), "
                        "only if still rotating forward; overrides --switch-rpm")
    p.add_argument("--min-switch-rpm", type=float, default=5.0,
                   help="time-trigger mode: refuse reversal if selected wheel is slower than this")
    p.add_argument("--reverse-confirm-rpm", type=float, default=3.0,
                   help="stop after two fresh feedback frames <= -this value")
    p.add_argument("--max-speed-rpm", type=float, default=45.0,
                   help="abort on absolute RPM of either wheel; default 45. >60 requires "
                        "--verified-safe-rpm")
    p.add_argument("--verified-safe-rpm", type=float, default=None,
                   help="documented maximum safe speed of the ACTUAL motor/drive/fixture; "
                        "required for any cutoff above 60 rpm")
    p.add_argument("--max-forward-s", type=float, default=0.25,
                   help="abort if no forward RPM trigger within this period")
    p.add_argument("--max-reverse-s", type=float, default=None,
                   help="abort if no reverse confirmation after sign change "
                        "(default 0.4s speed mode, 2.0s timed mode)")
    p.add_argument("--loop-hz", type=float, default=100.0)
    p.add_argument("--feedback-timeout-ms", type=float, default=100.0)
    p.add_argument("--settle-seconds", type=float, default=1.0)
    p.add_argument("--settle-timeout-seconds", type=float, default=5.0)
    p.add_argument("--settle-max-rpm", type=float, default=0.5)
    p.add_argument("--settle-max-current-a", type=float, default=0.1)
    p.add_argument("--output", default=None)
    p.add_argument("--execute", action="store_true", help="enable real motor commands")
    p.add_argument("--confirm-rated-current", action="store_true")
    p.add_argument("--confirm-wheels-raised", action="store_true")
    p.add_argument("--confirm-e-stop", action="store_true")
    p.add_argument("--confirm-exclusive-can", action="store_true")
    p.add_argument("--confirm-regeneration-safe", action="store_true",
                   help="I verified the drive/supply supports braking regeneration")
    args = p.parse_args(argv)
    if args.max_reverse_s is None:
        args.max_reverse_s = 2.0 if args.switch_after_s is not None else 0.4
    return args


def preflight(args):
    values = (args.current_a, args.switch_rpm, args.reverse_confirm_rpm,
              args.min_switch_rpm, args.switch_after_s if args.switch_after_s is not None else 0.0,
              args.verified_safe_rpm if args.verified_safe_rpm is not None else 0.0,
              args.max_speed_rpm, args.max_forward_s, args.max_reverse_s,
              args.loop_hz, args.feedback_timeout_ms, args.settle_seconds,
              args.settle_timeout_seconds, args.settle_max_rpm,
              args.settle_max_current_a)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("all numeric limits must be finite")
    if (not 1 <= args.node_id <= 127 or not args.interface or
            len(args.interface) > 15):
        raise ValueError("invalid CAN device/interface")
    if not (0.0 < args.current_a <= 1.0):
        raise ValueError("current magnitude must be within (0, 1.0] A")
    if not (5.0 <= args.switch_rpm <= 25.0):
        raise ValueError("switch-rpm must be 5..25 rpm")
    if args.switch_after_s is not None and not (0.05 <= args.switch_after_s <= 5.0):
        raise ValueError("switch-after-s must be within 0.05..5.0 seconds")
    if not (0.5 <= args.min_switch_rpm <= 20.0):
        raise ValueError("min-switch-rpm must be within 0.5..20.0 rpm")
    if not (1.0 <= args.reverse_confirm_rpm <= 5.0):
        raise ValueError("reverse-confirm-rpm must be 1..5 rpm")
    if not (args.switch_rpm + 10.0 <= args.max_speed_rpm <= 300.0):
        raise ValueError("speed cutoff must exceed switch-rpm by >=10 rpm and be <=300 rpm")
    if args.verified_safe_rpm is not None and not (20.0 <= args.verified_safe_rpm <= 3000.0):
        raise ValueError("verified-safe-rpm must be a verified 20..3000 rpm rating")
    if args.max_speed_rpm > 60.0 and (
            args.verified_safe_rpm is None or
            args.max_speed_rpm > args.verified_safe_rpm):
        raise ValueError(
            "cutoffs above 60 rpm require --verified-safe-rpm >= --max-speed-rpm; "
            "confirm documented motor/drive/fixture maximum before running")
    reverse_timeout_cap = 2.0 if args.switch_after_s is not None else 0.8
    if not (0.05 <= args.max_forward_s <= 0.5 and
            0.1 <= args.max_reverse_s <= reverse_timeout_cap):
        raise ValueError(
            f"forward timeout must be 0.05..0.5 s; "
            f"reverse timeout must be 0.1..{reverse_timeout_cap:.1f} s")
    if not (50.0 <= args.loop_hz <= 200.0 and
            30.0 <= args.feedback_timeout_ms <= 150.0):
        raise ValueError("invalid feedback or refresh rates")
    if (not 0.1 <= args.settle_seconds <= 5.0 or
            not args.settle_seconds < args.settle_timeout_seconds <= 30.0 or
            not 0.0 < args.settle_max_rpm <= 2.0 or
            not 0.0 <= args.settle_max_current_a <= 0.3):
        raise ValueError("invalid zero-current settling limits")
    print("TORQUE/CURRENT reversal (NOT speed command):")
    if args.switch_after_s is None:
        print(f"  wheel={args.wheel}, +{args.current_a:.2f} A until +{args.switch_rpm:.1f} rpm")
    else:
        print(f"  wheel={args.wheel}, +{args.current_a:.2f} A for {args.switch_after_s:.2f}s")
        print(f"  at switch time, requires wheel speed >=+{args.min_switch_rpm:.1f} rpm")
    print(f"  then -{args.current_a:.2f} A until two consecutive fresh readings "
          f"at <=-{args.reverse_confirm_rpm:.1f} rpm")
    print(f"  upper cutoff ±{args.max_speed_rpm:.1f} rpm on EITHER wheel, "
          + (f"forward timeout={args.max_forward_s:.2f}s, " if args.switch_after_s is None else
             f"timed hold={args.switch_after_s:.2f}s, ")
          + f"reverse timeout={args.max_reverse_s:.2f}s")
    print("  Current/velocity TPDO at driver sign-converted ROS-positive coordinates.")
    if args.verified_safe_rpm is not None:
        print(f"  operator supplied verified maximum safe speed: {args.verified_safe_rpm:.1f} rpm")
    print("  WARNING: may overspeed or abort before reverse; confirm physical safety.")
    if not args.execute:
        print("PREVIEW ONLY: no CAN socket opened; no current commands.")
        return False
    for key in ("confirm_rated_current", "confirm_wheels_raised", "confirm_e_stop",
                "confirm_exclusive_can", "confirm_regeneration_safe"):
        if not getattr(args, key):
            raise ValueError(f"live test requires --{key.replace('_','-')}")
    # Advisory process check; not a CAN bus ownership lock.
    ps = subprocess.run(["ps", "-eo", "pid=,args="],
                        capture_output=True, text=True, check=True)
    for line in ps.stdout.splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) < 2 or not fields[0].isdigit() or int(fields[0]) == os.getpid():
            continue
        cmd = fields[1]
        if ("ros2_control_node" in cmd or "can_launch_watchdog.py" in cmd or
                ("ros2 launch" in cmd and "zlac_can_bringup" in cmd)):
            raise RuntimeError(f"Concurrent CAN controller: {line.strip()}")
    return True


def selected_rpm(tester, wheel):
    return tester.rpm_l if wheel == "left" else tester.rpm_r


def switch_to_braking(tester, args, start, rpm, when):
    tester.stage = 1
    tester.step_started = when
    tester.step_current = -args.current_a
    tester.command(-args.current_a)
    tester.log("reverse_command", when=when)
    print(f"t={when-start:.4f}s: +{rpm:.1f} rpm; commanded "
          f"-{args.current_a:.2f} A", flush=True)


def run_reversal(tester, args):
    tester.wait_feedback()
    tester.wait_for_stable_zero()
    tester.read_preflight_snapshot()
    start = time.monotonic()
    tester.t0 = start
    tester.stage = 0
    tester.step_started = start
    tester.step_current = args.current_a
    tester.command(args.current_a)
    tester.log("forward_start", when=start)
    print(f"t=0.000 s +{args.current_a:.2f} A forward command started", flush=True)

    phase = "forward"
    reverse_at = None
    reverse_count = 0
    zero_logged = False
    period = 1.0 / args.loop_hz
    next_tx = start + period

    while True:
        now = time.monotonic()
        tester.check_safety(now)
        if phase == "forward":
            if args.switch_after_s is None:
                if now - start > args.max_forward_s:
                    raise RuntimeError("No forward RPM trigger before timeout")
            elif now - start >= args.switch_after_s:
                rpm = selected_rpm(tester, args.wheel)
                other_rpm = tester.rpm_r if args.wheel == "left" else tester.rpm_l
                if abs(other_rpm) > 5.0:
                    raise RuntimeError("Uncommanded other wheel rotation at timed switch")
                if rpm < args.min_switch_rpm:
                    raise RuntimeError(
                        f"Refusing timed reversal: wheel speed={rpm:+.1f} rpm "
                        f"is below +{args.min_switch_rpm:.1f} rpm")
                phase = "braking"
                reverse_at = time.monotonic()
                switch_to_braking(tester, args, start, rpm, reverse_at)
        if phase == "braking" and now - reverse_at > args.max_reverse_s:
            raise RuntimeError("Reverse not confirmed before timeout")
        if now >= next_tx:
            tester.command(tester.step_current)
            next_tx = now + period
        item = tester.receive(min(0.005, max(0.0, next_tx-time.monotonic())))
        if item is None:
            continue
        tester.process_feedback(item)
        tester.check_safety(time.monotonic())

        can_id, _data, when = item
        if can_id != 0x180 + tester.node:
            continue  # only fresh velocity TPDO can cause phase transition
        rpm = selected_rpm(tester, args.wheel)
        other_rpm = tester.rpm_r if args.wheel == "left" else tester.rpm_l
        if abs(other_rpm) > 5.0:
            raise RuntimeError(
                f"Uncommanded other wheel rotation: {other_rpm:+.1f} rpm")
        if phase == "forward":
            if rpm < -args.settle_max_rpm:
                raise RuntimeError("Wheel rotating opposite to requested forward torque")
            if args.switch_after_s is None and rpm >= args.switch_rpm:
                phase = "braking"
                reverse_at = time.monotonic()
                switch_to_braking(tester, args, start, rpm, reverse_at)
        else:
            if rpm <= 0.0 and not zero_logged:
                zero_logged = True
                tester.log("zero_speed_crossing", when=when)
                print(f"t={when-start:.4f}s: zero speed crossing reported", flush=True)
            if rpm <= -args.reverse_confirm_rpm:
                reverse_count += 1
            else:
                reverse_count = 0
            if reverse_count >= 2:
                tester.log("reverse_confirmed", when=when)
                tester.step_current = 0.0
                tester.command(0.0)
                tester.log("complete_zero")
                print(f"REVERSE CONFIRMED: {rpm:+.1f} rpm; zero current requested.",
                      flush=True)
                return


def main(argv=None):
    base.STOP = False
    args = parse_args(argv)
    try:
        if not preflight(args):
            return 0
    except (ValueError, RuntimeError) as exc:
        print(f"PRE-FLIGHT FAILED: {exc}", file=sys.stderr)
        return 2

    # Compatibility with existing current_step_test.ZlacCurrentTester fields.
    args.max_current_a = args.current_a
    args.step_seconds = 1.0
    args.probe_target = False
    signal.signal(signal.SIGINT, base.interrupted)
    signal.signal(signal.SIGTERM, base.interrupted)
    out_path = Path(args.output).expanduser() if args.output else (
        Path.home() / "ros2_ws" / "current_step_results" /
        f"current_reversal_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{args.wheel}.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tester = None
    try:
        with out_path.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=base.CSV_FIELDS)
            writer.writeheader()
            tester = base.ZlacCurrentTester(args, writer)
            print(f"CSV: {out_path}", flush=True)
            tester.open()
            tester.setup()
            run_reversal(tester, args)
            stream.flush()
        print("TEST COMPLETE: confirmed negative wheel rotation.", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError) as exc:
        print(f"TEST ABORTED: {exc}", file=sys.stderr)
        print(f"Partial CSV retained: {out_path}", file=sys.stderr)
        return 1
    finally:
        if tester is not None:
            tester.stop_motors()
            if tester.sock is not None:
                tester.sock.close()


if __name__ == "__main__":
    sys.exit(main())
