#!/usr/bin/env python3
"""ZLAC8015D open-loop current step experiment (NO ROS controller).

Sequence: +2, +4, -4, -2, +6, -2, +4 A, each for 3 seconds.
Runs ONLY by explicit --execute with safety acknowledgements; preview is default.
Outputs time-stamped commanded current, received actual current and wheel RPM.
CANopen mapping is copied from this repository's hardware plugin.
"""

import argparse
import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import time

PROFILE_A = (2.0, 4.0, -4.0, -2.0, 6.0, -2.0, 4.0)
FRAME = struct.Struct("=IB3x8s")  # Linux struct can_frame (16 bytes)
RAD_S_PER_01RPM = 2.0 * math.pi / 600.0
CSV_FIELDS = (
    "elapsed_s", "wall_time_iso", "event", "step_index", "step_time_s",
    "requested_step_A", "command_left_A", "command_right_A",
    "driver_left_mA", "driver_right_mA",
    "actual_left_A", "actual_right_A", "left_rpm", "right_rpm",
    "left_rad_s", "right_rad_s", "velocity_rx_age_ms",
    "current_rx_age_ms", "feedback_can_id", "can_rx_time_s",
)
STOP = False


def interrupted(_signum, _frame):
    global STOP
    STOP = True


def parser_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--interface", default="can0")
    p.add_argument("--node-id", type=int, default=1)
    p.add_argument("--wheel", choices=("left", "right", "both"), default="left",
                   help="one wheel by default; the other remains at 0 A")
    p.add_argument("--profile", default=",".join(str(v) for v in PROFILE_A),
                   help="comma-separated current profile (default: 2,4,-4,-2,6,-2,4)")
    p.add_argument("--step-seconds", type=float, default=3.0)
    p.add_argument("--loop-hz", type=float, default=100.0,
                   help="RPDO command refresh rate (not the ROS controller)")
    p.add_argument("--max-current-a", type=float, default=2.0,
                   help="test hard current limit; default 2 A, refuses the 6 A profile")
    p.add_argument("--max-speed-rpm", type=float, default=120.0,
                   help="abort the test if either wheel exceeds this speed")
    p.add_argument("--feedback-timeout-ms", type=float, default=100.0)
    p.add_argument("--output", default=None, help="CSV output path (default: ~/ros2_ws/current_step_results/...)")
    p.add_argument("--execute", action="store_true", help="send live motor commands; default is preview")
    p.add_argument("--confirm-rated-current", action="store_true",
                   help="I verified that motor and driver permit the requested signed current profile")
    p.add_argument("--confirm-wheels-raised", action="store_true",
                   help="I secured the robot with driven wheels lifted off the ground")
    p.add_argument("--confirm-e-stop", action="store_true",
                   help="An independent physical emergency stop is present and within reach")
    p.add_argument("--confirm-exclusive-can", action="store_true",
                   help="No ROS launch, CAN watchdog or other software controls this CANopen node")
    return p.parse_args(argv)


def parse_profile(raw):
    try:
        values = tuple(float(part.strip()) for part in raw.split(","))
    except ValueError as exc:
        raise ValueError("profile must be comma-separated finite numbers") from exc
    if not values or len(values) > 100 or any(not math.isfinite(x) for x in values):
        raise ValueError("profile must contain 1..100 finite currents")
    return values


def preflight(args, currents):
    if not (1 <= args.node_id <= 127):
        raise ValueError("CANopen node-id must be 1..127")
    if not args.interface or len(args.interface) > 15:
        raise ValueError("invalid CAN interface name")
    if not (0 < args.step_seconds <= 10 and 20 <= args.loop_hz <= 200):
        raise ValueError("step-seconds must be (0,10], loop-hz 20..200")
    if not (0 < args.max_current_a <= 30 and 0 < args.max_speed_rpm <= 3000):
        raise ValueError("max-current-a / max-speed-rpm out of permitted parameter range")
    if not (30 <= args.feedback_timeout_ms <= 500):
        raise ValueError("feedback-timeout-ms must be 30..500")
    max_requested = max(abs(v) for v in currents)
    if max_requested > 30.0:
        raise ValueError("requested current cannot exceed signed 16-bit mA envelope")
    print("Profile (open-loop current, ROS-positive; no speed feedback):")
    for i, amp in enumerate(currents):
        print(f"  {i*args.step_seconds:5.1f}–{(i+1)*args.step_seconds:5.1f} s : {amp:+.2f} A, wheel={args.wheel}")
    print(f"  then zero current; cutoff = {args.max_speed_rpm:.1f} rpm, "
          f"feedback timeout = {args.feedback_timeout_ms:.0f} ms")
    print("CAUTION: constant current can accelerate an unloaded wheel; the full profile may trip early.")
    if not args.execute:
        print("PREVIEW ONLY: CAN not opened, no motor command sent.")
        return False
    if max_requested > args.max_current_a:
        raise ValueError(f"Profile needs {max_requested:g} A but max-current-a={args.max_current_a:g} A. "
                         "No current has been sent. Verify motor/driver ratings before increasing this limit.")
    required = (
        "confirm_rated_current", "confirm_wheels_raised",
        "confirm_e_stop", "confirm_exclusive_can",
    )
    missing = [v.replace("_", "-") for v in required if not getattr(args, v)]
    if missing:
        raise ValueError("Real-hardware execution requires --" + ", --".join(missing))
    # This check is advisory; raw CAN has no exclusive-ownership mechanism.
    ps = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True, check=True)
    conflicting = []
    for row in ps.stdout.splitlines():
        fields = row.strip().split(maxsplit=1)
        if len(fields) < 2 or not fields[0].isdigit() or int(fields[0]) == os.getpid():
            continue
        command = fields[1]
        if ("ros2_control_node" in command or "can_launch_watchdog.py" in command or
                ("ros2 launch" in command and "zlac_can_bringup" in command)):
            conflicting.append(row.strip())
    if conflicting:
        raise RuntimeError("Potential concurrent CAN control process(es):\n" +
                           "\n".join(conflicting) + "\nStop them before the test.")
    return True


class ZlacCurrentTester:
    def __init__(self, args, writer):
        self.args = args
        self.writer = writer
        self.sock = None
        self.t0 = None
        self.last_vel_time = None
        self.last_cur_time = None
        self.measured_l = float("nan")
        self.measured_r = float("nan")
        self.rpm_l = float("nan")
        self.rpm_r = float("nan")
        self.stage = -1
        self.step_current = 0.0
        self.command_l = 0.0
        self.command_r = 0.0
        self.raw_l = 0
        self.raw_r = 0
        self.velocity_rx = 0
        self.current_rx = 0

    @property
    def node(self):
        return self.args.node_id

    def send(self, can_id, payload):
        if not self.sock:
            raise RuntimeError("CAN socket unavailable")
        if len(payload) > 8:
            raise ValueError("CAN payload exceeds 8 bytes")
        data = FRAME.pack(can_id, len(payload), bytes(payload).ljust(8, b"\x00"))
        written = self.sock.send(data)
        if written != FRAME.size:
            raise RuntimeError("short CAN send")

    def open(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.args.interface,))
        self.sock.setblocking(False)

    def receive(self, timeout):
        if not self.sock:
            raise RuntimeError("CAN socket unavailable")
        ready, _, _ = select.select([self.sock], [], [], max(0.0, timeout))
        if not ready:
            return None
        payload = self.sock.recv(FRAME.size)
        if len(payload) != FRAME.size:
            return None
        can_id, dlc, data = FRAME.unpack(payload)
        if can_id & 0xE0000000:  # error/RTR/extended frames ignored
            return None
        return (can_id & 0x7FF, data[:dlc], time.monotonic())

    def process_feedback(self, received, log=True):
        if received is None:
            return
        can_id, data, when = received
        event = None
        if can_id == 0x180 + self.node and len(data) >= 8:
            raw_l, raw_r = struct.unpack_from("<ii", data)
            self.rpm_l = -raw_l * 0.1  # ROS-positive, opposite driver sign on left
            self.rpm_r = raw_r * 0.1
            self.last_vel_time = when
            self.velocity_rx += 1
            event = "velocity_rx"
        elif can_id == 0x280 + self.node and len(data) >= 4:
            raw_l, raw_r = struct.unpack_from("<hh", data)
            self.measured_l = -raw_l * 0.1
            self.measured_r = raw_r * 0.1
            self.last_cur_time = when
            self.current_rx += 1
            event = "current_rx"
        if log and event:
            self.log(event, when, can_id)

    def log(self, event, when=None, can_id=None):
        when = time.monotonic() if when is None else when
        v_age = None if self.last_vel_time is None else max(0.0, (when-self.last_vel_time)*1000.0)
        i_age = None if self.last_cur_time is None else max(0.0, (when-self.last_cur_time)*1000.0)
        elapsed = None if self.t0 is None else when - self.t0
        self.writer.writerow({
            "elapsed_s": "" if elapsed is None else round(elapsed, 6),
            "wall_time_iso": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "event": event,
            "step_index": self.stage,
            "step_time_s": "" if elapsed is None or self.stage < 0
                           else round(elapsed-self.stage*self.args.step_seconds, 6),
            "requested_step_A": self.step_current,
            "command_left_A": self.command_l,
            "command_right_A": self.command_r,
            "driver_left_mA": self.raw_l,
            "driver_right_mA": self.raw_r,
            "actual_left_A": self.measured_l,
            "actual_right_A": self.measured_r,
            "left_rpm": self.rpm_l,
            "right_rpm": self.rpm_r,
            "left_rad_s": self.rpm_l * (2.0*math.pi/60.0),
            "right_rad_s": self.rpm_r * (2.0*math.pi/60.0),
            "velocity_rx_age_ms": "" if v_age is None else round(v_age, 3),
            "current_rx_age_ms": "" if i_age is None else round(i_age, 3),
            "feedback_can_id": "" if can_id is None else hex(can_id),
            "can_rx_time_s": round(when, 6),
        })

    def sdo(self, idx, sub, value, bits):
        command = {8: 0x2F, 16: 0x2B, 32: 0x23}[bits]
        packet = bytes([command, idx & 255, idx >> 8, sub]) + int(value).to_bytes(4, "little")
        self.send(0x600 + self.node, packet)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if STOP:
                raise InterruptedError("User interruption")
            item = self.receive(deadline-time.monotonic())
            if item is None:
                continue
            can_id, data, _when = item
            if can_id == 0x580 + self.node and len(data) >= 8 and data[1:4] == packet[1:4]:
                if data[0] == 0x60:
                    return
                if data[0] == 0x80:
                    abort = int.from_bytes(data[4:8], "little")
                    raise RuntimeError(f"SDO abort {idx:04X}:{sub:02X} code=0x{abort:08X}")
            self.process_feedback(item, log=False)
        raise TimeoutError(f"No SDO ACK for {idx:04X}:{sub:02X}")

    def configure_feedback(self, comm, mapping, cob, objects, event_ms):
        self.sdo(comm, 1, 0x80000000 | cob, 32)
        self.sdo(mapping, 0, 0, 8)
        for i, mapping_entry in enumerate(objects, start=1):
            self.sdo(mapping, i, mapping_entry, 32)
        self.sdo(comm, 2, 0xFF, 8)
        self.sdo(comm, 3, 0, 16)
        self.sdo(comm, 5, event_ms, 16)
        self.sdo(mapping, 0, len(objects), 8)
        self.sdo(comm, 1, cob, 32)

    def setup(self):
        self.send(0x000, bytes([0x80, self.node]))  # NMT pre-operational
        time.sleep(0.02)
        self.configure_feedback(0x1800, 0x1A00, 0x180+self.node,
                                (0x606C0120, 0x606C0220), 10)
        self.configure_feedback(0x1801, 0x1A01, 0x280+self.node,
                                (0x60770110, 0x60770210), 10)
        # RPDO0: 6071h:01 and :02, I16 mA, two motors.
        self.sdo(0x1400, 1, 0x80000000 | (0x200+self.node), 32)
        self.sdo(0x1600, 0, 0, 8)
        self.sdo(0x1600, 1, 0x60710110, 32)
        self.sdo(0x1600, 2, 0x60710210, 32)
        self.sdo(0x1400, 2, 0xFF, 8)
        self.sdo(0x1600, 0, 2, 8)
        self.sdo(0x1400, 1, 0x200+self.node, 32)
        self.sdo(0x200F, 0, 0, 16)      # asynchronous command update
        self.send(0x000, bytes([0x01, self.node]))  # NMT operational
        time.sleep(0.02)
        self.sdo(0x6060, 0, 4, 8)       # profile torque
        self.sdo(0x605A, 0, 6, 16)      # emergency quick stop configuration
        self.sdo(0x6085, 1, 10, 32)
        self.sdo(0x6085, 2, 10, 32)
        self.command(0.0)
        for cw in (6, 7, 15):          # CiA402 servo enable
            self.sdo(0x6040, 0, cw, 16)
            time.sleep(0.01)
        self.command(0.0)

    def command(self, amp):
        if abs(amp) > self.args.max_current_a + 1e-9:
            raise ValueError("attempt to exceed configured current hard limit")
        self.command_l = amp if self.args.wheel in ("left", "both") else 0.0
        self.command_r = amp if self.args.wheel in ("right", "both") else 0.0
        self.raw_l = int(round(-self.command_l * 1000.0))
        self.raw_r = int(round(self.command_r * 1000.0))
        self.send(0x200+self.node, struct.pack("<hh", self.raw_l, self.raw_r))

    def stop_motors(self):
        # Do not rely on software stop as a physical emergency stop.
        if self.sock is None:
            return
        try:
            for _ in range(12):
                self.command_l = self.command_r = 0.0
                self.raw_l = self.raw_r = 0
                self.send(0x200+self.node, b"\x00\x00\x00\x00")
                time.sleep(0.02)
            self.sdo(0x6040, 0, 6, 16)  # switch off operation (servo disabled)
        except (OSError, RuntimeError, TimeoutError) as exc:
            print(f"WARNING: software zero/disable not confirmed: {exc}", file=sys.stderr)

    def wait_feedback(self):
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            if STOP:
                raise InterruptedError("Interrupted during feedback preflight")
            self.command(0.0)
            item = self.receive(0.01)
            if item:
                self.process_feedback(item, log=False)
            if self.last_vel_time is not None and self.last_cur_time is not None:
                print(f"Initial feedback L/R: speed={self.rpm_l:.1f}/{self.rpm_r:.1f} rpm, "
                      f"current={self.measured_l:.1f}/{self.measured_r:.1f} A")
                if abs(self.rpm_l) > self.args.max_speed_rpm or abs(self.rpm_r) > self.args.max_speed_rpm:
                    raise RuntimeError("Wheel already exceeds speed safety threshold")
                return
        raise TimeoutError("No initial TPDO0 (speed) / TPDO1 (current) feedback")

    def check_safety(self, when):
        if STOP:
            raise InterruptedError("User interruption")
        timeout = self.args.feedback_timeout_ms / 1000.0
        if (self.last_vel_time is None or self.last_cur_time is None or
            when - self.last_vel_time > timeout or when - self.last_cur_time > timeout):
            raise RuntimeError("Speed or current CAN feedback timeout; aborting")
        if abs(self.rpm_l) > self.args.max_speed_rpm or abs(self.rpm_r) > self.args.max_speed_rpm:
            raise RuntimeError(f"Overspeed: left={self.rpm_l:.1f}, right={self.rpm_r:.1f} rpm")
        if (abs(self.measured_l) > self.args.max_current_a + 1.0 or
                abs(self.measured_r) > self.args.max_current_a + 1.0):
            raise RuntimeError("Measured motor current exceeds software limit by >1 A")

    def run(self, profile):
        self.wait_feedback()
        self.t0 = time.monotonic()
        self.log("start", self.t0)
        period = 1.0 / self.args.loop_hz
        deadline = self.t0
        total = len(profile) * self.args.step_seconds
        while True:
            now = time.monotonic()
            elapsed = now - self.t0
            if elapsed >= total:
                break
            self.check_safety(now)
            idx = min(int(elapsed / self.args.step_seconds), len(profile)-1)
            if idx != self.stage:
                self.stage = idx
                self.step_current = profile[idx]
                print(f"t={elapsed:.3f}s stage={idx+1}/{len(profile)} "
                      f"command {self.step_current:+.2f} A [{self.args.wheel}]", flush=True)
                self.command(self.step_current)
                self.log("step_change")
            else:
                self.command(self.step_current)
            # Collect feedback until the next 100 Hz RPDO refresh; log each
            # CAN TPDO as its own event, preserving data freshness/timestamps.
            deadline += period
            while time.monotonic() < deadline:
                if STOP:
                    raise InterruptedError("User interruption")
                item = self.receive(min(0.005, max(0, deadline-time.monotonic())))
                if item:
                    self.process_feedback(item)
                    self.check_safety(time.monotonic())
            if deadline < time.monotonic() - 2*period:
                deadline = time.monotonic()  # missed a period, avoid command burst
        self.step_current = 0.0
        self.command(0.0)
        self.log("complete_zero")
        print("Profile completed; zero command sent. Record coasting for 1 s.", flush=True)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            self.command(0.0)
            item = self.receive(0.01)
            if item:
                self.process_feedback(item)


def main(argv=None):
    global STOP
    STOP = False
    args = parser_args(argv)
    try:
        profile = parse_profile(args.profile)
        if not preflight(args, profile):
            return 0
    except (ValueError, RuntimeError) as exc:
        print(f"PRE-FLIGHT FAILED: {exc}", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    out_path = Path(args.output).expanduser() if args.output else (
        Path.home() / "ros2_ws" / "current_step_results" /
        f"current_steps_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{args.wheel}.csv"
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tester = None
    try:
        with out_path.open("x", newline="", encoding="utf-8") as f:
            csv_writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            csv_writer.writeheader()
            tester = ZlacCurrentTester(args, csv_writer)
            print(f"CSV: {out_path}", flush=True)
            tester.open()
            tester.setup()
            tester.run(profile)
            f.flush()
        print("COMPLETE: all stages executed without software safety trip.")
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
