#!/usr/bin/env python3
"""Analyze a signed-current reversal CSV without connecting to CAN.

Times are relative to the logged reverse_command event (PC monotonic clock).
"First current negative" and "first velocity decrease" are indicators,
NOT physical actuator response timestamps; TPDO timebase is asynchronous.
"""
import argparse
import csv
import math
from pathlib import Path
import sys


def val(r, key):
    try:
        x = float(r.get(key, ""))
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def inspect(rows, wheel="left"):
    marker = next((r for r in rows if r.get("event") == "reverse_command"), None)
    if marker is None:
        raise ValueError("No reverse_command marker; forward trigger not reached")
    t0 = val(marker, "elapsed_s")
    v0 = val(marker, f"{wheel}_rpm")
    if t0 is None or v0 is None:
        raise ValueError("Incomplete reverse_command event")
    vs = [(val(r, "elapsed_s"), val(r, f"{wheel}_rpm"))
          for r in rows if r.get("event") == "velocity_rx"]
    cs = [(val(r, "elapsed_s"), val(r, f"actual_{wheel}_A"))
          for r in rows if r.get("event") == "current_rx"]
    vs = [(t, v) for t, v in vs if t is not None and v is not None and t >= t0]
    cs = [(t, i) for t, i in cs if t is not None and i is not None and t >= t0]
    ends = {event: next((val(r, "elapsed_s") for r in rows
                         if r.get("event") == event and
                         val(r, "elapsed_s") is not None and
                         val(r, "elapsed_s") >= t0), None)
            for event in ("zero_speed_crossing", "reverse_confirmed",
                          "complete_zero")}
    # Threshold explicitly exceeds a few 0.1 rpm reporting quanta.
    drop_t = next((t for t, rpm in vs if rpm <= v0 - 0.5), None)
    current_neg_t = next((t for t, i in cs if i <= -0.1), None)
    speed_zero_t = next((t for t, rpm in vs if rpm <= 0), None)
    max_forward = max((rpm for t, rpm in vs if t <= (speed_zero_t or math.inf)),
                      default=None)
    return {
        "switch_elapsed_s": t0, "switch_rpm": v0,
        "first_current_negative_lag_s": None if current_neg_t is None else current_neg_t-t0,
        "first_speed_drop_0p5rpm_lag_s": None if drop_t is None else drop_t-t0,
        "first_zero_speed_lag_s": None if speed_zero_t is None else speed_zero_t-t0,
        "confirmed_reverse_lag_s": None if ends["reverse_confirmed"] is None
                                    else ends["reverse_confirmed"]-t0,
        "max_rpm_after_reverse": max_forward,
        "n_current_after": len(cs), "n_speed_after": len(vs),
        "confirmed": ends["reverse_confirmed"] is not None
    }


def inspect_before_reversal(rows, wheel="left"):
    """Describe a partial forward run without claiming a current reversal."""
    if any(r.get("event") == "reverse_command" for r in rows):
        raise ValueError("This function is for logs without reverse_command")
    start = next((r for r in rows if r.get("event") == "forward_start"), None)
    if start is None:
        raise ValueError("No forward_start or reverse_command event; invalid test log")
    t0 = val(start, "elapsed_s")
    if t0 is None:
        raise ValueError("forward_start event lacks elapsed_s")
    speeds = sorted(
        ((val(r, "elapsed_s"), val(r, f"{wheel}_rpm"))
         for r in rows if r.get("event") == "velocity_rx"),
        key=lambda pair: (pair[0] is None, pair[0] if pair[0] is not None else 0)
    )
    speeds = [(t, speed) for t, speed in speeds
              if t is not None and speed is not None and t >= t0]
    currents = [
        (val(r, "elapsed_s"), val(r, f"actual_{wheel}_A"))
        for r in rows if r.get("event") == "current_rx"
    ]
    currents = [(t, current) for t, current in currents
                if t is not None and current is not None and t >= t0]
    if not speeds:
        raise ValueError("No fresh speed feedback after forward_start")
    max_t, max_rpm = max(speeds, key=lambda item: item[1])
    thresholds = {
        rpm: next((t-t0 for t, speed in speeds if speed >= rpm), None)
        for rpm in (45, 100, 200, 300, 350, 370)
    }
    last_t, last_speed = speeds[-1]
    return {
        "duration_s": last_t-t0,
        "max_speed_rpm": max_rpm,
        "max_speed_time_s": max_t-t0,
        "last_speed_rpm": last_speed,
        "thresholds": thresholds,
        "current_min_A": min((i for _, i in currents), default=None),
        "current_max_A": max((i for _, i in currents), default=None),
        "current_mean_A": (
            sum(i for _, i in currents)/len(currents) if currents else None
        ),
        "n_current": len(currents),
        "n_speed": len(speeds),
    }


def fmt(value):
    return "NOT OBSERVED" if value is None else f"{value:+.4f}"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--wheel", choices=("left", "right"), default="left")
    args = p.parse_args(argv)
    try:
        with Path(args.csv_file).expanduser().open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not any(r.get("event") == "reverse_command" for r in rows):
            pre = inspect_before_reversal(rows, args.wheel)
            print(f"File: {args.csv_file}; wheel={args.wheel}")
            print("NO REVERSE COMMAND recorded: 5-second hold did not complete "
                  "(or the log is incomplete). Do not infer braking behavior.")
            print(f"Last fresh speed feedback t={pre['duration_s']:.4f}s "
                  f"speed={pre['last_speed_rpm']:+.1f} rpm")
            print(f"Maximum observed speed={pre['max_speed_rpm']:.1f} rpm "
                  f"at t={pre['max_speed_time_s']:.4f}s")
            for rpm, t in pre["thresholds"].items():
                print(f"First reported >= {rpm} rpm: "
                      + ("not observed" if t is None else f"t={t:.4f}s"))
            if pre['n_current']:
                print(f"6077h current min/mean/max = "
                      f"{pre['current_min_A']:+.3f}/"
                      f"{pre['current_mean_A']:+.3f}/"
                      f"{pre['current_max_A']:+.3f} A")
            else:
                print("No post-start fresh 6077h feedback")
            print(f"Fresh TPDO samples: speed={pre['n_speed']}, "
                  f"current={pre['n_current']}")
            print("Safety: do not disable overspeed protection or raise cutoff "
                  "to force 5-second hold.")
            return 0
        result = inspect(rows, args.wheel)
    except (OSError, ValueError) as exc:
        p.error(str(exc))
    print(f"File: {args.csv_file}; wheel={args.wheel}")
    print(f"Torque reversal requested at t={result['switch_elapsed_s']:.4f}s; "
          f"wheel speed={result['switch_rpm']:+.1f} rpm")
    for key, desc in (
        ("first_current_negative_lag_s", "First negative 6077h feedback lag [s]"),
        ("first_speed_drop_0p5rpm_lag_s", "First speed drop >=0.5 rpm lag [s]"),
        ("first_zero_speed_lag_s", "First reported zero crossing lag [s]"),
        ("confirmed_reverse_lag_s", "Two-sample reverse confirmation lag [s]"),
    ):
        print(f"{desc}: {fmt(result[key])}")
    print(f"Maximum reported forward speed after torque reversal: "
          f"{fmt(result['max_rpm_after_reverse'])} rpm")
    print(f"Fresh post-command TPDO rows: current={result['n_current_after']}, "
          f"speed={result['n_speed_after']}")
    print("Outcome: " + ("reverse confirmed" if result["confirmed"]
                        else "not confirmed (timeout, safety abort, or incomplete log)"))
    print("Caveat: reported CAN velocity and 6077h have their own sampling/filters.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
