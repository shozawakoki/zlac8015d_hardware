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


def fmt(value):
    return "NOT OBSERVED" if value is None else f"{value:+.4f}"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--wheel", choices=("left", "right"), default="left")
    args = p.parse_args(argv)
    try:
        with Path(args.csv_file).expanduser().open(newline="", encoding="utf-8") as f:
            result = inspect(list(csv.DictReader(f)), args.wheel)
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
