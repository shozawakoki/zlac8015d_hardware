#!/usr/bin/env python3
"""Read-only cross-trial comparison of legacy ZLAC8015D current-step CSV logs.

Never opens CAN or controls a motor. Uses only event=current_rx and
event=velocity_rx for measurements. Legacy logs do not have the preflight
stability snapshot; consequently same initial drive/mechanical condition
cannot be assumed. A step is not a steady-state current-control test.
"""
import argparse
import csv
import math
from pathlib import Path
import statistics
import sys


def number(row, key):
    try:
        v = float(row.get(key, ""))
    except (ValueError, TypeError):
        return None
    return v if math.isfinite(v) else None


def mean_or_none(values):
    return statistics.mean(values) if values else None


def slope(points):
    if len(points) < 3:
        return None
    xm = statistics.mean(t for t, _ in points)
    ym = statistics.mean(y for _, y in points)
    den = sum((t - xm) ** 2 for t, _ in points)
    return (sum((t - xm) * (y - ym) for t, y in points) / den
            if den > 1e-12 else None)


def first_crossing(points, threshold):
    for t, rpm in points:
        if abs(rpm) >= threshold:
            return t
    return None


def extract(path, *, window=0.14, wheel="left", bin_s=0.02):
    path = Path(path).expanduser()
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        needed = {
            "event", "elapsed_s", "step_index", f"actual_{wheel}_A",
            f"{wheel}_rpm", f"driver_{wheel}_mA", f"command_{wheel}_A",
        }
        missing = needed - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    steps = [r for r in rows if r["event"] == "step_change" and
             r.get("step_index") == "0" and number(r, "elapsed_s") is not None]
    if not steps:
        raise ValueError(f"{path}: no first step_change")
    t0 = number(steps[0], "elapsed_s")
    # Use only samples before the first zero-command completion or next step.
    boundaries = [
        number(r, "elapsed_s") for r in rows
        if r["event"] in ("complete_zero", "step_change") and
        number(r, "elapsed_s") is not None and
        number(r, "elapsed_s") > t0 + 1e-7
    ]
    stop = min(boundaries, default=math.inf)
    currents, speeds, sent = [], [], set()
    for r in rows:
        t = number(r, "elapsed_s")
        if t is None or not (t0 <= t < t0 + window) or t >= stop:
            continue
        if r["event"] not in ("current_rx", "velocity_rx"):
            continue
        packed = number(r, f"driver_{wheel}_mA")
        if packed is not None:
            sent.add(int(round(packed)))
        if r["event"] == "current_rx":
            c = number(r, f"actual_{wheel}_A")
            if c is not None:
                currents.append((t - t0, c))
        else:
            rpm = number(r, f"{wheel}_rpm")
            if rpm is not None:
                speeds.append((t - t0, rpm))
    if not currents or not speeds:
        raise ValueError(f"{path}: no fresh current/speed samples in requested window")
    if max(currents[-1][0], speeds[-1][0]) < window * 0.7:
        raise ValueError(f"{path}: recording too short for {window:g}s comparison")
    nums = [c for _, c in currents]
    points = [(t, rpm * 2 * math.pi / 60.0) for t, rpm in speeds]
    bins = []
    for k in range(math.ceil(window / bin_s)):
        lo = k * bin_s
        hi = min(window, (k + 1) * bin_s)
        cs = [c for t, c in currents if lo <= t < hi]
        ss = [rpm for t, rpm in speeds if lo <= t < hi]
        bins.append((lo, hi, mean_or_none(cs), mean_or_none(ss), len(cs), len(ss)))
    preflight = any(r["event"] == "preflight_verified" for r in rows)
    start_row = next((r for r in rows if r["event"] == "start"), None)
    baseline = None if start_row is None else number(start_row, f"{wheel}_rpm")
    intervals = [b[0] - a[0] for a, b in zip(currents, currents[1:])]
    return {
        "filename": path.name, "window_s": window,
        "cmd_A": number(steps[0], f"command_{wheel}_A"),
        "sent_mA": tuple(sorted(sent)),
        "mean_A": statistics.mean(nums),
        "std_A": statistics.pstdev(nums),
        "min_A": min(nums), "max_A": max(nums),
        "n_current": len(currents), "n_speed": len(speeds),
        "max_gap_current_ms": max(intervals) * 1000 if intervals else None,
        "max_rpm": max(abs(rpm) for _, rpm in speeds),
        "accel_rad_s2": slope(points),
        "reach10_s": first_crossing(speeds, 10.0),
        "reach30_s": first_crossing(speeds, 30.0),
        "start_held_rpm": baseline,
        "preflight": preflight,
        "bins": bins,
    }


def fmt(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


def print_report(reports):
    print("Read-only comparison; only independent event=current_rx / velocity_rx.")
    print("All means/accel fit are for the SAME post-step window; not steady state.")
    print("CSV driver_*_mA is a software-recorded packed command, not a CAN acknowledgment.")
    for r in reports:
        print(f"\n=== {r['filename']} ===")
        print(f"Cmd={fmt(r['cmd_A'])} A, packed command values={list(r['sent_mA'])} mA")
        print(f"Window=0..{r['window_s']:.3f}s, "
              f"Ncurrent={r['n_current']}, Nspeed={r['n_speed']}, "
              f"max current RX gap={fmt(r['max_gap_current_ms'], 1)}ms")
        print(f"I6077 mean={fmt(r['mean_A'])} A, "
              f"std={fmt(r['std_A'])} A, "
              f"range={fmt(r['min_A'])}..{fmt(r['max_A'])} A")
        print(f"Wheel max |rpm|={fmt(r['max_rpm'], 1)}, "
              f"acceleration OLS={fmt(r['accel_rad_s2'])} rad/s²")
        print(f"Cross 10rpm at t={fmt(r['reach10_s'])}s; "
              f"cross 30rpm at t={fmt(r['reach30_s'])}s")
        print(f"Start row held rpm={fmt(r['start_held_rpm'])}; "
              f"documented stable preflight={'YES' if r['preflight'] else 'NO (legacy CSV)'}")
        print("  time(s)      mean 6077h(A)  mean rpm   Ncurrent/Nspeed")
        for lo, hi, i, rpm, ni, ns in r["bins"]:
            print(f"  {lo:.3f}-{hi:.3f}      "
                  f"{fmt(i):>8}     {fmt(rpm, 1):>8}     {ni}/{ns}")
    print("\nINTERPRETATION:")
    print("* A missing stable preflight prevents confirming equal initial conditions.")
    print("* 6077h is driver-reported torque current (0.1 A quantization), "
          "not an independent phase-current sensor.")
    print("* Differing current and acceleration suggest different effective "
          "actuator/mechanical responses; not proof of a specific drive fault.")
    print("* Do not repeat an open-loop current pulse until the prior overspeed "
          "and physical safety conditions have been investigated.")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_files", nargs="+", help="existing current_step CSV logs")
    p.add_argument("--wheel", choices=("left", "right"), default="left")
    p.add_argument("--window", type=float, default=0.14)
    p.add_argument("--bin-s", type=float, default=0.02)
    args = p.parse_args(argv)
    if (not math.isfinite(args.window) or args.window <= 0 or
            not math.isfinite(args.bin_s) or args.bin_s <= 0 or
            args.bin_s > args.window):
        p.error("window and bin width must be finite, positive and bin <= window")
    try:
        reports = [extract(x, window=args.window, wheel=args.wheel,
                           bin_s=args.bin_s) for x in args.csv_files]
    except (OSError, ValueError) as exc:
        p.error(str(exc))
    print_report(reports)
    return 0


if __name__ == "__main__":
    sys.exit(main())
