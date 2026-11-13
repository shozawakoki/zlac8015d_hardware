#!/usr/bin/env python3
"""Offline analysis of a ZLAC8015D step log: Icmd, Iactual, RPM and acceleration.

Never opens CAN and never sends commands or changes drive settings.
Uses ONLY fresh TPDO event rows and excludes the post-test zero-current coast.
"""
import argparse
import csv
import math
from pathlib import Path
import statistics
import sys


def numeric(row, key):
    try:
        v = float(row[key])
    except (ValueError, KeyError, TypeError):
        return None
    return v if math.isfinite(v) else None


def linear_slope(points):
    """OLS slope of y versus t, in original units per second."""
    if len(points) < 3:
        return None
    xbar = statistics.mean(x for x, _ in points)
    ybar = statistics.mean(y for _, y in points)
    denominator = sum((x-xbar)**2 for x, _ in points)
    if denominator <= 1e-12:
        return None
    return sum((x-xbar)*(y-ybar) for x, y in points) / denominator


def stage_samples(rows, stage, wheel):
    stage_no = stage - 1
    active_end = min(
        (numeric(r, "elapsed_s") for r in rows
         if r.get("event") == "complete_zero" and numeric(r, "elapsed_s") is not None),
        default=math.inf
    )
    change = next((r for r in rows if r.get("event") == "step_change"
                   and r.get("step_index") == str(stage_no)), None)
    if change is None:
        raise ValueError(f"No step_change for stage {stage} in this CSV")
    start = numeric(change, "elapsed_s")
    if start is None:
        raise ValueError("Missing step start timestamp")
    subsequent = [
        numeric(r, "elapsed_s") for r in rows
        if r.get("event") == "step_change"
        and (numeric(r, "elapsed_s") is not None)
        and numeric(r, "elapsed_s") > start + 1e-7
    ]
    end = min([active_end] + subsequent)
    current_cmd = numeric(change, f"command_{wheel}_A")
    if current_cmd is None:
        raise ValueError("Invalid commanded current in step_change")
    currents = []
    speeds = []
    for row in rows:
        t = numeric(row, "elapsed_s")
        if t is None or not (start <= t < end):
            continue
        rel = t - start
        if row.get("event") == "current_rx":
            val = numeric(row, f"actual_{wheel}_A")
            if val is not None:
                currents.append((rel, val))
        elif row.get("event") == "velocity_rx":
            val = numeric(row, f"{wheel}_rpm")
            if val is not None:
                speeds.append((rel, val))
    return current_cmd, currents, speeds, start, end


def make_bins(currents, speeds, cmd, width, duration):
    result = []
    count = int(math.ceil(duration / width - 1e-9))
    for i in range(count):
        lo = i * width
        hi = min(duration, (i+1) * width)
        cs = [v for t,v in currents if lo <= t < hi]
        vs = [v for t,v in speeds if lo <= t < hi]
        if not cs and not vs:
            continue
        result.append({
            "bin_start_s": round(lo, 4),
            "bin_end_s": round(hi, 4),
            "target_A": cmd,
            "actual_mean_A": statistics.mean(cs) if cs else "",
            "actual_min_A": min(cs) if cs else "",
            "actual_max_A": max(cs) if cs else "",
            "error_mean_A": statistics.mean(cmd-v for v in cs) if cs else "",
            "rpm_mean": statistics.mean(vs) if vs else "",
            "rpm_max_abs": max(map(abs, vs)) if vs else "",
            "n_current": len(cs),
            "n_velocity": len(vs),
        })
    return result


def compute(rows, stage=1, wheel="left", bin_s=0.1):
    if bin_s <= 0:
        raise ValueError("bin_s must be positive")
    cmd, currents, speeds, start, end = stage_samples(rows, stage, wheel)
    if not currents or not speeds:
        raise ValueError("No fresh TPDO0 or TPDO1 feedback for the requested stage")
    duration = end-start if math.isfinite(end) else max(currents[-1][0], speeds[-1][0])
    bins = make_bins(currents, speeds, cmd, bin_s, duration)
    # The early window deliberately uses time stamps, not simple bin maxima.
    early_current = [(t,v) for t,v in currents if 0.0 <= t < min(0.20, duration)]
    later_current = [(t,v) for t,v in currents if 0.20 <= t < min(0.80, duration)]
    speed_range = [(t, rpm * 2.0*math.pi/60.0) for t,rpm in speeds
                   if 0.20 <= t < min(0.80, duration)]
    if len(speed_range) < 3:
        speed_range = [(t, rpm*2.0*math.pi/60.0) for t,rpm in speeds]
    return {
        "command_A": cmd,
        "duration_s": duration,
        "n_current": len(currents),
        "n_velocity": len(speeds),
        "mean_current_A": statistics.mean(v for _,v in currents),
        "max_abs_rpm": max(abs(v) for _,v in speeds),
        "early_dI_dt_A_s": linear_slope(early_current),
        "later_dI_dt_A_s": linear_slope(later_current),
        "wheel_accel_rad_s2": linear_slope(speed_range),
        "bins": bins,
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file", help="path to a current_steps_*.csv")
    p.add_argument("--wheel", choices=("left", "right"), default="left")
    p.add_argument("--stage", type=int, default=1, help="1-based step number")
    p.add_argument("--bin-s", type=float, default=0.1)
    p.add_argument("--slope-setting", type=int, default=None,
                   help="readback of 6087h; only printed for reference; unit interpretation is uncertain")
    p.add_argument("--output", help="optional path for binned summary CSV")
    args = p.parse_args(argv)
    with Path(args.csv_file).expanduser().open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        p.error("input CSV has no rows")
    try:
        report = compute(rows, args.stage, args.wheel, args.bin_s)
    except ValueError as exc:
        p.error(str(exc))
    print(f"File: {args.csv_file}")
    print(f"Wheel={args.wheel}; stage={args.stage}; duration={report['duration_s']:.3f}s")
    print(f"Command={report['command_A']:+.3f} A; actual mean={report['mean_current_A']:+.3f} A; "
          f"max wheel speed={report['max_abs_rpm']:.1f} rpm")
    print(f"Fresh TPDO current={report['n_current']}; speed={report['n_velocity']}")
    early_end = min(0.20, report["duration_s"])
    later_end = min(0.80, report["duration_s"])
    for key, name, unit in [
        ("early_dI_dt_A_s", f"Linear-fit actual-current slope 0–{early_end:.2f} s", "A/s"),
        ("later_dI_dt_A_s", f"Linear-fit actual-current slope 0.2–{later_end:.2f} s", "A/s"),
        ("wheel_accel_rad_s2",
         f"Linear-fit wheel angular acceleration (0.2–{later_end:.2f} s)", "rad/s^2"),
    ]:
        val = report[key]
        print(f"{name}: {'not enough samples' if val is None else f'{val:+.3f} ' + unit}")
    print("\nTime[s]       I_actual_mean[A]    I_error_mean[A]     max_abs_RPM     N(I)/N(rpm)")
    for b in report["bins"]:
        fmt = lambda v: f"{v:+.3f}" if isinstance(v, (int,float)) else "--"
        vr = b["rpm_max_abs"]
        print(f"{b['bin_start_s']:5.2f}-{b['bin_end_s']:<5.2f} "
              f"{fmt(b['actual_mean_A']):>15} {fmt(b['error_mean_A']):>19} "
              f"{f'{vr:.1f}' if isinstance(vr, (int,float)) else '--':>13} "
              f"{b['n_current']:>4}/{b['n_velocity']:<4}")
    if args.slope_setting is not None:
        print(f"\n6087h readback={args.slope_setting}, but the manufacturer's "
              "documentation has inconsistent units.")
        print("Do NOT equate this raw value directly to measured dI/dt without validation.")
    output = (Path(args.output).expanduser() if args.output else
              Path(args.csv_file).expanduser().with_name(
                  Path(args.csv_file).stem + f"_{args.wheel}_stage{args.stage}_binned.csv"))
    with output.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "bin_start_s","bin_end_s","target_A","actual_mean_A",
            "actual_min_A","actual_max_A","error_mean_A","rpm_mean",
            "rpm_max_abs","n_current","n_velocity"])
        writer.writeheader()
        writer.writerows(report["bins"])
    print(f"\nSaved summary: {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
