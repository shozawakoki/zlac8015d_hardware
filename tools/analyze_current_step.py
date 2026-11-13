#!/usr/bin/env python3
"""Summarize ZLAC current-step CSV and optionally plot measured current/RPM."""

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys


def load(path):
    with Path(path).open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError("CSV is empty")
    if "requested_step_A" not in rows[0]:
        raise ValueError("Not a current_step_test.py CSV")
    return rows


def number(row, key):
    try:
        x = float(row[key])
    except (ValueError, KeyError, TypeError):
        return None
    return x if math.isfinite(x) else None


def summarize(rows, wheel):
    measured = f"actual_{wheel}_A"
    commanded = f"command_{wheel}_A"
    velocity = f"{wheel}_rpm"
    groups = {}
    for row in rows:
        if row["event"] != "current_rx":
            continue
        try:
            stage = int(row["step_index"])
        except ValueError:
            continue
        if stage < 0:
            continue
        actual = number(row, measured)
        cmd = number(row, commanded)
        rpm = number(row, velocity)
        if actual is None or cmd is None:
            continue
        groups.setdefault(stage, []).append((actual, cmd, rpm))
    result = []
    for stage, samples in sorted(groups.items()):
        actual = [item[0] for item in samples]
        cmd = [item[1] for item in samples]
        speeds = [abs(item[2]) for item in samples if item[2] is not None]
        result.append({
            "stage": stage + 1,
            "requested_A": cmd[0],
            "count": len(samples),
            "mean_actual_A": statistics.mean(actual),
            "mean_abs_error_A": statistics.mean(abs(a-c) for a,c in zip(actual,cmd)),
            "max_abs_error_A": max(abs(a-c) for a,c in zip(actual,cmd)),
            "max_abs_rpm": max(speeds) if speeds else float("nan")
        })
    return result


def make_plots(rows, output_prefix):
    import matplotlib.pyplot as plt  # optional dependency
    for graph in ("current", "rpm"):
        fig, ax = plt.subplots(figsize=(11, 4))
        if graph == "current":
            for wheel in ("left", "right"):
                cmd = [(number(row, "elapsed_s"), number(row, f"command_{wheel}_A"))
                       for row in rows if row["event"] in ("step_change", "complete_zero")]
                cmd = [(t, v) for t,v in cmd if t is not None and v is not None]
                actual = [(number(row, "elapsed_s"), number(row, f"actual_{wheel}_A"))
                          for row in rows if row["event"] == "current_rx"]
                actual = [(t, v) for t,v in actual if t is not None and v is not None]
                if cmd:
                    ax.step([p[0] for p in cmd], [p[1] for p in cmd],
                            where="post", label=f"{wheel} commanded [A]")
                if actual:
                    ax.plot([p[0] for p in actual], [p[1] for p in actual],
                            linewidth=1.0, label=f"{wheel} measured [A]")
            ax.set_ylabel("Motor current [A]")
        else:
            for wheel in ("left", "right"):
                speed = [(number(row,"elapsed_s"), number(row,f"{wheel}_rpm"))
                         for row in rows if row["event"] == "velocity_rx"]
                speed = [(t,v) for t,v in speed if t is not None and v is not None]
                if speed:
                    ax.plot([p[0] for p in speed], [p[1] for p in speed],
                            label=f"{wheel} wheel RPM", linewidth=1)
            ax.set_ylabel("Wheel rotational speed [rpm]")
        ax.set_xlabel("Elapsed time [s]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
        fig.tight_layout()
        output = Path(str(output_prefix) + f"_{graph}.png")
        fig.savefig(output, dpi=160)
        plt.close(fig)
        print(f"Graph: {output}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file", help="CSV recorded by current_step_test.py")
    p.add_argument("--plots", action="store_true", help="write separate current / speed PNGs (matplotlib)")
    args = p.parse_args(argv)
    rows = load(args.csv_file)
    print("Current tracking by step. Includes the transient immediately after each step:")
    for wheel in ("left", "right"):
        print(f"\n{wheel.upper()} wheel")
        print("step  requested[A]  mean_actual[A]  MAE[A]  max_error[A]  max_rpm  samples")
        for rec in summarize(rows, wheel):
            print(f"{rec['stage']:>4}  {rec['requested_A']:>11.2f}  "
                  f"{rec['mean_actual_A']:>14.3f}  {rec['mean_abs_error_A']:>6.3f}  "
                  f"{rec['max_abs_error_A']:>12.3f}  {rec['max_abs_rpm']:>7.1f}  {rec['count']:>7}")
    if args.plots:
        try:
            make_plots(rows, Path(args.csv_file).with_suffix(""))
        except ImportError:
            print("matplotlib is not installed. Install python3-matplotlib for PNG plots.",
                  file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
