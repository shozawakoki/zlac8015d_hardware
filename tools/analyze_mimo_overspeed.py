#!/usr/bin/env python3
"""Offline MIMO-LADRC log audit: overspeed, saturation, braking direction.

Reads the existing /tmp/mimo_ladrc_track_200hz.csv, DOES NOT contact CAN or
change gains. All detection thresholds are explicitly printed. "Wrong direction"
means the SIGN of wheel current is opposite the instantaneous speed error,
not proof that the motor actually generated torque in that direction.
"""
import argparse
import csv
import math
import statistics
from pathlib import Path


REQUIRED = (
    "time_s", "ref_left_rad_s", "ref_right_rad_s",
    "vel_left_rad_s", "vel_right_rad_s",
    "cmd_current_left_A", "cmd_current_right_A",
    "actual_current_left_A", "actual_current_right_A",
    "tracking_left", "tracking_right",
    "ref_v_m_s", "ref_omega_rad_s", "v_m_s", "omega_rad_s",
)


def number_or_none(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def number(row, key):
    try:
        x = float(row[key])
    except (KeyError, ValueError, TypeError):
        return None
    return x if math.isfinite(x) else None


def classify(row, limit, speed_error, v_error, omega_error, min_v, min_omega):
    t = number(row, "time_s")
    if t is None:
        return None
    ref_v = number(row, "ref_v_m_s")
    ref_o = number(row, "ref_omega_rad_s")
    actual_v = number(row, "v_m_s")
    actual_o = number(row, "omega_rad_s")
    if any(x is None for x in (ref_v, ref_o, actual_v, actual_o)):
        return None
    composite = abs(ref_v) >= min_v and abs(ref_o) >= min_omega
    body_v_error = abs(ref_v - actual_v)
    body_omega_error = abs(ref_o - actual_o)
    row_out = {
        "time_s": t,
        "composite": composite,
        "error_v_m_s": body_v_error,
        "error_omega_rad_s": body_omega_error,
        "high_v_error": body_v_error >= v_error,
        "high_omega_error": body_omega_error >= omega_error,
    }
    count_sat = 0
    count_bad_direction = 0
    for wheel in ("left", "right"):
        ref = number(row, f"ref_{wheel}_rad_s")
        actual = number(row, f"vel_{wheel}_rad_s")
        current = number(row, f"cmd_current_{wheel}_A")
        if any(x is None for x in (ref, actual, current)):
            return None
        err = ref - actual
        saturated = abs(current) >= limit - 0.02
        # "Wrong direction" is a diagnostic, not necessarily a fault: low
        # speed or normal-stop ramp, forward friction, plant dynamics etc.
        wrong_dir = abs(err) >= speed_error and abs(current) >= 0.1 and err * current < 0
        overspeed = (ref * actual > 0 and abs(actual) > abs(ref) + speed_error)
        row_out[f"error_{wheel}_rad_s"] = err
        row_out[f"sat_{wheel}"] = saturated
        row_out[f"wrong_direction_{wheel}"] = wrong_dir
        row_out[f"overspeed_{wheel}"] = overspeed
        row_out[f"cmd_{wheel}_A"] = current
        row_out[f"actual_{wheel}_A"] = number(row, f"actual_current_{wheel}_A")
        count_sat += int(saturated)
        count_bad_direction += int(wrong_dir)
    row_out["either_saturated"] = count_sat > 0
    row_out["wrong_direction_count"] = count_bad_direction
    row_out["both_tracking"] = (
        number(row, "tracking_left") == 1 and
        number(row, "tracking_right") == 1
    )
    # Proxy for potential MIMO eligibility, not a definitive controller flag.
    row_out["wheel_ref_near_zero"] = (
        abs(number(row, "ref_left_rad_s")) <= 1e-6 or
        abs(number(row, "ref_right_rad_s")) <= 1e-6
    )
    return row_out


def contiguous_segments(rows, predicate, max_gap=.03):
    segments = []
    start = end = None
    count = 0
    for row in rows:
        t = row["time_s"]
        if predicate(row):
            if start is not None and t - end > max_gap:
                segments.append((start, end, count))
                start = None
            if start is None:
                start = t
                count = 0
            end = t
            count += 1
        elif start is not None:
            segments.append((start, end, count))
            start = end = None
            count = 0
    if start is not None:
        segments.append((start, end, count))
    return segments


def summarize(rows, limit, speed_error, v_error, omega_error, min_v, min_omega):
    data = [classify(row, limit, speed_error, v_error, omega_error,
                     min_v, min_omega) for row in rows]
    data = [x for x in data if x is not None]
    if not data:
        raise ValueError("No valid rows with required numeric fields")
    relevant = [x for x in data if x["composite"]]
    def pct(sample, fn):
        return 100.0 * sum(bool(fn(x)) for x in sample) / len(sample) if sample else 0.0
    result = {
        "all": data,
        "composite": relevant,
        "max_v_error": max(x["error_v_m_s"] for x in relevant) if relevant else None,
        "max_omega_error": max(x["error_omega_rad_s"] for x in relevant) if relevant else None,
        "saturation_pct": pct(relevant, lambda x: x["either_saturated"]),
        "wrong_dir_pct": pct(relevant, lambda x: x["wrong_direction_count"] > 0),
        "high_error_sat_pct": pct(
            relevant, lambda x: (x["high_v_error"] or x["high_omega_error"])
            and x["either_saturated"]),
        "near_zero_ref_pct": pct(relevant, lambda x: x["wheel_ref_near_zero"]),
        "not_both_tracking_pct": pct(relevant, lambda x: not x["both_tracking"]),
        "segments": contiguous_segments(
            data, lambda x: x["composite"] and x["either_saturated"]),
    }
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file", help="MIMO LADRC runtime CSV from existing log")
    p.add_argument("--limit-a", type=float, default=8.0,
                   help="actual URDF current_limit_a; NOT breakaway_max_a (branch: 8 A)")
    p.add_argument("--wheel-error", type=float, default=0.5, help="wheel rad/s")
    p.add_argument("--v-error", type=float, default=0.10, help="body m/s")
    p.add_argument("--omega-error", type=float, default=0.3, help="body rad/s")
    p.add_argument("--min-v", type=float, default=0.05, help="composite-motion |v_ref| m/s")
    p.add_argument("--min-omega", type=float, default=0.1, help="composite-motion |omega_ref| rad/s")
    p.add_argument("--output", help="optional diagnostic CSV output")
    args = p.parse_args(argv)
    for x in (args.limit_a, args.wheel_error, args.v_error,
              args.omega_error, args.min_v, args.min_omega):
        if not math.isfinite(x) or x <= 0:
            p.error("All limit and threshold parameters must be finite, positive values")
    file = Path(args.csv_file).expanduser()
    with file.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        absent = sorted(set(REQUIRED) - set(reader.fieldnames or []))
        if absent:
            p.error("Missing log fields: " + ", ".join(absent))
        rows = list(reader)
    try:
        results = summarize(rows, args.limit_a, args.wheel_error, args.v_error,
                            args.omega_error, args.min_v, args.min_omega)
    except ValueError as exc:
        p.error(str(exc))
    relevant = results["composite"]
    logged_currents = [
        abs(value) for row in rows
        for col in ("cmd_current_left_A", "cmd_current_right_A")
        if (value := number_or_none(row.get(col))) is not None
    ]
    observed_peak = max(logged_currents) if logged_currents else 0.0
    if observed_peak > args.limit_a + 0.05:
        p.error(f"--limit-a={args.limit_a:g} A is inconsistent with logged "
                f"|Icmd| peak {observed_peak:.3f} A. "
                "Use the URDF hardware current_limit_a, not breakaway_max_a.")
    print(f"Source: {file}")
    print(f"Observed peak |Icmd|: {observed_peak:.3f} A; "
          f"assumed controller hard current limit: {args.limit_a:.3f} A")
    print(f"Total valid rows: {len(results['all'])}; "
          f"composite v+omega rows: {len(relevant)}")
    print(f"Composite definition: |v_ref| >= {args.min_v} m/s, "
          f"|omega_ref| >= {args.min_omega} rad/s")
    print(f"Saturation threshold: |I_cmd| >= {args.limit_a - .02:.2f} A")
    if not relevant:
        print("No composite-motion rows. Check whether this run includes a combined v+omega command.")
        return 0
    for name, key, unit in (
        ("Max |v_ref-v|", "max_v_error", "m/s"),
        ("Max |omega_ref-omega|", "max_omega_error", "rad/s"),
        ("Percent of composite samples with any saturated wheel", "saturation_pct", "%"),
        ("Percent of composite samples with current opposing speed-error sign", "wrong_dir_pct", "%"),
        ("Percent with high body-speed error AND current saturation", "high_error_sat_pct", "%"),
        ("Percent with near-zero wheel reference", "near_zero_ref_pct", "%"),
        ("Percent with one or both TRACK flags off", "not_both_tracking_pct", "%"),
    ):
        print(f"{name}: {results[key]:.3f} {unit}")
    if relevant:
        for label, field in (("Worst linear-speed error", "error_v_m_s"),
                             ("Worst angular-speed error", "error_omega_rad_s")):
            peak = max(relevant, key=lambda x: x[field])
            print(f"{label}: t={peak['time_s']:.3f} s, "
                  f"absolute error={peak[field]:.3f}, "
                  f"IcmdL/R={peak['cmd_left_A']:+.2f}/"
                  f"{peak['cmd_right_A']:+.2f} A, "
                  f"bothTRACK={peak['both_tracking']}, "
                  f"saturated={peak['either_saturated']}")
    print("\nFirst 20 composite-motion saturation segments: start_s, end_s, samples")
    for a,b,n in results["segments"][:20]:
        print(f"  {a:.3f}, {b:.3f}, {n}")
    output = (Path(args.output).expanduser() if args.output else
              file.with_name(file.stem + "_overspeed_audit.csv"))
    with output.open("w", newline="", encoding="utf-8") as f:
        fields = list(results["all"][0])
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results["all"])
    print(f"Diagnostic CSV: {output}")
    print("NOTE: The measured 'actual current' columns are the driver's 6077h "
          "feedback, not independently measured motor phase currents.")
    print("NOTE: A single wrong-direction sample is not proof of destabilizing control. "
          "Review time intervals and reference changes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
