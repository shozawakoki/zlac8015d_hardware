#!/usr/bin/env python3
"""Read-only investigation of the worst body-speed errors in a MIMO CSV.

Prints exact event samples and ±time windows to distinguish:
  * target reversal / sign change,
  * BREAKAWAY and TRACK re-entry,
  * stall lockout,
  * velocity error and commanded/feedback motor current.
No ROS, CAN, file writes, or drive actuation. Requires only Python stdlib.
"""
import argparse
import csv
import math
from pathlib import Path
import statistics


REQUIRED = [
    "time_s", "ref_left_rad_s", "ref_right_rad_s",
    "vel_left_rad_s", "vel_right_rad_s",
    "cmd_current_left_A", "cmd_current_right_A",
    "actual_current_left_A", "actual_current_right_A",
    "tracking_left", "tracking_right",
    "ref_v_m_s", "ref_omega_rad_s", "v_m_s", "omega_rad_s",
]


def parse_float(raw):
    try:
        result = float(raw)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def row_numbers(row):
    nums = {}
    for k in REQUIRED:
        value = parse_float(row.get(k))
        if value is None:
            return None
        nums[k] = value
    for k in ("stall_lockout_left", "stall_lockout_right",
              "breakaway_target_left_A", "breakaway_target_right_A"):
        nums[k] = parse_float(row.get(k))
    nums["e_v"] = nums["ref_v_m_s"] - nums["v_m_s"]
    nums["e_omega"] = nums["ref_omega_rad_s"] - nums["omega_rad_s"]
    nums["both_track"] = (
        nums["tracking_left"] == 1 and nums["tracking_right"] == 1)
    return nums


def is_composite(r, min_v, min_omega):
    return abs(r["ref_v_m_s"]) >= min_v and abs(r["ref_omega_rad_s"]) >= min_omega


def find_extrema(rows, min_v=.05, min_omega=.1):
    subset = [r for r in rows if is_composite(r, min_v, min_omega)]
    if not subset:
        raise ValueError("No samples with combined linear and yaw references")
    return (
        max(subset, key=lambda r: abs(r["e_v"])),
        max(subset, key=lambda r: abs(r["e_omega"])),
        len(subset),
    )


def transitions(rows, start, end):
    """TRACK changes; reference sign changes or emergence from exact zero."""
    excerpt = [r for r in rows if start <= r["time_s"] <= end]
    out = []
    for prev, current in zip(excerpt, excerpt[1:]):
        keys = []
        for wheel in ("left", "right"):
            key = f"tracking_{wheel}"
            if prev[key] != current[key]:
                keys.append(f"{wheel} TRACK {int(prev[key])}->{int(current[key])}")
            refkey = f"ref_{wheel}_rad_s"
            a,b = prev[refkey],current[refkey]
            sign = lambda x: 1 if x>1e-6 else -1 if x < -1e-6 else 0
            if sign(a) != sign(b):
                keys.append(f"{wheel} reference-sign {sign(a)}->{sign(b)}")
            lock = f"stall_lockout_{wheel}"
            if prev[lock] is not None and current[lock] is not None and prev[lock] != current[lock]:
                keys.append(f"{wheel} lockout {prev[lock]:g}->{current[lock]:g}")
        if keys:
            out.append((current["time_s"], "; ".join(keys)))
    return out


def summarize_window(rows, peak_time, before, after, bin_s):
    start = max(0, peak_time-before)
    end = peak_time+after
    selected = [r for r in rows if start <= r["time_s"] <= end]
    grouped = []
    n = int(math.ceil((end-start)/bin_s))
    for k in range(n):
        lo = start+k*bin_s
        hi = min(end, lo+bin_s)
        sample=[r for r in selected if lo <= r["time_s"] < hi]
        if not sample:
            continue
        avg=lambda col: statistics.mean(r[col] for r in sample)
        grouped.append({
            "time": (lo+hi)*.5,
            "ref_v":avg("ref_v_m_s"),
            "v":avg("v_m_s"),
            "ref_o":avg("ref_omega_rad_s"),
            "omega":avg("omega_rad_s"),
            "Icmd_l":avg("cmd_current_left_A"),
            "Iact_l":avg("actual_current_left_A"),
            "Icmd_r":avg("cmd_current_right_A"),
            "Iact_r":avg("actual_current_right_A"),
            "track_pct":100*sum(r["both_track"] for r in sample)/len(sample),
            "lock_l":sum(r["stall_lockout_left"]==1 for r in sample),
            "lock_r":sum(r["stall_lockout_right"]==1 for r in sample),
            "count":len(sample)
        })
    return selected, grouped


def detail(r):
    print(f"t={r['time_s']:.3f}s  "
          f"ref(v,omega)=({r['ref_v_m_s']:+.3f},{r['ref_omega_rad_s']:+.3f}) "
          f"actual=({r['v_m_s']:+.3f},{r['omega_rad_s']:+.3f}) "
          f"errors=({r['e_v']:+.3f},{r['e_omega']:+.3f})")
    for w in ("left", "right"):
        print(f" {w}: ref={r[f'ref_{w}_rad_s']:+.3f}rad/s "
              f"speed={r[f'vel_{w}_rad_s']:+.3f}rad/s "
              f"Icmd={r[f'cmd_current_{w}_A']:+.3f}A "
              f"Iactual={r[f'actual_current_{w}_A']:+.3f}A "
              f"TRACK={int(r[f'tracking_{w}'])} "
              f"breakaway_target={r[f'breakaway_target_{w}_A']} "
              f"lockout={r[f'stall_lockout_{w}']}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--before", type=float, default=.8)
    p.add_argument("--after", type=float, default=1.2)
    p.add_argument("--bin-s", type=float, default=.1)
    p.add_argument("--min-v", type=float, default=.05)
    p.add_argument("--min-omega", type=float, default=.1)
    args=p.parse_args(argv)
    if any(not math.isfinite(x) or x<=0 for x in
           (args.before,args.after,args.bin_s,args.min_v,args.min_omega)):
        p.error("All windows and thresholds must be positive and finite.")
    path=Path(args.csv_file).expanduser()
    with path.open(newline="",encoding="utf-8") as f:
        reader=csv.DictReader(f)
        missing=sorted(set(REQUIRED)-set(reader.fieldnames or []))
        if missing:
            p.error("Missing columns: "+", ".join(missing))
        rows=[parsed for item in reader if (parsed:=row_numbers(item)) is not None]
    if not rows:
        p.error("No valid data rows")
    try:
        worst_v,worst_o,count=find_extrema(rows,args.min_v,args.min_omega)
    except ValueError as e:
        p.error(str(e))
    print(f"Source: {path}; valid rows={len(rows)}, combined-command rows={count}")
    print("IMPORTANT: these are maxima of ABSOLUTE TRACKING ERROR; not necessarily overshoot.")
    for name,peak in (("linear speed",worst_v),("yaw rate",worst_o)):
        print(f"\n===== MAX {name.upper()} ERROR =====")
        detail(peak)
        selected,binned=summarize_window(
            rows,peak["time_s"],args.before,args.after,args.bin_s)
        print(f"Time window {selected[0]['time_s']:.3f}.."
              f"{selected[-1]['time_s']:.3f} s, N={len(selected)}")
        print(" t[s]   vref/v      oref/o      IcmdL/IactL    IcmdR/IactR    TRACK% lockL/R")
        for b in binned:
            print(f"{b['time']:6.2f} "
                  f"{b['ref_v']:+5.2f}/{b['v']:+5.2f} "
                  f"{b['ref_o']:+6.2f}/{b['omega']:+6.2f} "
                  f"{b['Icmd_l']:+6.2f}/{b['Iact_l']:+6.2f} "
                  f"{b['Icmd_r']:+6.2f}/{b['Iact_r']:+6.2f} "
                  f"{b['track_pct']:6.0f} {b['lock_l']}/{b['lock_r']}")
        changes=transitions(rows,
                            max(0,peak["time_s"]-args.before),
                            peak["time_s"]+args.after)
        print("TRACK, lockout and wheel-target sign changes (first 35):")
        if not changes:
            print("  none found in focus interval")
        for t,msg in changes[:35]:
            print(f" {t:.3f}s: {msg}")
    print("\nNOTE: No mimo_ready, normal-stop-ramp, or controller-request BEFORE "
          "limiting was logged; TRACK changes and ref signs are only proxies.")
    print("NOTE: driver current feedback is 6077h, not independent phase-current measurement.")


if __name__=="__main__":
    main()
