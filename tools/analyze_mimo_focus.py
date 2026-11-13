#!/usr/bin/env python3
"""Read-only focus analysis of MIMO-LADRC runtime CSV; NEVER controls hardware.

Cross-tabulates saturated/breakaway/braking states, then bins a selected
time window. Input timestamps and feedback are those recorded by the
original ROS control plugin, not synchronized external measurements.
"""
import argparse
import csv
import math
from pathlib import Path
import statistics


REQUIRED = (
    "time_s", "ref_left_rad_s", "ref_right_rad_s",
    "vel_left_rad_s", "vel_right_rad_s",
    "cmd_current_left_A", "cmd_current_right_A",
    "actual_current_left_A", "actual_current_right_A",
    "tracking_left", "tracking_right",
    "ref_v_m_s", "ref_omega_rad_s", "v_m_s", "omega_rad_s",
)


def val(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else None
    except (KeyError, TypeError, ValueError):
        return None


def classify(row, limit, min_v, min_omega, wheel_err):
    a = {name: val(row, name) for name in REQUIRED}
    if any(x is None for x in a.values()):
        return None
    a["time_s"] = val(row, "time_s")
    if a["time_s"] is None:
        return None
    composite = abs(a["ref_v_m_s"]) >= min_v and abs(a["ref_omega_rad_s"]) >= min_omega
    both_track = a["tracking_left"] == 1 and a["tracking_right"] == 1
    saturated = any(abs(a[f"cmd_current_{w}_A"]) >= limit-0.02 for w in ("left", "right"))
    overspeed_wheels = []
    braking_wheels = []
    driving_while_overspeed = []
    for w in ("left", "right"):
        ref, speed = a[f"ref_{w}_rad_s"], a[f"vel_{w}_rad_s"]
        cmd = a[f"cmd_current_{w}_A"]
        overspeed = ref*speed > 0 and abs(speed)-abs(ref) >= wheel_err
        braking = overspeed and cmd*speed < -0.05
        driving = overspeed and cmd*speed > 0.05
        overspeed_wheels.append(overspeed)
        braking_wheels.append(braking)
        driving_while_overspeed.append(driving)
    a.update(
        composite=composite,
        both_track=both_track,
        saturated=saturated,
        overspeed=any(overspeed_wheels),
        braking_during_overspeed=any(braking_wheels),
        driving_during_overspeed=any(driving_while_overspeed),
    )
    return a


def summarize(rows):
    related = [r for r in rows if r["composite"]]
    if not related:
        raise ValueError("No composite-motion rows found")
    def describe(group, label):
        n=len(group)
        if n == 0:
            return (label, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
        percent=lambda key: 100.0*sum(bool(r[key]) for r in group)/n
        return (label, n, percent("saturated"), percent("overspeed"),
                percent("braking_during_overspeed"),
                percent("driving_during_overspeed"),
                statistics.mean(abs(r["ref_v_m_s"]-r["v_m_s"]) for r in group))
    return [
        describe(related, "All composite"),
        describe([r for r in related if r["both_track"]], "Both TRACK"),
        describe([r for r in related if not r["both_track"]], "Any BREAKAWAY"),
        describe([r for r in related if r["saturated"]], "Saturated subset"),
        describe([r for r in related if r["overspeed"]], "Overspeed subset"),
    ]


def bins(rows, start, end, step):
    n=int(math.ceil((end-start)/step))
    for i in range(n):
        a=start+i*step
        b=min(end, a+step)
        subset=[r for r in rows if a <= r["time_s"] < b]
        if not subset:
            continue
        mean=lambda k: statistics.mean(r[k] for r in subset)
        rate=lambda key: 100*sum(bool(r[key]) for r in subset)/len(subset)
        yield {
            "t_start_s": round(a, 3), "t_end_s": round(b, 3), "samples": len(subset),
            "ref_v": mean("ref_v_m_s"), "v": mean("v_m_s"),
            "ref_omega": mean("ref_omega_rad_s"), "omega": mean("omega_rad_s"),
            "ref_l": mean("ref_left_rad_s"), "speed_l": mean("vel_left_rad_s"),
            "ref_r": mean("ref_right_rad_s"), "speed_r": mean("vel_right_rad_s"),
            "cmd_l": mean("cmd_current_left_A"),
            "actual_l": mean("actual_current_left_A"),
            "cmd_r": mean("cmd_current_right_A"),
            "actual_r": mean("actual_current_right_A"),
            "sat_pct": rate("saturated"), "track_pct": rate("both_track"),
            "overspeed_pct": rate("overspeed"),
            "drive_overspeed_pct": rate("driving_during_overspeed"),
            "brake_overspeed_pct": rate("braking_during_overspeed"),
        }


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--start",type=float,default=22.5)
    p.add_argument("--end",type=float,default=24.3)
    p.add_argument("--bin-s",type=float,default=0.1)
    p.add_argument("--limit-a",type=float,default=2.0)
    p.add_argument("--wheel-overspeed",type=float,default=0.5,
                   help="minimum wheel velocity excess in rad/s")
    p.add_argument("--min-v",type=float,default=0.05)
    p.add_argument("--min-omega",type=float,default=0.1)
    p.add_argument("--out",help="save binned records to a CSV path")
    args=p.parse_args(argv)
    if not (0<=args.start<args.end and 0<args.bin_s<=5 and
            args.limit_a>0.02 and args.wheel_overspeed>0 and
            args.min_v>0 and args.min_omega>0):
        p.error("Invalid positive ranges/thresholds")
    file=Path(args.csv_file).expanduser()
    with file.open(newline="",encoding="utf-8") as f:
        reader=csv.DictReader(f)
        absent=set(REQUIRED)|{"time_s"}-set(reader.fieldnames or [])
        if absent:
            p.error("Missing CSV fields: "+", ".join(sorted(absent)))
        records=list(reader)
    analyzed=[classify(x,args.limit_a,args.min_v,args.min_omega,
                       args.wheel_overspeed) for x in records]
    analyzed=[x for x in analyzed if x is not None]
    if not analyzed:
        p.error("No valid records")
    print(f"File: {file}; parsed rows: {len(analyzed)}")
    print(f"Composite threshold |v_ref|>={args.min_v}m/s, "
          f"|omega_ref|>={args.min_omega}rad/s; "
          f"current saturation >= {args.limit_a-.02:.2f} A")
    print(f"Wheel overspeed threshold: >= {args.wheel_overspeed}rad/s above |reference|")
    print("Group              rows      sat%  overspeed% brake-over% drive-over%  mean |v error|")
    for label,n,sat,over,brake,drive,verr in summarize(analyzed):
        print(f"{label:<18} {n:7d} {sat:8.1f} {over:10.1f} "
              f"{brake:11.1f} {drive:11.1f} {verr:13.3f}")
    focus=list(bins(analyzed,args.start,args.end,args.bin_s))
    print(f"\nFocus {args.start:.2f}-{args.end:.2f}s; "
          f"per-bin means and percent of samples within each bin")
    print("Time[s]    v_ref/v    omega_ref/omega    left Icmd/Iact    right Icmd/Iact   SAT% TRACK% OVR% DRV% BRK%")
    for b in focus:
        print(f"{b['t_start_s']:6.2f} "
              f"{b['ref_v']:5.2f}/{b['v']:5.2f} "
              f"{b['ref_omega']:6.2f}/{b['omega']:6.2f} "
              f"{b['cmd_l']:+6.2f}/{b['actual_l']:+6.2f} "
              f"{b['cmd_r']:+6.2f}/{b['actual_r']:+6.2f} "
              f"{b['sat_pct']:5.0f} {b['track_pct']:6.0f} "
              f"{b['overspeed_pct']:4.0f} {b['drive_overspeed_pct']:4.0f} "
              f"{b['brake_overspeed_pct']:4.0f}")
    if not focus:
        print("No samples in requested focus interval")
    out=Path(args.out).expanduser() if args.out else file.with_name(
        file.stem+f"_focus_{args.start:g}_{args.end:g}.csv")
    if focus:
        with out.open("w",newline="",encoding="utf-8") as f:
            writer=csv.DictWriter(f,fieldnames=list(focus[0]))
            writer.writeheader()
            writer.writerows(focus)
        print("Saved:",out)
    print("CAUTION: percentages are sample-based; repeated/irregular timestamps "
          "could bias time-weighted interpretation.")
    print("DRV/BRK only classify sign of commanded current relative to measured "
          "overspeeding wheel; they do not establish actual torque.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
