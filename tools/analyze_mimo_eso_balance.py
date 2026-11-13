#!/usr/bin/env python3
"""Offline MIMO-LADRC yaw-current decomposition using existing 200 Hz CSV only.

Formula (b_left=b_right=b0):
    I_R - I_L = [ wc_omega*(omega_ref - hat_omega)
                  - hat_d_omega ] / [ b0*wheel_radius/wheel_separation ].

The predicted value is the *raw LADRC* yaw differential current. Actual final
RPDO command can differ because of current slew limit, floor, breakaway,
saturation, controller reset or changed configurations. A close match does
NOT validate the ESO disturbance as a physical estimate.
"""
import argparse
import csv
import math
from pathlib import Path
import statistics


REQUIRED = (
    "time_s", "ref_omega_rad_s", "omega_rad_s",
    "hat_omega_rad_s", "hat_d_omega_rad_s2",
    "cmd_current_left_A", "cmd_current_right_A",
    "tracking_left", "tracking_right", "ref_v_m_s",
    "ref_left_rad_s", "ref_right_rad_s",
)


def parse_num(row, field):
    try:
        value = float(row[field])
        return value if math.isfinite(value) else None
    except (KeyError, ValueError, TypeError):
        return None


def analyze_row(row, wc, gain, min_v):
    v = {name: parse_num(row, name) for name in REQUIRED}
    if any(n is None for n in v.values()):
        return None
    t = v["time_s"]
    if (abs(v["ref_v_m_s"]) < min_v or
            v["tracking_left"] != 1 or v["tracking_right"] != 1 or
            abs(v["ref_left_rad_s"]) <= 1e-6 or
            abs(v["ref_right_rad_s"]) <= 1e-6):
        return None
    if parse_num(row, "stall_lockout_left") not in (None, 0) or (
            parse_num(row, "stall_lockout_right") not in (None, 0)):
        return None
    error = v["ref_omega_rad_s"] - v["hat_omega_rad_s"]
    feedback_term = wc * error
    disturbance_term = -v["hat_d_omega_rad_s2"]
    virtual_acceleration = feedback_term + disturbance_term
    pred = virtual_acceleration / gain
    logged = v["cmd_current_right_A"] - v["cmd_current_left_A"]
    yaw_error = v["ref_omega_rad_s"] - v["omega_rad_s"]
    measured_diff = (parse_num(row,"actual_current_right_A")
                     if parse_num(row,"actual_current_right_A") is not None
                     else math.nan) - (parse_num(row,"actual_current_left_A")
                     if parse_num(row,"actual_current_left_A") is not None
                     else math.nan)
    return {
        "time_s": t,
        "ref_omega": v["ref_omega_rad_s"],
        "omega": v["omega_rad_s"],
        "hat_omega": v["hat_omega_rad_s"],
        "yaw_error": yaw_error,
        "hat_d_omega": v["hat_d_omega_rad_s2"],
        "feedback_accel_rad_s2": feedback_term,
        "disturbance_accel_rad_s2": disturbance_term,
        "requested_accel_rad_s2": virtual_acceleration,
        "predicted_diff_A": pred,
        "commanded_diff_A": logged,
        "measured_diff_A": measured_diff,
        "command_residual_A": logged - pred,
        "opposite_error_sign": (
            abs(yaw_error) >= .15 and abs(logged) >= .2 and
            yaw_error * logged < 0),
        "disturbance_dominates": abs(disturbance_term) > abs(feedback_term),
    }


def mean(values):
    a=[v for v in values if isinstance(v,(float,int)) and math.isfinite(v)]
    return statistics.mean(a) if a else math.nan


def summarize(rows,start,end,bin_s):
    chosen=[r for r in rows if start <= r["time_s"] < end]
    result=[]
    num_bins=int(math.ceil((end-start)/bin_s))
    for i in range(num_bins):
        lo=start+i*bin_s
        hi=min(end,lo+bin_s)
        subset=[r for r in chosen if lo <= r["time_s"] < hi]
        if not subset:
            continue
        record={"start_s":round(lo,4),"end_s":round(hi,4),"n":len(subset)}
        for col in (
                "ref_omega","omega","hat_omega","yaw_error",
                "hat_d_omega","feedback_accel_rad_s2",
                "disturbance_accel_rad_s2","requested_accel_rad_s2",
                "predicted_diff_A","commanded_diff_A","measured_diff_A",
                "command_residual_A"):
            record[col]=mean([r[col] for r in subset])
        record["opposite_pct"]=100*sum(r["opposite_error_sign"] for r in subset)/len(subset)
        record["disturbance_dominates_pct"]=(
            100*sum(r["disturbance_dominates"] for r in subset)/len(subset))
        result.append(record)
    return chosen,result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--start",type=float,default=23.5)
    p.add_argument("--end",type=float,default=24.3)
    p.add_argument("--bin-s",type=float,default=.1)
    p.add_argument("--wc-omega",type=float,default=5.0)
    p.add_argument("--b0",type=float,default=18.0,
                   help="equal left/right wheel b0 in rad/s^2/A")
    p.add_argument("--wheel-radius",type=float,default=.064)
    p.add_argument("--wheel-separation",type=float,default=.354)
    p.add_argument("--min-v",type=float,default=.05)
    p.add_argument("--output",help="binned report CSV")
    args=p.parse_args(argv)
    for name in ("wc_omega","b0","wheel_radius","wheel_separation","bin_s","min_v"):
        v=getattr(args,name)
        if not math.isfinite(v) or v<=0:
            p.error(f"--{name.replace('_','-')} must be finite positive")
    if not (math.isfinite(args.start) and math.isfinite(args.end) and 0<=args.start<args.end):
        p.error("start/end range must be finite and valid")
    gain=args.b0*args.wheel_radius/args.wheel_separation
    src=Path(args.csv_file).expanduser()
    with src.open(newline="",encoding="utf-8") as f:
        reader=csv.DictReader(f)
        missing=sorted(set(REQUIRED)-set(reader.fieldnames or []))
        if missing:
            p.error("missing CSV fields: "+", ".join(missing))
        raw=list(reader)
    data=[analyze_row(r,args.wc_omega,gain,args.min_v) for r in raw]
    data=[r for r in data if r is not None]
    if not data:
        p.error("No suitable both-TRACK, moving MIMO samples in CSV")
    chosen,binned=summarize(data,args.start,args.end,args.bin_s)
    print(f"File: {src}; valid both-TRACK moving MIMO samples: {len(data)}")
    print(f"Model: wc={args.wc_omega:g}rad/s, b0={args.b0:g}rad/s^2/A, "
          f"r={args.wheel_radius:g}m, track={args.wheel_separation:g}m")
    print(f"Yaw current-difference gain K=b0*r/track={gain:.6f}rad/s^2/A")
    print("I_R-I_L = [wc*(omega_ref-hat_omega)-hat_d_omega]/K")
    print(f"Window {args.start:g}..{args.end:g}s: {len(chosen)} samples")
    if not chosen:
        print("No eligible MIMO samples in requested time window")
        return 0
    residual=mean([abs(r["command_residual_A"]) for r in chosen])
    print(f"Mean |final commanded differential - raw LADRC prediction|: "
          f"{residual:.4f} A")
    print(f"Disturbance magnitude > feedback magnitude: "
          f"{100*sum(r['disturbance_dominates'] for r in chosen)/len(chosen):.1f}%")
    print(f"Yaw-error sign opposed by final differential command "
          f"(thresholded): "
          f"{100*sum(r['opposite_error_sign'] for r in chosen)/len(chosen):.1f}%")
    print("\n t[s]   yaw_err   wc*ESOerr    -hat_d  I_diff_pred I_diff_cmd  I_diff_act residual dist-dom%")
    for r in binned:
        print(f"{r['start_s']:5.2f} {r['yaw_error']:+8.3f} "
              f"{r['feedback_accel_rad_s2']:+11.3f} "
              f"{r['disturbance_accel_rad_s2']:+9.3f} "
              f"{r['predicted_diff_A']:+11.3f} "
              f"{r['commanded_diff_A']:+10.3f} "
              f"{r['measured_diff_A']:+11.3f} "
              f"{r['command_residual_A']:+8.3f} "
              f"{r['disturbance_dominates_pct']:8.1f}")
    output=(Path(args.output).expanduser() if args.output else
            src.with_name(src.stem+
                          f"_eso_balance_{args.start:g}_{args.end:g}.csv"))
    with output.open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=list(binned[0]))
        writer.writeheader()
        writer.writerows(binned)
    print(f"Saved: {output}")
    print("NOTE: Command reconstruction assumes identical wheel gains and no "
          "stop-acceleration feedforward. Mismatch may be from slew limit, "
          "floor, stale tracking states, and differing model parameters.")
    print("NOTE: z2 is an estimated TOTAL disturbance, not a separately "
          "measured physical disturbance or independently verified current.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
