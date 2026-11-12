#!/usr/bin/env python3
"""Analyze existing nine virtual-wheel traces without ROS or CAN.

Separates transition flag lifetime from nonzero influence on actual current.
Outputs per-friction diagnostics and a JSON file for reproducibility.
"""
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/pi_three_stage_sim")
REFERENCE = 0.05 / 0.064
CUTOFF = 0.8 * REFERENCE


def load(mode, tag):
    path = ROOT / f"pi_{mode}_friction_{tag}.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        records = list(csv.DictReader(stream))
    if len(records) != 1000:
        raise RuntimeError(f"{path}: expected 1000 rows, got {len(records)}")
    return records


def val(row, field):
    result = float(row[field])
    if not math.isfinite(result):
        raise RuntimeError(f"non-finite {field}")
    return result


def diagnostic(tag):
    legacy = load("legacy", tag)
    three = load("three_stage", tag)
    starts = [
        i for i, row in enumerate(three)
        if row["transition"] == "1" and
        (i == 0 or three[i - 1]["transition"] != "1")
    ]
    if len(starts) != 1:
        raise RuntimeError(f"friction {tag}: expected one transition, got {len(starts)}")
    start = starts[0]
    indices = [
        i for i, row in enumerate(three) if row["transition"] == "1"
    ]
    if not indices or indices[0] != start:
        raise RuntimeError("unexpected transition trace")
    end_exclusive = indices[-1] + 1
    start_speed = val(three[start], "speed_rad_s")
    start_time = val(three[start], "time_s")
    # A transition flag means the state is enabled; it does not prove the
    # transition actually changed current. Compare final commands to legacy.
    differences = [
        abs(val(r3, "command_current_a") -
            val(r0, "command_current_a"))
        for r3, r0 in zip(three, legacy)
    ]
    nonzero_indices = [
        i for i, value in enumerate(differences) if value > 1.0e-8
    ]
    overlap = [i for i in indices if differences[i] > 1.0e-8]
    last_track = end_exclusive
    first_off = three[last_track] if last_track < len(three) else None
    return {
        "friction_A": int(tag) / 100,
        "transition_start_s": start_time,
        "transition_start_speed_rad_s": start_speed,
        "transition_start_speed_ratio": start_speed / REFERENCE,
        "cutoff_speed_rad_s": CUTOFF,
        "cutoff_already_reached_at_start": start_speed >= CUTOFF,
        "active_flag_samples": len(indices),
        "active_flag_duration_s": len(indices) * 0.005,
        "first_inactive_s": val(first_off, "time_s") if first_off else None,
        "first_inactive_speed_rad_s": val(first_off, "speed_rad_s")
            if first_off else None,
        "max_abs_current_difference_from_legacy_A":
            max(differences),
        "nonzero_current_difference_samples_total":
            len(nonzero_indices),
        "nonzero_current_difference_samples_while_flag_active":
            len(overlap),
        "first_nonzero_difference_s":
            val(three[nonzero_indices[0]], "time_s") if nonzero_indices else None,
        "peak_three_stage_speed_rad_s":
            max(abs(val(row, "speed_rad_s")) for row in three),
    }


def main():
    results = [diagnostic(tag) for tag in ("0", "20", "40")]
    print("friction_A,start_s,start_speed_rad_s,start_ratio,"
          "flag_duration_s,max_command_delta_A,"
          "nonzero_command_delta_samples,cutoff_at_start")
    for item in results:
        print(
            f'{item["friction_A"]:.1f},'
            f'{item["transition_start_s"]:.3f},'
            f'{item["transition_start_speed_rad_s"]:.5f},'
            f'{item["transition_start_speed_ratio"]:.3f},'
            f'{item["active_flag_duration_s"]:.3f},'
            f'{item["max_abs_current_difference_from_legacy_A"]:.7f},'
            f'{item["nonzero_current_difference_samples_total"]},'
            f'{item["cutoff_already_reached_at_start"]}'
        )
    output = ROOT / "transition_diagnostics.json"
    output.write_text(
        json.dumps(results, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
