#!/usr/bin/env python3
"""Validate offline PI-plant simulator CSVs; no ROS, no CAN."""
import csv
import math
from pathlib import Path
import sys

directory = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp")
count = 0
for friction_tag in ("0", "20", "40"):
    for mode in ("legacy", "bumpless", "three_stage"):
        path = directory / f"pi_{mode}_friction_{friction_tag}.csv"
        if not path.is_file():
            raise AssertionError(f"Missing simulator output: {path}")
        with path.open(newline="") as file:
            rows = list(csv.DictReader(file))
        assert len(rows) == 1000, (path, len(rows))
        for row in rows:
            assert row["mode"] == mode, path
            for key in ("time_s", "reference_rad_s", "speed_rad_s",
                        "command_current_a", "integral_a", "friction_a"):
                assert math.isfinite(float(row[key])), (path, key)
            assert abs(float(row["command_current_a"])) <= 4.0 + 1e-9, path
            if mode != "three_stage":
                assert row["transition"] == "0", path
        assert any(row["track"] == "1" for row in rows), path
        assert all(row["reference_rad_s"] == "0" for row in rows[:200]), path
        assert all(row["reference_rad_s"] == "0" for row in rows[600:]), path
        # Transition must never remain active beyond the 0.10s timeout.
        transition_rows = [row for row in rows if row["transition"] == "1"]
        if transition_rows:
            duration = (float(transition_rows[-1]["time_s"]) -
                        float(transition_rows[0]["time_s"]))
            assert duration <= 0.105, (path, duration)
        count += 1
print(f"PASS: validated {count} offline PI simulation CSV files")
