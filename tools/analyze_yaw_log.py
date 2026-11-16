#!/usr/bin/env python3
"""Summarize recorded UAV yaw telemetry. This is NOT a flight-safety validator.

Usage: python3 tools/analyze_yaw_log.py /path/to/uav_YYYYMMDD_HHMMSS.csv
"""
import csv
import math
import statistics
import sys
from pathlib import Path


def num(row, key):
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def pct(data, percent):
    if not data:
        return float("nan")
    values = sorted(data)
    return values[round((len(values) - 1) * percent / 100)]


def main(path):
    total = motor_on = rail = pid_sat = neutral_count = 0
    abs_rates = []
    rate_values = []
    diff_command = []
    for row in csv.DictReader(path.open(newline="", encoding="utf-8-sig")):
        total += 1
        active = row.get("spinning", "").lower() == "true"
        outputs = [num(row, f"out_m{i}") for i in range(1, 5)]
        if not active or any(v is None for v in outputs) or max(outputs) <= 0:
            continue
        motor_on += 1
        yaw = num(row, "gyro_yaw_dps")
        corr = num(row, "pid_yaw")
        if yaw is None:
            continue
        abs_rates.append(abs(yaw))
        rate_values.append(yaw)
        if abs(yaw) >= 490:
            rail += 1
        if corr is not None and abs(corr) >= 190:
            pid_sat += 1
        cmd_diff = (outputs[0] + outputs[3] - outputs[1] - outputs[2]) / 4
        diff_command.append(cmd_diff)
        rc_yaw = num(row, "rc_yaw")
        if rc_yaw is not None and abs(rc_yaw - 1092.0) <= 15:
            neutral_count += 1
    print(f"File: {path.name}")
    print(f"All samples: {total}, active motor samples: {motor_on}")
    if not abs_rates:
        print("No powered yaw samples available; motor-off manual rotation cannot validate torque response.")
        return
    print(f"Yaw rate: min={min(rate_values):.1f}, max={max(rate_values):.1f} deg/s")
    print(f"|yaw| P95={pct(abs_rates, 95):.1f}, max={max(abs_rates):.1f} deg/s")
    print(f"Samples near OLD +/-500 deg/s gyro range: {rail} ({100*rail/len(abs_rates):.1f}%)")
    print(f"Samples near PID +/-200 output rail: {pid_sat} ({100*pid_sat/len(abs_rates):.1f}%)")
    print(f"Yaw stick approximately neutral: {neutral_count}/{len(abs_rates)}")
    print("Motor command pair difference = (M1 + M4 - M2 - M3)/4,")
    print(f"  measured from TELEMETRY, median={statistics.median(diff_command):.1f} DShot.")
    print("NOTE: command does not measure physical torque, RPM, ESC sync, or real yaw direction.")
    print("This tool must not be used to declare the UAV safe for free flight.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 tools/analyze_yaw_log.py path/to/uav_log.csv")
    file_path = Path(sys.argv[1])
    if not file_path.is_file():
        raise SystemExit(f"Missing file: {file_path}")
    main(file_path)
