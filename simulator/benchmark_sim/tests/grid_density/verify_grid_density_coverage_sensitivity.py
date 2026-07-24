#!/usr/bin/env python3
"""Verify row counts for COVERAGE grid-density sensitivity outputs."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List

CSV_TYPES = ["system_performance.csv", "trial_summary.csv", "robot_performance.csv"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", default="/home/jlott/dcta_benchmark_sim/runs/sensitivity_grid_density_coverage_50")
    parser.add_argument("--combined", action="store_true", help="Also verify combined CSV totals.")
    return parser.parse_args()


def count_rows(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="") as f:
        return max(0, sum(1 for _ in f) - 1)


def read_manifest(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main() -> None:
    args = parse_args()
    run_root = Path(args.run_root).resolve()
    manifest_path = run_root / "condition_manifest.csv"
    rows = read_manifest(manifest_path)

    failures: List[str] = []
    expected_combined = {name: 0 for name in CSV_TYPES}

    for row in rows:
        out_dir = Path(row["out_dir"])
        expected_trials = int(row["num_trials"])
        expected_robot_rows = expected_trials * int(row["robot_count"])
        expected = {
            "system_performance.csv": expected_trials,
            "trial_summary.csv": expected_trials,
            "robot_performance.csv": expected_robot_rows,
        }
        for name, exp in expected.items():
            got = count_rows(out_dir / name)
            expected_combined[name] += exp
            if got != exp:
                failures.append(f"{row['condition_id']} {name}: got {got}, expected {exp}")

    print(f"[INFO] checked conditions: {len(rows)}")
    if failures:
        print(f"[FAIL] incomplete/mismatched raw outputs: {len(failures)}")
        missing_path = run_root / "coverage_grid_density_missing_outputs.csv"
        with missing_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["problem"])
            for item in failures:
                writer.writerow([item])
        print(f"[INFO] wrote details: {missing_path}")
        raise SystemExit(1)

    print("[OK] all raw outputs have expected row counts")

    if args.combined:
        combined_dir = run_root / "combined"
        for name, exp in expected_combined.items():
            got = count_rows(combined_dir / name)
            if got != exp:
                raise SystemExit(f"[FAIL] combined {name}: got {got}, expected {exp}")
            print(f"[OK] combined {name}: {got} rows")


if __name__ == "__main__":
    main()
