#!/usr/bin/env python3
"""
Run one partition of the COVERAGE grid-density sensitivity condition manifest.

Designed to be launched by run_grid_density_coverage_sensitivity.sh.
"""
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path
from typing import Dict, List


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--worker-index", type=int, required=True)
    parser.add_argument("--num-workers", type=int, required=True)
    parser.add_argument("--force", action="store_true", help="Rerun even if output CSV already appears complete.")
    return parser.parse_args()


def count_data_rows(csv_path: Path) -> int:
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        return 0
    with csv_path.open(newline="") as f:
        return max(0, sum(1 for _ in f) - 1)


def should_skip(row: Dict[str, str], force: bool) -> bool:
    if force:
        return False
    out_dir = Path(row["out_dir"])
    expected = int(row["num_trials"])
    expected_robot_rows = expected * int(row["robot_count"])
    system_csv = out_dir / "system_performance.csv"
    trial_csv = out_dir / "trial_summary.csv"
    robot_csv = out_dir / "robot_performance.csv"
    return (
        count_data_rows(system_csv) == expected
        and count_data_rows(trial_csv) == expected
        and count_data_rows(robot_csv) == expected_robot_rows
    )


def _append_optional(cmd: List[str], flag: str, value: str) -> None:
    if value is not None and str(value).strip() != "" and str(value).strip().lower() != "none":
        cmd.extend([flag, str(value)])


def build_command(row: Dict[str, str]) -> List[str]:
    cmd = [
        sys.executable,
        "-m",
        "benchmark_sim.run_trials",
        "--trial-mode", "coverage",
        "--num-trials", row["num_trials"],
        "--algorithm", row["algorithm_import"],
        "--algorithm-name", row["algorithm_name"],
        "--comm-model", row["comm_model"],
        "--comm-level", row["comm_level"],
        "--seed", row["seed"],
        "--out-dir", row["out_dir"],
        "--grid-size", row["grid_size"],
        "--num-robots", row["robot_count"],
        "--robot-start-layout", row["robot_start_layout"],
        "--condition-id", row["condition_id"],
        "--target-cells-per-robot", row["target_cells_per_robot"],
        "--actual-cells-per-robot", row["actual_cells_per_robot"],
        "--target-decay-exp", row["target_decay_exp"],
        "--commitment-horizon", row["commitment_horizon"],
        "--max-candidate-cells", row["max_candidate_cells"],
        "--no-parquet",
    ]

    # Kept for compatibility if you later decide to provide coverage scenario files.
    _append_optional(cmd, "--scenario-file", row.get("scenario_file", ""))
    return cmd


def main() -> None:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    manifest = Path(args.manifest).resolve()
    if args.num_workers <= 0:
        raise SystemExit("--num-workers must be positive")
    if not 0 <= args.worker_index < args.num_workers:
        raise SystemExit("--worker-index must be in [0, num-workers)")

    with manifest.open(newline="") as f:
        rows = list(csv.DictReader(f))

    selected = [row for i, row in enumerate(rows) if i % args.num_workers == args.worker_index]
    print(f"[WORKER {args.worker_index:02d}] selected {len(selected)} of {len(rows)} conditions")

    failed: List[str] = []
    for local_idx, row in enumerate(selected, start=1):
        out_dir = Path(row["out_dir"])
        out_dir.mkdir(parents=True, exist_ok=True)
        log_path = out_dir / "run.log"

        if should_skip(row, args.force):
            print(f"[WORKER {args.worker_index:02d}] skip complete {row['condition_id']} ({local_idx}/{len(selected)})")
            continue

        cmd = build_command(row)
        print(f"[WORKER {args.worker_index:02d}] run {row['condition_id']} ({local_idx}/{len(selected)})")
        for filename in (
            "system_performance.csv",
            "trial_summary.csv",
            "robot_performance.csv",
            "config_used.json",
        ):
            stale_path = out_dir / filename
            if stale_path.exists():
                stale_path.unlink()
        with log_path.open("w") as log:
            log.write("COMMAND:\n")
            log.write(" ".join(cmd) + "\n\n")
            log.flush()
            result = subprocess.run(cmd, cwd=str(repo_root), stdout=log, stderr=subprocess.STDOUT)

        if result.returncode != 0:
            print(
                f"[WORKER {args.worker_index:02d}] ERROR "
                f"{row['condition_id']} returncode={result.returncode}",
                file=sys.stderr,
            )
            print(f"[WORKER {args.worker_index:02d}] see log: {log_path}", file=sys.stderr)
            failed.append(row["condition_id"])

    if failed:
        print(
            f"[WORKER {args.worker_index:02d}] failed conditions: {','.join(failed)}",
            file=sys.stderr,
        )
        raise SystemExit(1)
    print(f"[WORKER {args.worker_index:02d}] done")


if __name__ == "__main__":
    main()
