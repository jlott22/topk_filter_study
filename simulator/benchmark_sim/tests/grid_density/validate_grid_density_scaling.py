from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path


ALGORITHMS = {
    "cbaa": "benchmark_sim.algorithms.CBAA:CBAAAllocator",
    "acbba": "benchmark_sim.algorithms.ACBBA:ACBBAAllocator",
    "pi": "benchmark_sim.algorithms.PI:PIAllocator",
    "hipc": "benchmark_sim.algorithms.HIPC:HIPCAllocator",
    "dmchba": "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator",
    "dga": "benchmark_sim.algorithms.DGA:DGAAllocator",
}

CONDITIONS = (
    {"grid_size": 14, "robot_count": 1, "target": (1, 6), "target_cpr": 196.0},
    {"grid_size": 48, "robot_count": 46, "target": (1, 0), "target_cpr": 50.0},
)

METADATA_COLUMNS = {
    "trial_id",
    "algorithm",
    "comm_model",
    "comm_level",
    "grid_size",
    "grid_cells",
    "robot_count",
    "target_cells_per_robot",
    "actual_cells_per_robot",
    "condition_id",
    "scenario_file",
}


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def validate_run(out_dir: Path, robot_count: int) -> None:
    for filename in ("system_performance.csv", "trial_summary.csv", "robot_performance.csv"):
        path = out_dir / filename
        if not path.is_file():
            raise AssertionError(f"missing output: {path}")
        fieldnames, rows = read_rows(path)
        missing = METADATA_COLUMNS.difference(fieldnames)
        if missing:
            raise AssertionError(f"{path} missing metadata columns: {sorted(missing)}")
        expected_rows = robot_count if filename == "robot_performance.csv" else 1
        if len(rows) != expected_rows:
            raise AssertionError(f"{path} has {len(rows)} rows; expected {expected_rows}")

    _, robot_rows = read_rows(out_dir / "robot_performance.csv")
    expected_ids = [f"{index:02d}" for index in range(robot_count)]
    actual_ids = [row["robot_id"] for row in robot_rows]
    if actual_ids != expected_ids:
        raise AssertionError(f"unexpected robot IDs in {out_dir}: {actual_ids}")


def run_validation(output_root: Path) -> None:
    package_dir = Path(__file__).resolve().parents[2]
    repo_root = package_dir.parent
    output_root.mkdir(parents=True, exist_ok=True)

    for condition in CONDITIONS:
        grid_size = condition["grid_size"]
        robot_count = condition["robot_count"]
        condition_id = f"g{grid_size}_r{robot_count}"
        actual_cpr = grid_size * grid_size / robot_count
        scenario_path = output_root / f"scenario_{condition_id}.json"
        scenario_path.write_text(json.dumps([{
            "trial_id": 0,
            "target": list(condition["target"]),
            "clues": [list(condition["target"])],
        }]))

        for algorithm_name, algorithm_spec in ALGORITHMS.items():
            out_dir = output_root / condition_id / algorithm_name
            command = [
                sys.executable,
                "-m",
                "benchmark_sim.run_trials",
                "--scenario-file",
                str(scenario_path.resolve()),
                "--algorithm",
                algorithm_spec,
                "--algorithm-name",
                algorithm_name,
                "--comm-model",
                "ideal",
                "--max-trials",
                "1",
                "--grid-size",
                str(grid_size),
                "--num-robots",
                str(robot_count),
                "--robot-start-layout",
                "edge_even",
                "--condition-id",
                condition_id,
                "--target-cells-per-robot",
                str(condition["target_cpr"]),
                "--actual-cells-per-robot",
                str(actual_cpr),
                "--out-dir",
                str(out_dir.resolve()),
            ]
            subprocess.run(command, cwd=repo_root, check=True)
            validate_run(out_dir, robot_count)
            print(f"validated {condition_id}/{algorithm_name}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the quick grid-density scaling validation matrix.")
    parser.add_argument("--out-dir", type=Path, default=Path("runs/grid_density_validation"))
    args = parser.parse_args()
    run_validation(args.out_dir.resolve())


if __name__ == "__main__":
    main()
