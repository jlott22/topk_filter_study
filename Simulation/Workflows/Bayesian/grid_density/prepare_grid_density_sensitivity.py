#!/usr/bin/env python3
"""
Prepare scenario files and a condition manifest for the grid-density saturation sensitivity study.

Designed to be saved in:
  /home/jlott/dcta_benchmark_sim/benchmark_sim/tests/grid_density/prepare_grid_density_sensitivity.py

Run indirectly through run_grid_density_sensitivity.sh, or directly from repo root:
  python3 benchmark_sim/tests/grid_density/prepare_grid_density_sensitivity.py --repo-root /home/jlott/dcta_benchmark_sim
"""
from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

Cell = Tuple[int, int]

GRIDS: List[int] = [14, 19, 25, 34, 48]
TARGET_DENSITIES: List[int] = [220, 180, 140, 110, 85, 65, 50]

# Exact planned robot counts. Columns are TARGET_DENSITIES in order.
ROBOT_COUNTS: Dict[int, Dict[int, int]] = {
    14: {220: 1, 180: 1, 140: 2, 110: 2, 85: 3, 65: 3, 50: 4},
    19: {220: 2, 180: 2, 140: 3, 110: 4, 85: 5, 65: 6, 50: 8},
    25: {220: 3, 180: 4, 140: 5, 110: 6, 85: 8, 65: 10, 50: 13},
    34: {220: 6, 180: 7, 140: 9, 110: 11, 85: 14, 65: 18, 50: 24},
    48: {220: 11, 180: 13, 140: 17, 110: 21, 85: 28, 65: 36, 50: 46},
}

COMM_ENVS = [
    # label, comm_model, comm_level
    ("ideal", "ideal", "1.0"),
    ("bernoulli_drop_0_10", "bernoulli", "0.1"),
    ("gilbert_elliot_0_90", "gilbert_elliot", "0.9"),
    ("rayleigh_sens_-59_4", "rayleigh_style", "-59.4"),
]

ALGORITHMS = [
    # cli_name, import_path, paper/output name
    ("cbaa", "benchmark_sim.algorithms.CBAA:CBAAAllocator", "CBAA"),
    ("acbba", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator", "ACBBA"),
    ("dmchba", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator", "DMCHBA"),
    ("hipc", "benchmark_sim.algorithms.HIPC:HIPCAllocator", "HIPC"),
    ("pi", "benchmark_sim.algorithms.PI:PIAllocator", "PI"),
    ("dga", "benchmark_sim.algorithms.DGA:DGAAllocator", "DGA"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default="/home/jlott/dcta_benchmark_sim")
    parser.add_argument("--run-root", default=None, help="Default: <repo-root>/runs/sensitivity_grid_density_50")
    parser.add_argument("--num-trials", type=int, default=50)
    parser.add_argument("--num-clues", type=int, default=4)
    parser.add_argument("--scenario-seed", type=int, default=20260630)
    parser.add_argument("--sim-seed-base", type=int, default=700000)
    parser.add_argument("--target-decay-exp", type=float, default=1.0)
    return parser.parse_args()


def weighted_sample_without_replacement(
    rng: random.Random,
    items: Sequence[Cell],
    weights: Sequence[float],
    k: int,
) -> List[Cell]:
    available = list(items)
    available_weights = list(weights)
    selected: List[Cell] = []

    for _ in range(min(k, len(available))):
        total = sum(available_weights)
        if total <= 0:
            idx = rng.randrange(len(available))
        else:
            r = rng.random() * total
            acc = 0.0
            idx = len(available) - 1
            for i, w in enumerate(available_weights):
                acc += w
                if acc >= r:
                    idx = i
                    break
        selected.append(available.pop(idx))
        available_weights.pop(idx)

    return selected


def planned_start_cells(grid_size: int) -> set[Cell]:
    """Return every edge_even start used by any planned density at this grid size."""
    starts: set[Cell] = set()
    for robot_count in set(ROBOT_COUNTS[grid_size].values()):
        if robot_count == 1:
            starts.add((0, (grid_size - 1) // 2))
            continue
        for index in range(robot_count):
            y = round(index * (grid_size - 1) / (robot_count - 1))
            starts.add((0, y))
    return starts


def generate_scenarios_for_grid(
    grid_size: int,
    num_trials: int,
    num_clues: int,
    seed: int,
    out_path: Path,
    target_decay_exp: float,
) -> None:
    rng = random.Random(seed + grid_size * 1009)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    header = ["trial_id", "episode", "object_x", "object_y", "target_x", "target_y"]
    for i in range(1, num_clues + 1):
        header.extend([f"clue{i}_x", f"clue{i}_y"])

    all_cells = [(x, y) for y in range(grid_size) for x in range(grid_size)]
    # RobotShell marks start cells searched during initialization but does not sense
    # clues or targets there. Reserving every planned start keeps all scenario items
    # observable for every robot-density condition sharing this scenario file.
    reserved_starts = planned_start_cells(grid_size)
    scenario_cells = [cell for cell in all_cells if cell not in reserved_starts]

    with out_path.open("w", newline="") as f:
        f.write(f"# scenario_set=grid_density_sensitivity\n")
        f.write(f"# grid_size={grid_size}\n")
        f.write(f"# num_trials={num_trials}\n")
        f.write(f"# num_clues={num_clues}\n")
        f.write(f"# clue_weight=1/((1+manhattan_distance)**{target_decay_exp})\n")
        writer = csv.writer(f)
        writer.writerow(header)

        for trial_id in range(num_trials):
            target = rng.choice(scenario_cells)
            candidate_cells = [c for c in scenario_cells if c != target]
            weights = []
            for cell in candidate_cells:
                dist = abs(cell[0] - target[0]) + abs(cell[1] - target[1])
                weights.append(1.0 / ((1.0 + dist) ** target_decay_exp))
            clues = weighted_sample_without_replacement(rng, candidate_cells, weights, num_clues)

            row: List[object] = [trial_id, trial_id, target[0], target[1], target[0], target[1]]
            for clue in clues:
                row.extend([clue[0], clue[1]])
            writer.writerow(row)


def main() -> None:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    run_root = Path(args.run_root).resolve() if args.run_root else repo_root / "runs" / "sensitivity_grid_density_50"
    scenario_dir = run_root / "scenarios"
    raw_dir = run_root / "raw"
    combined_dir = run_root / "combined"

    scenario_dir.mkdir(parents=True, exist_ok=True)
    raw_dir.mkdir(parents=True, exist_ok=True)
    combined_dir.mkdir(parents=True, exist_ok=True)

    scenario_files: Dict[int, Path] = {}
    for grid_size in GRIDS:
        scenario_path = scenario_dir / f"scaling_grid{grid_size}_trials{args.num_trials}.csv"
        generate_scenarios_for_grid(
            grid_size=grid_size,
            num_trials=args.num_trials,
            num_clues=args.num_clues,
            seed=args.scenario_seed,
            out_path=scenario_path,
            target_decay_exp=args.target_decay_exp,
        )
        scenario_files[grid_size] = scenario_path

    manifest_path = run_root / "condition_manifest.csv"
    header = [
        "condition_index",
        "condition_id",
        "grid_size",
        "grid_cells",
        "target_cells_per_robot",
        "robot_count",
        "actual_cells_per_robot",
        "density_error",
        "density_ratio",
        "comm_label",
        "comm_model",
        "comm_level",
        "algorithm_cli",
        "algorithm_import",
        "algorithm_name",
        "scenario_file",
        "num_trials",
        "out_dir",
        "seed",
        "target_decay_exp",
        "commitment_horizon",
        "max_candidate_cells",
        "robot_start_layout",
    ]

    rows: List[Dict[str, object]] = []
    condition_index = 0
    for grid_i, grid_size in enumerate(GRIDS):
        grid_cells = grid_size * grid_size
        for dens_i, target_density in enumerate(TARGET_DENSITIES):
            robot_count = ROBOT_COUNTS[grid_size][target_density]
            actual_density = grid_cells / float(robot_count)
            density_error = actual_density - float(target_density)
            density_ratio = actual_density / float(target_density)
            for comm_i, (comm_label, comm_model, comm_level) in enumerate(COMM_ENVS):
                # Same seed across algorithms for the same grid/density/comm condition.
                # This keeps non-scenario randomness as paired as practical.
                sim_seed = args.sim_seed_base + grid_i * 10000 + dens_i * 1000 + comm_i * 100
                for alg_cli, alg_import, alg_name in ALGORITHMS:
                    condition_id = f"g{grid_size}_d{target_density}_{comm_label}_{alg_cli}"
                    out_dir = raw_dir / f"grid{grid_size}" / f"density{target_density}" / comm_label / alg_cli
                    rows.append({
                        "condition_index": condition_index,
                        "condition_id": condition_id,
                        "grid_size": grid_size,
                        "grid_cells": grid_cells,
                        "target_cells_per_robot": target_density,
                        "robot_count": robot_count,
                        "actual_cells_per_robot": f"{actual_density:.6f}",
                        "density_error": f"{density_error:.6f}",
                        "density_ratio": f"{density_ratio:.6f}",
                        "comm_label": comm_label,
                        "comm_model": comm_model,
                        "comm_level": comm_level,
                        "algorithm_cli": alg_cli,
                        "algorithm_import": alg_import,
                        "algorithm_name": alg_name,
                        "scenario_file": str(scenario_files[grid_size]),
                        "num_trials": args.num_trials,
                        "out_dir": str(out_dir),
                        "seed": sim_seed,
                        "target_decay_exp": args.target_decay_exp,
                        "commitment_horizon": 3,
                        "max_candidate_cells": "all",
                        "robot_start_layout": "edge_even",
                    })
                    condition_index += 1

    with manifest_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[OK] wrote scenarios: {scenario_dir}")
    print(f"[OK] wrote condition manifest: {manifest_path} ({len(rows)} conditions)")
    print(f"[INFO] expected runs: {len(rows) * args.num_trials}")


if __name__ == "__main__":
    main()
