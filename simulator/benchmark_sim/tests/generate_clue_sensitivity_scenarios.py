#!/usr/bin/env python3
"""Generate a reproducible clue-search tuning set independent of final evaluation data."""
from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path
from typing import List, Sequence, Tuple

from benchmark_sim.config import edge_even_start_positions, generate_robot_ids

Cell = Tuple[int, int]


def weighted_sample_without_replacement(
    rng: random.Random,
    items: Sequence[Cell],
    weights: Sequence[float],
    count: int,
) -> List[Cell]:
    available = list(items)
    available_weights = list(weights)
    selected: List[Cell] = []

    for _ in range(min(count, len(available))):
        total = sum(available_weights)
        if total <= 0.0:
            index = rng.randrange(len(available))
        else:
            threshold = rng.random() * total
            cumulative = 0.0
            index = len(available) - 1
            for candidate_index, weight in enumerate(available_weights):
                cumulative += weight
                if cumulative >= threshold:
                    index = candidate_index
                    break
        selected.append(available.pop(index))
        available_weights.pop(index)

    return selected


def generate_scenario_file(
    output: Path,
    *,
    grid_size: int,
    num_trials: int,
    num_clues: int,
    num_robots: int,
    seed: int,
    target_decay_exp: float,
) -> None:
    rng = random.Random(seed)
    robot_ids = generate_robot_ids(num_robots)
    reserved_starts = set(edge_even_start_positions(grid_size, robot_ids).values())
    cells = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if (x, y) not in reserved_starts
    ]
    if len(cells) < num_clues + 1:
        raise ValueError("grid does not contain enough non-start cells for the target and clues")

    output.parent.mkdir(parents=True, exist_ok=True)
    header = ["trial_id", "episode", "object_x", "object_y", "target_x", "target_y"]
    for clue_index in range(1, num_clues + 1):
        header.extend([f"clue{clue_index}_x", f"clue{clue_index}_y"])

    with output.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# scenario_set=independent_sensitivity_tuning\n")
        handle.write(f"# seed={seed}\n")
        handle.write(f"# grid_size={grid_size}\n")
        handle.write(f"# num_trials={num_trials}\n")
        handle.write(f"# num_clues={num_clues}\n")
        handle.write(f"# num_robots={num_robots}\n")
        handle.write(f"# clue_weight=1/((1+manhattan_distance)**{target_decay_exp})\n")
        writer = csv.writer(handle)
        writer.writerow(header)

        for trial_id in range(num_trials):
            target = rng.choice(cells)
            clue_candidates = [cell for cell in cells if cell != target]
            weights = [
                1.0
                / (
                    (1.0 + abs(cell[0] - target[0]) + abs(cell[1] - target[1]))
                    ** target_decay_exp
                )
                for cell in clue_candidates
            ]
            clues = weighted_sample_without_replacement(
                rng,
                clue_candidates,
                weights,
                num_clues,
            )

            row: List[object] = [
                trial_id,
                trial_id,
                target[0],
                target[1],
                target[0],
                target[1],
            ]
            for clue in clues:
                row.extend([clue[0], clue[1]])
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--grid-size", type=int, default=19)
    parser.add_argument("--num-trials", type=int, default=300)
    parser.add_argument("--num-clues", type=int, default=4)
    parser.add_argument("--num-robots", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20260702)
    parser.add_argument("--target-decay-exp", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    generate_scenario_file(
        args.output,
        grid_size=args.grid_size,
        num_trials=args.num_trials,
        num_clues=args.num_clues,
        num_robots=args.num_robots,
        seed=args.seed,
        target_decay_exp=args.target_decay_exp,
    )
    print(f"wrote {args.num_trials} independent sensitivity scenarios to {args.output}")


if __name__ == "__main__":
    main()
