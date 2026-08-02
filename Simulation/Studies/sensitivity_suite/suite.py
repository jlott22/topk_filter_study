from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
import queue
import random
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "Simulation" / "Architecture" / "simulator"
KNOWN_VISIT_ROOT = REPO_ROOT / "Simulation" / "dcta_benchmark_sim"
DEFAULT_RUN_ROOT = REPO_ROOT / "Results" / "Simulation" / "Sensitivity"

TRIALS_PER_CONDITION = 50
MULTITARGET_TRIALS_PER_CONDITION = 100
DEFAULT_WORKERS = 12
DEBUG_MAX_EVENTS = 20_000
COMMITMENT_HORIZON = 3
CLUE_COUNT = 4
TARGET_COUNT = 50
TARGET_DECAY_EXP = 1.0
SCENARIO_SEED = 20260726
SIM_SEED_BASE = 820_000
RAYLEIGH_SENSITIVITY_DBM = -50.63
TOP_K_RATES = (1.0, 0.75, 0.50, 0.25, 0.10, 0.05)

ALGORITHMS = (
    ("cbaa", "CBAA", "benchmark_sim.algorithms.CBAA:CBAAAllocator",
     "known_visit_sim.algorithms.CBAA:CBAAAllocator", 10),
    ("acbba", "ACBBA", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator",
     "known_visit_sim.algorithms.ACBBA:ACBBAAllocator", 25),
    ("pi", "PI", "benchmark_sim.algorithms.PI:PIAllocator",
     "known_visit_sim.algorithms.PI:PIAllocator", 25),
    ("hipc", "HIPC", "benchmark_sim.algorithms.HIPC:HIPCAllocator",
     "known_visit_sim.algorithms.HIPC:HIPCAllocator", 30),
    ("dmchba", "DMCHBA", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator",
     "known_visit_sim.algorithms.DMCHBA:DMCHBAAllocator", 80),
    ("dga", "DGA", "benchmark_sim.algorithms.DGA:DGAAllocator",
     "known_visit_sim.algorithms.DGA:DGAAllocator", 100),
)

EXPECTED_CONDITIONS = 324
CAMPAIGN_RECORDS_DIR = "campaign_records"


@dataclass(frozen=True)
class Environment:
    suite: str
    key: str
    runner: str
    grid_size: int
    robot_count: int
    scenario_name: str
    pair_key: str
    comm_label: str = "ideal"
    comm_model: str = "ideal"
    comm_level: str = ""
    target_count: int = 1
    trials_per_condition: int = TRIALS_PER_CONDITION


def campaign_records_dir(run_root: Path) -> Path:
    return run_root / CAMPAIGN_RECORDS_DIR


def raw_environment_dir(run_root: Path, environment: Environment) -> Path:
    raw_root = run_root / "raw"
    if environment.runner == "known_visit":
        return (
            raw_root
            / "collaborative_known_target_visit"
            / "topk_sensitivity"
            / environment.key
        )
    return raw_root / "bayesian_clue_search" / environment.suite / environment.key


SCALE_ENVIRONMENTS = (
    Environment("scale", "scale_g19_r2", "clue", 19, 2, "clue_g19_n50.csv", "clue_g19"),
    Environment("scale", "scale_g19_r4", "clue", 19, 4, "clue_g19_n50.csv", "clue_g19"),
    Environment("scale", "scale_g19_r8", "clue", 19, 8, "clue_g19_n50.csv", "clue_g19"),
    Environment("scale", "scale_g14_r4", "clue", 14, 4, "clue_g14_n50.csv", "clue_g14"),
    Environment("scale", "scale_g28_r4", "clue", 28, 4, "clue_g28_n50.csv", "clue_g28"),
)

COMM_ENVIRONMENTS = (
    Environment(
        "communication", "comm_g19_r4_bernoulli_drop025", "clue", 19, 4,
        "clue_g19_n50.csv", "clue_g19", "bernoulli_drop025", "bernoulli", "0.25",
    ),
    Environment(
        "communication", "comm_g19_r4_ge_drop025", "clue", 19, 4,
        "clue_g19_n50.csv", "clue_g19", "ge_drop025_rho08", "gilbert_elliot", "0.75",
    ),
    Environment(
        "communication", "comm_g19_r4_rayleigh_drop025", "clue", 19, 4,
        "clue_g19_n50.csv", "clue_g19", "rayleigh_drop025", "rayleigh_style",
        str(RAYLEIGH_SENSITIVITY_DBM),
    ),
)

MULTITARGET_ENVIRONMENTS = (
    Environment(
        "multitarget", "multitarget_g19_r4_t50", "known_visit", 19, 4,
        "known_targets_g19_t50_n100.csv", "known_targets_g19_t50",
        target_count=TARGET_COUNT,
        trials_per_condition=MULTITARGET_TRIALS_PER_CONDITION,
    ),
)

ALL_ENVIRONMENTS = SCALE_ENVIRONMENTS + COMM_ENVIRONMENTS + MULTITARGET_ENVIRONMENTS
EXPECTED_TRIALS = sum(
    environment.trials_per_condition for environment in ALL_ENVIRONMENTS
) * len(ALGORITHMS) * len(TOP_K_RATES)

MANIFEST_FIELDS = (
    "condition_index", "condition_id", "suite", "environment", "runner",
    "grid_size", "grid_cells", "robot_count", "target_count", "pair_key",
    "algorithm_key", "algorithm_name", "algorithm_import", "top_k_rate",
    "top_k_max_cells", "top_k_basis", "comm_label", "comm_model", "comm_level",
    "scenario_file", "scenario_sha256", "seed", "num_trials", "out_dir",
    "working_directory", "weight", "command",
)


def round_half_up(value: float) -> int:
    return max(1, int(math.floor(float(value) + 0.5)))


def top_k_limit(grid_size: int, rate: float, runner: str) -> int:
    reference = TARGET_COUNT if runner == "known_visit" else grid_size * grid_size
    return round_half_up(reference * rate)


def top_k_label(rate: float) -> str:
    return {
        1.0: "100",
        0.75: "075",
        0.50: "050",
        0.25: "025",
        0.10: "010",
        0.05: "005",
    }[rate]


def edge_even_starts(grid_size: int, robot_count: int) -> set[tuple[int, int]]:
    if grid_size <= 0 or not 1 <= robot_count <= grid_size:
        raise ValueError("edge-even starts require 1 <= robot_count <= grid_size")
    if robot_count == 1:
        return {(0, (grid_size - 1) // 2)}
    return {
        (0, round(index * (grid_size - 1) / (robot_count - 1)))
        for index in range(robot_count)
    }


def weighted_sample_without_replacement(
    rng: random.Random,
    cells: Sequence[tuple[int, int]],
    weights: Sequence[float],
    count: int,
) -> list[tuple[int, int]]:
    available = list(cells)
    available_weights = list(weights)
    selected: list[tuple[int, int]] = []
    for _ in range(count):
        total = sum(available_weights)
        threshold = rng.random() * total if total > 0.0 else 0.0
        cumulative = 0.0
        chosen = len(available) - 1
        for index, weight in enumerate(available_weights):
            cumulative += weight
            if cumulative >= threshold:
                chosen = index
                break
        selected.append(available.pop(chosen))
        available_weights.pop(chosen)
    return selected


def write_clue_scenarios(
    path: Path,
    grid_size: int,
    reserved_robot_counts: Sequence[int],
    seed: int,
) -> None:
    reserved = set().union(
        *(edge_even_starts(grid_size, count) for count in reserved_robot_counts)
    )
    eligible = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if (x, y) not in reserved
    ]
    if len(eligible) < CLUE_COUNT + 1:
        raise ValueError(f"grid {grid_size} has insufficient eligible cells")
    rng = random.Random(seed)
    header = [
        "trial_id", "episode", "object_x", "object_y", "target_x", "target_y",
    ]
    for clue_index in range(1, CLUE_COUNT + 1):
        header.extend((f"clue{clue_index}_x", f"clue{clue_index}_y"))

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# scenario_set=topk_readonly_sensitivity\n")
        handle.write(f"# seed={seed}\n")
        handle.write(f"# grid_size={grid_size}\n")
        handle.write(f"# num_trials={TRIALS_PER_CONDITION}\n")
        handle.write(f"# reserved_robot_counts={','.join(map(str, reserved_robot_counts))}\n")
        handle.write(f"# target_decay_exp={TARGET_DECAY_EXP}\n")
        writer = csv.writer(handle)
        writer.writerow(header)
        for trial_id in range(TRIALS_PER_CONDITION):
            target = rng.choice(eligible)
            candidates = [cell for cell in eligible if cell != target]
            weights = [
                1.0 / (
                    (1.0 + abs(cell[0] - target[0]) + abs(cell[1] - target[1]))
                    ** TARGET_DECAY_EXP
                )
                for cell in candidates
            ]
            clues = weighted_sample_without_replacement(
                rng, candidates, weights, CLUE_COUNT
            )
            row: list[object] = [
                trial_id, trial_id, target[0], target[1], target[0], target[1],
            ]
            for clue in clues:
                row.extend(clue)
            writer.writerow(row)


def write_known_target_scenarios(
    path: Path,
    seed: int,
    trial_count: int = MULTITARGET_TRIALS_PER_CONDITION,
) -> None:
    starts = edge_even_starts(19, 4)
    eligible = [
        (x, y)
        for y in range(19)
        for x in range(19)
        if (x, y) not in starts
    ]
    rng = random.Random(seed)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ["trial_id"]
    for target_index in range(1, TARGET_COUNT + 1):
        header.extend((f"target{target_index}_x", f"target{target_index}_y"))
    with path.open("w", newline="", encoding="utf-8") as handle:
        handle.write(
            f"# grid_size=19, num_targets={TARGET_COUNT}, num_robots=4, "
            f"layout=edge_even, seed={seed}\n"
        )
        writer = csv.writer(handle)
        writer.writerow(header)
        for trial_id in range(trial_count):
            targets = rng.sample(eligible, TARGET_COUNT)
            writer.writerow([trial_id, *[value for cell in targets for value in cell]])


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(
            line for line in handle if line.strip() and not line.lstrip().startswith("#")
        ))


def validate_scenario_files(scenario_dir: Path) -> None:
    specs = (
        ("clue_g19_n50.csv", 19, (2, 4, 8), "clue", TRIALS_PER_CONDITION),
        ("clue_g14_n50.csv", 14, (4,), "clue", TRIALS_PER_CONDITION),
        ("clue_g28_n50.csv", 28, (4,), "clue", TRIALS_PER_CONDITION),
        (
            "known_targets_g19_t50_n100.csv",
            19,
            (4,),
            "known",
            MULTITARGET_TRIALS_PER_CONDITION,
        ),
    )
    for filename, grid_size, robot_counts, kind, expected_rows in specs:
        rows = read_csv_rows(scenario_dir / filename)
        if len(rows) != expected_rows:
            raise RuntimeError(
                f"{filename}: expected {expected_rows} rows, found {len(rows)}"
            )
        reserved = set().union(
            *(edge_even_starts(grid_size, count) for count in robot_counts)
        )
        for row in rows:
            if kind == "clue":
                target = (int(row["target_x"]), int(row["target_y"]))
                clues = [
                    (int(row[f"clue{i}_x"]), int(row[f"clue{i}_y"]))
                    for i in range(1, CLUE_COUNT + 1)
                ]
                cells = [target, *clues]
                if len(set(cells)) != len(cells):
                    raise RuntimeError(f"{filename}: duplicate target/clue cell")
            else:
                cells = [
                    (int(row[f"target{i}_x"]), int(row[f"target{i}_y"]))
                    for i in range(1, TARGET_COUNT + 1)
                ]
                if len(set(cells)) != TARGET_COUNT:
                    raise RuntimeError(f"{filename}: duplicate known target")
            if any(cell in reserved for cell in cells):
                raise RuntimeError(f"{filename}: scenario overlaps a reserved start")
            if any(not (0 <= x < grid_size and 0 <= y < grid_size) for x, y in cells):
                raise RuntimeError(f"{filename}: out-of-bounds scenario cell")


def rayleigh_calibration(samples_per_distance: int = 20_000) -> dict[str, object]:
    if str(SIMULATOR_ROOT) not in sys.path:
        sys.path.insert(0, str(SIMULATOR_ROOT))
    from benchmark_sim.comms.message import Message
    from benchmark_sim.comms.models import make_comm_model

    model = make_comm_model("rayleigh_style", RAYLEIGH_SENSITIVITY_DBM)
    rng = random.Random(SCENARIO_SEED + 991)
    message = Message("00", "robot/00/state", {}, 0.0)
    receiver_positions = ((1, 0), (5, 0), (10, 0), (18, 0), (18, 18))
    drops = attempts = 0
    by_distance: list[dict[str, object]] = []
    for receiver in receiver_positions:
        distance_drops = 0
        for _ in range(samples_per_distance):
            delivered = model.should_deliver(
                message, (0, 0), receiver, rng, ("00", "01")
            )
            distance_drops += int(not delivered)
        attempts += samples_per_distance
        drops += distance_drops
        by_distance.append({
            "receiver": list(receiver),
            "drop_fraction": distance_drops / samples_per_distance,
        })
    fraction = drops / attempts
    if not 0.235 <= fraction <= 0.265:
        raise RuntimeError(
            f"Rayleigh calibration at {RAYLEIGH_SENSITIVITY_DBM} dBm "
            f"produced drop fraction {fraction:.4f}, expected approximately 0.25"
        )
    return {
        "model": "rayleigh_style",
        "sensitivity_dbm": RAYLEIGH_SENSITIVITY_DBM,
        "seed": SCENARIO_SEED + 991,
        "samples_per_distance": samples_per_distance,
        "attempts": attempts,
        "drop_fraction": fraction,
        "representative_links": by_distance,
    }


def build_command(
    environment: Environment,
    algorithm_name: str,
    clue_import: str,
    known_import: str,
    rate: float,
    limit: int,
    scenario_path: Path,
    out_dir: Path,
    seed: int,
    condition_id: str,
) -> tuple[Path, list[str]]:
    if environment.runner == "clue":
        command = [
            sys.executable, "-m", "benchmark_sim.run_trials",
            "--study-profile", "custom",
            "--no-scenario-manifest-lock",
            "--scenario-file", str(scenario_path),
            "--trial-mode", "clue_search",
            "--algorithm", clue_import,
            "--algorithm-name", algorithm_name,
            "--comm-model", environment.comm_model,
            "--max-trials", str(environment.trials_per_condition),
            "--seed", str(seed),
            "--out-dir", str(out_dir),
            "--grid-size", str(environment.grid_size),
            "--num-robots", str(environment.robot_count),
            "--robot-start-layout", "edge_even",
            "--condition-id", condition_id,
            "--target-cells-per-robot",
            f"{environment.grid_size * environment.grid_size / environment.robot_count:.9f}",
            "--actual-cells-per-robot",
            f"{environment.grid_size * environment.grid_size / environment.robot_count:.9f}",
            "--target-decay-exp", str(TARGET_DECAY_EXP),
            "--commitment-horizon", str(COMMITMENT_HORIZON),
            "--top-k-rate", f"{rate:g}",
            "--debug-max-events", str(DEBUG_MAX_EVENTS),
            "--no-parquet",
        ]
        if environment.comm_level:
            command.extend(("--comm-level", environment.comm_level))
        return SIMULATOR_ROOT, command

    command = [
        sys.executable, "-m", "known_visit_sim.run_trials",
        "--scenario-file", str(scenario_path),
        "--algorithm", known_import,
        "--algorithm-name", algorithm_name,
        "--comm-model", environment.comm_model,
        "--max-trials", str(environment.trials_per_condition),
        "--seed", str(seed),
        "--out-dir", str(out_dir),
        "--grid-size", str(environment.grid_size),
        "--num-robots", str(environment.robot_count),
        "--robot-start-layout", "edge_even",
        "--condition-id", condition_id,
        "--commitment-horizon", str(COMMITMENT_HORIZON),
        "--max-candidate-cells", str(limit),
    ]
    if environment.comm_level:
        command.extend(("--comm-level", environment.comm_level))
    return KNOWN_VISIT_ROOT, command


def build_manifest_rows(run_root: Path) -> list[dict[str, object]]:
    scenario_dir = run_root / "scenarios"
    rows: list[dict[str, object]] = []
    index = 0
    for environment in ALL_ENVIRONMENTS:
        scenario_path = (scenario_dir / environment.scenario_name).resolve()
        scenario_hash = file_sha256(scenario_path)
        seed = SIM_SEED_BASE + environment.grid_size * 1009
        for (
            algorithm_key, algorithm_name, clue_import, known_import, algorithm_weight,
        ) in ALGORITHMS:
            algorithm_import = (
                known_import if environment.runner == "known_visit" else clue_import
            )
            for rate in TOP_K_RATES:
                limit = top_k_limit(environment.grid_size, rate, environment.runner)
                label = top_k_label(rate)
                condition_id = (
                    f"{environment.key}_{algorithm_key}_topk{label}"
                )
                out_dir = (
                    raw_environment_dir(run_root, environment)
                    / algorithm_key / f"topk_{label}"
                ).resolve()
                cwd, command = build_command(
                    environment=environment,
                    algorithm_name=algorithm_name,
                    clue_import=clue_import,
                    known_import=known_import,
                    rate=rate,
                    limit=limit,
                    scenario_path=scenario_path,
                    out_dir=out_dir,
                    seed=seed,
                    condition_id=condition_id,
                )
                work_weight = (
                    algorithm_weight
                    + int(limit / 25)
                    + environment.robot_count * 2
                    + (30 if environment.runner == "known_visit" else 0)
                )
                rows.append({
                    "condition_index": index,
                    "condition_id": condition_id,
                    "suite": environment.suite,
                    "environment": environment.key,
                    "runner": environment.runner,
                    "grid_size": environment.grid_size,
                    "grid_cells": environment.grid_size ** 2,
                    "robot_count": environment.robot_count,
                    "target_count": environment.target_count,
                    "pair_key": environment.pair_key,
                    "algorithm_key": algorithm_key,
                    "algorithm_name": algorithm_name,
                    "algorithm_import": algorithm_import,
                    "top_k_rate": f"{rate:g}",
                    "top_k_max_cells": limit,
                    "top_k_basis": (
                        "initial_targets"
                        if environment.runner == "known_visit"
                        else "grid_cells"
                    ),
                    "comm_label": environment.comm_label,
                    "comm_model": environment.comm_model,
                    "comm_level": environment.comm_level,
                    "scenario_file": str(scenario_path),
                    "scenario_sha256": scenario_hash,
                    "seed": seed,
                    "num_trials": environment.trials_per_condition,
                    "out_dir": str(out_dir),
                    "working_directory": str(cwd.resolve()),
                    "weight": work_weight,
                    "command": json.dumps(command),
                })
                index += 1
    if len(rows) != EXPECTED_CONDITIONS:
        raise RuntimeError(
            f"expected {EXPECTED_CONDITIONS} conditions, built {len(rows)}"
        )
    return rows


def write_manifest(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def prepare(run_root: Path) -> None:
    if not (SIMULATOR_ROOT / "benchmark_sim" / "run_trials.py").exists():
        raise FileNotFoundError(f"missing benchmark simulator: {SIMULATOR_ROOT}")
    if not (KNOWN_VISIT_ROOT / "known_visit_sim" / "run_trials.py").exists():
        raise FileNotFoundError(f"missing known-visit simulator: {KNOWN_VISIT_ROOT}")

    scenario_dir = run_root / "scenarios"
    write_clue_scenarios(
        scenario_dir / "clue_g19_n50.csv", 19, (2, 4, 8),
        SCENARIO_SEED + 19 * 1009,
    )
    write_clue_scenarios(
        scenario_dir / "clue_g14_n50.csv", 14, (4,),
        SCENARIO_SEED + 14 * 1009,
    )
    write_clue_scenarios(
        scenario_dir / "clue_g28_n50.csv", 28, (4,),
        SCENARIO_SEED + 28 * 1009,
    )
    write_known_target_scenarios(
        scenario_dir / "known_targets_g19_t50_n100.csv",
        SCENARIO_SEED + 50 * 1009,
        MULTITARGET_TRIALS_PER_CONDITION,
    )
    validate_scenario_files(scenario_dir)

    records_dir = campaign_records_dir(run_root)
    records_dir.mkdir(parents=True, exist_ok=True)
    calibration = rayleigh_calibration()
    (records_dir / "channel_calibration.json").write_text(
        json.dumps(calibration, indent=2) + "\n", encoding="utf-8"
    )
    rows = build_manifest_rows(run_root)
    write_manifest(records_dir / "condition_manifest.csv", rows)
    metadata = {
        "schema": 1,
        "created_at_unix": time.time(),
        "repository_root": str(REPO_ROOT),
        "benchmark_simulator_root": str(SIMULATOR_ROOT),
        "known_visit_simulator_root": str(KNOWN_VISIT_ROOT),
        "conditions": len(rows),
        "default_trials_per_condition": TRIALS_PER_CONDITION,
        "multitarget_trials_per_condition": MULTITARGET_TRIALS_PER_CONDITION,
        "expected_trials": EXPECTED_TRIALS,
        "default_workers": DEFAULT_WORKERS,
        "top_k_rates": list(TOP_K_RATES),
        "debug_max_events_clue": DEBUG_MAX_EVENTS,
        "known_visit_debug_max_events": "existing simulator default",
    }
    (records_dir / "suite_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Prepared {len(rows)} conditions and {EXPECTED_TRIALS} trials at {run_root}")
    print(
        "Rayleigh calibration drop fraction: "
        f"{float(calibration['drop_fraction']):.4f}"
    )


def load_manifest(run_root: Path) -> list[dict[str, str]]:
    path = campaign_records_dir(run_root) / "condition_manifest.csv"
    if not path.exists():
        raise FileNotFoundError(f"missing manifest: {path}; run prepare first")
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != EXPECTED_CONDITIONS:
        raise RuntimeError(
            f"manifest has {len(rows)} conditions, expected {EXPECTED_CONDITIONS}"
        )
    return rows


def output_counts(row: dict[str, str]) -> dict[str, int]:
    out_dir = Path(row["out_dir"])
    counts = {
        "trial": len(read_csv_rows(out_dir / "trial_summary.csv")),
        "system": len(read_csv_rows(out_dir / "system_performance.csv")),
        "robot": len(read_csv_rows(out_dir / "robot_performance.csv")),
    }
    if row["runner"] == "known_visit":
        counts["extra"] = len(read_csv_rows(out_dir / "target_performance.csv"))
    else:
        counts["extra"] = len(read_csv_rows(out_dir / "computational_performance.csv"))
    return counts


def output_complete(row: dict[str, str]) -> bool:
    expected = int(row["num_trials"])
    counts = output_counts(row)
    expected_extra = (
        expected * int(row["target_count"])
        if row["runner"] == "known_visit"
        else expected * int(row["robot_count"])
    )
    return (
        counts["trial"] == expected
        and counts["system"] == expected
        and counts["robot"] == expected * int(row["robot_count"])
        and counts["extra"] == expected_extra
    )


def trial_failures(row: dict[str, str]) -> list[dict[str, str]]:
    return [
        item
        for item in read_csv_rows(Path(row["out_dir"]) / "system_performance.csv")
        if item.get("trial_status", "").strip().lower() == "failed"
    ]


def progress_snapshot(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    complete = recorded = failures = 0
    by_suite: dict[str, dict[str, int]] = {}
    for row in rows:
        counts = output_counts(row)
        recorded += min(counts["system"], int(row["num_trials"]))
        is_complete = output_complete(row)
        complete += int(is_complete)
        failed = len(trial_failures(row))
        failures += failed
        suite_counts = by_suite.setdefault(
            row["suite"], {"conditions": 0, "complete": 0, "recorded_trials": 0}
        )
        suite_counts["conditions"] += 1
        suite_counts["complete"] += int(is_complete)
        suite_counts["recorded_trials"] += min(
            counts["system"], int(row["num_trials"])
        )
    return {
        "updated_at_unix": time.time(),
        "conditions_complete": complete,
        "conditions_total": len(rows),
        "recorded_trials": recorded,
        "expected_trials": sum(int(row["num_trials"]) for row in rows),
        "failed_trial_rows": failures,
        "by_suite": by_suite,
    }


def write_progress(run_root: Path, rows: Sequence[dict[str, str]]) -> None:
    snapshot = progress_snapshot(rows)
    path = campaign_records_dir(run_root) / "progress.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(snapshot, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


class SleepInhibitor:
    def __enter__(self):
        if os.name == "nt":
            ctypes.windll.kernel32.SetThreadExecutionState(
                0x80000000 | 0x00000001 | 0x00000040
            )
        return self

    def __exit__(self, exc_type, exc, traceback):
        if os.name == "nt":
            ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)


def run_condition(row: dict[str, str]) -> dict[str, object]:
    if output_complete(row):
        return {
            "condition_id": row["condition_id"],
            "status": "already_complete",
            "counts": output_counts(row),
        }
    out_dir = Path(row["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)
    command = json.loads(row["command"])
    log_path = out_dir / "run.log"
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    started = time.time()
    with log_path.open("a", encoding="utf-8") as log_handle:
        log_handle.write(
            f"\nSTART {time.strftime('%Y-%m-%dT%H:%M:%S')} "
            f"{json.dumps(command)}\n"
        )
        log_handle.flush()
        process = subprocess.run(
            command,
            cwd=row["working_directory"],
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            check=False,
            creationflags=creationflags,
        )
        log_handle.write(
            f"END {time.strftime('%Y-%m-%dT%H:%M:%S')} "
            f"returncode={process.returncode}\n"
        )
    counts = output_counts(row)
    failures = len(trial_failures(row))
    complete = output_complete(row)
    status = "complete" if process.returncode == 0 and complete else "incomplete"
    return {
        "condition_id": row["condition_id"],
        "status": status,
        "returncode": process.returncode,
        "elapsed_seconds": time.time() - started,
        "counts": counts,
        "failed_trial_rows": failures,
    }


def run_campaign(run_root: Path, workers: int) -> None:
    if workers != DEFAULT_WORKERS:
        raise ValueError(
            f"this campaign is locked to {DEFAULT_WORKERS} workers "
            "(three-quarters of 16 physical cores)"
        )
    rows = load_manifest(run_root)
    job_queue: queue.Queue[dict[str, str]] = queue.Queue()
    for row in sorted(rows, key=lambda item: -int(item["weight"])):
        if not output_complete(row):
            job_queue.put(row)
    print(
        f"Launching {job_queue.qsize()} incomplete conditions with {workers} workers",
        flush=True,
    )
    write_progress(run_root, rows)
    log_lock = threading.Lock()
    errors: list[str] = []

    def worker(worker_index: int) -> None:
        while True:
            try:
                row = job_queue.get_nowait()
            except queue.Empty:
                return
            try:
                result = run_condition(row)
            except Exception as exc:
                result = {
                    "condition_id": row["condition_id"],
                    "status": "launcher_error",
                    "error": repr(exc),
                }
            with log_lock:
                with (campaign_records_dir(run_root) / "launcher.jsonl").open(
                    "a", encoding="utf-8"
                ) as handle:
                    handle.write(json.dumps({
                        "worker": worker_index,
                        "timestamp": time.time(),
                        **result,
                    }) + "\n")
                if result["status"] not in {"complete", "already_complete"}:
                    errors.append(str(result["condition_id"]))
                write_progress(run_root, rows)
                print(json.dumps(result), flush=True)
            job_queue.task_done()

    with SleepInhibitor():
        threads = [
            threading.Thread(target=worker, args=(index,), daemon=False)
            for index in range(workers)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    write_progress(run_root, rows)
    if errors:
        raise RuntimeError(
            f"{len(errors)} condition launch(es) incomplete; run verify and resume"
        )
    print("Campaign execution finished", flush=True)


def verify(run_root: Path, allow_incomplete: bool = False) -> dict[str, object]:
    rows = load_manifest(run_root)
    scenario_dir = run_root / "scenarios"
    validate_scenario_files(scenario_dir)
    issues: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    seen: set[str] = set()
    for row in rows:
        condition_id = row["condition_id"]
        if condition_id in seen:
            issues.append({"condition_id": condition_id, "issue": "duplicate ID"})
        seen.add(condition_id)
        scenario_path = Path(row["scenario_file"])
        if file_sha256(scenario_path) != row["scenario_sha256"]:
            issues.append({
                "condition_id": condition_id,
                "issue": "scenario SHA-256 mismatch",
            })
        counts = output_counts(row)
        if not output_complete(row):
            issues.append({
                "condition_id": condition_id,
                "issue": "incomplete output",
                "counts": counts,
            })
        for failure in trial_failures(row):
            failures.append({
                "condition_id": condition_id,
                "suite": row["suite"],
                "trial_id": failure.get("trial_id", ""),
                "failure_type": failure.get("failure_type", ""),
                "failure_message": failure.get("failure_message", ""),
            })
        if output_complete(row) and row["runner"] == "clue":
            for item in read_csv_rows(
                Path(row["out_dir"]) / "system_performance.csv"
            ):
                if item.get("condition_id") != condition_id:
                    issues.append({
                        "condition_id": condition_id,
                        "issue": "condition ID mismatch in system output",
                    })
                    break
                if round_half_up(float(item["top_k_max_cells"])) != int(
                    row["top_k_max_cells"]
                ):
                    issues.append({
                        "condition_id": condition_id,
                        "issue": "Top-K limit mismatch in system output",
                    })
                    break
    summary = {
        **progress_snapshot(rows),
        "verification_issues": len(issues),
        "failed_trials": len(failures),
    }
    records_dir = campaign_records_dir(run_root)
    records_dir.mkdir(parents=True, exist_ok=True)
    (records_dir / "verification_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    write_dict_rows(records_dir / "verification_issues.csv", issues)
    write_dict_rows(records_dir / "failures.csv", failures)
    print(json.dumps(summary, indent=2))
    if issues and not allow_incomplete:
        raise RuntimeError(f"verification found {len(issues)} issue(s)")
    return summary


def percentile(values: Sequence[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = (len(ordered) - 1) * probability
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def bootstrap_mean_interval(
    values: Sequence[float],
    seed: int,
    samples: int = 2_000,
) -> tuple[float, float]:
    if not values:
        return math.nan, math.nan
    if len(values) == 1:
        return values[0], values[0]
    rng = random.Random(seed)
    count = len(values)
    means = [
        sum(values[rng.randrange(count)] for _ in range(count)) / count
        for _ in range(samples)
    ]
    return percentile(means, 0.025), percentile(means, 0.975)


def safe_float(value: str | None) -> float | None:
    try:
        parsed = float(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None
    return parsed if parsed is not None and math.isfinite(parsed) else None


def write_dict_rows(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for field in row:
            if field not in seen:
                seen.add(field)
                fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def report(run_root: Path) -> None:
    rows = load_manifest(run_root)
    lookup: dict[tuple[str, str, str], dict[str, str]] = {
        (row["environment"], row["algorithm_key"], row["top_k_rate"]): row
        for row in rows
    }
    paired_rows: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []

    for row in rows:
        baseline = lookup.get((row["environment"], row["algorithm_key"], "1"))
        if baseline is None:
            raise RuntimeError(
                f"missing 100% Top-K baseline for {row['condition_id']}"
            )
        current_system = {
            item["trial_id"]: item
            for item in read_csv_rows(
                Path(row["out_dir"]) / "system_performance.csv"
            )
            if item.get("trial_status", "").strip().lower() == "completed"
        }
        baseline_system = {
            item["trial_id"]: item
            for item in read_csv_rows(
                Path(baseline["out_dir"]) / "system_performance.csv"
            )
            if item.get("trial_status", "").strip().lower() == "completed"
        }
        for trial_id in sorted(
            set(current_system).intersection(baseline_system), key=int
        ):
            current = current_system[trial_id]
            reference = baseline_system[trial_id]
            total = safe_float(current.get("total_team_steps"))
            baseline_total = safe_float(reference.get("total_team_steps"))
            max_steps = safe_float(
                current.get("max_robot_steps")
                if row["runner"] == "known_visit"
                else current.get("max_steps_any_robot")
            )
            if total is None or baseline_total is None or max_steps is None:
                failures.append({
                    "condition_id": row["condition_id"],
                    "trial_id": trial_id,
                    "failure_type": "missing_metric",
                })
                continue
            degradation = total - baseline_total
            degradation_pct = (
                100.0 * degradation / baseline_total
                if baseline_total != 0.0 else 0.0
            )
            paired_rows.append({
                "suite": row["suite"],
                "environment": row["environment"],
                "pair_key": row["pair_key"],
                "grid_size": row["grid_size"],
                "robot_count": row["robot_count"],
                "algorithm": row["algorithm_name"],
                "top_k_rate": row["top_k_rate"],
                "top_k_max_cells": row["top_k_max_cells"],
                "top_k_basis": row["top_k_basis"],
                "comm_model": row["comm_model"],
                "comm_level": row["comm_level"],
                "trial_id": trial_id,
                "max_steps_any_robot": max_steps,
                "total_step_degradation": degradation,
                "total_step_degradation_pct": degradation_pct,
            })

    grouped: dict[tuple[str, ...], list[dict[str, object]]] = {}
    group_fields = (
        "suite", "environment", "pair_key", "grid_size", "robot_count",
        "algorithm", "top_k_rate", "top_k_max_cells", "top_k_basis",
        "comm_model", "comm_level",
    )
    for row in paired_rows:
        key = tuple(str(row[field]) for field in group_fields)
        grouped.setdefault(key, []).append(row)

    summaries: list[dict[str, object]] = []
    for group_index, (key, items) in enumerate(sorted(grouped.items())):
        max_values = [float(item["max_steps_any_robot"]) for item in items]
        degradation_values = [
            float(item["total_step_degradation"]) for item in items
        ]
        degradation_pct_values = [
            float(item["total_step_degradation_pct"]) for item in items
        ]
        max_ci = bootstrap_mean_interval(max_values, SCENARIO_SEED + group_index)
        degradation_ci = bootstrap_mean_interval(
            degradation_values, SCENARIO_SEED + 10_000 + group_index
        )
        degradation_pct_ci = bootstrap_mean_interval(
            degradation_pct_values, SCENARIO_SEED + 20_000 + group_index
        )
        metadata = dict(zip(group_fields, key))
        summaries.append({
            **metadata,
            "paired_trials": len(items),
            "max_steps_mean": statistics.fmean(max_values),
            "max_steps_median": statistics.median(max_values),
            "max_steps_mean_ci95_low": max_ci[0],
            "max_steps_mean_ci95_high": max_ci[1],
            "total_step_degradation_mean": statistics.fmean(degradation_values),
            "total_step_degradation_median": statistics.median(degradation_values),
            "total_step_degradation_mean_ci95_low": degradation_ci[0],
            "total_step_degradation_mean_ci95_high": degradation_ci[1],
            "total_step_degradation_pct_mean": statistics.fmean(
                degradation_pct_values
            ),
            "total_step_degradation_pct_median": statistics.median(
                degradation_pct_values
            ),
            "total_step_degradation_pct_mean_ci95_low": degradation_pct_ci[0],
            "total_step_degradation_pct_mean_ci95_high": degradation_pct_ci[1],
        })

    report_dir = run_root / "reports"
    report_sets = {
        "all_missions": ({"scale", "communication", "multitarget"}),
        "bayesian_clue_search": ({"scale", "communication"}),
        "collaborative_known_target_visit": ({"multitarget"}),
    }
    for name, suites in report_sets.items():
        destination = report_dir / name
        write_dict_rows(
            destination / "paired_step_results.csv",
            [row for row in paired_rows if str(row["suite"]) in suites],
        )
        write_dict_rows(
            destination / "step_summary.csv",
            [row for row in summaries if str(row["suite"]) in suites],
        )
        write_dict_rows(
            destination / "report_failures.csv",
            [row for row in failures if str(row.get("suite", "")) in suites],
        )
    print(
        f"Wrote {len(paired_rows)} paired rows and {len(summaries)} "
        f"condition summaries to {report_dir}"
    )


def status(run_root: Path) -> None:
    rows = load_manifest(run_root)
    snapshot = progress_snapshot(rows)
    print(json.dumps(snapshot, indent=2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only Top-K sensitivity campaign orchestration."
    )
    parser.add_argument(
        "--run-root", type=Path, default=DEFAULT_RUN_ROOT,
        help=f"Campaign root (default: {DEFAULT_RUN_ROOT})",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("prepare")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("--allow-incomplete", action="store_true")
    subparsers.add_parser("report")
    subparsers.add_parser("status")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_root = args.run_root.expanduser().resolve()
    if args.command == "prepare":
        prepare(run_root)
    elif args.command == "run":
        run_campaign(run_root, args.workers)
    elif args.command == "verify":
        verify(run_root, args.allow_incomplete)
    elif args.command == "report":
        report(run_root)
    elif args.command == "status":
        status(run_root)
    else:
        raise AssertionError(args.command)
