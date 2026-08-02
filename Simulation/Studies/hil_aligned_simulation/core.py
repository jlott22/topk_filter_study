from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import queue
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


REPO_ROOT = Path(__file__).resolve().parents[3]
DCTA_ROOT = REPO_ROOT / "Simulation" / "dcta_benchmark_sim"
RUN_ROOT = (
    REPO_ROOT
    / "Results"
    / "Simulation"
    / "TopKLowKSupplement"
    / "AllMissionsFullCounts"
)
RECORDS_DIR = RUN_ROOT / "campaign_records"
RAW_ROOT = RUN_ROOT / "raw"
COMBINED_ROOT = RUN_ROOT / "combined"
REPORTS_ROOT = RUN_ROOT / "reports"

BAYESIAN_SIMULATOR_ROOT = REPO_ROOT / "Simulation" / "Architecture" / "simulator"
BAYESIAN_SCENARIO = BAYESIAN_SIMULATOR_ROOT / "scenarios" / "final_trial_500.csv"
COLLABORATIVE_SCENARIO = (
    REPO_ROOT
    / "Results"
    / "Simulation"
    / "Sensitivity"
    / "scenarios"
    / "known_targets_g19_t50_n100.csv"
)

BAYESIAN_TRIALS = 500
COLLABORATIVE_TRIALS = 100
BAYESIAN_FIRST_PASS_EVENT_CAP = 15_000
BAYESIAN_RETRY_EVENT_CAP = 20_000
BAYESIAN_EXTENDED_EVENT_CAPS = (50_000, 100_000)
SEED = 0
GRID_SIZE = 19
ROBOT_COUNT = 4
COLLABORATIVE_TARGET_COUNT = 50
COMMITMENT_HORIZON = 3

BAYESIAN_HIL_TRIAL_IDS = (
    7, 12, 32, 45, 50, 56, 67, 80, 94, 108, 127, 132, 164,
    166, 178, 216, 226, 233, 274, 313, 358, 371, 388, 398, 424,
)
COLLABORATIVE_HIL_TRIAL_IDS = (10, 12, 14, 26, 30, 31, 32, 41, 49, 74)

METRIC_FILES = (
    "trial_summary.csv",
    "system_performance.csv",
    "robot_performance.csv",
    "computational_performance.csv",
)
COLLABORATIVE_EXTRA_FILES = ("target_performance.csv",)

ALGORITHMS = (
    ("cbaa", "CBAA", "benchmark_sim.algorithms.CBAA:CBAAAllocator", "known_visit_sim.algorithms.CBAA:CBAAAllocator", 4),
    ("acbba", "ACBBA", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator", "known_visit_sim.algorithms.ACBBA:ACBBAAllocator", 8),
    ("pi", "PI", "benchmark_sim.algorithms.PI:PIAllocator", "known_visit_sim.algorithms.PI:PIAllocator", 8),
    ("hipc", "HIPC", "benchmark_sim.algorithms.HIPC:HIPCAllocator", "known_visit_sim.algorithms.HIPC:HIPCAllocator", 10),
    ("dmchba", "DMCHBA", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator", "known_visit_sim.algorithms.DMCHBA:DMCHBAAllocator", 16),
    ("dga", "DGA", "benchmark_sim.algorithms.DGA:DGAAllocator", "known_visit_sim.algorithms.DGA:DGAAllocator", 24),
)

BAYESIAN_LEVELS = (
    ("fixed_k1", "K=1", 1.0 / 361.0, 1),
    ("topk_001", "1%", 0.01, 4),
    ("topk_003", "3%", 0.03, 11),
)
COLLABORATIVE_LEVELS = (
    ("fixed_k1", "K=1", 1.0 / 50.0, 1),
    ("fixed_k2", "K=2", 2.0 / 50.0, 2),
)

MANIFEST_FIELDS = (
    "condition_index",
    "condition_id",
    "mission",
    "algorithm_key",
    "algorithm_name",
    "algorithm_import",
    "top_k_level",
    "top_k_rate",
    "top_k_max_cells",
    "top_k_basis",
    "scenario_file",
    "scenario_sha256",
    "num_trials",
    "out_dir",
    "working_directory",
    "weight",
    "command",
)


@dataclass(frozen=True)
class CommandResult:
    condition_id: str
    returncode: int
    attempts: int
    failed_trials_after_retry: int
    log_path: str
    error: str = ""


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
        return list(csv.DictReader(handle))


def write_csv_rows(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for field in row:
            if field not in seen:
                seen.add(field)
                fieldnames.append(field)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        if fieldnames:
            writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
    temporary.replace(path)


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def load_manifest() -> list[dict[str, str]]:
    path = RECORDS_DIR / "condition_manifest.csv"
    if not path.exists():
        raise FileNotFoundError(f"run prepare first: {path}")
    return read_csv_rows(path)


def subprocess_environment() -> dict[str, str]:
    environment = dict(os.environ)
    environment.update(
        {
            "PYTHONHASHSEED": "0",
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    return environment


def build_manifest_rows() -> list[dict[str, object]]:
    if not BAYESIAN_SCENARIO.exists():
        raise FileNotFoundError(BAYESIAN_SCENARIO)
    if not COLLABORATIVE_SCENARIO.exists():
        raise FileNotFoundError(COLLABORATIVE_SCENARIO)
    if not (DCTA_ROOT / "known_visit_sim" / "run_trials.py").exists():
        raise FileNotFoundError(DCTA_ROOT / "known_visit_sim" / "run_trials.py")

    rows: list[dict[str, object]] = []
    index = 0
    bayesian_sha = file_sha256(BAYESIAN_SCENARIO)
    collaborative_sha = file_sha256(COLLABORATIVE_SCENARIO)
    for algorithm_key, algorithm_name, bayesian_import, known_import, weight in ALGORITHMS:
        for level_key, level_label, rate, cells in BAYESIAN_LEVELS:
            condition_id = f"bayesian_{algorithm_key}_{level_key}_k{cells}"
            out_dir = RAW_ROOT / "bayesian" / algorithm_key / level_key
            command = [
                sys.executable,
                "-m",
                "benchmark_sim.run_trials",
                "--study-profile",
                "custom",
                "--scenario-file",
                str(BAYESIAN_SCENARIO),
                "--max-trials",
                str(BAYESIAN_TRIALS),
                "--seed",
                str(SEED),
                "--algorithm",
                bayesian_import,
                "--algorithm-name",
                algorithm_name,
                "--comm-model",
                "ideal",
                "--top-k-rate",
                f"{rate:.18g}",
                "--condition-id",
                condition_id,
                "--debug-max-events",
                str(BAYESIAN_FIRST_PASS_EVENT_CAP),
                "--out-dir",
                str(out_dir),
                "--no-scenario-manifest-lock",
            ]
            rows.append(
                {
                    "condition_index": index,
                    "condition_id": condition_id,
                    "mission": "bayesian",
                    "algorithm_key": algorithm_key,
                    "algorithm_name": algorithm_name,
                    "algorithm_import": bayesian_import,
                    "top_k_level": level_label,
                    "top_k_rate": f"{rate:.18g}",
                    "top_k_max_cells": cells,
                    "top_k_basis": "grid_cells",
                    "scenario_file": str(BAYESIAN_SCENARIO),
                    "scenario_sha256": bayesian_sha,
                    "num_trials": BAYESIAN_TRIALS,
                    "out_dir": str(out_dir),
                    "working_directory": str(BAYESIAN_SIMULATOR_ROOT),
                    "weight": weight + cells,
                    "command": json.dumps(command),
                }
            )
            index += 1

        for level_key, level_label, rate, cells in COLLABORATIVE_LEVELS:
            condition_id = f"collaborative_{algorithm_key}_{level_key}_k{cells}"
            out_dir = RAW_ROOT / "collaborative" / algorithm_key / level_key
            command = [
                sys.executable,
                "-m",
                "known_visit_sim.run_trials",
                "--scenario-file",
                str(COLLABORATIVE_SCENARIO),
                "--max-trials",
                str(COLLABORATIVE_TRIALS),
                "--seed",
                str(SEED),
                "--algorithm",
                known_import,
                "--algorithm-name",
                algorithm_name,
                "--comm-model",
                "ideal",
                "--out-dir",
                str(out_dir),
                "--grid-size",
                str(GRID_SIZE),
                "--num-robots",
                str(ROBOT_COUNT),
                "--robot-start-layout",
                "edge_even",
                "--condition-id",
                condition_id,
                "--commitment-horizon",
                str(COMMITMENT_HORIZON),
                "--max-candidate-cells",
                str(cells),
            ]
            rows.append(
                {
                    "condition_index": index,
                    "condition_id": condition_id,
                    "mission": "collaborative",
                    "algorithm_key": algorithm_key,
                    "algorithm_name": algorithm_name,
                    "algorithm_import": known_import,
                    "top_k_level": level_label,
                    "top_k_rate": f"{rate:.18g}",
                    "top_k_max_cells": cells,
                    "top_k_basis": "initial_targets",
                    "scenario_file": str(COLLABORATIVE_SCENARIO),
                    "scenario_sha256": collaborative_sha,
                    "num_trials": COLLABORATIVE_TRIALS,
                    "out_dir": str(out_dir),
                    "working_directory": str(DCTA_ROOT),
                    "weight": weight + cells + 30,
                    "command": json.dumps(command),
                }
            )
            index += 1

    return rows


def prepare() -> None:
    rows = build_manifest_rows()
    RECORDS_DIR.mkdir(parents=True, exist_ok=True)
    write_csv_rows(RECORDS_DIR / "condition_manifest.csv", rows)
    write_json(
        RECORDS_DIR / "campaign_config.json",
        {
            "campaign": "missing_lowk_full_counts",
            "output_root": str(RUN_ROOT),
            "bayesian_trials_per_condition": BAYESIAN_TRIALS,
            "collaborative_trials_per_condition": COLLABORATIVE_TRIALS,
            "condition_count": len(rows),
            "expected_trials": sum(int(row["num_trials"]) for row in rows),
            "bayesian_hil_trial_ids": list(BAYESIAN_HIL_TRIAL_IDS),
            "collaborative_hil_trial_ids": list(COLLABORATIVE_HIL_TRIAL_IDS),
        },
    )
    write_json(
        RECORDS_DIR / "progress.json",
        {
            "status": "prepared",
            "completed_conditions": 0,
            "total_conditions": len(rows),
            "updated_at_unix": time.time(),
        },
    )


def _failed_trial_count(out_dir: Path) -> int:
    return sum(
        row.get("trial_status", "").strip().lower() == "failed"
        for row in read_csv_rows(out_dir / "trial_summary.csv")
    )


def _completed_trial_count(out_dir: Path) -> int:
    return len(read_csv_rows(out_dir / "system_performance.csv"))


def _command_with_event_cap(command: list[str], event_cap: int) -> list[str]:
    result = list(command)
    try:
        index = result.index("--debug-max-events")
    except ValueError:
        result.extend(["--debug-max-events", str(event_cap)])
    else:
        result[index + 1] = str(event_cap)
    return result


def _run_subprocess(
    command: list[str],
    *,
    cwd: Path,
    log_path: Path,
) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as log:
        log.write(f"START {time.strftime('%Y-%m-%dT%H:%M:%S')} {json.dumps(command)}\n")
        log.flush()
        process = subprocess.run(
            command,
            cwd=str(cwd),
            env=subprocess_environment(),
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        log.write(f"END returncode={process.returncode}\n")
        log.flush()
        return int(process.returncode)


def backfill_system_timing(out_dir: Path) -> None:
    system_rows = read_csv_rows(out_dir / "system_performance.csv")
    compute_rows = read_csv_rows(out_dir / "computational_performance.csv")
    if not system_rows or not compute_rows:
        return
    by_trial: dict[str, list[dict[str, str]]] = {}
    for row in compute_rows:
        by_trial.setdefault(row["trial_id"], []).append(row)
    for system in system_rows:
        rows = by_trial.get(system["trial_id"], [])
        if not rows:
            continue
        system["allocator_calls_total"] = str(sum(int(float(row.get("allocator_calls") or 0)) for row in rows))
        system["allocator_time_ms_team_total"] = str(sum(float(row.get("allocator_time_ms_total") or 0.0) for row in rows))
        system["allocator_time_ms_team_max"] = str(max(float(row.get("allocator_time_ms_max") or 0.0) for row in rows))
        system["allocator_solve_time_ms_team_total"] = str(sum(float(row.get("allocator_solve_time_ms_total") or 0.0) for row in rows))
        system["allocator_solve_time_ms_team_max"] = str(max(float(row.get("allocator_solve_time_ms_max") or 0.0) for row in rows))
        system["candidate_filter_calls_total"] = str(sum(int(float(row.get("candidate_filter_calls") or 0)) for row in rows))
        system["candidate_filter_time_ms_team_total"] = str(sum(float(row.get("candidate_filter_time_ms_total") or 0.0) for row in rows))
        system["candidate_filter_time_ms_team_max"] = str(max(float(row.get("candidate_filter_time_ms_max") or 0.0) for row in rows))
    write_csv_rows(out_dir / "system_performance.csv", system_rows)


def _validate_output_counts(row: dict[str, str]) -> tuple[bool, str]:
    out_dir = Path(row["out_dir"])
    expected_trials = int(row["num_trials"])
    expected_robots = expected_trials * ROBOT_COUNT
    expected = {
        "trial_summary.csv": expected_trials,
        "system_performance.csv": expected_trials,
        "robot_performance.csv": expected_robots,
        "computational_performance.csv": expected_robots,
    }
    for filename, count in expected.items():
        actual = len(read_csv_rows(out_dir / filename))
        if actual != count:
            return False, f"{filename}: {actual} != {count}"
    if row["mission"] == "collaborative":
        actual = len(read_csv_rows(out_dir / "target_performance.csv"))
        expected_targets = expected_trials * COLLABORATIVE_TARGET_COUNT
        if actual != expected_targets:
            return False, f"target_performance.csv: {actual} != {expected_targets}"
    return True, ""


def run_condition(row: dict[str, str]) -> CommandResult:
    out_dir = Path(row["out_dir"])
    log_path = RECORDS_DIR / "logs" / f"{row['condition_id']}.log"
    command = json.loads(row["command"])
    cwd = Path(row["working_directory"])
    ok, _ = _validate_output_counts(row) if out_dir.exists() else (False, "")
    if ok and _failed_trial_count(out_dir) == 0:
        backfill_system_timing(out_dir)
        return CommandResult(row["condition_id"], 0, 0, 0, str(log_path))

    attempts = 1
    returncode = _run_subprocess(command, cwd=cwd, log_path=log_path)
    if row["mission"] == "bayesian":
        failed = _failed_trial_count(out_dir)
        if returncode == 0 and failed:
            retry = _command_with_event_cap(command, BAYESIAN_RETRY_EVENT_CAP)
            retry.append("--retry-failed")
            attempts += 1
            returncode = _run_subprocess(retry, cwd=cwd, log_path=log_path)
            failed = _failed_trial_count(out_dir)
        for cap in BAYESIAN_EXTENDED_EVENT_CAPS:
            if returncode != 0 or failed == 0:
                break
            retry = _command_with_event_cap(command, cap)
            retry.append("--retry-failed")
            attempts += 1
            returncode = _run_subprocess(retry, cwd=cwd, log_path=log_path)
            failed = _failed_trial_count(out_dir)
    else:
        failed = _failed_trial_count(out_dir)

    if returncode == 0:
        backfill_system_timing(out_dir)
    ok, error = _validate_output_counts(row)
    if not ok and returncode == 0:
        returncode = 1
    return CommandResult(
        row["condition_id"],
        returncode,
        attempts,
        failed,
        str(log_path),
        error,
    )


def run_campaign(workers: int) -> None:
    rows = load_manifest()
    work = sorted(rows, key=lambda item: (-int(float(item["weight"])), int(item["condition_index"])))
    q: queue.Queue[dict[str, str]] = queue.Queue()
    for row in work:
        q.put(row)
    results: list[CommandResult] = []
    lock = threading.Lock()

    def worker() -> None:
        while True:
            try:
                row = q.get_nowait()
            except queue.Empty:
                return
            result = run_condition(row)
            with lock:
                results.append(result)
                completed = sum(item.returncode == 0 for item in results)
                write_json(
                    RECORDS_DIR / "progress.json",
                    {
                        "status": "running" if completed < len(rows) else "complete",
                        "completed_conditions": completed,
                        "total_conditions": len(rows),
                        "results": [item.__dict__ for item in results],
                        "updated_at_unix": time.time(),
                    },
                )
                print(
                    f"{result.condition_id}: returncode={result.returncode} "
                    f"failed_trials={result.failed_trials_after_retry}",
                    flush=True,
                )
            q.task_done()

    threads = [threading.Thread(target=worker, daemon=True) for _ in range(max(1, workers))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    failures = [result for result in results if result.returncode != 0]
    if failures:
        write_json(RECORDS_DIR / "run_failures.json", [item.__dict__ for item in failures])
        raise SystemExit(f"{len(failures)} condition(s) failed")


def _float(row: dict[str, str], field: str) -> float:
    try:
        return float(row.get(field, ""))
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{field} is not numeric: {row.get(field)!r}") from exc


def verify() -> None:
    rows = load_manifest()
    failures: list[dict[str, object]] = []
    for row in rows:
        out_dir = Path(row["out_dir"])
        ok, error = _validate_output_counts(row)
        if not ok:
            failures.append({"condition_id": row["condition_id"], "failure": error})
            continue
        system_rows = read_csv_rows(out_dir / "system_performance.csv")
        compute_rows = read_csv_rows(out_dir / "computational_performance.csv")
        compute_by_trial: dict[str, list[dict[str, str]]] = {}
        for item in compute_rows:
            compute_by_trial.setdefault(item["trial_id"], []).append(item)
            if int(float(item["top_k_max_cells"])) != int(row["top_k_max_cells"]):
                failures.append({"condition_id": row["condition_id"], "failure": "wrong top_k_max_cells"})
            completed_compute = item.get("trial_status", "completed").lower() == "completed"
            if not completed_compute:
                continue
            total = _float(item, "allocator_time_ms_total")
            solve = _float(item, "allocator_solve_time_ms_total")
            filtered = _float(item, "candidate_filter_time_ms_total")
            if abs(total - solve - filtered) > max(1e-6, total * 1e-9):
                failures.append({"condition_id": row["condition_id"], "failure": "robot timing arithmetic"})
        for system in system_rows:
            if system.get("trial_status", "completed").lower() != "completed":
                continue
            trial_rows = compute_by_trial.get(system["trial_id"], [])
            expected_calls = sum(int(float(item.get("allocator_calls") or 0)) for item in trial_rows)
            has_post_clue_split = any("allocator_calls_post_clue" in item for item in trial_rows)
            expected_post_clue_calls = sum(int(float(item.get("allocator_calls_post_clue") or 0)) for item in trial_rows)
            expected_filter_calls = sum(int(float(item.get("candidate_filter_calls") or 0)) for item in trial_rows)
            expected_total = sum(_float(item, "allocator_time_ms_total") for item in trial_rows)
            expected_solve = sum(_float(item, "allocator_solve_time_ms_total") for item in trial_rows)
            expected_filter = sum(_float(item, "candidate_filter_time_ms_total") for item in trial_rows)
            requires_filter = expected_post_clue_calls > 0 if has_post_clue_split else expected_calls > 0
            if requires_filter and expected_filter_calls <= 0:
                failures.append({"condition_id": row["condition_id"], "failure": "candidate_filter_calls <= 0"})
            checks = (
                ("allocator_calls_total", expected_calls, 0.0),
                ("candidate_filter_calls_total", expected_filter_calls, 0.0),
                ("allocator_time_ms_team_total", expected_total, max(1e-6, expected_total * 1e-9)),
                ("allocator_solve_time_ms_team_total", expected_solve, max(1e-6, expected_solve * 1e-9)),
                ("candidate_filter_time_ms_team_total", expected_filter, max(1e-6, expected_filter * 1e-9)),
            )
            for field, expected, tolerance in checks:
                actual = _float(system, field)
                if abs(actual - expected) > tolerance:
                    failures.append({"condition_id": row["condition_id"], "failure": f"system {field} mismatch"})
    write_json(
        RECORDS_DIR / "verification_summary.json",
        {
            "status": "failed" if failures else "passed",
            "condition_count": len(rows),
            "failures": failures,
        },
    )
    if failures:
        raise SystemExit(f"verification failed with {len(failures)} issue(s)")


def combine_outputs() -> None:
    rows = load_manifest()
    for filename in (*METRIC_FILES, *COLLABORATIVE_EXTRA_FILES):
        combined: list[dict[str, object]] = []
        for row in rows:
            path = Path(row["out_dir"]) / filename
            if not path.exists():
                continue
            for item in read_csv_rows(path):
                item.update(
                    {
                        "mission": row["mission"],
                        "top_k_level": row["top_k_level"],
                        "campaign_condition_id": row["condition_id"],
                    }
                )
                combined.append(item)
        if combined:
            write_csv_rows(COMBINED_ROOT / f"all_{filename}", combined)


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return statistics.fmean(values) if values else 0.0


def _condition_summary(row: dict[str, str], *, subset: set[int] | None = None) -> dict[str, object]:
    out_dir = Path(row["out_dir"])
    system_rows = read_csv_rows(out_dir / "system_performance.csv")
    if subset is not None:
        system_rows = [item for item in system_rows if int(item["trial_id"]) in subset]
    completed = [
        item for item in system_rows
        if item.get("trial_status", "completed").strip().lower() == "completed"
    ]
    if row["mission"] == "bayesian":
        search_metric = "post_clue_steps_to_find"
        fallback_metric = "total_team_steps"
    else:
        search_metric = "total_team_steps"
        fallback_metric = "total_team_steps"
    return {
        "condition_id": row["condition_id"],
        "mission": row["mission"],
        "algorithm": row["algorithm_name"],
        "top_k_level": row["top_k_level"],
        "top_k_rate": row["top_k_rate"],
        "top_k_max_cells": row["top_k_max_cells"],
        "trials": len(system_rows),
        "completed_trials": len(completed),
        "failed_trials": len(system_rows) - len(completed),
        "mean_primary_steps": _mean(
            _float(item, search_metric)
            if item.get(search_metric, "") not in {"", "nan", "None"}
            else _float(item, fallback_metric)
            for item in completed
        ),
        "mean_total_team_steps": _mean(_float(item, "total_team_steps") for item in completed),
        "mean_allocator_time_ms_team_total": _mean(
            _float(item, "allocator_time_ms_team_total") for item in completed
        ),
        "mean_allocator_solve_time_ms_team_total": _mean(
            _float(item, "allocator_solve_time_ms_team_total") for item in completed
        ),
        "mean_candidate_filter_time_ms_team_total": _mean(
            _float(item, "candidate_filter_time_ms_team_total") for item in completed
        ),
    }


def report() -> None:
    combine_outputs()
    rows = load_manifest()
    full = [_condition_summary(row) for row in rows]
    subset_rows = []
    for row in rows:
        subset = (
            set(BAYESIAN_HIL_TRIAL_IDS)
            if row["mission"] == "bayesian"
            else set(COLLABORATIVE_HIL_TRIAL_IDS)
        )
        subset_rows.append(_condition_summary(row, subset=subset))
    write_csv_rows(REPORTS_ROOT / "condition_summary_full_counts.csv", full)
    write_csv_rows(REPORTS_ROOT / "condition_summary_hil_trial_subset.csv", subset_rows)


def workers_arg(default: int = 12) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=default)
    return parser
