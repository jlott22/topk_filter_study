#!/usr/bin/env python3
"""Run the paired 500-trial Top-K filtering campaign.

The launcher performs a serial smoke pass first, then executes sharded
condition jobs on CPU-pinned workers. All 15,000-event first-pass jobs finish
before any failed trial is retried with the 20,000-event cap.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import queue
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from benchmark_sim.run_trials import (
    DEFAULT_TOPK_SCENARIO_MANIFEST_LOCK,
    enforce_scenario_manifest_lock,
    file_sha256,
    scenario_selection_sha256,
)
from benchmark_sim.core.scenario_loader import load_scenarios, validate_scenarios
from benchmark_sim.config import LOGIC_REVISION, edge_even_start_positions, generate_robot_ids


SIMULATOR_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = SIMULATOR_ROOT.parent
DEFAULT_SCENARIO_FILE = SIMULATOR_ROOT / "scenarios" / "final_trial_500.csv"
DEFAULT_RUN_ROOT = REPO_ROOT / "results" / "topk_500_event15000"
EXPECTED_SCENARIO_SELECTION_SHA256 = (
    "823213c90703fd83224ad7122ee730ba64af3769ea517af252103bddd907f681"
)
TRIAL_COUNT = 500
FIRST_PASS_EVENT_CAP = 15_000
RETRY_EVENT_CAP = 20_000
TOP_K_RATES = (1.0, 0.75, 0.50, 0.25, 0.10, 0.05)
METRIC_FILENAMES = (
    "trial_summary.csv",
    "system_performance.csv",
    "robot_performance.csv",
    "computational_performance.csv",
)


@dataclass(frozen=True)
class Algorithm:
    name: str
    module: str
    fallback_weight: float


ALGORITHMS = (
    Algorithm("ACBBA", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator", 3.0),
    Algorithm("CBAA", "benchmark_sim.algorithms.CBAA:CBAAAllocator", 1.0),
    Algorithm("DGA", "benchmark_sim.algorithms.DGA:DGAAllocator", 30.0),
    Algorithm("DMCHBA", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator", 15.0),
    Algorithm("HIPC", "benchmark_sim.algorithms.HIPC:HIPCAllocator", 4.0),
    Algorithm("PI", "benchmark_sim.algorithms.PI:PIAllocator", 3.0),
)


@dataclass(frozen=True)
class Condition:
    algorithm: Algorithm
    top_k_rate: float
    top_k_max_cells: int
    condition_id: str


@dataclass(frozen=True)
class ShardJob:
    condition: Condition
    shard_index: int
    shard_count: int
    expected_trials: int
    out_dir: Path
    estimated_runtime_ms: float


def atomic_write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def rate_label(rate: float) -> str:
    return str(int(round(rate * 100)))


def top_k_cells(rate: float) -> int:
    return max(1, int(19 * 19 * rate + 0.5))


def build_conditions() -> list[Condition]:
    return [
        Condition(
            algorithm=algorithm,
            top_k_rate=rate,
            top_k_max_cells=top_k_cells(rate),
            condition_id=f"{algorithm.name.lower()}_topk_{rate_label(rate)}",
        )
        for algorithm in ALGORITHMS
        for rate in TOP_K_RATES
    ]


def shard_size(total: int, shard_count: int, shard_index: int) -> int:
    if not 0 <= shard_index < shard_count:
        raise ValueError("invalid shard index")
    if shard_index >= total:
        return 0
    return ((total - 1 - shard_index) // shard_count) + 1


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8") as handle:
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


def completed_and_failed(path: Path) -> tuple[int, int]:
    rows = read_csv_rows(path)
    failures = sum(row.get("trial_status", "").strip().lower() == "failed" for row in rows)
    return len(rows), failures


def common_run_command(
    condition: Condition,
    scenario_file: Path,
    out_dir: Path,
    *,
    max_trials: int,
    debug_max_events: int,
) -> list[str]:
    return [
        sys.executable,
        "-m",
        "benchmark_sim.run_trials",
        "--scenario-file",
        str(scenario_file),
        "--max-trials",
        str(max_trials),
        "--seed",
        "0",
        "--algorithm",
        condition.algorithm.module,
        "--algorithm-name",
        condition.algorithm.name,
        "--comm-model",
        "ideal",
        "--top-k-rate",
        f"{condition.top_k_rate:g}",
        "--condition-id",
        condition.condition_id,
        "--debug-max-events",
        str(debug_max_events),
        "--out-dir",
        str(out_dir),
    ]


def smoke_command(
    condition: Condition,
    scenario_file: Path,
    out_dir: Path,
) -> list[str]:
    return common_run_command(
        condition,
        scenario_file,
        out_dir,
        max_trials=1,
        debug_max_events=FIRST_PASS_EVENT_CAP,
    ) + ["--no-scenario-manifest-lock"]


def shard_command(
    job: ShardJob,
    scenario_file: Path,
    *,
    retry: bool,
) -> list[str]:
    command = common_run_command(
        job.condition,
        scenario_file,
        job.out_dir,
        max_trials=TRIAL_COUNT,
        debug_max_events=RETRY_EVENT_CAP if retry else FIRST_PASS_EVENT_CAP,
    )
    command.extend(
        [
            "--expected-scenario-sha256",
            EXPECTED_SCENARIO_SELECTION_SHA256,
            "--trial-shard-count",
            str(job.shard_count),
            "--trial-shard-index",
            str(job.shard_index),
        ]
    )
    if retry:
        command.append("--retry-failed")
    return command


def pinned_command(core: int, command: Sequence[str]) -> list[str]:
    return ["taskset", "-c", str(core), *command]


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


def _float(row: dict[str, str], field: str) -> float:
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{field} is not numeric: {row.get(field)!r}") from exc


def _int(row: dict[str, str], field: str) -> int:
    value = _float(row, field)
    if not value.is_integer():
        raise ValueError(f"{field} is not an integer: {value}")
    return int(value)


def validate_smoke_output(condition: Condition, out_dir: Path) -> dict[str, object]:
    trial_rows = read_csv_rows(out_dir / "trial_summary.csv")
    system_rows = read_csv_rows(out_dir / "system_performance.csv")
    robot_rows = read_csv_rows(out_dir / "robot_performance.csv")
    compute_rows = read_csv_rows(out_dir / "computational_performance.csv")
    errors: list[str] = []

    def require(assertion: bool, message: str) -> None:
        if not assertion:
            errors.append(message)

    require(len(trial_rows) == 1, f"trial_summary rows={len(trial_rows)}, expected 1")
    require(len(system_rows) == 1, f"system rows={len(system_rows)}, expected 1")
    require(len(robot_rows) == 4, f"robot rows={len(robot_rows)}, expected 4")
    require(len(compute_rows) == 4, f"compute rows={len(compute_rows)}, expected 4")
    if errors:
        return {"passed": False, "errors": errors}

    trial = trial_rows[0]
    system = system_rows[0]
    require(trial.get("trial_status") == "completed", "trial did not complete")
    require(system.get("trial_status") == "completed", "system row did not complete")
    require(bool(trial.get("target_found_by_robot")), "target finder is missing")
    for row in [trial, system, *robot_rows, *compute_rows]:
        require(
            abs(_float(row, "top_k_rate") - condition.top_k_rate) < 1e-12,
            f"wrong top_k_rate in {row.get('robot_id', 'team')} row",
        )
        require(
            _int(row, "top_k_max_cells") == condition.top_k_max_cells,
            f"wrong top_k_max_cells in {row.get('robot_id', 'team')} row",
        )

    total_team_steps = _int(system, "total_team_steps")
    robot_steps = [_int(row, "steps_total") for row in robot_rows]
    unique_cells = _int(system, "unique_cells_searched")
    require(total_team_steps == sum(robot_steps), "team steps do not equal robot-step sum")
    require(_int(system, "max_steps_any_robot") == max(robot_steps), "max robot steps mismatch")
    require(total_team_steps > 0, "team steps are not positive")
    require(4 <= unique_cells <= 361, f"unique cells outside [4, 361]: {unique_cells}")
    require(
        _int(system, "messages_sent_total")
        == sum(_int(row, "messages_sent") for row in robot_rows),
        "team sent-message count does not equal robot sum",
    )

    host_runtimes: list[float] = []
    for row in compute_rows:
        calls = _int(row, "allocator_calls")
        pre_calls = _int(row, "allocator_calls_pre_clue")
        post_calls = _int(row, "allocator_calls_post_clue")
        candidate_calls = _int(row, "candidate_filter_calls")
        allocator_total = _float(row, "allocator_time_ms_total")
        allocator_solve_total = _float(row, "allocator_solve_time_ms_total")
        candidate_total = _float(row, "candidate_filter_time_ms_total")
        host_runtime = _float(row, "host_trial_runtime_ms")
        host_runtimes.append(host_runtime)
        require(calls == pre_calls + post_calls, "allocator phase-call counts do not add up")
        require(
            0 <= candidate_calls <= calls,
            "candidate-filter calls exceed allocator calls",
        )
        require(allocator_total >= 0.0, "negative allocator time")
        require(allocator_solve_total >= 0.0, "negative allocator solve time")
        require(candidate_total >= 0.0, "negative candidate-filter time")
        require(
            candidate_total <= allocator_total + 1e-6,
            "candidate-filter time exceeds enclosing allocator time",
        )
        require(
            abs(allocator_solve_total + candidate_total - allocator_total) < 1e-6,
            "solve-only plus candidate-filter time does not equal allocator time",
        )
        require(host_runtime > 0.0, "host runtime is not positive")
        require(
            abs(_float(row, "allocator_time_pct") - _float(row, "allocator_host_runtime_pct"))
            < 1e-9,
            "allocator percentage aliases differ",
        )
        if calls:
            require(
                abs(_float(row, "allocator_time_ms_mean") - allocator_total / calls) < 1e-6,
                "allocator mean does not equal total/calls",
            )
            require(
                abs(
                    _float(row, "allocator_solve_time_ms_mean")
                    - allocator_solve_total / calls
                )
                < 1e-6,
                "allocator solve mean does not equal total/calls",
            )
            require(
                _float(row, "allocator_time_ms_median")
                <= _float(row, "allocator_time_ms_p95")
                <= _float(row, "allocator_time_ms_max"),
                "allocator median/p95/max are not ordered",
            )

    require(max(host_runtimes) - min(host_runtimes) < 1e-9, "host runtime differs by robot")
    return {
        "passed": not errors,
        "errors": errors,
        "trial_id": trial.get("trial_id"),
        "total_team_steps": total_team_steps,
        "post_clue_steps_to_find": _int(system, "post_clue_steps_to_find"),
        "unique_cells_searched": unique_cells,
        "messages_sent_total": _int(system, "messages_sent_total"),
        "host_trial_runtime_ms": host_runtimes[0],
        "allocator_time_ms_team_total": sum(
            _float(row, "allocator_time_ms_total") for row in compute_rows
        ),
        "allocator_solve_time_ms_team_total": sum(
            _float(row, "allocator_solve_time_ms_total") for row in compute_rows
        ),
        "candidate_filter_time_ms_team_total": sum(
            _float(row, "candidate_filter_time_ms_total") for row in compute_rows
        ),
    }


def run_smoke(
    conditions: Sequence[Condition],
    scenario_file: Path,
    run_root: Path,
    core: int,
) -> dict[str, object]:
    smoke_root = run_root / "smoke"
    results: dict[str, dict[str, object]] = {}
    started = time.time()
    for index, condition in enumerate(conditions, start=1):
        out_dir = smoke_root / condition.condition_id
        out_dir.mkdir(parents=True, exist_ok=True)
        command = pinned_command(core, smoke_command(condition, scenario_file, out_dir))
        log_path = out_dir / "run.log"
        with log_path.open("a", encoding="utf-8") as log:
            log.write(f"\nSTART {time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
            log.write(json.dumps(command) + "\n")
            log.flush()
            process = subprocess.run(
                command,
                cwd=SIMULATOR_ROOT,
                env=subprocess_environment(),
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )
        validation = validate_smoke_output(condition, out_dir)
        validation["returncode"] = process.returncode
        if process.returncode != 0:
            validation["passed"] = False
            validation.setdefault("errors", []).append(
                f"simulator exited with status {process.returncode}"
            )
        results[condition.condition_id] = validation
        print(
            json.dumps(
                {
                    "smoke": f"{index}/{len(conditions)}",
                    "condition": condition.condition_id,
                    "passed": validation["passed"],
                    "host_trial_runtime_ms": validation.get("host_trial_runtime_ms"),
                }
            ),
            flush=True,
        )
        if not validation["passed"]:
            break

    report = {
        "passed": len(results) == len(conditions)
        and all(bool(result["passed"]) for result in results.values()),
        "conditions_tested": len(results),
        "conditions_expected": len(conditions),
        "serial_cpu_core": core,
        "elapsed_s": time.time() - started,
        "results": results,
    }
    atomic_write_json(run_root / "smoke_validation.json", report)
    return report


def measured_condition_weights(
    conditions: Sequence[Condition],
    smoke_report: dict[str, object],
) -> dict[str, float]:
    raw_results = smoke_report.get("results", {})
    results = raw_results if isinstance(raw_results, dict) else {}
    weights: dict[str, float] = {}
    for condition in conditions:
        record = results.get(condition.condition_id, {})
        measured = (
            record.get("host_trial_runtime_ms")
            if isinstance(record, dict)
            else None
        )
        try:
            weight = float(measured)
        except (TypeError, ValueError):
            # Scale the fallback by retained-grid fraction without allowing
            # tiny Top-K conditions to collapse to zero estimated work.
            fraction = max(0.15, condition.top_k_max_cells / 361.0)
            weight = condition.algorithm.fallback_weight * fraction
        weights[condition.condition_id] = max(weight, 0.001)
    return weights


def build_shard_jobs(
    conditions: Sequence[Condition],
    run_root: Path,
    shard_count: int,
    condition_weights: dict[str, float],
) -> list[ShardJob]:
    jobs: list[ShardJob] = []
    for condition in conditions:
        for shard_index in range(shard_count):
            expected = shard_size(TRIAL_COUNT, shard_count, shard_index)
            if expected == 0:
                continue
            jobs.append(
                ShardJob(
                    condition=condition,
                    shard_index=shard_index,
                    shard_count=shard_count,
                    expected_trials=expected,
                    out_dir=(
                        run_root
                        / "raw"
                        / condition.algorithm.name.lower()
                        / f"topk_{rate_label(condition.top_k_rate)}"
                        / f"shard_{shard_index:03d}"
                    ),
                    estimated_runtime_ms=condition_weights[condition.condition_id] * expected,
                )
            )
    return jobs


def job_state(job: ShardJob) -> dict[str, object]:
    rows, failures = completed_and_failed(job.out_dir / "system_performance.csv")
    return {
        "rows": rows,
        "failures": failures,
        "recorded": rows == job.expected_trials,
    }


def write_progress(
    run_root: Path,
    jobs: Sequence[ShardJob],
    *,
    phase: str,
    started_at: float,
) -> dict[str, object]:
    recorded_trials = 0
    failed_trials = 0
    recorded_jobs = 0
    for job in jobs:
        state = job_state(job)
        recorded_trials += min(int(state["rows"]), job.expected_trials)
        failed_trials += int(state["failures"])
        if state["recorded"]:
            recorded_jobs += 1
    progress = {
        "phase": phase,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "elapsed_s": time.time() - started_at,
        "recorded_trials": recorded_trials,
        "expected_trials": len(build_conditions()) * TRIAL_COUNT,
        "failed_trial_rows": failed_trials,
        "recorded_shard_jobs": recorded_jobs,
        "total_shard_jobs": len(jobs),
    }
    atomic_write_json(run_root / "progress.json", progress)
    return progress


def run_one_job(
    job: ShardJob,
    scenario_file: Path,
    *,
    core: int,
    retry: bool,
) -> dict[str, object]:
    before = job_state(job)
    if not retry and before["recorded"]:
        return {
            "condition": job.condition.condition_id,
            "shard": job.shard_index,
            "core": core,
            "phase": "first_pass",
            "status": "already_recorded",
            **before,
        }
    if retry and (not before["recorded"] or int(before["failures"]) == 0):
        return {
            "condition": job.condition.condition_id,
            "shard": job.shard_index,
            "core": core,
            "phase": "retry",
            "status": "nothing_to_retry",
            **before,
        }

    job.out_dir.mkdir(parents=True, exist_ok=True)
    command = pinned_command(core, shard_command(job, scenario_file, retry=retry))
    log_path = job.out_dir / ("retry.log" if retry else "run.log")
    with log_path.open("a", encoding="utf-8") as log:
        log.write(f"\nSTART {time.strftime('%Y-%m-%dT%H:%M:%S%z')} core={core}\n")
        log.write(json.dumps(command) + "\n")
        log.flush()
        process = subprocess.run(
            command,
            cwd=SIMULATOR_ROOT,
            env=subprocess_environment(),
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        )
        log.write(
            f"END {time.strftime('%Y-%m-%dT%H:%M:%S%z')} status={process.returncode}\n"
        )
    after = job_state(job)
    if process.returncode != 0:
        status = "launcher_failed"
    elif not after["recorded"]:
        status = "incomplete"
    elif retry and int(after["failures"]) > 0:
        status = "retry_failures_remain"
    else:
        status = "complete"
    return {
        "condition": job.condition.condition_id,
        "shard": job.shard_index,
        "core": core,
        "phase": "retry" if retry else "first_pass",
        "status": status,
        "returncode": process.returncode,
        **after,
    }


def run_job_phase(
    jobs: Sequence[ShardJob],
    scenario_file: Path,
    run_root: Path,
    worker_cores: Sequence[int],
    *,
    retry: bool,
    campaign_started_at: float,
) -> list[dict[str, object]]:
    phase = "retry_20000" if retry else "first_pass_15000"
    work_queue: queue.PriorityQueue[tuple[float, int, ShardJob]] = queue.PriorityQueue()
    candidates = [
        job
        for job in jobs
        if (
            int(job_state(job)["failures"]) > 0
            if retry
            else not bool(job_state(job)["recorded"])
        )
    ]
    for index, job in enumerate(candidates):
        work_queue.put((-job.estimated_runtime_ms, index, job))

    results: list[dict[str, object]] = []
    results_lock = threading.Lock()

    def worker(core: int) -> None:
        while True:
            try:
                _, _, job = work_queue.get_nowait()
            except queue.Empty:
                return
            try:
                result = run_one_job(
                    job,
                    scenario_file,
                    core=core,
                    retry=retry,
                )
            except Exception as exc:  # preserve the remaining queue on one bad job
                result = {
                    "condition": job.condition.condition_id,
                    "shard": job.shard_index,
                    "core": core,
                    "phase": phase,
                    "status": "launcher_exception",
                    "error": repr(exc),
                }
            finally:
                work_queue.task_done()
            with results_lock:
                results.append(result)
                with (run_root / "launcher.log").open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(result, sort_keys=True) + "\n")
                progress = write_progress(
                    run_root,
                    jobs,
                    phase=phase,
                    started_at=campaign_started_at,
                )
                print(
                    json.dumps(
                        {
                            **result,
                            "campaign_recorded_trials": progress["recorded_trials"],
                            "campaign_failed_trials": progress["failed_trial_rows"],
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )

    threads = [
        threading.Thread(target=worker, args=(core,), name=f"cpu-{core}", daemon=False)
        for core in worker_cores
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    write_progress(run_root, jobs, phase=phase, started_at=campaign_started_at)
    return results


def _row_sort_key(row: dict[str, str]) -> tuple[int, str]:
    try:
        trial_id = int(row.get("trial_id", ""))
    except ValueError:
        trial_id = sys.maxsize
    return trial_id, row.get("robot_id", "")


def combine_condition(
    condition: Condition,
    jobs: Sequence[ShardJob],
    run_root: Path,
) -> dict[str, object]:
    condition_jobs = [job for job in jobs if job.condition == condition]
    combined_dir = run_root / "combined" / condition.condition_id
    counts: dict[str, int] = {}
    duplicate_keys: dict[str, int] = {}
    for filename in METRIC_FILENAMES:
        rows = [
            row
            for job in condition_jobs
            for row in read_csv_rows(job.out_dir / filename)
        ]
        rows.sort(key=_row_sort_key)
        keys = [
            (row.get("trial_id", ""), row.get("robot_id", ""))
            if filename in {"robot_performance.csv", "computational_performance.csv"}
            else (row.get("trial_id", ""),)
            for row in rows
        ]
        counts[filename] = len(rows)
        duplicate_keys[filename] = len(keys) - len(set(keys))
        write_csv_rows(combined_dir / filename, rows)

    system_rows = read_csv_rows(combined_dir / "system_performance.csv")
    failures = sum(row.get("trial_status") == "failed" for row in system_rows)
    expected_counts = {
        "trial_summary.csv": TRIAL_COUNT,
        "system_performance.csv": TRIAL_COUNT,
        "robot_performance.csv": TRIAL_COUNT * 4,
        "computational_performance.csv": TRIAL_COUNT * 4,
    }
    errors = [
        f"{filename}: rows={counts[filename]}, expected={expected}"
        for filename, expected in expected_counts.items()
        if counts[filename] != expected
    ]
    errors.extend(
        f"{filename}: duplicate keys={duplicates}"
        for filename, duplicates in duplicate_keys.items()
        if duplicates
    )
    result = {
        "condition_id": condition.condition_id,
        "algorithm": condition.algorithm.name,
        "top_k_rate": condition.top_k_rate,
        "top_k_max_cells": condition.top_k_max_cells,
        "counts": counts,
        "failed_trials_after_retry": failures,
        "passed_structure_validation": not errors,
        "errors": errors,
    }
    atomic_write_json(combined_dir / "validation.json", result)
    return result


def combine_campaign(
    conditions: Sequence[Condition],
    jobs: Sequence[ShardJob],
    run_root: Path,
) -> dict[str, object]:
    condition_results = [
        combine_condition(condition, jobs, run_root)
        for condition in conditions
    ]
    all_counts: dict[str, int] = {}
    for filename in METRIC_FILENAMES:
        rows = [
            row
            for condition in conditions
            for row in read_csv_rows(
                run_root / "combined" / condition.condition_id / filename
            )
        ]
        rows.sort(
            key=lambda row: (
                row.get("algorithm", ""),
                _float(row, "top_k_rate"),
                *_row_sort_key(row),
            )
        )
        all_counts[filename] = len(rows)
        write_csv_rows(run_root / "combined" / f"all_{filename}", rows)

    report = {
        "passed_structure_validation": all(
            bool(result["passed_structure_validation"])
            for result in condition_results
        ),
        "conditions": condition_results,
        "conditions_total": len(condition_results),
        "conditions_with_remaining_failures": sum(
            int(result["failed_trials_after_retry"]) > 0
            for result in condition_results
        ),
        "failed_trials_after_retry": sum(
            int(result["failed_trials_after_retry"])
            for result in condition_results
        ),
        "all_counts": all_counts,
    }
    atomic_write_json(run_root / "final_validation.json", report)
    return report


def write_condition_manifest(
    conditions: Sequence[Condition],
    run_root: Path,
    condition_weights: dict[str, float],
    shard_count: int,
) -> None:
    rows = [
        {
            "condition_id": condition.condition_id,
            "algorithm": condition.algorithm.name,
            "algorithm_module": condition.algorithm.module,
            "top_k_rate": condition.top_k_rate,
            "top_k_max_cells": condition.top_k_max_cells,
            "comm_model": "ideal",
            "comm_level": "1.0",
            "seed": 0,
            "expected_trials": TRIAL_COUNT,
            "first_pass_event_cap": FIRST_PASS_EVENT_CAP,
            "retry_event_cap": RETRY_EVENT_CAP,
            "shard_count": shard_count,
            "smoke_host_trial_runtime_ms": condition_weights[condition.condition_id],
        }
        for condition in conditions
    ]
    write_csv_rows(run_root / "condition_manifest.csv", rows)


def git_metadata() -> dict[str, object]:
    def capture(*args: str) -> str:
        result = subprocess.run(
            ["git", *args],
            cwd=REPO_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.stdout.strip()

    return {
        "commit": capture("rev-parse", "HEAD"),
        "branch": capture("branch", "--show-current"),
        "dirty_files": capture("status", "--short").splitlines(),
    }


def validate_scenario_selection(scenario_file: Path) -> tuple[str, str]:
    scenarios = load_scenarios(scenario_file, max_trials=TRIAL_COUNT)
    robot_ids = generate_robot_ids(4)
    validate_scenarios(
        scenarios,
        grid_size=19,
        start_positions=edge_even_start_positions(19, robot_ids),
        trial_mode="clue_search",
        expected_count=TRIAL_COUNT,
    )
    selection_hash = scenario_selection_sha256(scenarios)
    if selection_hash != EXPECTED_SCENARIO_SELECTION_SHA256:
        raise RuntimeError(
            "500-trial scenario selection hash mismatch: "
            f"expected {EXPECTED_SCENARIO_SELECTION_SHA256}, got {selection_hash}"
        )
    return file_sha256(scenario_file), selection_hash


def parse_args() -> argparse.Namespace:
    available = sorted(os.sched_getaffinity(0))
    default_workers = max(1, int(len(available) * 0.75))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--scenario-file", type=Path, default=DEFAULT_SCENARIO_FILE)
    parser.add_argument("--workers", type=int, default=default_workers)
    parser.add_argument(
        "--shard-count",
        type=int,
        default=None,
        help="Round-robin shards per condition (default: four per worker).",
    )
    parser.add_argument(
        "--smoke-only",
        action="store_true",
        help="Run and validate the serial smoke suite, then stop.",
    )
    parser.add_argument(
        "--skip-smoke",
        action="store_true",
        help="Reuse an existing passing smoke_validation.json.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.workers <= len(available):
        parser.error(f"--workers must be in [1, {len(available)}]")
    if args.shard_count is None:
        args.shard_count = args.workers * 4
    if args.shard_count <= 0:
        parser.error("--shard-count must be positive")
    return args


def main() -> None:
    args = parse_args()
    available_cores = sorted(os.sched_getaffinity(0))
    worker_cores = available_cores[: args.workers]
    orchestrator_cores = available_cores[args.workers :]
    if orchestrator_cores:
        os.sched_setaffinity(0, set(orchestrator_cores))
    run_root = args.run_root.expanduser().resolve()
    scenario_file = args.scenario_file.expanduser().resolve()
    run_root.mkdir(parents=True, exist_ok=True)
    conditions = build_conditions()
    raw_hash, selection_hash = validate_scenario_selection(scenario_file)
    started_at = time.time()

    metadata = {
        "campaign": "topk_filter_500",
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "trial_count_per_condition": TRIAL_COUNT,
        "condition_count": len(conditions),
        "expected_condition_trials": len(conditions) * TRIAL_COUNT,
        "first_pass_event_cap": FIRST_PASS_EVENT_CAP,
        "retry_event_cap": RETRY_EVENT_CAP,
        "retry_policy": "after_all_first_pass_trials",
        "scenario_file": str(scenario_file),
        "scenario_file_sha256": raw_hash,
        "scenario_selection_sha256": selection_hash,
        "logic_revision": LOGIC_REVISION,
        "workers": args.workers,
        "worker_cores": worker_cores,
        "orchestrator_cores": orchestrator_cores,
        "cpu_affinity_rule": "one single-threaded simulator process per pinned worker core",
        "available_cpu_count": len(available_cores),
        "shards_per_condition": args.shard_count,
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "processor": platform.processor(),
        },
        "git": git_metadata(),
    }
    atomic_write_json(run_root / "campaign_config.json", metadata)

    smoke_path = run_root / "smoke_validation.json"
    if args.skip_smoke:
        if not smoke_path.exists():
            raise RuntimeError(f"--skip-smoke requires {smoke_path}")
        smoke_report = json.loads(smoke_path.read_text(encoding="utf-8"))
        if not smoke_report.get("passed"):
            raise RuntimeError("existing smoke report did not pass")
    else:
        smoke_report = run_smoke(
            conditions,
            scenario_file,
            run_root,
            core=worker_cores[0],
        )
        if not smoke_report["passed"]:
            raise RuntimeError(
                f"smoke validation failed; inspect {smoke_path} before launching"
            )
    if args.smoke_only:
        print(f"smoke validation passed: {smoke_path}", flush=True)
        return

    condition_weights = measured_condition_weights(conditions, smoke_report)
    jobs = build_shard_jobs(
        conditions,
        run_root,
        args.shard_count,
        condition_weights,
    )
    write_condition_manifest(
        conditions,
        run_root,
        condition_weights,
        args.shard_count,
    )
    atomic_write_json(
        run_root / "scheduler_plan.json",
        {
            "strategy": "dynamic longest-estimated-shard-first",
            "weight_source": "serial smoke host_trial_runtime_ms",
            "worker_cores": worker_cores,
            "jobs": [
                {
                    "condition_id": job.condition.condition_id,
                    "shard_index": job.shard_index,
                    "expected_trials": job.expected_trials,
                    "estimated_runtime_ms": job.estimated_runtime_ms,
                    "out_dir": str(job.out_dir),
                }
                for job in sorted(
                    jobs,
                    key=lambda item: item.estimated_runtime_ms,
                    reverse=True,
                )
            ],
        },
    )
    if args.dry_run:
        print(f"dry-run plan written to {run_root}", flush=True)
        return

    # Establish the full-selection lock only after the smoke suite passes.
    scenarios = load_scenarios(scenario_file, max_trials=TRIAL_COUNT)
    enforce_scenario_manifest_lock(
        DEFAULT_TOPK_SCENARIO_MANIFEST_LOCK,
        scenarios,
        grid_size=19,
        logic_revision=LOGIC_REVISION,
    )

    write_progress(
        run_root,
        jobs,
        phase="first_pass_15000",
        started_at=started_at,
    )
    run_job_phase(
        jobs,
        scenario_file,
        run_root,
        worker_cores,
        retry=False,
        campaign_started_at=started_at,
    )

    # Deliberate phase barrier: no retry starts until every first-pass shard
    # has recorded all of its assigned trials.
    incomplete = [job for job in jobs if not bool(job_state(job)["recorded"])]
    if incomplete:
        atomic_write_json(
            run_root / "campaign_blocked.json",
            {
                "reason": "first-pass shards incomplete",
                "jobs": [
                    {
                        "condition_id": job.condition.condition_id,
                        "shard": job.shard_index,
                        "state": job_state(job),
                    }
                    for job in incomplete
                ],
            },
        )
        raise RuntimeError(
            f"{len(incomplete)} first-pass shards are incomplete; rerun the launcher to resume"
        )

    run_job_phase(
        jobs,
        scenario_file,
        run_root,
        worker_cores,
        retry=True,
        campaign_started_at=started_at,
    )
    final_report = combine_campaign(conditions, jobs, run_root)
    final_report["elapsed_s"] = time.time() - started_at
    final_report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    atomic_write_json(run_root / "final_validation.json", final_report)
    write_progress(
        run_root,
        jobs,
        phase="complete",
        started_at=started_at,
    )
    print(json.dumps({"campaign_complete": True, **final_report}, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
