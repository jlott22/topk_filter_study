from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Iterable, Sequence

from .suite import (
    campaign_records_dir,
    raw_environment_dir,
    DEFAULT_RUN_ROOT,
    Environment,
    KNOWN_VISIT_ROOT,
    MULTITARGET_TRIALS_PER_CONDITION,
    read_csv_rows,
    report,
    verify,
)


ENVIRONMENT = "multitarget_g19_r4_t50"
SUITE_NAME = "multitarget"
CONDITION_COUNT = 36
ROBOT_COUNT = 4
TARGET_COUNT = 50
SCENARIO_COUNT = MULTITARGET_TRIALS_PER_CONDITION
OUTPUT_FILES = (
    "trial_summary.csv",
    "system_performance.csv",
    "robot_performance.csv",
    "target_performance.csv",
)
TIMING_FIELDS = (
    "allocator_calls",
    "allocator_time_ms_total",
    "allocator_time_ms_max",
    "allocator_solve_time_ms_total",
    "allocator_solve_time_ms_max",
    "candidate_filter_calls",
    "candidate_filter_time_ms_total",
    "candidate_filter_time_ms_max",
)
SYSTEM_TIMING_FIELDS = (
    "allocator_calls_total",
    "allocator_time_ms_team_total",
    "allocator_time_ms_team_max",
    "allocator_solve_time_ms_team_total",
    "allocator_solve_time_ms_team_max",
    "candidate_filter_calls_total",
    "candidate_filter_time_ms_team_total",
    "candidate_filter_time_ms_team_max",
)
SYSTEM_MAXIMUM_SOURCES = {
    "allocator_time_ms_team_max": "allocator_time_ms_max",
    "allocator_solve_time_ms_team_max": "allocator_solve_time_ms_max",
    "candidate_filter_time_ms_team_max": "candidate_filter_time_ms_max",
}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json_atomic(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def scenario_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(
            line
            for line in handle
            if line.strip() and not line.lstrip().startswith("#")
        )
        if reader.fieldnames is None:
            raise RuntimeError(f"missing scenario header: {path}")
        return list(reader.fieldnames), list(reader)


def write_single_scenario(
    path: Path,
    fields: Sequence[str],
    row: dict[str, str],
    source_hash: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# scenario_set=collaborative_100_shard\n")
        handle.write(f"# source_sha256={source_hash}\n")
        handle.write(f"# source_trial_id={row['trial_id']}\n")
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)


def replace_arg(command: list[str], option: str, value: str) -> None:
    try:
        index = command.index(option)
    except ValueError as exc:
        raise RuntimeError(f"command does not contain {option}") from exc
    command[index + 1] = value


def load_collaborative_conditions(run_root: Path) -> list[dict[str, str]]:
    path = campaign_records_dir(run_root) / "condition_manifest.csv"
    if not path.exists():
        raise FileNotFoundError(f"missing campaign manifest: {path}")
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = [
            row
            for row in csv.DictReader(handle)
            if row["suite"] == SUITE_NAME and row["environment"] == ENVIRONMENT
        ]
    if len(rows) != CONDITION_COUNT:
        raise RuntimeError(
            f"expected {CONDITION_COUNT} collaborative conditions, found {len(rows)}"
        )
    for row in rows:
        if int(row["num_trials"]) != SCENARIO_COUNT:
            raise RuntimeError(
                f"{row['condition_id']}: expected {SCENARIO_COUNT} trials"
            )
        if row["runner"] != "known_visit":
            raise RuntimeError(f"{row['condition_id']}: wrong runner")
    return rows


def validate_scenario_extension(run_root: Path, scenario_path: Path) -> None:
    fields, current = scenario_rows(scenario_path)
    if len(current) != SCENARIO_COUNT:
        raise RuntimeError(
            f"{scenario_path.name}: expected {SCENARIO_COUNT} rows, found {len(current)}"
        )
    expected_ids = list(range(SCENARIO_COUNT))
    actual_ids = [int(row["trial_id"]) for row in current]
    if actual_ids != expected_ids:
        raise RuntimeError("100-scenario cohort IDs are not exactly 0 through 99")
    old_path = run_root / "scenarios" / "known_targets_g19_t50_n50.csv"
    if old_path.exists():
        old_fields, old = scenario_rows(old_path)
        if fields != old_fields:
            raise RuntimeError("new and prior collaborative scenario schemas differ")
        if len(old) != 50 or current[:50] != old:
            raise RuntimeError(
                "new scenario cohort does not preserve the prior seeded first 50 rows"
            )
    starts = {(0, 0), (0, 6), (0, 12), (0, 18)}
    for row in current:
        targets = [
            (int(row[f"target{index}_x"]), int(row[f"target{index}_y"]))
            for index in range(1, TARGET_COUNT + 1)
        ]
        if len(set(targets)) != TARGET_COUNT:
            raise RuntimeError(f"trial {row['trial_id']}: duplicate target")
        if set(targets).intersection(starts):
            raise RuntimeError(f"trial {row['trial_id']}: target overlaps start")
        if any(not (0 <= x < 19 and 0 <= y < 19) for x, y in targets):
            raise RuntimeError(f"trial {row['trial_id']}: target is out of bounds")


def prepare(run_root: Path, staging_root: Path) -> list[dict[str, object]]:
    manifest_path = staging_root / "campaign_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        scenario_path = Path(str(manifest["scenario_file"]))
        if file_sha256(scenario_path) != manifest["scenario_sha256"]:
            raise RuntimeError("scenario changed after collaborative campaign preparation")
        return list(manifest["jobs"])
    if staging_root.exists() and any(staging_root.iterdir()):
        raise RuntimeError(
            f"staging directory exists without a campaign manifest: {staging_root}"
        )
    staging_root.mkdir(parents=True, exist_ok=True)

    conditions = load_collaborative_conditions(run_root)
    scenario_path = Path(conditions[0]["scenario_file"])
    scenario_hash = file_sha256(scenario_path)
    validate_scenario_extension(run_root, scenario_path)
    fields, scenarios = scenario_rows(scenario_path)

    jobs: list[dict[str, object]] = []
    for condition_index, condition in enumerate(conditions):
        command_template = list(json.loads(condition["command"]))
        if Path(condition["working_directory"]).resolve() != KNOWN_VISIT_ROOT.resolve():
            raise RuntimeError(
                f"{condition['condition_id']}: unexpected working directory"
            )
        for scenario in scenarios:
            current_trial = int(scenario["trial_id"])
            job_dir = (
                staging_root
                / "shards"
                / f"c{condition_index:02d}"
                / f"t{current_trial:03d}"
            ).resolve()
            shard_scenario = job_dir / "scenario.csv"
            shard_output = job_dir / "output"
            write_single_scenario(
                shard_scenario,
                fields,
                scenario,
                scenario_hash,
            )
            command = list(command_template)
            replace_arg(command, "--scenario-file", str(shard_scenario))
            replace_arg(command, "--max-trials", "1")
            replace_arg(command, "--out-dir", str(shard_output))
            weight = int(condition["weight"]) * (
                1000 + (current_trial * 37 % 101)
            )
            jobs.append({
                "condition_index": condition_index,
                "condition_id": condition["condition_id"],
                "algorithm_key": condition["algorithm_key"],
                "algorithm_name": condition["algorithm_name"],
                "top_k_rate": condition["top_k_rate"],
                "top_k_max_cells": int(condition["top_k_max_cells"]),
                "trial_id": current_trial,
                "weight": weight,
                "working_directory": condition["working_directory"],
                "scenario_file": str(shard_scenario),
                "scenario_sha256": file_sha256(shard_scenario),
                "out_dir": str(shard_output),
                "log_path": str(job_dir / "run.log"),
                "command": command,
            })

    jobs.sort(key=lambda item: (-int(item["weight"]), int(item["trial_id"])))
    manifest = {
        "schema": 1,
        "created_at_unix": time.time(),
        "run_root": str(run_root.resolve()),
        "staging_root": str(staging_root.resolve()),
        "logical_processors": os.cpu_count(),
        "worker_rule": "floor(logical_processors * 0.75)",
        "scenario_file": str(scenario_path.resolve()),
        "scenario_sha256": scenario_hash,
        "scenario_count": SCENARIO_COUNT,
        "condition_count": len(conditions),
        "job_count": len(jobs),
        "output_files": list(OUTPUT_FILES),
        "jobs": jobs,
    }
    write_json_atomic(manifest_path, manifest)
    write_json_atomic(
        staging_root / "progress.json",
        {
            "status": "prepared",
            "jobs_complete": 0,
            "jobs_total": len(jobs),
            "updated_at_unix": time.time(),
        },
    )
    return jobs


def _numeric(row: dict[str, str], field: str) -> float:
    value = row.get(field, "")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"missing/non-numeric {field}: {value!r}") from exc
    if not math.isfinite(number) or number < 0.0:
        raise RuntimeError(f"invalid {field}: {value!r}")
    return number


def validate_timing_rows(
    system_rows: Sequence[dict[str, str]],
    robot_rows: Sequence[dict[str, str]],
    context: str,
) -> None:
    if len(system_rows) != 1:
        raise RuntimeError(f"{context}: expected one system row")
    if len(robot_rows) != ROBOT_COUNT:
        raise RuntimeError(f"{context}: expected {ROBOT_COUNT} robot rows")
    system = system_rows[0]
    for field in SYSTEM_TIMING_FIELDS:
        _numeric(system, field)
    for robot in robot_rows:
        for field in TIMING_FIELDS:
            _numeric(robot, field)
        allocator_calls = int(_numeric(robot, "allocator_calls"))
        filter_calls = int(_numeric(robot, "candidate_filter_calls"))
        if allocator_calls <= 0:
            raise RuntimeError(f"{context}: allocator recorded no calls")
        if filter_calls <= 0:
            raise RuntimeError(
                f"{context}: candidate filter recorded no calls"
            )
        total = _numeric(robot, "allocator_time_ms_total")
        solve = _numeric(robot, "allocator_solve_time_ms_total")
        candidate_filter = _numeric(robot, "candidate_filter_time_ms_total")
        tolerance = max(1e-6, total * 1e-10)
        if abs(total - solve - candidate_filter) > tolerance:
            raise RuntimeError(
                f"{context}: allocator total does not equal solve + filter"
            )
    expected = {
        "allocator_calls_total": sum(
            int(_numeric(row, "allocator_calls")) for row in robot_rows
        ),
        "candidate_filter_calls_total": sum(
            int(_numeric(row, "candidate_filter_calls")) for row in robot_rows
        ),
        "allocator_time_ms_team_total": sum(
            _numeric(row, "allocator_time_ms_total") for row in robot_rows
        ),
        "allocator_solve_time_ms_team_total": sum(
            _numeric(row, "allocator_solve_time_ms_total") for row in robot_rows
        ),
        "candidate_filter_time_ms_team_total": sum(
            _numeric(row, "candidate_filter_time_ms_total") for row in robot_rows
        ),
        **{
            system_field: max(
                _numeric(row, robot_field) for row in robot_rows
            )
            for system_field, robot_field in SYSTEM_MAXIMUM_SOURCES.items()
        },
    }
    for field, expected_value in expected.items():
        actual = _numeric(system, field)
        tolerance = (
            0.0
            if field.endswith("calls_total")
            else max(1e-6, expected_value * 1e-10)
        )
        if abs(actual - expected_value) > tolerance:
            raise RuntimeError(
                f"{context}: system {field}={actual} != robot sum {expected_value}"
            )


def backfill_system_maxima(environment_root: Path) -> int:
    updated_rows = 0
    system_paths = sorted(environment_root.glob("*/topk_*/system_performance.csv"))
    if not system_paths:
        raise RuntimeError(f"no collaborative system CSVs found under {environment_root}")
    for system_path in system_paths:
        robot_path = system_path.with_name("robot_performance.csv")
        system_rows = read_csv_rows(system_path)
        robot_rows = read_csv_rows(robot_path)
        robots_by_trial: dict[int, list[dict[str, str]]] = {}
        for row in robot_rows:
            robots_by_trial.setdefault(int(row["trial_id"]), []).append(row)
        for system in system_rows:
            trial_id = int(system["trial_id"])
            trial_robots = robots_by_trial.get(trial_id, [])
            if len(trial_robots) != ROBOT_COUNT:
                raise RuntimeError(
                    f"{system_path}: trial {trial_id} has {len(trial_robots)} robot rows"
                )
            for system_field, robot_field in SYSTEM_MAXIMUM_SOURCES.items():
                system[system_field] = str(
                    max(_numeric(row, robot_field) for row in trial_robots)
                )
            updated_rows += 1
        write_csv_atomic(system_path, system_rows)
    return updated_rows


def validate_job_output(job: dict[str, object], returncode: int) -> dict[str, int]:
    out_dir = Path(str(job["out_dir"]))
    rows = {name: read_csv_rows(out_dir / name) for name in OUTPUT_FILES}
    counts = {name: len(items) for name, items in rows.items()}
    expected = {
        "trial_summary.csv": 1,
        "system_performance.csv": 1,
        "robot_performance.csv": ROBOT_COUNT,
        "target_performance.csv": TARGET_COUNT,
    }
    if returncode != 0 or counts != expected:
        raise RuntimeError(
            f"{job['condition_id']} trial {job['trial_id']}: "
            f"returncode={returncode}, counts={counts}"
        )
    for name, items in rows.items():
        for item in items:
            if int(item["trial_id"]) != int(job["trial_id"]):
                raise RuntimeError(
                    f"{job['condition_id']} trial {job['trial_id']}: "
                    f"wrong trial ID in {name}"
                )
            if item.get("trial_status", "").strip().lower() != "completed":
                raise RuntimeError(
                    f"{job['condition_id']} trial {job['trial_id']}: failed row"
                )
    validate_timing_rows(
        rows["system_performance.csv"],
        rows["robot_performance.csv"],
        f"{job['condition_id']} trial {job['trial_id']}",
    )
    return counts


def run_job(job: dict[str, object]) -> dict[str, object]:
    log_path = Path(str(job["log_path"]))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    returncode = -1
    error = ""
    with log_path.open("a", encoding="utf-8") as log_handle:
        log_handle.write(
            f"START {time.strftime('%Y-%m-%dT%H:%M:%S')} "
            f"{json.dumps(job['command'])}\n"
        )
        log_handle.flush()
        process = subprocess.run(
            list(job["command"]),
            cwd=str(job["working_directory"]),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            check=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        returncode = process.returncode
        log_handle.write(
            f"END {time.strftime('%Y-%m-%dT%H:%M:%S')} "
            f"returncode={returncode}\n"
        )
    try:
        counts = validate_job_output(job, returncode)
        complete = True
    except Exception as exc:
        counts = {
            name: len(read_csv_rows(Path(str(job["out_dir"])) / name))
            for name in OUTPUT_FILES
        }
        complete = False
        error = repr(exc)
    return {
        "condition_id": job["condition_id"],
        "trial_id": job["trial_id"],
        "returncode": returncode,
        "complete": complete,
        "counts": counts,
        "error": error,
        "elapsed_seconds": time.time() - started,
    }


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


def completed_job_keys(results_path: Path) -> set[tuple[str, int]]:
    complete: set[tuple[str, int]] = set()
    if not results_path.exists():
        return complete
    for line in results_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        if item.get("complete"):
            complete.add((str(item["condition_id"]), int(item["trial_id"])))
    return complete


def execute(staging_root: Path, jobs: Sequence[dict[str, object]], workers: int) -> None:
    logical = os.cpu_count() or workers
    expected_workers = max(1, int(logical * 0.75))
    if workers != expected_workers:
        raise ValueError(
            f"workers must equal floor({logical} * 0.75) = {expected_workers}"
        )
    results_path = staging_root / "job_results.jsonl"
    complete = completed_job_keys(results_path)
    pending = [
        job
        for job in jobs
        if (str(job["condition_id"]), int(job["trial_id"])) not in complete
    ]
    job_queue: queue.Queue[dict[str, object]] = queue.Queue()
    for job in pending:
        job_queue.put(job)
    lock = threading.Lock()
    failures: list[tuple[str, int]] = []
    worker_stats = [
        {"worker": index, "jobs": 0, "elapsed_seconds": 0.0}
        for index in range(workers)
    ]
    write_json_atomic(
        staging_root / "progress.json",
        {
            "status": "running",
            "workers": workers,
            "logical_processors": logical,
            "jobs_complete": len(complete),
            "jobs_total": len(jobs),
            "jobs_pending": len(pending),
            "updated_at_unix": time.time(),
        },
    )

    def worker(worker_index: int) -> None:
        while True:
            try:
                job = job_queue.get_nowait()
            except queue.Empty:
                return
            result = run_job(job)
            with lock:
                with results_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps({
                        "worker": worker_index,
                        "finished_at_unix": time.time(),
                        **result,
                    }) + "\n")
                worker_stats[worker_index]["jobs"] += 1
                worker_stats[worker_index]["elapsed_seconds"] += float(
                    result["elapsed_seconds"]
                )
                if result["complete"]:
                    complete.add((
                        str(result["condition_id"]),
                        int(result["trial_id"]),
                    ))
                else:
                    failures.append((
                        str(result["condition_id"]),
                        int(result["trial_id"]),
                    ))
                write_json_atomic(
                    staging_root / "progress.json",
                    {
                        "status": "running",
                        "workers": workers,
                        "logical_processors": logical,
                        "jobs_complete": len(complete),
                        "jobs_total": len(jobs),
                        "jobs_pending": job_queue.qsize(),
                        "job_failures": len(failures),
                        "worker_stats": worker_stats,
                        "updated_at_unix": time.time(),
                    },
                )
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
    status = "jobs_complete" if not failures and len(complete) == len(jobs) else "incomplete"
    write_json_atomic(
        staging_root / "progress.json",
        {
            "status": status,
            "workers": workers,
            "logical_processors": logical,
            "jobs_complete": len(complete),
            "jobs_total": len(jobs),
            "job_failures": len(failures),
            "failures": failures,
            "worker_stats": worker_stats,
            "updated_at_unix": time.time(),
        },
    )
    if status != "jobs_complete":
        raise RuntimeError(
            f"campaign has {len(failures)} failed jobs and "
            f"{len(jobs) - len(complete)} incomplete jobs"
        )


def write_csv_atomic(path: Path, rows: Iterable[dict[str, str]]) -> None:
    materialized = list(rows)
    if not materialized:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    fields = list(materialized[0])
    if any(list(row) != fields for row in materialized[1:]):
        raise RuntimeError(f"schema mismatch while merging {path.name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(materialized)
    temporary.replace(path)


def merge(
    run_root: Path,
    staging_root: Path,
    jobs: Sequence[dict[str, object]],
) -> Path:
    conditions = load_collaborative_conditions(run_root)
    conditions_by_id = {row["condition_id"]: row for row in conditions}
    jobs_by_condition: dict[str, list[dict[str, object]]] = {}
    for job in jobs:
        jobs_by_condition.setdefault(str(job["condition_id"]), []).append(job)
    merged_environment = staging_root / "merged" / ENVIRONMENT
    if merged_environment.exists():
        validate_merged(run_root, merged_environment)
        return merged_environment

    for condition_id, condition_jobs in jobs_by_condition.items():
        condition = conditions_by_id[condition_id]
        label = {
            "1": "100",
            "0.75": "075",
            "0.5": "050",
            "0.25": "025",
            "0.1": "010",
            "0.05": "005",
        }[condition["top_k_rate"]]
        condition_out = (
            merged_environment / condition["algorithm_key"] / f"topk_{label}"
        )
        canonical_scenario = str(Path(condition["scenario_file"]).resolve())
        for name in OUTPUT_FILES:
            rows: list[dict[str, str]] = []
            for job in condition_jobs:
                shard_rows = read_csv_rows(Path(str(job["out_dir"])) / name)
                if not shard_rows:
                    raise RuntimeError(
                        f"{condition_id} trial {job['trial_id']}: missing {name}"
                    )
                for row in shard_rows:
                    row["scenario_file"] = canonical_scenario
                rows.extend(shard_rows)
            rows.sort(key=lambda row: (
                int(row["trial_id"]),
                row.get("robot_id", ""),
                int(row.get("target_index", "0") or 0),
            ))
            write_csv_atomic(condition_out / name, rows)
        first_config = Path(str(condition_jobs[0]["out_dir"])) / "config_used.json"
        config = json.loads(first_config.read_text(encoding="utf-8"))
        config["scenario_file"] = canonical_scenario
        config["scenario_sha256"] = file_sha256(Path(canonical_scenario))
        config["num_trials"] = SCENARIO_COUNT
        config["timing_clock"] = "time.perf_counter_ns"
        config["allocator_time_definition"] = "complete choose_goal call including filter"
        config["allocator_solve_time_definition"] = "allocator total minus nested filter time"
        config["candidate_filter_time_definition"] = (
            "candidate discovery, ranking, and truncation"
        )
        write_json_atomic(condition_out / "config_used.json", config)
        write_json_atomic(
            condition_out / "campaign_provenance.json",
            {
                "condition_id": condition_id,
                "scenario_sha256": config["scenario_sha256"],
                "trial_ids": list(range(SCENARIO_COUNT)),
                "source_shards": len(condition_jobs),
                "merged_at_unix": time.time(),
            },
        )
        (condition_out / "run.log").write_text(
            f"merged {len(condition_jobs)} isolated timing-enabled trials\n",
            encoding="utf-8",
        )
    validate_merged(run_root, merged_environment)
    return merged_environment


def validate_merged(run_root: Path, environment_root: Path) -> None:
    conditions = load_collaborative_conditions(run_root)
    for condition in conditions:
        label = {
            "1": "100",
            "0.75": "075",
            "0.5": "050",
            "0.25": "025",
            "0.1": "010",
            "0.05": "005",
        }[condition["top_k_rate"]]
        out_dir = environment_root / condition["algorithm_key"] / f"topk_{label}"
        rows = {name: read_csv_rows(out_dir / name) for name in OUTPUT_FILES}
        expected = {
            "trial_summary.csv": SCENARIO_COUNT,
            "system_performance.csv": SCENARIO_COUNT,
            "robot_performance.csv": SCENARIO_COUNT * ROBOT_COUNT,
            "target_performance.csv": SCENARIO_COUNT * TARGET_COUNT,
        }
        counts = {name: len(items) for name, items in rows.items()}
        if counts != expected:
            raise RuntimeError(
                f"{condition['condition_id']}: merged counts {counts} != {expected}"
            )
        for name, items in rows.items():
            ids = {int(item["trial_id"]) for item in items}
            if ids != set(range(SCENARIO_COUNT)):
                raise RuntimeError(
                    f"{condition['condition_id']}: wrong trial IDs in {name}"
                )
            if any(
                item.get("trial_status", "").strip().lower() != "completed"
                for item in items
            ):
                raise RuntimeError(
                    f"{condition['condition_id']}: failed row in {name}"
                )
        robots_by_trial: dict[int, list[dict[str, str]]] = {}
        for row in rows["robot_performance.csv"]:
            robots_by_trial.setdefault(int(row["trial_id"]), []).append(row)
        for system in rows["system_performance.csv"]:
            trial_id = int(system["trial_id"])
            validate_timing_rows(
                [system],
                robots_by_trial[trial_id],
                f"{condition['condition_id']} trial {trial_id}",
            )


def _require_descendant(path: Path, parent: Path) -> None:
    resolved_path = path.resolve()
    resolved_parent = parent.resolve()
    if resolved_path == resolved_parent or resolved_parent not in resolved_path.parents:
        raise RuntimeError(f"unsafe path outside expected parent: {resolved_path}")


def write_computational_summary(run_root: Path) -> None:
    rows: list[dict[str, object]] = []
    for condition in load_collaborative_conditions(run_root):
        out_dir = Path(condition["out_dir"])
        robot_rows = read_csv_rows(out_dir / "robot_performance.csv")
        allocator_calls = sum(int(_numeric(row, "allocator_calls")) for row in robot_rows)
        filter_calls = sum(
            int(_numeric(row, "candidate_filter_calls")) for row in robot_rows
        )
        allocator_total = sum(
            _numeric(row, "allocator_time_ms_total") for row in robot_rows
        )
        solve_total = sum(
            _numeric(row, "allocator_solve_time_ms_total") for row in robot_rows
        )
        filter_total = sum(
            _numeric(row, "candidate_filter_time_ms_total") for row in robot_rows
        )
        rows.append({
            "condition_id": condition["condition_id"],
            "algorithm": condition["algorithm_name"],
            "top_k_rate": condition["top_k_rate"],
            "top_k_max_cells": condition["top_k_max_cells"],
            "scenario_count": SCENARIO_COUNT,
            "robot_rows": len(robot_rows),
            "allocator_calls": allocator_calls,
            "allocator_time_ms_total": allocator_total,
            "allocator_time_ms_mean": (
                allocator_total / allocator_calls if allocator_calls else 0.0
            ),
            "allocator_solve_time_ms_total": solve_total,
            "allocator_solve_time_ms_mean": (
                solve_total / allocator_calls if allocator_calls else 0.0
            ),
            "candidate_filter_calls": filter_calls,
            "candidate_filter_time_ms_total": filter_total,
            "candidate_filter_time_ms_mean": (
                filter_total / filter_calls if filter_calls else 0.0
            ),
        })
    write_csv_atomic(run_root / "collaborative_computational_summary.csv", rows)


def install_results(run_root: Path, staging_root: Path, merged_root: Path) -> None:
    canonical = raw_environment_dir(
        run_root,
        Environment(
            SUITE_NAME,
            ENVIRONMENT,
            "known_visit",
            19,
            ROBOT_COUNT,
            "known_targets_g19_t50_n100.csv",
            "known_targets_g19_t50",
            target_count=TARGET_COUNT,
            trials_per_condition=SCENARIO_COUNT,
        ),
    )
    backup = staging_root / "previous_collaborative_results"
    _require_descendant(canonical, run_root)
    _require_descendant(backup, staging_root)
    _require_descendant(merged_root, staging_root)
    if backup.exists():
        raise RuntimeError(f"unexpected existing replacement backup: {backup}")
    if not canonical.exists():
        raise RuntimeError(f"missing prior collaborative results: {canonical}")
    canonical.rename(backup)
    try:
        merged_root.rename(canonical)
        validate_merged(run_root, canonical)
        verify(run_root)
        report(run_root)
        write_computational_summary(run_root)
    except Exception:
        if canonical.exists():
            failed = staging_root / "failed_new_results"
            if failed.exists():
                raise RuntimeError(
                    "new result installation failed and failed_new_results exists"
                )
            canonical.rename(failed)
        backup.rename(canonical)
        raise
    shutil.rmtree(backup)
    write_json_atomic(
        staging_root / "progress.json",
        {
            "status": "complete",
            "conditions": CONDITION_COUNT,
            "scenarios_per_condition": SCENARIO_COUNT,
            "jobs_total": CONDITION_COUNT * SCENARIO_COUNT,
            "canonical_results": str(canonical.resolve()),
            "completed_at_unix": time.time(),
        },
    )


def status(staging_root: Path) -> None:
    progress = staging_root / "progress.json"
    if not progress.exists():
        print(json.dumps({"status": "not_prepared"}, indent=2))
        return
    print(progress.read_text(encoding="utf-8"))


def main() -> None:
    logical = os.cpu_count() or 1
    default_workers = max(1, int(logical * 0.75))
    parser = argparse.ArgumentParser(
        description="Run and replace the 100-scenario collaborative visit campaign."
    )
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--staging-root", type=Path)
    parser.add_argument("--workers", type=int, default=default_workers)
    parser.add_argument(
        "command",
        choices=("prepare", "run", "finalize", "all", "status", "backfill-maxima"),
    )
    args = parser.parse_args()
    run_root = args.run_root.expanduser().resolve()
    staging_root = (
        args.staging_root.expanduser().resolve()
        if args.staging_root
        else (run_root / "_cv100").resolve()
    )
    if args.command == "status":
        status(staging_root)
        return
    if args.command == "backfill-maxima":
        environment_root = raw_environment_dir(
            run_root,
            Environment(
                SUITE_NAME,
                ENVIRONMENT,
                "known_visit",
                19,
                ROBOT_COUNT,
                "known_targets_g19_t50_n100.csv",
                "known_targets_g19_t50",
                target_count=TARGET_COUNT,
                trials_per_condition=SCENARIO_COUNT,
            ),
        )
        updated = backfill_system_maxima(environment_root)
        validate_merged(run_root, environment_root)
        print(f"backfilled and verified {updated} collaborative system rows")
        return
    jobs = prepare(run_root, staging_root)
    if args.command == "prepare":
        print(f"prepared {len(jobs)} jobs at {staging_root}")
        return
    if args.command in {"run", "all"}:
        execute(staging_root, jobs, args.workers)
    if args.command in {"finalize", "all"}:
        if len(completed_job_keys(staging_root / "job_results.jsonl")) != len(jobs):
            raise RuntimeError("cannot finalize before every shard is complete")
        merged_root = merge(run_root, staging_root, jobs)
        install_results(run_root, staging_root, merged_root)
        print(f"installed verified collaborative results at {run_root}")


if __name__ == "__main__":
    main()
