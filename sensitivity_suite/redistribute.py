from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Iterable, Sequence

from .suite import DEFAULT_RUN_ROOT, load_manifest, output_complete, read_csv_rows


OUTPUT_FILES = (
    "trial_summary.csv",
    "system_performance.csv",
    "robot_performance.csv",
    "computational_performance.csv",
)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def trial_id(row: dict[str, str]) -> int:
    return int(row["trial_id"])


def rows_by_trial(path: Path) -> dict[int, list[dict[str, str]]]:
    grouped: dict[int, list[dict[str, str]]] = {}
    for row in read_csv_rows(path):
        grouped.setdefault(trial_id(row), []).append(row)
    return grouped


def validate_checkpoint(row: dict[str, str]) -> set[int]:
    out_dir = Path(row["out_dir"])
    expected_robots = int(row["robot_count"])
    trial_groups = {
        name: rows_by_trial(out_dir / name)
        for name in OUTPUT_FILES
    }
    recorded = set(trial_groups["trial_summary.csv"])
    if recorded != set(trial_groups["system_performance.csv"]):
        raise RuntimeError(f"{row['condition_id']}: trial/system trial IDs differ")
    if recorded != set(trial_groups["robot_performance.csv"]):
        raise RuntimeError(f"{row['condition_id']}: robot trial IDs differ")
    if recorded != set(trial_groups["computational_performance.csv"]):
        raise RuntimeError(f"{row['condition_id']}: computational trial IDs differ")
    for current_trial in recorded:
        if len(trial_groups["trial_summary.csv"][current_trial]) != 1:
            raise RuntimeError(f"{row['condition_id']}: duplicate trial row {current_trial}")
        if len(trial_groups["system_performance.csv"][current_trial]) != 1:
            raise RuntimeError(f"{row['condition_id']}: duplicate system row {current_trial}")
        if len(trial_groups["robot_performance.csv"][current_trial]) != expected_robots:
            raise RuntimeError(f"{row['condition_id']}: wrong robot rows for {current_trial}")
        if (
            len(trial_groups["computational_performance.csv"][current_trial])
            != expected_robots
        ):
            raise RuntimeError(
                f"{row['condition_id']}: wrong computational rows for {current_trial}"
            )
    return recorded


def scenario_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(
            line for line in handle
            if line.strip() and not line.lstrip().startswith("#")
        )
        if reader.fieldnames is None:
            raise RuntimeError(f"missing scenario header: {path}")
        return list(reader.fieldnames), list(reader)


def write_scenario(path: Path, fields: Sequence[str], row: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# scenario_set=topk_readonly_sensitivity_redistributed\n")
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


def prepare(run_root: Path, work_root: Path) -> list[dict[str, object]]:
    manifest_rows = load_manifest(run_root)
    incomplete = [row for row in manifest_rows if not output_complete(row)]
    if not incomplete:
        raise RuntimeError("campaign is already complete")

    work_root.mkdir(parents=True, exist_ok=False)
    jobs: list[dict[str, object]] = []
    condition_records: list[dict[str, object]] = []
    for condition in incomplete:
        recorded = validate_checkpoint(condition)
        source_path = Path(condition["scenario_file"])
        fields, source_rows = scenario_rows(source_path)
        source_by_id = {trial_id(item): item for item in source_rows}
        expected_ids = set(source_by_id)
        pending = sorted(expected_ids - recorded)
        if len(expected_ids) != int(condition["num_trials"]):
            raise RuntimeError(
                f"{condition['condition_id']}: scenario count does not match manifest"
            )
        condition_records.append({
            "condition_id": condition["condition_id"],
            "canonical_out_dir": condition["out_dir"],
            "source_scenario": str(source_path),
            "source_scenario_sha256": file_sha256(source_path),
            "recorded_trial_ids": sorted(recorded),
            "pending_trial_ids": pending,
        })
        for pending_id in pending:
            job_dir = (
                work_root / "shards" / condition["condition_id"]
                / f"trial_{pending_id:03d}"
            ).resolve()
            shard_scenario = job_dir / "scenario.csv"
            shard_out = job_dir / "output"
            write_scenario(shard_scenario, fields, source_by_id[pending_id])
            command = list(json.loads(condition["command"]))
            replace_arg(command, "--scenario-file", str(shard_scenario))
            replace_arg(command, "--max-trials", "1")
            replace_arg(command, "--out-dir", str(shard_out))
            jobs.append({
                "condition_id": condition["condition_id"],
                "trial_id": pending_id,
                "canonical_out_dir": condition["out_dir"],
                "working_directory": condition["working_directory"],
                "scenario_file": str(shard_scenario),
                "scenario_sha256": file_sha256(shard_scenario),
                "out_dir": str(shard_out),
                "log_path": str(job_dir / "run.log"),
                "command": command,
            })

    provenance = {
        "schema": 1,
        "created_at_unix": time.time(),
        "pid": os.getpid(),
        "run_root": str(run_root.resolve()),
        "work_root": str(work_root.resolve()),
        "conditions": condition_records,
        "jobs": jobs,
    }
    (work_root / "redistribution_manifest.json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8"
    )
    return jobs


def run_job(job: dict[str, object]) -> dict[str, object]:
    log_path = Path(str(job["log_path"]))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
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
        log_handle.write(
            f"END {time.strftime('%Y-%m-%dT%H:%M:%S')} "
            f"returncode={process.returncode}\n"
        )
    out_dir = Path(str(job["out_dir"]))
    counts = {name: len(read_csv_rows(out_dir / name)) for name in OUTPUT_FILES}
    expected_robots = 4
    complete = (
        process.returncode == 0
        and counts["trial_summary.csv"] == 1
        and counts["system_performance.csv"] == 1
        and counts["robot_performance.csv"] == expected_robots
        and counts["computational_performance.csv"] == expected_robots
    )
    return {
        "condition_id": job["condition_id"],
        "trial_id": job["trial_id"],
        "returncode": process.returncode,
        "complete": complete,
        "counts": counts,
        "elapsed_seconds": time.time() - started,
    }


def execute(work_root: Path, workers: int) -> None:
    manifest_path = work_root / "redistribution_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    jobs = list(manifest["jobs"])
    results_path = work_root / "job_results.jsonl"
    completed_keys: set[tuple[str, int]] = set()
    if results_path.exists():
        for line in results_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            item = json.loads(line)
            if item.get("complete"):
                completed_keys.add((str(item["condition_id"]), int(item["trial_id"])))
    pending = [
        job for job in jobs
        if (str(job["condition_id"]), int(job["trial_id"])) not in completed_keys
    ]
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(run_job, job): job for job in pending}
        for future in as_completed(futures):
            result = future.result()
            with results_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(result) + "\n")
            print(json.dumps(result), flush=True)
            if not result["complete"]:
                raise RuntimeError(
                    f"shard failed: {result['condition_id']} trial {result['trial_id']}"
                )


def write_csv_atomic(path: Path, rows: Iterable[dict[str, str]]) -> None:
    materialized = list(rows)
    if not materialized:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    fields = list(materialized[0])
    for row in materialized[1:]:
        if list(row) != fields:
            raise RuntimeError(f"schema mismatch while merging {path.name}")
    temporary = path.with_suffix(path.suffix + ".redistribute.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(materialized)
    temporary.replace(path)


def merge(run_root: Path, work_root: Path) -> None:
    provenance = json.loads(
        (work_root / "redistribution_manifest.json").read_text(encoding="utf-8")
    )
    jobs = list(provenance["jobs"])
    by_condition: dict[str, list[dict[str, object]]] = {}
    for job in jobs:
        by_condition.setdefault(str(job["condition_id"]), []).append(job)

    manifest_by_id = {
        row["condition_id"]: row for row in load_manifest(run_root)
    }
    backup_root = work_root / "canonical_backup"
    merge_records: list[dict[str, object]] = []
    for condition_id, condition_jobs in by_condition.items():
        condition = manifest_by_id[condition_id]
        canonical = Path(condition["out_dir"])
        validate_checkpoint(condition)
        backup_dir = backup_root / condition_id
        backup_dir.mkdir(parents=True, exist_ok=False)
        for name in OUTPUT_FILES:
            shutil.copy2(canonical / name, backup_dir / name)

        merged_by_file: dict[str, list[dict[str, str]]] = {}
        for name in OUTPUT_FILES:
            rows = read_csv_rows(canonical / name)
            for job in condition_jobs:
                shard_rows = read_csv_rows(Path(str(job["out_dir"])) / name)
                if not shard_rows:
                    raise RuntimeError(
                        f"{condition_id} trial {job['trial_id']}: missing {name}"
                    )
                rows.extend(shard_rows)
            rows.sort(key=lambda item: (trial_id(item), item.get("robot_id", "")))
            merged_by_file[name] = rows

        expected_trials = int(condition["num_trials"])
        expected_robots = int(condition["robot_count"])
        expected_counts = {
            "trial_summary.csv": expected_trials,
            "system_performance.csv": expected_trials,
            "robot_performance.csv": expected_trials * expected_robots,
            "computational_performance.csv": expected_trials * expected_robots,
        }
        for name, expected in expected_counts.items():
            rows = merged_by_file[name]
            if len(rows) != expected:
                raise RuntimeError(
                    f"{condition_id}: {name} has {len(rows)}, expected {expected}"
                )
            grouped = {}
            for item in rows:
                grouped.setdefault(trial_id(item), []).append(item)
            if set(grouped) != set(range(expected_trials)):
                raise RuntimeError(f"{condition_id}: missing or unexpected IDs in {name}")
            per_trial = 1 if name in {
                "trial_summary.csv", "system_performance.csv"
            } else expected_robots
            if any(len(items) != per_trial for items in grouped.values()):
                raise RuntimeError(f"{condition_id}: duplicate or missing rows in {name}")

        for name, rows in merged_by_file.items():
            write_csv_atomic(canonical / name, rows)
        if not output_complete(condition):
            raise RuntimeError(f"{condition_id}: canonical output incomplete after merge")
        merge_records.append({
            "condition_id": condition_id,
            "merged_at_unix": time.time(),
            "trials_added": sorted(int(job["trial_id"]) for job in condition_jobs),
            "backup_dir": str(backup_dir.resolve()),
            "canonical_out_dir": str(canonical.resolve()),
            "canonical_hashes": {
                name: file_sha256(canonical / name) for name in OUTPUT_FILES
            },
        })

    (work_root / "merge_provenance.json").write_text(
        json.dumps(merge_records, indent=2) + "\n", encoding="utf-8"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Safely redistribute incomplete sensitivity trials."
    )
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--work-root", type=Path)
    parser.add_argument("--workers", type=int, default=12)
    args = parser.parse_args()
    if args.workers != 12:
        raise ValueError("redistribution is locked to 12 workers")
    work_root = args.work_root
    if work_root is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        work_root = args.run_root / "redistribution" / stamp
    jobs = prepare(args.run_root, work_root)
    print(f"Prepared {len(jobs)} isolated trial shards in {work_root}", flush=True)
    execute(work_root, args.workers)
    merge(args.run_root, work_root)
    print(f"Redistribution complete and merged from {work_root}", flush=True)


if __name__ == "__main__":
    main()
