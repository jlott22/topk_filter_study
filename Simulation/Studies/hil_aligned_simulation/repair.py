from __future__ import annotations

import argparse
import csv
import json
import queue
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from .core import (
    BAYESIAN_FIRST_PASS_EVENT_CAP,
    BAYESIAN_TRIALS,
    RECORDS_DIR,
    ROBOT_COUNT,
    backfill_system_timing,
    load_manifest,
    read_csv_rows,
    subprocess_environment,
    write_csv_rows,
    write_json,
)


REPAIR_ROOT = RECORDS_DIR / "bayesian_trial_repair"


@dataclass(frozen=True)
class RepairJob:
    condition_id: str
    row: dict[str, str]
    trial_id: int
    event_cap: int
    out_dir: Path
    log_path: Path


@dataclass(frozen=True)
class RepairResult:
    condition_id: str
    trial_id: int
    returncode: int
    event_cap: int
    out_dir: str
    log_path: str


def _replace_option(command: list[str], option: str, value: str) -> list[str]:
    result = list(command)
    try:
        index = result.index(option)
    except ValueError:
        result.extend([option, value])
    else:
        result[index + 1] = value
    return result


def _trial_id(row: dict[str, str]) -> int | None:
    value = row.get("trial_id", "")
    try:
        return int(value)
    except ValueError:
        return None


def _ids_in_rows(rows: list[dict[str, str]]) -> set[int]:
    ids: set[int] = set()
    for row in rows:
        trial_id = _trial_id(row)
        if trial_id is not None:
            ids.add(trial_id)
    return ids


def _failed_ids(out_dir: Path) -> set[int]:
    failed: set[int] = set()
    for row in read_csv_rows(out_dir / "trial_summary.csv"):
        if row.get("trial_status", "").strip().lower() == "failed":
            trial_id = _trial_id(row)
            if trial_id is not None:
                failed.add(trial_id)
    return failed


def _missing_ids(row: dict[str, str]) -> set[int]:
    out_dir = Path(row["out_dir"])
    recorded = _ids_in_rows(read_csv_rows(out_dir / "trial_summary.csv"))
    return set(range(int(row["num_trials"]))) - recorded


def _needs_repair(row: dict[str, str]) -> bool:
    return row["mission"] == "bayesian" and bool(_missing_ids(row))


def _jobs_for_condition(row: dict[str, str], *, retry_event_cap: int) -> list[RepairJob]:
    out_dir = Path(row["out_dir"])
    missing = _missing_ids(row)
    if not missing:
        return []

    # Complete conditions with residual failed rows have already exhausted the
    # campaign retry ladder. This repair pass only fills genuinely missing rows.
    target_ids = missing
    jobs: list[RepairJob] = []
    use_retry_cap_for_missing = row["algorithm_key"] != "dga"
    for trial_id in sorted(target_ids):
        if trial_id not in missing or use_retry_cap_for_missing:
            event_cap = retry_event_cap
        else:
            event_cap = BAYESIAN_FIRST_PASS_EVENT_CAP
        shard_dir = REPAIR_ROOT / row["condition_id"] / f"trial_{trial_id:03d}_cap_{event_cap}"
        jobs.append(
            RepairJob(
                condition_id=row["condition_id"],
                row=row,
                trial_id=trial_id,
                event_cap=event_cap,
                out_dir=shard_dir,
                log_path=REPAIR_ROOT / "logs" / f"{row['condition_id']}_trial_{trial_id:03d}.log",
            )
        )
    return jobs


def _job_has_output(job: RepairJob) -> bool:
    rows = read_csv_rows(job.out_dir / "trial_summary.csv")
    return any(_trial_id(row) == job.trial_id for row in rows)


def _command_for_job(job: RepairJob) -> list[str]:
    command = json.loads(job.row["command"])
    command = _replace_option(command, "--out-dir", str(job.out_dir))
    command = _replace_option(command, "--debug-max-events", str(job.event_cap))
    command = _replace_option(command, "--trial-shard-count", str(BAYESIAN_TRIALS))
    command = _replace_option(command, "--trial-shard-index", str(job.trial_id))
    return command


def _run_job(job: RepairJob) -> RepairResult:
    job.log_path.parent.mkdir(parents=True, exist_ok=True)
    command = _command_for_job(job)
    with job.log_path.open("a", encoding="utf-8") as log:
        log.write(f"START {time.strftime('%Y-%m-%dT%H:%M:%S')} {json.dumps(command)}\n")
        log.flush()
        process = subprocess.run(
            command,
            cwd=job.row["working_directory"],
            env=subprocess_environment(),
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        log.write(f"END returncode={process.returncode}\n")
        log.flush()
    return RepairResult(
        condition_id=job.condition_id,
        trial_id=job.trial_id,
        returncode=int(process.returncode),
        event_cap=job.event_cap,
        out_dir=str(job.out_dir),
        log_path=str(job.log_path),
    )


def _sort_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    def key(row: dict[str, str]) -> tuple[int, str, str]:
        trial_id = _trial_id(row)
        return (
            trial_id if trial_id is not None else sys.maxsize,
            row.get("robot_id", ""),
            row.get("target_id", ""),
        )

    return sorted(rows, key=key)


def _merge_condition(row: dict[str, str], jobs: list[RepairJob]) -> None:
    out_dir = Path(row["out_dir"])
    repaired_ids = {job.trial_id for job in jobs}
    for filename in (
        "trial_summary.csv",
        "system_performance.csv",
        "robot_performance.csv",
        "computational_performance.csv",
    ):
        merged = [
            item
            for item in read_csv_rows(out_dir / filename)
            if (_trial_id(item) not in repaired_ids)
        ]
        for job in jobs:
            merged.extend(read_csv_rows(job.out_dir / filename))
        write_csv_rows(out_dir / filename, _sort_rows(merged))
    backfill_system_timing(out_dir)


def repair_bayesian_missing_trials(workers: int, retry_event_cap: int) -> None:
    rows = [row for row in load_manifest() if _needs_repair(row)]
    all_jobs: list[RepairJob] = []
    for row in rows:
        all_jobs.extend(_jobs_for_condition(row, retry_event_cap=retry_event_cap))
    if not all_jobs:
        print("no incomplete Bayesian conditions to repair", flush=True)
        return
    pending_jobs = [job for job in all_jobs if not _job_has_output(job)]

    work: queue.Queue[RepairJob] = queue.Queue()
    for job in pending_jobs:
        work.put(job)
    results: list[RepairResult] = []
    lock = threading.Lock()

    def worker() -> None:
        while True:
            try:
                job = work.get_nowait()
            except queue.Empty:
                return
            result = _run_job(job)
            with lock:
                results.append(result)
                write_json(
                    REPAIR_ROOT / "progress.json",
                    {
                        "status": "running",
                        "completed_jobs": len(results),
                        "skipped_existing_jobs": len(all_jobs) - len(pending_jobs),
                        "pending_jobs": len(pending_jobs),
                        "total_jobs": len(all_jobs),
                        "results": [item.__dict__ for item in results],
                        "updated_at_unix": time.time(),
                    },
                )
                print(
                    f"{result.condition_id} trial={result.trial_id} "
                    f"returncode={result.returncode}",
                    flush=True,
                )
            work.task_done()

    threads = [threading.Thread(target=worker, daemon=True) for _ in range(max(1, workers))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    failed_results = [result for result in results if result.returncode != 0]
    write_json(
        REPAIR_ROOT / "progress.json",
        {
            "status": "failed" if failed_results else "complete",
            "completed_jobs": len(results),
            "skipped_existing_jobs": len(all_jobs) - len(pending_jobs),
            "pending_jobs": len(pending_jobs),
            "total_jobs": len(all_jobs),
            "results": [item.__dict__ for item in results],
            "updated_at_unix": time.time(),
        },
    )
    if failed_results:
        raise SystemExit(f"{len(failed_results)} repair shard(s) failed")

    by_condition: dict[str, list[RepairJob]] = {}
    for job in all_jobs:
        if _job_has_output(job):
            by_condition.setdefault(job.condition_id, []).append(job)
    for row in rows:
        _merge_condition(row, by_condition.get(row["condition_id"], []))
        print(f"merged {row['condition_id']}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--retry-event-cap", type=int, default=100_000)
    args = parser.parse_args()
    repair_bayesian_missing_trials(args.workers, args.retry_event_cap)


if __name__ == "__main__":
    main()
