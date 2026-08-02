from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from .suite import (
    DEFAULT_RUN_ROOT,
    load_manifest,
    read_csv_rows,
    report,
    trial_failures,
    verify,
)


def replace_arg(command: list[str], option: str, value: str) -> None:
    try:
        index = command.index(option)
    except ValueError as exc:
        raise RuntimeError(f"missing required command option {option}") from exc
    command[index + 1] = value


def failure_ids(row: dict[str, str]) -> list[int]:
    return sorted(int(item["trial_id"]) for item in trial_failures(row))


def run_job(job: dict[str, object]) -> dict[str, object]:
    started = time.time()
    log_path = Path(str(job["log_path"]))
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
    condition = dict(job["manifest_row"])
    remaining = failure_ids(condition)
    return {
        "condition_id": condition["condition_id"],
        "retried_trial_ids": job["failed_trial_ids"],
        "remaining_failed_trial_ids": remaining,
        "returncode": process.returncode,
        "elapsed_seconds": time.time() - started,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Retry sensitivity trials that reached the debug event cap."
    )
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--work-root", type=Path)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--event-cap", type=int, default=50_000)
    args = parser.parse_args()
    if args.workers != 12:
        raise ValueError("event-cap retries are locked to 12 workers")
    if args.event_cap < 1:
        raise ValueError("event cap must be positive")

    run_root = args.run_root.resolve()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    work_root = (
        args.work_root.resolve()
        if args.work_root is not None
        else run_root / "event_cap_retries" / stamp
    )
    work_root.mkdir(parents=True, exist_ok=False)

    affected: list[dict[str, str]] = []
    original_failures: list[dict[str, str]] = []
    for row in load_manifest(run_root):
        failures = trial_failures(row)
        if not failures:
            continue
        for failure in failures:
            if (
                failure.get("failure_type") != "RuntimeError"
                or not failure.get("failure_message", "").startswith(
                    "Debug safety cap reached"
                )
            ):
                raise RuntimeError(
                    f"refusing to retry non-cap failure in {row['condition_id']}"
                )
            original_failures.append({
                "condition_id": row["condition_id"],
                "trial_id": failure["trial_id"],
                "failure_type": failure["failure_type"],
                "failure_message": failure["failure_message"],
            })
        affected.append(row)

    if not affected:
        raise RuntimeError("no event-cap failures found")

    backup_root = work_root / "canonical_backup"
    log_root = work_root / "logs"
    backup_root.mkdir()
    log_root.mkdir()
    jobs: list[dict[str, object]] = []
    for row in affected:
        shutil.copytree(
            Path(row["out_dir"]),
            backup_root / row["condition_id"],
        )
        command = list(json.loads(row["command"]))
        replace_arg(command, "--debug-max-events", str(args.event_cap))
        if "--retry-failed" not in command:
            command.append("--retry-failed")
        jobs.append({
            "condition_id": row["condition_id"],
            "failed_trial_ids": failure_ids(row),
            "working_directory": row["working_directory"],
            "manifest_row": row,
            "command": command,
            "log_path": str(log_root / f"{row['condition_id']}.log"),
        })

    provenance = {
        "schema": 1,
        "created_at_unix": time.time(),
        "pid": os.getpid(),
        "event_caps_before": sorted({
            int(item.get("debug_max_events") or 0)
            for row in affected
            for item in trial_failures(row)
        }),
        "event_cap_after": args.event_cap,
        "workers": args.workers,
        "run_root": str(run_root),
        "work_root": str(work_root),
        "original_failures": original_failures,
        "jobs": jobs,
    }
    (work_root / "retry_manifest.json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8"
    )

    results_path = work_root / "retry_results.jsonl"
    results: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {executor.submit(run_job, job): job for job in jobs}
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            with results_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(result) + "\n")
            print(json.dumps(result), flush=True)

    bad_processes = [item for item in results if item["returncode"] != 0]
    if bad_processes:
        raise RuntimeError(
            f"{len(bad_processes)} retry process(es) exited nonzero; inspect {work_root}"
        )

    verification = verify(run_root)
    report(run_root)
    remaining_failures = read_csv_rows(run_root / "failures.csv")
    summary = {
        "completed_at_unix": time.time(),
        "conditions_retried": len(affected),
        "trials_retried": len(original_failures),
        "remaining_failures": len(remaining_failures),
        "verification_issues": verification["verification_issues"],
    }
    (work_root / "retry_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
