#!/usr/bin/env python3
"""Rerun clue-search or coverage core GE conditions with corrected burst loss."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RHO = 0.8
SUITES = {
    "clue": {
        "source_manifest": REPO_ROOT / "clue_500_combined" / "condition_manifest.csv",
        "run_root": REPO_ROOT / "runs" / "clue_core_500_ge_bursty_rho08",
        "expected_conditions": 48,
        "expected_trials": 500,
    },
    "coverage": {
        "source_manifest": REPO_ROOT / "coverage_100_combined" / "condition_manifest.csv",
        "run_root": REPO_ROOT / "runs" / "coverage_core_100_ge_bursty_rho08",
        "expected_conditions": 48,
        "expected_trials": 100,
    },
}


@dataclass(frozen=True)
class Job:
    suite: str
    algorithm: str
    level: float
    drop: float
    p_gg: float
    p_bb: float
    expected_trials: int
    condition_id: str
    out_dir: Path
    command: tuple[str, ...]
    source_command: str
    scenario_file: str


def replace_arg(command: list[str], option: str, value: str) -> None:
    index = command.index(option)
    command[index + 1] = value


def row_count(path: Path) -> tuple[int, int]:
    if not path.exists() or path.stat().st_size == 0:
        return 0, 0
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    failures = sum(str(row.get("trial_status", "")).lower() == "failed" for row in rows)
    return len(rows), failures


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_jobs(
    suite: str,
    run_root: Path,
    debug_max_events: int | None = None,
) -> list[Job]:
    config = SUITES[suite]
    with config["source_manifest"].open(newline="", encoding="utf-8-sig") as handle:
        source_rows = list(csv.DictReader(handle))
    ge_rows = [row for row in source_rows if row["comm_model"] == "gilbert_elliot"]
    if len(ge_rows) != config["expected_conditions"]:
        raise RuntimeError(f"Expected {config['expected_conditions']} GE rows, found {len(ge_rows)}")

    jobs: list[Job] = []
    for row in ge_rows:
        level = float(row["comm_level"])
        drop = 1.0 - level
        p_gg = level + RHO * (1.0 - level)
        p_bb = (1.0 - level) + RHO * level
        algorithm = row["algorithm"]
        condition_id = f"{algorithm.lower()}_gilbert_elliott_drop_{drop:.2f}_rho_0.8".replace(".", "_")
        folder = f"gilbert_elliott_drop_{drop:.2f}_rho_0.8".replace(".", "_")
        out_dir = run_root / "raw" / algorithm.lower() / folder
        source_command = row["command"]
        command = list(json.loads(source_command))
        command[0] = sys.executable
        replace_arg(command, "--out-dir", str(out_dir))
        replace_arg(command, "--condition-id", condition_id)
        if debug_max_events is not None:
            if "--debug-max-events" in command:
                replace_arg(command, "--debug-max-events", str(debug_max_events))
            else:
                command.extend(["--debug-max-events", str(debug_max_events)])
        jobs.append(Job(
            suite=suite,
            algorithm=algorithm,
            level=level,
            drop=drop,
            p_gg=p_gg,
            p_bb=p_bb,
            expected_trials=int(row["expected_trials"]),
            condition_id=condition_id,
            out_dir=out_dir,
            command=tuple(command),
            source_command=source_command,
            scenario_file=row.get("scenario_file", ""),
        ))
    return jobs


def write_manifest(run_root: Path, jobs: list[Job], workers: int) -> None:
    run_root.mkdir(parents=True, exist_ok=True)
    fields = [
        "stage", "algorithm", "comm_model", "target_drop_fraction",
        "comm_level_stationary_delivery", "state_correlation", "p_gg", "p_bb",
        "scenario_file", "scenario_sha256", "seed", "expected_trials",
        "condition_id", "out_dir", "source_command", "command",
    ]
    with (run_root / "condition_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for job in jobs:
            scenario_hash = ""
            if job.scenario_file:
                scenario_path = REPO_ROOT / job.scenario_file
                if not scenario_path.exists():
                    raise RuntimeError(f"Original scenario file is missing: {scenario_path}")
                scenario_hash = file_sha256(scenario_path)
            writer.writerow({
                "stage": f"{job.suite}_core_ge_bursty",
                "algorithm": job.algorithm,
                "comm_model": "gilbert_elliot",
                "target_drop_fraction": f"{job.drop:g}",
                "comm_level_stationary_delivery": f"{job.level:g}",
                "state_correlation": RHO,
                "p_gg": f"{job.p_gg:g}",
                "p_bb": f"{job.p_bb:g}",
                "scenario_file": job.scenario_file,
                "scenario_sha256": scenario_hash,
                "seed": 0,
                "expected_trials": job.expected_trials,
                "condition_id": job.condition_id,
                "out_dir": str(job.out_dir),
                "source_command": job.source_command,
                "command": json.dumps(job.command),
            })
    metadata = {
        "suite": jobs[0].suite,
        "workers": workers,
        "logical_processors": os.cpu_count(),
        "conditions": len(jobs),
        "expected_trials": sum(job.expected_trials for job in jobs),
        "state_correlation": RHO,
    }
    (run_root / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def update_progress(run_root: Path, jobs: list[Job]) -> None:
    trials = failures = complete = failed_conditions = 0
    for job in jobs:
        rows, failed = row_count(job.out_dir / "system_performance.csv")
        trials += rows
        failures += failed
        if rows == job.expected_trials and failed == 0:
            complete += 1
        if failed or (job.out_dir / "_FAILED.txt").exists():
            failed_conditions += 1
    progress = {
        "updated_at_unix": time.time(),
        "completed_trials": trials,
        "expected_trials": sum(job.expected_trials for job in jobs),
        "conditions_complete": complete,
        "conditions_total": len(jobs),
        "conditions_with_failures": failed_conditions,
        "failed_trial_rows": failures,
    }
    temporary = run_root / "progress.json.tmp"
    temporary.write_text(json.dumps(progress, indent=2) + "\n", encoding="utf-8")
    temporary.replace(run_root / "progress.json")


def run_job(job: Job) -> dict[str, object]:
    job.out_dir.mkdir(parents=True, exist_ok=True)
    rows, failures = row_count(job.out_dir / "system_performance.csv")
    if rows == job.expected_trials and failures == 0:
        (job.out_dir / "_COMPLETE.txt").write_text("complete\n", encoding="utf-8")
        return {"condition": job.condition_id, "status": "already_complete", "rows": rows}
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    with (job.out_dir / "run.log").open("a", encoding="utf-8") as log:
        log.write(f"\nSTART {time.strftime('%Y-%m-%dT%H:%M:%S')}\n{json.dumps(job.command)}\n")
        log.flush()
        process = subprocess.run(
            job.command, cwd=REPO_ROOT, stdout=log, stderr=subprocess.STDOUT,
            creationflags=creationflags, check=False,
        )
        log.write(f"END status={process.returncode} {time.strftime('%Y-%m-%dT%H:%M:%S')}\n")
    rows, failures = row_count(job.out_dir / "system_performance.csv")
    status = "complete" if process.returncode == 0 and rows == job.expected_trials and failures == 0 else "failed"
    marker = job.out_dir / ("_COMPLETE.txt" if status == "complete" else "_FAILED.txt")
    marker.write_text(f"status={process.returncode}\nrows={rows}\nfailed_trial_rows={failures}\n", encoding="utf-8")
    return {"condition": job.condition_id, "status": status, "rows": rows, "failed": failures}


def inhibit_sleep(enable: bool) -> None:
    if os.name != "nt":
        return
    import ctypes
    continuous, system, away = 0x80000000, 0x00000001, 0x00000040
    ctypes.windll.kernel32.SetThreadExecutionState(continuous | system | away if enable else continuous)


def main() -> None:
    logical = os.cpu_count() or 1
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", required=True, choices=sorted(SUITES))
    parser.add_argument("--workers", type=int, default=max(1, int(logical * 0.75)))
    parser.add_argument("--run-root", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug-max-events", type=int)
    args = parser.parse_args()
    if not 1 <= args.workers <= logical:
        parser.error(f"--workers must be in [1, {logical}]")
    if args.debug_max_events is not None and args.debug_max_events <= 0:
        parser.error("--debug-max-events must be positive")
    run_root = (args.run_root or SUITES[args.suite]["run_root"]).resolve()
    jobs = build_jobs(args.suite, run_root, args.debug_max_events)
    write_manifest(run_root, jobs, args.workers)
    update_progress(run_root, jobs)
    print(f"Prepared {len(jobs)} {args.suite} GE conditions at {run_root}", flush=True)
    if args.dry_run:
        return
    lock = threading.Lock()
    inhibit_sleep(True)
    try:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(run_job, job): job for job in jobs}
            for future in as_completed(futures):
                job = futures[future]
                try:
                    result = future.result()
                except Exception as exc:
                    result = {"condition": job.condition_id, "status": "launcher_error", "error": repr(exc)}
                with lock:
                    with (run_root / "launcher.log").open("a", encoding="utf-8") as handle:
                        handle.write(json.dumps(result) + "\n")
                    update_progress(run_root, jobs)
                    print(json.dumps(result), flush=True)
    finally:
        inhibit_sleep(False)
        update_progress(run_root, jobs)


if __name__ == "__main__":
    main()
