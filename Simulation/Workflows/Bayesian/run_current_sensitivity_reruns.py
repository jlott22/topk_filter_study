from __future__ import annotations

import csv
import ctypes
import json
import os
import queue
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2] / "dcta_benchmark_sim"
PYTHON = sys.executable
N_WORKERS = 12
HEARTBEAT_S = 120

CLUE_SCENARIO = Path("runs/sensitivity_scenarios/clue_tuning_g19_n300_seed20260702.csv")
KNOWN_SCENARIO = Path("scenarios/known_visit_10target_300.csv")
MAX_TRIALS = "300"
GRID_SIZE = "19"

CLUE_HORIZON_ROOT = Path("clue_sensitivity_test_results/horizon_results")
KNOWN_DGA_ITER_ROOT = Path("known_target_sensitivity_test_results/dga_iteration")
COVERAGE_HORIZON_ROOT = Path("coverage_sensitivity_test_results/horizon_results")
COVERAGE_TRIALS = "50"

HORIZONS = [1, 2, 3, 5, 8, 12]
DGA_ITERATIONS = [1, 2, 5, 10, 25, 50]

COMM_CONDITIONS = [
    ("ideal", "ideal", ""),
    ("bernoulli_025", "bernoulli", "0.25"),
]

HORIZON_ALGORITHMS = [
    ("acbba", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator", 2),
    ("pi", "benchmark_sim.algorithms.PI:PIAllocator", 2),
    ("hipc", "benchmark_sim.algorithms.HIPC:HIPCAllocator", 2),
    ("dmchba", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator", 5),
    ("dga", "benchmark_sim.algorithms.DGA:DGAAllocator", 7),
]


@dataclass(frozen=True)
class Job:
    study: str
    run_id: str
    out_dir: Path
    cmd: list[str]
    weight: int
    metadata: dict[str, str]


def log(message: str) -> None:
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {message}", flush=True)


def skipped_run_ids() -> set[str]:
    raw = os.environ.get("SKIP_RUN_IDS", "")
    return {item.strip() for item in raw.split(",") if item.strip()}


class SleepInhibitor:
    def __enter__(self):
        if os.name == "nt":
            # ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED
            ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001 | 0x00000040)
            log("Windows sleep prevention enabled for sensitivity runs.")
        return self

    def __exit__(self, exc_type, exc, tb):
        if os.name == "nt":
            ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
            log("Windows sleep prevention released.")


def count_scenario_rows(path: Path) -> int:
    full_path = REPO_ROOT / path
    with full_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(line for line in handle if not line.lstrip().startswith("#"))
        return len(list(reader))


def ensure_inputs() -> None:
    for path in (CLUE_SCENARIO, KNOWN_SCENARIO):
        full = REPO_ROOT / path
        if not full.exists():
            raise FileNotFoundError(f"Missing scenario file: {full}")
        rows = count_scenario_rows(path)
        if rows < int(MAX_TRIALS):
            raise RuntimeError(f"{path} has {rows} rows, expected at least {MAX_TRIALS}")


def write_known_dga_wrappers() -> None:
    package = REPO_ROOT / "known_visit_sim/algorithms/dga_iter_wrappers"
    package.mkdir(parents=True, exist_ok=True)
    (package / "__init__.py").write_text("", encoding="utf-8")
    for iteration in DGA_ITERATIONS:
        class_name = f"DGAIter{iteration}Allocator"
        (package / f"DGA_iter_{iteration}.py").write_text(
            "\n".join(
                [
                    "from known_visit_sim.algorithms.DGA import DGAAllocator as BaseDGAAllocator",
                    "",
                    f"class {class_name}(BaseDGAAllocator):",
                    f"    name = \"DGA_iter_{iteration}\"",
                    f"    DGA_ITERATIONS_PER_TRIGGER = {iteration}",
                    "",
                ]
            ),
            encoding="utf-8",
        )


def build_horizon_jobs() -> list[Job]:
    jobs: list[Job] = []
    for horizon in HORIZONS:
        setting = f"h{horizon}"
        for alg_key, algorithm, weight in HORIZON_ALGORITHMS:
            for comm_label, comm_model, comm_level in COMM_CONDITIONS:
                run_id = f"{alg_key}_{setting}_{comm_label}"
                out_dir = CLUE_HORIZON_ROOT / "raw" / setting / comm_model / run_id
                cmd = [
                    PYTHON,
                    "-m",
                    "benchmark_sim.run_trials",
                    "--scenario-file",
                    str(CLUE_SCENARIO),
                    "--trial-mode",
                    "clue_search",
                    "--algorithm",
                    algorithm,
                    "--algorithm-name",
                    alg_key.upper(),
                    "--comm-model",
                    comm_model,
                    "--grid-size",
                    GRID_SIZE,
                    "--max-trials",
                    MAX_TRIALS,
                    "--out-dir",
                    str(out_dir),
                    "--condition-id",
                    run_id,
                    "--commitment-horizon",
                    str(horizon),
                ]
                if comm_level:
                    cmd += ["--comm-level", comm_level]
                metadata = {
                    "study": "clue_horizon",
                    "setting": setting,
                    "run_id": run_id,
                    "algorithm_key": alg_key,
                    "algorithm_module": algorithm,
                    "comm_label": comm_label,
                    "comm_model": comm_model,
                    "comm_level": comm_level,
                    "commitment_horizon": str(horizon),
                    "scenario_file": str(CLUE_SCENARIO),
                    "max_trials": MAX_TRIALS,
                    "out_dir": str(out_dir),
                }
                jobs.append(Job("clue_horizon", run_id, out_dir, cmd, weight, metadata))
    return jobs


def build_known_dga_iteration_jobs() -> list[Job]:
    jobs: list[Job] = []
    for iteration in DGA_ITERATIONS:
        algorithm = (
            f"known_visit_sim.algorithms.dga_iter_wrappers.DGA_iter_{iteration}:"
            f"DGAIter{iteration}Allocator"
        )
        for comm_label, comm_model, comm_level in COMM_CONDITIONS:
            run_id = f"dga_iter_{iteration}_{comm_label}"
            out_dir = KNOWN_DGA_ITER_ROOT / "raw" / f"iter_{iteration}" / comm_model / run_id
            cmd = [
                PYTHON,
                "-m",
                "known_visit_sim.run_trials",
                "--scenario-file",
                str(KNOWN_SCENARIO),
                "--algorithm",
                algorithm,
                "--algorithm-name",
                f"DGA_iter_{iteration}",
                "--comm-model",
                comm_model,
                "--grid-size",
                GRID_SIZE,
                "--max-trials",
                MAX_TRIALS,
                "--out-dir",
                str(out_dir),
                "--condition-id",
                run_id,
            ]
            if comm_level:
                cmd += ["--comm-level", comm_level]
            metadata = {
                "study": "known_target_dga_iteration",
                "setting": f"iter_{iteration}",
                "run_id": run_id,
                "algorithm_key": "dga",
                "algorithm_module": algorithm,
                "comm_label": comm_label,
                "comm_model": comm_model,
                "comm_level": comm_level,
                "dga_iterations": str(iteration),
                "scenario_file": str(KNOWN_SCENARIO),
                "max_trials": MAX_TRIALS,
                "out_dir": str(out_dir),
            }
            jobs.append(Job("known_target_dga_iteration", run_id, out_dir, cmd, 7, metadata))
    return jobs


def build_coverage_horizon_jobs() -> list[Job]:
    jobs: list[Job] = []
    for horizon in HORIZONS:
        setting = f"h{horizon}"
        for alg_key, algorithm, weight in HORIZON_ALGORITHMS:
            for comm_label, comm_model, comm_level in COMM_CONDITIONS:
                run_id = f"{alg_key}_{setting}_{comm_label}"
                out_dir = COVERAGE_HORIZON_ROOT / "raw" / setting / comm_model / run_id
                cmd = [
                    PYTHON,
                    "-m",
                    "benchmark_sim.run_trials",
                    "--trial-mode",
                    "coverage",
                    "--num-trials",
                    COVERAGE_TRIALS,
                    "--algorithm",
                    algorithm,
                    "--algorithm-name",
                    alg_key.upper(),
                    "--comm-model",
                    comm_model,
                    "--grid-size",
                    GRID_SIZE,
                    "--out-dir",
                    str(out_dir),
                    "--condition-id",
                    run_id,
                    "--commitment-horizon",
                    str(horizon),
                ]
                if comm_level:
                    cmd += ["--comm-level", comm_level]
                metadata = {
                    "study": "coverage_horizon",
                    "setting": setting,
                    "run_id": run_id,
                    "algorithm_key": alg_key,
                    "algorithm_module": algorithm,
                    "comm_label": comm_label,
                    "comm_model": comm_model,
                    "comm_level": comm_level,
                    "commitment_horizon": str(horizon),
                    "trial_mode": "coverage",
                    "num_trials": COVERAGE_TRIALS,
                    "out_dir": str(out_dir),
                }
                jobs.append(Job("coverage_horizon", run_id, out_dir, cmd, weight, metadata))
    return jobs


def build_job_queue(jobs: list[Job]) -> queue.Queue[Job]:
    job_queue: queue.Queue[Job] = queue.Queue()
    for job in sorted(jobs, key=lambda item: (-item.weight, item.run_id)):
        job_queue.put(job)
    log(f"Queued {len(jobs)} jobs for {N_WORKERS} shared workers.")
    return job_queue


def expected_trial_rows(job: Job) -> int:
    if "max_trials" in job.metadata:
        return int(job.metadata["max_trials"])
    if "num_trials" in job.metadata:
        return int(job.metadata["num_trials"])
    return int(MAX_TRIALS)


def csv_row_count(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="", encoding="utf-8") as handle:
        return sum(1 for _ in csv.DictReader(handle))


def output_is_complete(job: Job) -> bool:
    if not expected_outputs_exist(job):
        return False
    system_rows = csv_row_count(REPO_ROOT / job.out_dir / "system_performance.csv")
    return system_rows >= expected_trial_rows(job)


def process_exists(pid: int) -> bool:
    if os.name == "nt":
        result = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-Command",
                f"if (Get-Process -Id {pid} -ErrorAction SilentlyContinue) {{ exit 0 }} else {{ exit 1 }}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return result.returncode == 0
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def wait_for_process_exit(pid: int) -> None:
    while process_exists(pid):
        time.sleep(5)


def running_trial_processes(jobs: list[Job]) -> dict[str, int]:
    if os.name != "nt":
        return {}
    by_run_id: dict[str, int] = {}
    command = (
        "Get-CimInstance Win32_Process | "
        "Where-Object { $_.Name -match 'python' -and $_.CommandLine -match 'run_trials' } | "
        "Select-Object ProcessId,CommandLine | ConvertTo-Json -Compress"
    )
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", command],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
    except OSError:
        return {}
    if result.returncode != 0 or not result.stdout.strip():
        return {}
    try:
        parsed = json.loads(result.stdout)
    except json.JSONDecodeError:
        return {}
    processes = parsed if isinstance(parsed, list) else [parsed]
    for process in processes:
        pid = process.get("ProcessId")
        cmd = process.get("CommandLine") or ""
        if not pid or int(pid) == os.getpid():
            continue
        for job in jobs:
            if job.run_id in by_run_id:
                continue
            if "run_trials" in cmd and job.run_id in cmd:
                by_run_id[job.run_id] = int(pid)
                break
    return by_run_id


def apply_skip_list(jobs: list[Job]) -> list[Job]:
    skipped = skipped_run_ids()
    if not skipped:
        return jobs
    kept = [job for job in jobs if job.run_id not in skipped]
    removed = [job.run_id for job in jobs if job.run_id in skipped]
    if removed:
        log(f"Skipping requested run_ids: {', '.join(sorted(removed))}")
    return kept


def expected_outputs_exist(job: Job) -> bool:
    required = ["system_performance.csv", "trial_summary.csv", "robot_performance.csv", "config_used.json"]
    if job.study == "known_target_dga_iteration":
        required.append("target_performance.csv")
    return all((REPO_ROOT / job.out_dir / name).exists() for name in required)


def mark_job_complete(job: Job, state: dict, lock: threading.Lock, label: str) -> None:
    done_file = REPO_ROOT / job.out_dir / "_COMPLETE.txt"
    done_file.write_text(f"{label} {job.run_id} at {time.strftime('%Y-%m-%d %H:%M:%S')}\n", encoding="utf-8")
    with lock:
        state["completed"] += 1
    log(f"{label} {job.run_id}")


def run_job(job: Job, state: dict, lock: threading.Lock) -> None:
    full_out = REPO_ROOT / job.out_dir
    full_out.mkdir(parents=True, exist_ok=True)
    log_file = full_out / "run.log"
    done_file = full_out / "_COMPLETE.txt"

    if done_file.exists() and output_is_complete(job):
        with lock:
            state["completed"] += 1
            state["skipped"] += 1
        log(f"SKIP completed {job.run_id}")
        return

    with lock:
        state["running"][job.run_id] = time.time()

    log(f"START {job.run_id}")
    with log_file.open("a", encoding="utf-8") as output:
        output.write("\nCommand: " + " ".join(job.cmd) + "\n\n")
        output.flush()
        proc = subprocess.Popen(
            job.cmd,
            cwd=REPO_ROOT,
            stdout=output,
            stderr=subprocess.STDOUT,
            text=True,
        )
        return_code = proc.wait()

    with lock:
        state["running"].pop(job.run_id, None)

    if return_code != 0:
        with lock:
            state["failed"] += 1
            state["failures"].append(job.run_id)
        raise RuntimeError(f"{job.run_id} failed with exit code {return_code}; see {log_file}")

    if not output_is_complete(job):
        with lock:
            state["failed"] += 1
            state["failures"].append(job.run_id)
        rows = csv_row_count(REPO_ROOT / job.out_dir / "system_performance.csv")
        raise RuntimeError(
            f"{job.run_id} exited successfully but only has {rows}/{expected_trial_rows(job)} rows; "
            f"see {log_file}"
        )

    mark_job_complete(job, state, lock, "DONE")


def adopt_running_job(job: Job, pid: int, state: dict, lock: threading.Lock) -> None:
    with lock:
        state["running"][job.run_id] = time.time()
    rows = csv_row_count(REPO_ROOT / job.out_dir / "system_performance.csv")
    log(f"ADOPT {job.run_id} pid={pid} rows={rows}/{expected_trial_rows(job)}")
    wait_for_process_exit(pid)
    with lock:
        state["running"].pop(job.run_id, None)

    if output_is_complete(job):
        mark_job_complete(job, state, lock, "ADOPTED DONE")
        return

    rows = csv_row_count(REPO_ROOT / job.out_dir / "system_performance.csv")
    log(f"ADOPTED incomplete {job.run_id}: rows={rows}/{expected_trial_rows(job)}; resuming condition")
    run_job(job, state, lock)


def worker(
    worker_id: int,
    job_queue: queue.Queue[Job],
    state: dict,
    lock: threading.Lock,
    slots: threading.Semaphore,
) -> None:
    while True:
        try:
            job = job_queue.get_nowait()
        except queue.Empty:
            break
        slots.acquire()
        try:
            run_job(job, state, lock)
        finally:
            slots.release()
            job_queue.task_done()
    log(f"Worker {worker_id} finished.")


def adopted_worker(
    job: Job,
    pid: int,
    state: dict,
    lock: threading.Lock,
    slots: threading.Semaphore,
) -> None:
    slots.acquire()
    try:
        adopt_running_job(job, pid, state, lock)
    finally:
        slots.release()


def heartbeat(label: str, total: int, state: dict, lock: threading.Lock, stop: threading.Event) -> None:
    while not stop.wait(HEARTBEAT_S):
        with lock:
            completed = state["completed"]
            failed = state["failed"]
            skipped = state["skipped"]
            running = dict(state["running"])
        running_text = ", ".join(
            f"{name}({int(time.time() - started)}s)" for name, started in sorted(running.items())
        )
        log(
            f"PROGRESS {label}: completed={completed}/{total} skipped={skipped} "
            f"failed={failed} running=[{running_text}]"
        )


def run_study(label: str, jobs: list[Job]) -> None:
    state = {"completed": 0, "failed": 0, "skipped": 0, "running": {}, "failures": []}
    lock = threading.Lock()
    stop = threading.Event()
    active_by_run_id = running_trial_processes(jobs)
    active_jobs = [job for job in jobs if job.run_id in active_by_run_id]
    queued_jobs = [job for job in jobs if job.run_id not in active_by_run_id]
    if active_jobs:
        log(
            "Adopting active jobs: "
            + ", ".join(f"{job.run_id}(pid={active_by_run_id[job.run_id]})" for job in active_jobs)
        )
    slots = threading.Semaphore(N_WORKERS)
    job_queue = build_job_queue(queued_jobs)
    hb = threading.Thread(target=heartbeat, args=(label, len(jobs), state, lock, stop), daemon=True)
    hb.start()
    threads = []
    for job in active_jobs:
        threads.append(
            threading.Thread(
                target=adopted_worker,
                args=(job, active_by_run_id[job.run_id], state, lock, slots),
                daemon=False,
            )
        )
    threads.extend(
        threading.Thread(target=worker, args=(idx, job_queue, state, lock, slots), daemon=False)
        for idx in range(min(N_WORKERS, len(queued_jobs)))
    )
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    stop.set()
    with lock:
        if state["failed"]:
            raise RuntimeError(f"{label} failed jobs: {state['failures']}")
        log(f"FINISHED {label}: completed={state['completed']} skipped={state['skipped']}")


def write_manifest(root: Path, jobs: Iterable[Job]) -> None:
    combined = REPO_ROOT / root / "combined"
    combined.mkdir(parents=True, exist_ok=True)
    manifest = combined / "condition_manifest.csv"
    fields: list[str] = []
    rows = [job.metadata for job in jobs]
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    log(f"Wrote manifest {manifest}")


def combine_outputs(root: Path, jobs: list[Job]) -> None:
    combined = REPO_ROOT / root / "combined"
    combined.mkdir(parents=True, exist_ok=True)
    write_manifest(root, jobs)
    for filename in ["system_performance.csv", "trial_summary.csv", "robot_performance.csv", "target_performance.csv"]:
        rows: list[dict[str, str]] = []
        fields: list[str] = []
        for job in jobs:
            path = REPO_ROOT / job.out_dir / filename
            if not path.exists():
                continue
            with path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                if reader.fieldnames:
                    for field in reader.fieldnames:
                        if field not in fields:
                            fields.append(field)
                for row in reader:
                    row.update(job.metadata)
                    for key in job.metadata:
                        if key not in fields:
                            fields.append(key)
                    rows.append(row)
        if not rows:
            continue
        out_path = combined / filename
        with out_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        log(f"Wrote combined {out_path} rows={len(rows)}")


def main() -> None:
    ensure_inputs()
    write_known_dga_wrappers()
    horizon_jobs = apply_skip_list(build_horizon_jobs())
    known_jobs = apply_skip_list(build_known_dga_iteration_jobs())
    coverage_jobs = apply_skip_list(build_coverage_horizon_jobs())

    with SleepInhibitor():
        log("Starting clue-informed horizon sensitivity first.")
        run_study("clue_horizon", horizon_jobs)
        combine_outputs(CLUE_HORIZON_ROOT, horizon_jobs)

        log("Starting known-target DGA iteration sensitivity after horizon completion.")
        run_study("known_target_dga_iteration", known_jobs)
        combine_outputs(KNOWN_DGA_ITER_ROOT, known_jobs)

        log("Starting coverage horizon sensitivity after known-target DGA iteration completion.")
        run_study("coverage_horizon", coverage_jobs)
        combine_outputs(COVERAGE_HORIZON_ROOT, coverage_jobs)

    log("All requested sensitivity reruns completed.")


if __name__ == "__main__":
    main()
