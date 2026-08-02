from __future__ import annotations

from pathlib import Path

from benchmark_sim.tests.run_current_sensitivity_reruns import (
    COMM_CONDITIONS,
    COVERAGE_HORIZON_ROOT,
    COVERAGE_TRIALS,
    GRID_SIZE,
    HORIZON_ALGORITHMS,
    HORIZONS,
    PYTHON,
    Job,
    SleepInhibitor,
    combine_outputs,
    log,
    run_study,
)

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


def main() -> None:
    jobs = build_coverage_horizon_jobs()
    with SleepInhibitor():
        log("Starting coverage horizon sensitivity.")
        run_study("coverage_horizon", jobs)
        combine_outputs(COVERAGE_HORIZON_ROOT, jobs)
    log("Coverage horizon sensitivity completed.")


if __name__ == "__main__":
    main()
