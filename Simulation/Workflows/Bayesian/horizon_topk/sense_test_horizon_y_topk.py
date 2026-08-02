#!/usr/bin/env python3
from __future__ import annotations

import csv
import shlex
from pathlib import Path

REPO_ROOT = Path("/home/jlott/dcta_benchmark_sim")
SCENARIO_FILE = "runs/sensitivity_scenarios/clue_tuning_g19_n300_seed20260702.csv"
SCENARIO_SEED = 20260702
SCENARIO_TRIALS = 300
GRID_SIZE = "19"
MAX_TRIALS = "50"
PYTHON_BIN = "${PYTHON_BIN:-python3}"

HORIZON_ROOT = Path("runs/sensitivity_horizon_50")
TOPK_ROOT = Path("runs/sensitivity_topk_50")

N_HORIZON_PARTS = 8
N_TOPK_PARTS = 8

# Adjust this if your DGA file/class name is different.
DGA_MODULE = "benchmark_sim.algorithms.DGA:DGAAllocator"

CONDITIONS = [
    ("ideal", "0%", "ideal", ""),
    ("bernoulli_025", "25%", "bernoulli", "0.25"),
    ("ge_075", "25%", "gilbert_elliot", "0.75"),
    ("rayleigh_m50_66", "25%", "rayleigh_style", "-50.66"),
]

HORIZONS = [1, 2, 3, 5, 8, 12]
TOPKS = ["10", "25", "50", "100", "all"]

HORIZON_ALGORITHMS = [
    ("acbba", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator"),
    ("pi", "benchmark_sim.algorithms.PI:PIAllocator"),
    ("hipc", "benchmark_sim.algorithms.HIPC:HIPCAllocator"),
    ("dmchba", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator"),
    ("dga", DGA_MODULE),
]

TOPK_ALGORITHMS = [
    ("cbaa", "benchmark_sim.algorithms.CBAA:CBAAAllocator"),
    ("acbba", "benchmark_sim.algorithms.ACBBA:ACBBAAllocator"),
    ("pi", "benchmark_sim.algorithms.PI:PIAllocator"),
    ("hipc", "benchmark_sim.algorithms.HIPC:HIPCAllocator"),
    ("dmchba", "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator"),
    ("dga", DGA_MODULE),
]

# Simple load-balancing weights so DGA/DMCHBA get spread out.
WEIGHTS = {
    "dga": 4,
    "dmchba": 4,
    "hipc": 2,
    "pi": 2,
    "acbba": 2,
    "cbaa": 1,
}


def q(x: str | Path) -> str:
    return shlex.quote(str(x))


def generate_independent_sensitivity_scenarios() -> None:
    import sys

    repo_root_text = str(REPO_ROOT)
    if repo_root_text not in sys.path:
        sys.path.insert(0, repo_root_text)

    from benchmark_sim.tests.generate_clue_sensitivity_scenarios import generate_scenario_file

    generate_scenario_file(
        REPO_ROOT / SCENARIO_FILE,
        grid_size=int(GRID_SIZE),
        num_trials=SCENARIO_TRIALS,
        num_clues=4,
        num_robots=4,
        seed=SCENARIO_SEED,
        target_decay_exp=1.0,
    )
    print(f"Wrote independent sensitivity scenarios to {REPO_ROOT / SCENARIO_FILE}")


def build_jobs(study: str):
    jobs = []

    if study == "horizon":
        root = HORIZON_ROOT
        for horizon in HORIZONS:
            setting = f"h{horizon}"
            for alg_key, alg_module in HORIZON_ALGORITHMS:
                for label, target_drop, model, level in CONDITIONS:
                    run_id = f"{alg_key}_{setting}_{label}"
                    out_dir = root / "raw" / setting / model / run_id
                    args = [
                        PYTHON_BIN,
                        "-m", "benchmark_sim.run_trials",
                        "--scenario-file", SCENARIO_FILE,
                        "--algorithm", alg_module,
                        "--comm-model", model,
                        "--grid-size", GRID_SIZE,
                        "--max-trials", MAX_TRIALS,
                        "--out-dir", str(out_dir),
                        "--commitment-horizon", str(horizon),
                    ]
                    if level:
                        args += ["--comm-level", level]

                    jobs.append({
                        "study": study,
                        "setting": setting,
                        "run_id": run_id,
                        "algorithm_key": alg_key,
                        "algorithm_module": alg_module,
                        "target_drop": target_drop,
                        "comm_label": label,
                        "comm_model": model,
                        "comm_level": level,
                        "scenario_file": SCENARIO_FILE,
                        "grid_size": GRID_SIZE,
                        "max_trials": MAX_TRIALS,
                        "out_dir": str(out_dir),
                        "args": args,
                        "weight": WEIGHTS.get(alg_key, 1),
                    })

    elif study == "topk":
        root = TOPK_ROOT
        for topk in TOPKS:
            setting = f"k{topk}"
            for alg_key, alg_module in TOPK_ALGORITHMS:
                for label, target_drop, model, level in CONDITIONS:
                    run_id = f"{alg_key}_{setting}_{label}"
                    out_dir = root / "raw" / setting / model / run_id
                    args = [
                        PYTHON_BIN,
                        "-m", "benchmark_sim.run_trials",
                        "--scenario-file", SCENARIO_FILE,
                        "--algorithm", alg_module,
                        "--comm-model", model,
                        "--grid-size", GRID_SIZE,
                        "--max-trials", MAX_TRIALS,
                        "--out-dir", str(out_dir),
                        "--max-candidate-cells", str(topk),
                    ]
                    if level:
                        args += ["--comm-level", level]

                    jobs.append({
                        "study": study,
                        "setting": setting,
                        "run_id": run_id,
                        "algorithm_key": alg_key,
                        "algorithm_module": alg_module,
                        "target_drop": target_drop,
                        "comm_label": label,
                        "comm_model": model,
                        "comm_level": level,
                        "scenario_file": SCENARIO_FILE,
                        "grid_size": GRID_SIZE,
                        "max_trials": MAX_TRIALS,
                        "out_dir": str(out_dir),
                        "args": args,
                        "weight": WEIGHTS.get(alg_key, 1),
                    })

    else:
        raise ValueError(study)

    return jobs


def assign_jobs(jobs, n_parts: int):
    # Put slow jobs first, then greedily assign to currently lightest part.
    jobs = sorted(jobs, key=lambda j: (-j["weight"], j["algorithm_key"], j["setting"], j["comm_label"]))
    parts = [[] for _ in range(n_parts)]
    loads = [0 for _ in range(n_parts)]

    for job in jobs:
        idx = min(range(n_parts), key=lambda i: loads[i])
        parts[idx].append(job)
        loads[idx] += job["weight"]

    return parts, loads


def write_manifest(root: Path, jobs):
    combined = REPO_ROOT / root / "combined"
    combined.mkdir(parents=True, exist_ok=True)
    manifest_path = combined / "manifest.csv"

    fields = [
        "study", "setting", "run_id", "algorithm_key", "algorithm_module",
        "target_drop", "comm_label", "comm_model", "comm_level",
        "scenario_file", "grid_size", "max_trials", "out_dir"
    ]

    with manifest_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for job in jobs:
            writer.writerow({k: job[k] for k in fields})

    print(f"Wrote {manifest_path}")


def write_part_script(root: Path, study: str, part_idx: int, jobs):
    parts_dir = REPO_ROOT / root / "parts"
    parts_dir.mkdir(parents=True, exist_ok=True)

    script_path = parts_dir / f"{study}_core_{part_idx}.sh"

    lines = [
        "#!/usr/bin/env bash",
        f"# Auto-generated {study} sensitivity part {part_idx}",
        "set -u",
        "set -o pipefail",
        "",
        f'REPO_ROOT="{REPO_ROOT}"',
        'cd "$REPO_ROOT" || exit 1',
        "",
        f'echo "Starting {study} core {part_idx} at $(date -Is)"',
        "",
    ]

    for job in jobs:
        out_dir = job["out_dir"]
        log_file = f"{out_dir}/run.log"
        done_file = f"{out_dir}/_COMPLETE.txt"

        cmd = " ".join(q(a) if not a.startswith("${PYTHON_BIN") else a for a in job["args"])

        lines += [
            "echo ''",
            f'echo "============================================================"',
            f'echo "RUN {job["run_id"]}"',
            f'echo "============================================================"',
            f'mkdir -p {q(out_dir)}',
            f'if [[ -f {q(done_file)} ]]; then',
            f'  echo "Skipping completed: {job["run_id"]}"',
            "else",
            f'  echo "Command: {cmd}"',
            f'  {cmd} 2>&1 | tee {q(log_file)}',
            "  STATUS=${PIPESTATUS[0]}",
            "  if [[ \"$STATUS\" -ne 0 ]]; then",
            f'    echo "ERROR: failed {job["run_id"]} with status $STATUS"',
            "    exit \"$STATUS\"",
            "  fi",
            f'  echo "Completed {job["run_id"]} at $(date -Is)" > {q(done_file)}',
            "fi",
            "",
        ]

    lines += [
        f'echo "Finished {study} core {part_idx} at $(date -Is)"',
        "",
    ]

    script_path.write_text("\n".join(lines), encoding="utf-8")
    script_path.chmod(0o755)
    print(f"Wrote {script_path} ({len(jobs)} jobs)")


def write_start_script():
    script_path = REPO_ROOT / "runs" / "start_all_sensitivity_parts.sh"
    script_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "#!/usr/bin/env bash",
        "set -u",
        "set -o pipefail",
        "",
        f'cd "{REPO_ROOT}" || exit 1',
        'mkdir -p runs/sensitivity_logs',
        "",
        "echo \"Starting all 16 sensitivity part scripts at $(date -Is)\"",
        "",
    ]

    for i in range(N_HORIZON_PARTS):
        lines.append(f'./{HORIZON_ROOT}/parts/horizon_core_{i}.sh > runs/sensitivity_logs/horizon_core_{i}.master.log 2>&1 &')

    for i in range(N_TOPK_PARTS):
        lines.append(f'./{TOPK_ROOT}/parts/topk_core_{i}.sh > runs/sensitivity_logs/topk_core_{i}.master.log 2>&1 &')

    lines += [
        "",
        "wait",
        'echo "All sensitivity parts finished at $(date -Is)"',
        "",
    ]

    script_path.write_text("\n".join(lines), encoding="utf-8")
    script_path.chmod(0o755)
    print(f"Wrote {script_path}")


def write_combine_script():
    script_path = REPO_ROOT / "runs" / "combine_sensitivity_results.py"

    script = f'''#!/usr/bin/env python3
from __future__ import annotations

import csv
from pathlib import Path

REPO_ROOT = Path("{REPO_ROOT}")
RUN_ROOTS = [
    REPO_ROOT / "{HORIZON_ROOT}",
    REPO_ROOT / "{TOPK_ROOT}",
]

FILES = [
    ("system_performance.csv", "system_performance_all.csv"),
    ("robot_performance.csv", "robot_performance_all.csv"),
    ("trial_summary.csv", "trial_summary_all.csv"),
]

METADATA_FIELDS = [
    "study",
    "setting",
    "run_id",
    "algorithm_key",
    "algorithm_module",
    "target_drop",
    "comm_label",
    "comm_model",
    "comm_level",
    "scenario_file",
    "grid_size",
    "max_trials",
]

for run_root in RUN_ROOTS:
    combined_root = run_root / "combined"
    manifest_path = combined_root / "manifest.csv"

    if not manifest_path.exists():
        print(f"Missing manifest: {{manifest_path}}")
        continue

    with manifest_path.open(newline="", encoding="utf-8") as f:
        manifest = list(csv.DictReader(f))

    print(f"Combining {{run_root}} with {{len(manifest)}} manifest entries")

    for input_name, output_name in FILES:
        rows = []
        fieldnames = []
        seen = set()

        for entry in manifest:
            path = Path(entry["out_dir"]) / input_name
            if not path.exists():
                print(f"Warning: missing {{path}}")
                continue

            with path.open(newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)

                if reader.fieldnames:
                    for name in reader.fieldnames:
                        if name not in seen:
                            seen.add(name)
                            fieldnames.append(name)

                for row in reader:
                    for meta in METADATA_FIELDS:
                        row[meta] = entry.get(meta, "")
                    rows.append(row)

        for meta in METADATA_FIELDS:
            if meta not in seen:
                seen.add(meta)
                fieldnames.append(meta)

        output_path = combined_root / output_name
        if not rows:
            print(f"No rows found for {{input_name}} in {{run_root}}")
            continue

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

        print(f"Wrote {{output_path}} ({{len(rows)}} rows)")
'''

    script_path.write_text(script, encoding="utf-8")
    script_path.chmod(0o755)
    print(f"Wrote {script_path}")


def main():
    generate_independent_sensitivity_scenarios()

    for study, root, n_parts in [
        ("horizon", HORIZON_ROOT, N_HORIZON_PARTS),
        ("topk", TOPK_ROOT, N_TOPK_PARTS),
    ]:
        jobs = build_jobs(study)
        parts, loads = assign_jobs(jobs, n_parts)

        write_manifest(root, jobs)

        for idx, part_jobs in enumerate(parts):
            write_part_script(root, study, idx, part_jobs)

        print(f"{study} loads:", loads)

    write_start_script()
    write_combine_script()

    print("")
    print("Done. Next commands:")
    print("  ./runs/start_all_sensitivity_parts.sh")
    print("  python3 runs/combine_sensitivity_results.py")


if __name__ == "__main__":
    main()
