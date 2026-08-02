#!/usr/bin/env bash
set -u
set -o pipefail

REPO_ROOT="/home/jlott/dcta_benchmark_sim"
cd "$REPO_ROOT" || exit 1

# Rebuilds combined CSVs for the 300-trial horizon and top-k sensitivity runs.
# It DOES NOT rerun simulations. It only reads raw CSV outputs and writes labeled combined CSVs.
#
# Labels are inferred from folder names:
#   runs/sensitivity_horizon_300/raw/h3/bernoulli/<condition>/system_performance.csv
#       -> sensitivity_suite=horizon, sensitivity_parameter=commitment_horizon, sensitivity_value=3
#   runs/sensitivity_topk_300/raw/k100/ideal/<condition>/system_performance.csv
#       -> sensitivity_suite=topk, sensitivity_parameter=max_candidate_cells, sensitivity_value=100
#   runs/sensitivity_topk_300/raw/kall/ideal/<condition>/system_performance.csv
#       -> sensitivity_suite=topk, sensitivity_parameter=max_candidate_cells, sensitivity_value=all

PYTHON_BIN="${PYTHON_BIN:-python3}"

"$PYTHON_BIN" - <<'PY'
import csv
import os
from pathlib import Path

ROOTS = [
    Path("runs/sensitivity_horizon_300"),
    Path("runs/sensitivity_topk_300"),
]
FILENAMES = [
    "system_performance.csv",
    "trial_summary.csv",
    "robot_performance.csv",
]

METADATA_COLUMNS = [
    "sensitivity_suite",
    "sensitivity_parameter",
    "sensitivity_value",
    "sensitivity_label",
    "source_comm_folder",
    "source_condition_folder",
    "source_out_dir",
]


def infer_metadata(root: Path, csv_file: Path) -> dict:
    """Infer metadata from raw output path."""
    rel = csv_file.relative_to(root)
    parts = rel.parts

    # Expected: raw/<label>/<comm_folder>/<condition_folder>/<filename>
    if len(parts) < 5 or parts[0] != "raw":
        raise ValueError(f"Unexpected path under {root}: {csv_file}")

    label = parts[1]              # h3, h12, k100, kall, etc.
    comm_folder = parts[2]        # ideal, bernoulli, gilbert_elliot, rayleigh_style
    condition_folder = parts[3]   # dga_h3_ideal, acbba_k100_bernoulli_025, etc.
    out_dir = csv_file.parent

    root_name = root.name.lower()
    if "horizon" in root_name:
        suite = "horizon"
        parameter = "commitment_horizon"
        if not label.startswith("h"):
            raise ValueError(f"Expected horizon label like h3, got {label} from {csv_file}")
        value = label[1:]
    elif "topk" in root_name:
        suite = "topk"
        parameter = "max_candidate_cells"
        if not label.startswith("k"):
            raise ValueError(f"Expected top-k label like k100 or kall, got {label} from {csv_file}")
        value = label[1:]
    else:
        suite = root.name
        parameter = "unknown"
        value = label

    return {
        "sensitivity_suite": suite,
        "sensitivity_parameter": parameter,
        "sensitivity_value": value,
        "sensitivity_label": label,
        "source_comm_folder": comm_folder,
        "source_condition_folder": condition_folder,
        "source_out_dir": str(out_dir),
    }


def find_raw_csvs(root: Path, filename: str):
    raw = root / "raw"
    if not raw.exists():
        return []
    return sorted(p for p in raw.rglob(filename) if p.is_file())


def combine_one(root: Path, filename: str) -> None:
    combined_dir = root / "combined"
    combined_dir.mkdir(parents=True, exist_ok=True)
    outfile = combined_dir / filename

    files = find_raw_csvs(root, filename)
    if not files:
        print(f"[SKIP] {root}: no {filename} files found")
        return

    rows_written = 0
    header_written = False
    output_header = None

    with outfile.open("w", newline="") as fout:
        writer = None

        for csv_file in files:
            meta = infer_metadata(root, csv_file)
            with csv_file.open("r", newline="") as fin:
                reader = csv.DictReader(fin)
                if reader.fieldnames is None:
                    print(f"[WARN] Empty CSV skipped: {csv_file}")
                    continue

                # Preserve original columns first, then append metadata columns.
                current_header = list(reader.fieldnames)
                if not header_written:
                    output_header = current_header + [c for c in METADATA_COLUMNS if c not in current_header]
                    writer = csv.DictWriter(fout, fieldnames=output_header, extrasaction="ignore")
                    writer.writeheader()
                    header_written = True
                else:
                    # Warn but continue if files do not share identical original schema.
                    expected_original = [c for c in output_header if c not in METADATA_COLUMNS]
                    if current_header != expected_original:
                        print(f"[WARN] Header differs in {csv_file}")
                        print(f"       expected={expected_original}")
                        print(f"       got     ={current_header}")

                for row in reader:
                    for k, v in meta.items():
                        row[k] = v
                    writer.writerow(row)
                    rows_written += 1

    print(f"[OK] {root}: combined {filename} data_rows={rows_written} -> {outfile}")


def write_manifest(root: Path) -> None:
    combined_dir = root / "combined"
    combined_dir.mkdir(parents=True, exist_ok=True)
    manifest = combined_dir / "condition_manifest.csv"

    # Use system_performance.csv as the condition-level source because every successful condition should have it.
    files = find_raw_csvs(root, "system_performance.csv")
    with manifest.open("w", newline="") as fout:
        fieldnames = METADATA_COLUMNS + ["system_rows", "trial_summary_rows", "robot_rows", "complete_marker"]
        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()
        for f in files:
            meta = infer_metadata(root, f)
            out_dir = Path(meta["source_out_dir"])
            row = dict(meta)
            row["system_rows"] = count_data_rows(out_dir / "system_performance.csv")
            row["trial_summary_rows"] = count_data_rows(out_dir / "trial_summary.csv")
            row["robot_rows"] = count_data_rows(out_dir / "robot_performance.csv")
            row["complete_marker"] = "yes" if (out_dir / "_COMPLETE.txt").exists() else "no"
            writer.writerow(row)
    print(f"[OK] {root}: wrote manifest -> {manifest}")


def count_data_rows(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", newline="") as f:
        # subtract header if present
        n = sum(1 for _ in f)
    return max(n - 1, 0)


for root in ROOTS:
    print()
    print("============================================================")
    print(f"[COMBINE] {root}")
    print("============================================================")
    for filename in FILENAMES:
        combine_one(root, filename)
    write_manifest(root)

print()
print("[DONE] Labeled combine complete. Simulations were not rerun.")
PY
