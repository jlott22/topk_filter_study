#!/usr/bin/env bash
set -euo pipefail

# Combine condition-partitioned DGA final-500 outputs and verify completeness.

REPO_ROOT="${DCTA_REPO_ROOT:-/home/jlott/dcta_benchmark_sim}"
RAW_ROOT="$REPO_ROOT/runs/final_500_all/raw/dga"
COMBINED_ROOT="$REPO_ROOT/runs/final_500_all/combined_dga"
SCENARIO_FILE="$REPO_ROOT/scenarios/final_trial_500.csv"

if [[ ! -d "$RAW_ROOT" ]]; then
  echo "ERROR: raw DGA output root not found: $RAW_ROOT" >&2
  exit 2
fi
if [[ ! -f "$SCENARIO_FILE" ]]; then
  echo "ERROR: scenario file not found: $SCENARIO_FILE" >&2
  exit 2
fi

mkdir -p "$COMBINED_ROOT"
cd "$REPO_ROOT"

python3 - "$SCENARIO_FILE" "$RAW_ROOT" "$COMBINED_ROOT" <<'PY'
import csv
import sys
from pathlib import Path

scenario_path = Path(sys.argv[1])
raw_root = Path(sys.argv[2])
combined_root = Path(sys.argv[3])

# folder, comm_model, forced output comm_level
conditions = [
    ("ideal_1_0", "ideal", "1.0"),
    ("bernoulli_drop_0_05", "bernoulli", "drop_0.05"),
    ("bernoulli_drop_0_1", "bernoulli", "drop_0.1"),
    ("bernoulli_drop_0_2", "bernoulli", "drop_0.2"),
    ("bernoulli_drop_0_3", "bernoulli", "drop_0.3"),
    ("bernoulli_drop_0_4", "bernoulli", "drop_0.4"),
    ("bernoulli_drop_0_5", "bernoulli", "drop_0.5"),
    ("bernoulli_drop_0_6", "bernoulli", "drop_0.6"),
    ("bernoulli_drop_0_7", "bernoulli", "drop_0.7"),
    ("gilbert_elliot_pGG_0_3_pBB_0_7", "gilbert_elliot", "pGG_0.3_pBB_0.7"),
    ("gilbert_elliot_pGG_0_4_pBB_0_6", "gilbert_elliot", "pGG_0.4_pBB_0.6"),
    ("gilbert_elliot_pGG_0_5_pBB_0_5", "gilbert_elliot", "pGG_0.5_pBB_0.5"),
    ("gilbert_elliot_pGG_0_6_pBB_0_4", "gilbert_elliot", "pGG_0.6_pBB_0.4"),
    ("gilbert_elliot_pGG_0_7_pBB_0_3", "gilbert_elliot", "pGG_0.7_pBB_0.3"),
    ("gilbert_elliot_pGG_0_8_pBB_0_2", "gilbert_elliot", "pGG_0.8_pBB_0.2"),
    ("gilbert_elliot_pGG_0_9_pBB_0_1", "gilbert_elliot", "pGG_0.9_pBB_0.1"),
    ("gilbert_elliot_pGG_0_95_pBB_0_05", "gilbert_elliot", "pGG_0.95_pBB_0.05"),
    ("rayleigh_style_sens_neg32_58", "rayleigh_style", "sens_-32.58"),
    ("rayleigh_style_sens_neg37_79", "rayleigh_style", "sens_-37.79"),
    ("rayleigh_style_sens_neg42_16", "rayleigh_style", "sens_-42.16"),
    ("rayleigh_style_sens_neg46_04", "rayleigh_style", "sens_-46.04"),
    ("rayleigh_style_sens_neg49_17", "rayleigh_style", "sens_-49.17"),
    ("rayleigh_style_sens_neg52_15", "rayleigh_style", "sens_-52.15"),
    ("rayleigh_style_sens_neg56_04", "rayleigh_style", "sens_-56.04"),
    ("rayleigh_style_sens_neg59_4", "rayleigh_style", "sens_-59.4"),
]
filenames = ("system_performance.csv", "trial_summary.csv", "robot_performance.csv")

with scenario_path.open(newline="") as handle:
    scenario_rows = list(csv.DictReader(line for line in handle if line.strip() and not line.lstrip().startswith("#")))
if scenario_rows and "trial_id" in scenario_rows[0]:
    expected_ids = [str(row["trial_id"]) for row in scenario_rows]
elif scenario_rows and "episode" in scenario_rows[0]:
    expected_ids = [str(row["episode"]) for row in scenario_rows]
else:
    expected_ids = [str(index) for index in range(len(scenario_rows))]
expected_id_set = set(expected_ids)
expected_trials = len(expected_ids)
expected_robot_rows = expected_trials * 4

combined_by_file = {filename: [] for filename in filenames}
fieldnames_by_file = {filename: [] for filename in filenames}
completeness_rows = []
missing_rows = []

for folder, model, label in conditions:
    condition_dir = raw_root / folder
    condition_data = {}
    for filename in filenames:
        path = condition_dir / filename
        rows = []
        read_error = ""
        if path.is_file():
            try:
                with path.open(newline="") as handle:
                    reader = csv.DictReader(handle)
                    source_fields = list(reader.fieldnames or [])
                    for field in source_fields:
                        if field not in fieldnames_by_file[filename]:
                            fieldnames_by_file[filename].append(field)
                    rows = list(reader)
            except Exception as exc:
                read_error = str(exc)
        else:
            read_error = "missing_file"

        for row in rows:
            row["algorithm"] = "DGA"
            row["comm_model"] = model
            row["comm_level"] = label
            row["source_condition_folder"] = folder
            combined_by_file[filename].append(row)

        trial_ids = {str(row.get("trial_id", row.get("episode", ""))) for row in rows}
        trial_ids.discard("")
        condition_data[filename] = (len(rows), trial_ids, read_error)

    system_count, system_ids, system_error = condition_data["system_performance.csv"]
    trial_count, trial_ids, trial_error = condition_data["trial_summary.csv"]
    robot_count, robot_ids, robot_error = condition_data["robot_performance.csv"]
    missing_system = expected_id_set - system_ids
    missing_trial = expected_id_set - trial_ids
    missing_robot = expected_id_set - robot_ids
    missing_union = missing_system | missing_trial | missing_robot
    status = "OK" if (
        system_count == expected_trials
        and trial_count == expected_trials
        and robot_count == expected_robot_rows
        and system_ids == expected_id_set
        and trial_ids == expected_id_set
        and robot_ids == expected_id_set
        and not system_error
        and not trial_error
        and not robot_error
    ) else "INCOMPLETE"

    completeness_rows.append({
        "comm_model": model,
        "comm_level": label,
        "condition_folder": folder,
        "system_rows": system_count,
        "expected_system_rows": expected_trials,
        "system_unique_trial_ids": len(system_ids),
        "trial_summary_rows": trial_count,
        "expected_trial_summary_rows": expected_trials,
        "trial_summary_unique_trial_ids": len(trial_ids),
        "robot_rows": robot_count,
        "expected_robot_rows": expected_robot_rows,
        "robot_unique_trial_ids": len(robot_ids),
        "missing_trial_count": len(missing_union),
        "read_errors": ";".join(error for error in (system_error, trial_error, robot_error) if error),
        "status": status,
    })
    for trial_id in expected_ids:
        if trial_id not in missing_union:
            continue
        missing_outputs = []
        if trial_id in missing_system:
            missing_outputs.append("system_performance")
        if trial_id in missing_trial:
            missing_outputs.append("trial_summary")
        if trial_id in missing_robot:
            missing_outputs.append("robot_performance")
        missing_rows.append({
            "comm_model": model,
            "comm_level": label,
            "condition_folder": folder,
            "trial_id": trial_id,
            "missing_outputs": ";".join(missing_outputs),
        })

def sortable_id(value):
    text = str(value)
    try:
        return (0, int(text))
    except ValueError:
        return (1, text)

for filename in filenames:
    fields = fieldnames_by_file[filename]
    for required in ("algorithm", "comm_model", "comm_level", "source_condition_folder"):
        if required not in fields:
            fields.append(required)

    unique_rows = []
    seen = set()
    for row in combined_by_file[filename]:
        key = tuple(row.get(field, "") for field in fields)
        if key in seen:
            continue
        seen.add(key)
        unique_rows.append(row)
    unique_rows.sort(key=lambda row: (
        row.get("comm_model", ""),
        row.get("comm_level", ""),
        sortable_id(row.get("trial_id", row.get("episode", ""))),
        sortable_id(row.get("robot_id", "")),
    ))

    out_path = combined_root / filename
    with out_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(unique_rows)
    print(f"[COMBINED] {filename}: rows={len(unique_rows)}")

def write_report(path, fieldnames, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

write_report(combined_root / "completeness_report.csv", list(completeness_rows[0].keys()), completeness_rows)
write_report(
    combined_root / "missing_dga_trials_to_rerun.csv",
    ["comm_model", "comm_level", "condition_folder", "trial_id", "missing_outputs"],
    missing_rows,
)

expected_combined = {
    "system_performance.csv": len(conditions) * expected_trials,
    "trial_summary.csv": len(conditions) * expected_trials,
    "robot_performance.csv": len(conditions) * expected_robot_rows,
}
print("\n[COMBINED COUNT SUMMARY]")
all_counts_ok = True
for filename in filenames:
    with (combined_root / filename).open(newline="") as handle:
        actual = sum(1 for _ in csv.DictReader(handle))
    expected = expected_combined[filename]
    status = "OK" if actual == expected else "INCOMPLETE"
    all_counts_ok = all_counts_ok and actual == expected
    print(f"{filename}: {actual}/{expected} => {status}")

print("\n[TRIALS NEEDING RERUN]")
if not missing_rows:
    print("None")
else:
    for row in missing_rows:
        print(f"{row['comm_model']},{row['comm_level']},trial_id={row['trial_id']},missing={row['missing_outputs']}")

print(f"\n[REPORT] {combined_root / 'completeness_report.csv'}")
print(f"[REPORT] {combined_root / 'missing_dga_trials_to_rerun.csv'}")
if all_counts_ok and not missing_rows:
    print("[DONE] combined DGA final-500 outputs are complete")
else:
    print("[DONE WITH GAPS] combined outputs are incomplete; see reports")
PY
