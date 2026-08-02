#!/usr/bin/env bash
set -uo pipefail

# Run one complete DGA final-500 job per communication condition.
# Usage: bash benchmark_sim/tests/dga_final500/run_dga_final500_by_condition.sh [max_parallel_conditions]

REPO_ROOT="${DCTA_REPO_ROOT:-/home/jlott/dcta_benchmark_sim}"
SCENARIO_FILE="$REPO_ROOT/scenarios/final_trial_500.csv"
RAW_ROOT="$REPO_ROOT/runs/final_500_all/raw/dga"
LOG_DIR="$RAW_ROOT/_logs"
STATE_DIR="$RAW_ROOT/_condition_status"
MAX_PARALLEL="${1:-1}"
GRID_SIZE=19
ROBOT_COUNT=4
SEED=0
ALGORITHM="benchmark_sim.algorithms.DGA:DGAAllocator"
SIMULATOR=(python3 -m benchmark_sim.run_trials)

# folder|comm_model|output_comm_level_label|CLI_comm_level
CONDITIONS=(
  "ideal_1_0|ideal|1.0|1.0"
  "bernoulli_drop_0_05|bernoulli|drop_0.05|0.05"
  "bernoulli_drop_0_1|bernoulli|drop_0.1|0.1"
  "bernoulli_drop_0_2|bernoulli|drop_0.2|0.2"
  "bernoulli_drop_0_3|bernoulli|drop_0.3|0.3"
  "bernoulli_drop_0_4|bernoulli|drop_0.4|0.4"
  "bernoulli_drop_0_5|bernoulli|drop_0.5|0.5"
  "bernoulli_drop_0_6|bernoulli|drop_0.6|0.6"
  "bernoulli_drop_0_7|bernoulli|drop_0.7|0.7"
  "gilbert_elliot_pGG_0_3_pBB_0_7|gilbert_elliot|pGG_0.3_pBB_0.7|0.3"
  "gilbert_elliot_pGG_0_4_pBB_0_6|gilbert_elliot|pGG_0.4_pBB_0.6|0.4"
  "gilbert_elliot_pGG_0_5_pBB_0_5|gilbert_elliot|pGG_0.5_pBB_0.5|0.5"
  "gilbert_elliot_pGG_0_6_pBB_0_4|gilbert_elliot|pGG_0.6_pBB_0.4|0.6"
  "gilbert_elliot_pGG_0_7_pBB_0_3|gilbert_elliot|pGG_0.7_pBB_0.3|0.7"
  "gilbert_elliot_pGG_0_8_pBB_0_2|gilbert_elliot|pGG_0.8_pBB_0.2|0.8"
  "gilbert_elliot_pGG_0_9_pBB_0_1|gilbert_elliot|pGG_0.9_pBB_0.1|0.9"
  "gilbert_elliot_pGG_0_95_pBB_0_05|gilbert_elliot|pGG_0.95_pBB_0.05|0.95"
  "rayleigh_style_sens_neg32_58|rayleigh_style|sens_-32.58|-32.58"
  "rayleigh_style_sens_neg37_79|rayleigh_style|sens_-37.79|-37.79"
  "rayleigh_style_sens_neg42_16|rayleigh_style|sens_-42.16|-42.16"
  "rayleigh_style_sens_neg46_04|rayleigh_style|sens_-46.04|-46.04"
  "rayleigh_style_sens_neg49_17|rayleigh_style|sens_-49.17|-49.17"
  "rayleigh_style_sens_neg52_15|rayleigh_style|sens_-52.15|-52.15"
  "rayleigh_style_sens_neg56_04|rayleigh_style|sens_-56.04|-56.04"
  "rayleigh_style_sens_neg59_4|rayleigh_style|sens_-59.4|-59.4"
)

if ! [[ "$MAX_PARALLEL" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: max_parallel_conditions must be a positive integer" >&2
  exit 2
fi
if [[ ! -d "$REPO_ROOT/benchmark_sim" ]]; then
  echo "ERROR: repository package not found: $REPO_ROOT/benchmark_sim" >&2
  exit 2
fi
if [[ ! -f "$SCENARIO_FILE" ]]; then
  echo "ERROR: scenario file not found: $SCENARIO_FILE" >&2
  exit 2
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 is not available" >&2
  exit 2
fi

cd "$REPO_ROOT" || exit 2
mkdir -p "$RAW_ROOT" "$LOG_DIR" "$STATE_DIR"
rm -f "$STATE_DIR"/*.status "$STATE_DIR/conditions.tsv"

EXPECTED_TRIALS="$(python3 - "$SCENARIO_FILE" <<'PY'
import csv
import sys
from pathlib import Path

path = Path(sys.argv[1])
with path.open(newline="") as handle:
    rows = list(csv.DictReader(line for line in handle if line.strip() and not line.lstrip().startswith("#")))
print(len(rows))
PY
)"
EXPECTED_ROBOT_ROWS=$((EXPECTED_TRIALS * ROBOT_COUNT))
if [[ "$EXPECTED_TRIALS" -ne 500 ]]; then
  echo "ERROR: expected exactly 500 scenario rows, found $EXPECTED_TRIALS in $SCENARIO_FILE" >&2
  exit 2
fi

for condition in "${CONDITIONS[@]}"; do
  IFS="|" read -r folder model label cli_value <<< "$condition"
  printf '%s\t%s\t%s\t%s\n' "$folder" "$model" "$label" "$cli_value" >> "$STATE_DIR/conditions.tsv"
done

echo "============================================================"
echo "[SETUP] DGA final-500 by communication condition"
echo "[INFO] repo root: $REPO_ROOT"
echo "[INFO] scenario file: $SCENARIO_FILE"
echo "[INFO] scenario rows: $EXPECTED_TRIALS"
echo "[INFO] raw output root: $RAW_ROOT"
echo "[INFO] simulator entrypoint: ${SIMULATOR[*]}"
echo "[INFO] simulator command template: ${SIMULATOR[*]} --scenario-file $SCENARIO_FILE --trial-mode clue_search --algorithm $ALGORITHM --algorithm-name DGA --comm-model <model> --comm-level <value> --grid-size $GRID_SIZE --num-robots $ROBOT_COUNT --robot-start-layout edge_even --condition-id <folder> --max-trials $EXPECTED_TRIALS --seed $SEED --out-dir <condition_dir>"
echo "[INFO] max parallel conditions: $MAX_PARALLEL"
echo "[INFO] algorithm CLI value: $ALGORITHM"
echo "[INFO] grid size: $GRID_SIZE"
echo "[INFO] robot count: $ROBOT_COUNT"
echo "[INFO] expected trials per condition: $EXPECTED_TRIALS"
echo "[INFO] condition matrix (${#CONDITIONS[@]} conditions):"
for condition in "${CONDITIONS[@]}"; do
  IFS="|" read -r folder model label cli_value <<< "$condition"
  echo "  $model:$label cli=$cli_value folder=$folder"
done
echo "============================================================"

run_condition() {
  local folder="$1"
  local model="$2"
  local label="$3"
  local cli_value="$4"
  local out_dir="$RAW_ROOT/$folder"
  local log_file="$LOG_DIR/$folder.log"
  local status_file="$STATE_DIR/$folder.status"
  local rc

  mkdir -p "$out_dir"
  rm -f \
    "$out_dir/system_performance.csv" \
    "$out_dir/trial_summary.csv" \
    "$out_dir/robot_performance.csv" \
    "$out_dir/config_used.json"

  echo "[START CONDITION] $model:$label cli=$cli_value"
  echo "[OUTPUT] $out_dir"

  "${SIMULATOR[@]}" \
    --scenario-file "$SCENARIO_FILE" \
    --trial-mode clue_search \
    --algorithm "$ALGORITHM" \
    --algorithm-name DGA \
    --comm-model "$model" \
    --comm-level "$cli_value" \
    --grid-size "$GRID_SIZE" \
    --num-robots "$ROBOT_COUNT" \
    --robot-start-layout edge_even \
    --condition-id "$folder" \
    --max-trials "$EXPECTED_TRIALS" \
    --seed "$SEED" \
    --out-dir "$out_dir" \
    > "$log_file" 2>&1
  rc=$?

  printf '%s\n' "$rc" > "$status_file"
  if [[ "$rc" -eq 0 ]]; then
    echo "[CONDITION OK] $model:$label"
  else
    echo "[CONDITION FAILED] $model:$label exit=$rc log=$log_file"
  fi
  return 0
}

for condition in "${CONDITIONS[@]}"; do
  while (( $(jobs -pr | wc -l) >= MAX_PARALLEL )); do
    wait -n || true
  done
  IFS="|" read -r folder model label cli_value <<< "$condition"
  run_condition "$folder" "$model" "$label" "$cli_value" &
done
wait || true

python3 - "$SCENARIO_FILE" "$RAW_ROOT" "$STATE_DIR/conditions.tsv" "$EXPECTED_TRIALS" "$EXPECTED_ROBOT_ROWS" <<'PY'
import csv
import sys
from pathlib import Path

scenario_path = Path(sys.argv[1])
raw_root = Path(sys.argv[2])
conditions_path = Path(sys.argv[3])
expected_trials = int(sys.argv[4])
expected_robot_rows = int(sys.argv[5])

with conditions_path.open() as handle:
    conditions = [line.rstrip("\n").split("\t") for line in handle if line.strip()]

with scenario_path.open(newline="") as handle:
    scenario_rows = list(csv.DictReader(line for line in handle if line.strip() and not line.lstrip().startswith("#")))

if scenario_rows and "trial_id" in scenario_rows[0]:
    expected_ids = [str(row["trial_id"]) for row in scenario_rows]
elif scenario_rows and "episode" in scenario_rows[0]:
    expected_ids = [str(row["episode"]) for row in scenario_rows]
else:
    expected_ids = [str(index) for index in range(len(scenario_rows))]
expected_id_set = set(expected_ids)

def read_metric(path):
    if not path.is_file():
        return 0, set(), "missing"
    try:
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
    except Exception as exc:
        return 0, set(), f"read_error:{exc}"
    ids = {str(row.get("trial_id", row.get("episode", ""))) for row in rows}
    ids.discard("")
    return len(rows), ids, ""

self_check_rows = []
missing_rows = []
failed_rows = []

print("\n[RAW COUNT SUMMARY]")
for folder, model, label, cli_value in conditions:
    out_dir = raw_root / folder
    system_count, system_ids, system_error = read_metric(out_dir / "system_performance.csv")
    trial_count, trial_ids, trial_error = read_metric(out_dir / "trial_summary.csv")
    robot_count, robot_ids, robot_error = read_metric(out_dir / "robot_performance.csv")
    status_path = raw_root / "_condition_status" / f"{folder}.status"
    process_rc = status_path.read_text().strip() if status_path.is_file() else "missing"

    missing_system = expected_id_set - system_ids
    missing_trial = expected_id_set - trial_ids
    missing_robot = expected_id_set - robot_ids
    unexpected_ids = (system_ids | trial_ids | robot_ids) - expected_id_set
    complete = (
        process_rc == "0"
        and system_count == expected_trials
        and trial_count == expected_trials
        and robot_count == expected_robot_rows
        and system_ids == expected_id_set
        and trial_ids == expected_id_set
        and robot_ids == expected_id_set
        and not system_error
        and not trial_error
        and not robot_error
    )
    result = "OK" if complete else "FAILED"
    print(
        f"{model},{label}: system={system_count}/{expected_trials}, "
        f"trial={trial_count}/{expected_trials}, robot={robot_count}/{expected_robot_rows} => {result}"
    )

    missing_union = missing_system | missing_trial | missing_robot
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
            "cli_value": cli_value,
            "condition_folder": folder,
            "trial_id": trial_id,
            "missing_outputs": ";".join(missing_outputs),
        })

    errors = ";".join(error for error in (system_error, trial_error, robot_error) if error)
    self_check_rows.append({
        "comm_model": model,
        "comm_level": label,
        "cli_value": cli_value,
        "condition_folder": folder,
        "process_exit_code": process_rc,
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
        "unexpected_trial_ids": ";".join(sorted(unexpected_ids)),
        "read_errors": errors,
        "status": result,
    })
    if not complete:
        reasons = []
        if process_rc != "0":
            reasons.append(f"process_exit_code={process_rc}")
        if system_count != expected_trials:
            reasons.append(f"system_rows={system_count}")
        if trial_count != expected_trials:
            reasons.append(f"trial_rows={trial_count}")
        if robot_count != expected_robot_rows:
            reasons.append(f"robot_rows={robot_count}")
        if missing_union:
            reasons.append(f"missing_trials={len(missing_union)}")
        if unexpected_ids:
            reasons.append(f"unexpected_trials={len(unexpected_ids)}")
        if errors:
            reasons.append(errors)
        failed_rows.append({
            "comm_model": model,
            "comm_level": label,
            "cli_value": cli_value,
            "condition_folder": folder,
            "exit_code": process_rc,
            "reason": ";".join(reasons),
            "log_file": str(raw_root / "_logs" / f"{folder}.log"),
        })

def write_report(path, fieldnames, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

write_report(
    raw_root / "dga_final500_raw_self_check.csv",
    list(self_check_rows[0].keys()),
    self_check_rows,
)
write_report(
    raw_root / "dga_final500_missing_trials_to_rerun.csv",
    ["comm_model", "comm_level", "cli_value", "condition_folder", "trial_id", "missing_outputs"],
    missing_rows,
)
write_report(
    raw_root / "dga_final500_failed_conditions.csv",
    ["comm_model", "comm_level", "cli_value", "condition_folder", "exit_code", "reason", "log_file"],
    failed_rows,
)

print("\n[TRIALS NEEDING RERUN]")
if not missing_rows:
    print("None")
else:
    for row in missing_rows:
        print(f"{row['comm_model']},{row['comm_level']},trial_id={row['trial_id']},missing={row['missing_outputs']}")

print(f"\n[REPORT] {raw_root / 'dga_final500_raw_self_check.csv'}")
print(f"[REPORT] {raw_root / 'dga_final500_missing_trials_to_rerun.csv'}")
print(f"[REPORT] {raw_root / 'dga_final500_failed_conditions.csv'}")
if failed_rows:
    print(f"[DONE WITH FAILURES] {len(failed_rows)} of {len(conditions)} conditions failed validation")
else:
    print(f"[DONE] all {len(conditions)} conditions passed validation")
PY
