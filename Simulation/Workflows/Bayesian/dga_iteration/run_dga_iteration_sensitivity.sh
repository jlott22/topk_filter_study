#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# DGA iteration sensitivity test
#
# Usage:
#   bash benchmark_sim/tests/dga_iteration/run_dga_iteration_sensitivity.sh
#   bash benchmark_sim/tests/dga_iteration/run_dga_iteration_sensitivity.sh 8
#   bash benchmark_sim/tests/dga_iteration/run_dga_iteration_sensitivity.sh 4
#
# This partitions TRIALS across NUM_CORES workers.
# Each worker runs the same iteration/communication conditions,
# but only on its assigned subset of trials.
# ============================================================

REPO_ROOT="/home/jlott/dcta_benchmark_sim"
cd "$REPO_ROOT"

NUM_CORES="${1:-8}"

SCENARIO_FILE="runs/sensitivity_scenarios/clue_tuning_g19_n300_seed20260702.csv"
SCENARIO_SEED=20260702
GRID_SIZE=19
MAX_TRIALS_TOTAL=300

OUT_ROOT="runs/sensitivity_dga_iterations"
OVERWRITE_PREVIOUS=1
SCRIPT_ROOT="benchmark_sim/tests/dga_iteration/dga_iteration_sensitivity_generated"
PARTITION_DIR="$SCRIPT_ROOT/scenario_partitions"
WORKER_DIR="$SCRIPT_ROOT/workers"
LOG_DIR="$SCRIPT_ROOT/logs"
WRAPPER_DIR="benchmark_sim/algorithms/dga_iter_wrappers"

ITERATIONS=(1 2 5 10 25 50)

# Format: label|comm_model|comm_level
COMM_CONDITIONS=(
  "ideal|ideal|0"
  "bernoulli_025|bernoulli|0.25"
  "ge_075|gilbert_elliot|0.75"
  "rayleigh_m50p66|rayleigh_style|-50.66"
)

echo "============================================================"
echo "[SETUP] DGA iteration sensitivity"
echo "[INFO] repo root: $REPO_ROOT"
echo "[INFO] cores/workers: $NUM_CORES"
echo "[INFO] scenario file: $SCENARIO_FILE"
echo "[INFO] total paired trials per condition: $MAX_TRIALS_TOTAL"
echo "============================================================"

if [[ "$OVERWRITE_PREVIOUS" -eq 1 ]]; then
  echo "[SETUP] Removing previous outputs under $OUT_ROOT"
  rm -rf "$OUT_ROOT"
fi

mkdir -p "$OUT_ROOT" "$SCRIPT_ROOT" "$PARTITION_DIR" "$WORKER_DIR" "$LOG_DIR" "$WRAPPER_DIR"
touch benchmark_sim/algorithms/dga_iter_wrappers/__init__.py

echo "[SETUP] Generating an independent tuning scenario set..."
python3 -m benchmark_sim.tests.generate_clue_sensitivity_scenarios \
  --output "$SCENARIO_FILE" \
  --grid-size "$GRID_SIZE" \
  --num-trials "$MAX_TRIALS_TOTAL" \
  --num-clues 4 \
  --num-robots 4 \
  --seed "$SCENARIO_SEED"

# ------------------------------------------------------------
# Create DGA wrapper modules for each iteration count
# ------------------------------------------------------------
for ITER in "${ITERATIONS[@]}"; do
  WRAPPER_FILE="$WRAPPER_DIR/DGA_iter_${ITER}.py"
  cat > "$WRAPPER_FILE" <<EOF
from benchmark_sim.algorithms.DGA import DGAAllocator as BaseDGAAllocator

class DGAIter${ITER}Allocator(BaseDGAAllocator):
    name = "DGA_iter_${ITER}"
    DGA_ITERATIONS_PER_TRIGGER = ${ITER}
EOF
done

# ------------------------------------------------------------
# Partition scenario trials across workers
# ------------------------------------------------------------
echo "[SETUP] Partitioning $MAX_TRIALS_TOTAL trials across $NUM_CORES workers..."

python3 - <<PY
import csv
import os

scenario_file = "$SCENARIO_FILE"
partition_dir = "$PARTITION_DIR"
num_parts = int("$NUM_CORES")
max_trials = int("$MAX_TRIALS_TOTAL")

os.makedirs(partition_dir, exist_ok=True)

with open(scenario_file, newline="") as f:
    raw_lines = f.readlines()

comment_lines = [ln for ln in raw_lines if ln.lstrip().startswith("#")]
data_lines = [ln for ln in raw_lines if not ln.lstrip().startswith("#") and ln.strip()]

if not data_lines:
    raise SystemExit(f"No CSV data rows found in {scenario_file}")

reader = csv.DictReader(data_lines)
fieldnames = reader.fieldnames
rows = list(reader)[:max_trials]

if not fieldnames:
    raise SystemExit(f"Could not read header from {scenario_file}")

parts = [[] for _ in range(num_parts)]
for idx, row in enumerate(rows):
    parts[idx % num_parts].append(row)

for part_idx, part_rows in enumerate(parts):
    out_path = os.path.join(partition_dir, f"scenario_part_{part_idx}.csv")
    with open(out_path, "w", newline="") as out:
        for ln in comment_lines:
            out.write(ln)
        writer = csv.DictWriter(out, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(part_rows)
    print(f"[PARTITION] part={part_idx} trials={len(part_rows)} file={out_path}")

print(f"[PARTITION_DONE] total_trials={sum(len(p) for p in parts)}")
PY

# ------------------------------------------------------------
# Clear old worker scripts/logs
# ------------------------------------------------------------
rm -f "$WORKER_DIR"/worker_*.sh
rm -f "$LOG_DIR"/worker_*.log

# ------------------------------------------------------------
# Create one worker per core
# ------------------------------------------------------------
for PART in $(seq 0 $((NUM_CORES - 1))); do
  WORKER_SCRIPT="$WORKER_DIR/worker_${PART}.sh"
  PART_SCENARIO="$PARTITION_DIR/scenario_part_${PART}.csv"

  cat > "$WORKER_SCRIPT" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "$REPO_ROOT"

PART="$PART"
PART_SCENARIO="$PART_SCENARIO"
GRID_SIZE="$GRID_SIZE"
OUT_ROOT="$OUT_ROOT"

echo "[START] worker=\$PART at \$(date)"
echo "[INFO] scenario partition: \$PART_SCENARIO"
echo "[INFO] trial rows in partition:"
python3 - <<PY2
import csv
p = "\$PART_SCENARIO"
with open(p, newline="") as f:
    rows = [r for r in csv.DictReader(line for line in f if not line.lstrip().startswith("#"))]
print(len(rows))
PY2

EOF

  for ITER in "${ITERATIONS[@]}"; do
    for COMM in "${COMM_CONDITIONS[@]}"; do
      IFS="|" read -r LABEL MODEL LEVEL <<< "$COMM"

      ALG_PATH="benchmark_sim.algorithms.dga_iter_wrappers.DGA_iter_${ITER}:DGAIter${ITER}Allocator"
      OUT_DIR="${OUT_ROOT}/iter_${ITER}/${LABEL}/part_${PART}"

      cat >> "$WORKER_SCRIPT" <<EOF

echo "============================================================"
echo "[RUN] worker=\$PART iter=$ITER comm=$LABEL model=$MODEL level=$LEVEL"
echo "[TIME] \$(date)"

mkdir -p "$OUT_DIR"

python3 -m benchmark_sim.run_trials \\
  --scenario-file "\$PART_SCENARIO" \\
  --algorithm "$ALG_PATH" \\
  --comm-model "$MODEL" \\
  --comm-level "$LEVEL" \\
  --grid-size "\$GRID_SIZE" \\
  --max-trials 999999 \\
  --out-dir "$OUT_DIR" \\
  > "$OUT_DIR/run.log" 2>&1

echo "[DONE] worker=\$PART iter=$ITER comm=$LABEL at \$(date)"

EOF

    done
  done

  cat >> "$WORKER_SCRIPT" <<EOF

echo "[FINISHED] worker=\$PART at \$(date)"
EOF

done

# ------------------------------------------------------------
# Launch all workers using bash, no chmod needed
# ------------------------------------------------------------
echo "[LAUNCH] Starting $NUM_CORES workers..."

for PART in $(seq 0 $((NUM_CORES - 1))); do
  nohup bash "$WORKER_DIR/worker_${PART}.sh" > "$LOG_DIR/worker_${PART}.log" 2>&1 &
  echo "[LAUNCHED] worker_$PART pid=$!"
done

echo
echo "============================================================"
echo "[RUNNING] DGA iteration sensitivity started."
echo
echo "Monitor all workers:"
echo "  tail -f $LOG_DIR/worker_*.log"
echo
echo "Check active workers:"
echo "  ps -ef | grep dga_iteration_sensitivity_generated/workers | grep -v grep"
echo
echo "Check completed worker logs:"
echo "  grep FINISHED $LOG_DIR/worker_*.log"
echo
echo "Outputs:"
echo "  $OUT_ROOT"
echo "============================================================"
