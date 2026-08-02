#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
RUN_ROOT="$REPO_ROOT/runs/sensitivity_grid_density_50"

cd "$REPO_ROOT"
python3 "$SCRIPT_DIR/combine_grid_density_sensitivity.py" --run-root "$RUN_ROOT"

echo ""
echo "[DONE] Combined CSVs are in:"
echo "$RUN_ROOT/combined"
echo ""
echo "Optional verification:"
echo "python3 benchmark_sim/tests/grid_density/verify_grid_density_sensitivity.py --run-root $RUN_ROOT"
