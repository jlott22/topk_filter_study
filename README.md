# Top-K Filtering Study

This repository combines the Bayesian DCTA simulator, the current hardware
implementation, and study outputs used to evaluate Top-K candidate filtering.

## Repository Layout

```text
hardware/
  Pololu_*.py
  esp32_DTA_BENCHMARK/
  metrics_hub.py
  README.md
simulator/
  benchmark_sim/
  scenarios/
  runs/sensitivity_scenarios/
  README.md
results/
  README.md
  trials_d1_c4_g19.csv
  trials_d1_c4_g19_readable.txt
Log.md
```

- `hardware/` contains the hardware working-tree snapshot imported from
  `dtca_benchmark_hardware`.
- `simulator/` contains the complete Bayesian benchmark simulator, tests, and
  bundled scenario inputs.
- `results/` contains committed study outputs. New runs should also write here.
- `Log.md` is the experiment plan, implementation record, validation ledger,
  and physical pre-campaign gate checklist.

## Simulator Quick Start

Run simulator commands from the `simulator` directory so the
`benchmark_sim` package is importable:

```powershell
cd simulator
python -m benchmark_sim.run_trials `
  --study-profile custom `
  --trial-mode coverage `
  --num-trials 1 `
  --algorithm benchmark_sim.algorithms.ACBBA:ACBBAAllocator `
  --comm-model ideal `
  --max-candidate-cells 25 `
  --out-dir ../results/acbba_topk_25
```

Run the simulator test suite with:

```powershell
cd simulator
python -m unittest discover -s benchmark_sim/tests -v
```
