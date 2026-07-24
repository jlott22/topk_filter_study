# Bayesian DCTA Simulator

The `benchmark_sim` package is the complete clue-informed and coverage
simulation architecture used by the Top-K filtering study.

Key components:

- `benchmark_sim/core/belief.py`: per-robot Bayesian target belief
- `benchmark_sim/core/`: world, robot, planner, scheduler, and scenario loading
- `benchmark_sim/algorithms/`: allocation algorithms and registry
- `benchmark_sim/comms/`: message bus and degraded communication models
- `benchmark_sim/metrics/`: trial, robot, and system metric exports
- `benchmark_sim/visualization/`: optional pygame viewer
- `benchmark_sim/tests/horizon_topk/`: Top-K and horizon study drivers
- `scenarios/`: bundled scenario inputs

Run commands from this directory. The candidate prefilter is controlled by
`--max-candidate-cells <n|all>`. Write study output to `../results/<run-name>`
to keep generated results separate from simulator source.

Headless trials and unit tests require Python 3.10 or newer and use only the
standard library. The live viewer additionally requires `pygame`.
