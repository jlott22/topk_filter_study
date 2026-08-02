# Simulation Architecture

This directory contains executable experiment architecture and study workflows.
It does not contain the canonical published result CSVs.

- `Architecture/simulator/`: Bayesian CLUE-search simulator.
- `Architecture/allocator_replay/`: HIL campaign, replay, reporting, and device
  tooling.
- `Studies/sensitivity_suite/`: sensitivity-campaign preparation and reporting.
- `Studies/hil_aligned_simulation/`: low-K and HIL-aligned simulation workflows.
- `Workflows/Bayesian/`: Bayesian experiment and optimization workflows.
- `Archive/Legacy/`: superseded implementations retained for provenance.
- `dcta_benchmark_sim/`: ignored local junction to the separately versioned
  collaborative-visit simulator repository at the same parent workspace.

Published outputs are under [`../Results/`](../Results/README.md).
