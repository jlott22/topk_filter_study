# Results

Store committed Top-K filtering study outputs in this directory.

The simulator and hardware hub also keep small cross-condition scenario-lock
records here by default:

- `topk_simulation_scenario_manifest.json` locks the exact ordered simulator
  selection across all algorithm and Top-K conditions.
- `topk_study_scenario_manifest.json` locks the exact ordered hardware
  selection across all hardware conditions.

These are intentionally separate because the simulation campaign and physical
hardware campaign can use different numbers of scenarios. Do not reuse a lock
for a different cohort; pass an explicit alternate lock path for smoke tests.

The imported `trials_d1_c4_g19` files are the hardware trial outputs that were
present when the repository was organized. Simulator runs should use an output
directory beneath `results/`, for example:

```text
results/acbba_topk_25/
results/acbba_topk_50/
results/acbba_all_candidates/
```
