# Results

This directory contains result artifacts only. Start with the published files
below; raw campaign material is retained in deeper source-record directories.

## Published datasets

| Mission | Execution | State | Primary file |
|---|---|---|---|
| Bayesian CLUE search | Simulation | Complete | [`Simulation/Bayesian/Published/bayesian_topk_all_k_trial_level_system_and_timing_results.csv`](Simulation/Bayesian/Published/bayesian_topk_all_k_trial_level_system_and_timing_results.csv) |
| Collaborative known-target visit | Simulation | Complete | [`Simulation/Collaborative/Published/collaborative_topk_all_k_trial_level_system_and_timing_results.csv`](Simulation/Collaborative/Published/collaborative_topk_all_k_trial_level_system_and_timing_results.csv) |
| Bayesian CLUE search | HIL | Terminal with capped conditions | [`HIL/Bayesian/CompletedTopKCampaignV8/bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv`](HIL/Bayesian/CompletedTopKCampaignV8/bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv) |
| Collaborative known-target visit | HIL | Incomplete / restart later | [`HIL/Collaborative/`](HIL/Collaborative/README.md) |

The two simulation CSVs are the canonical all-K datasets. They already combine
the original 5%-100% campaigns with the later lower-K runs; lower-K data is not
published as a separate study dataset.

See [`results_catalog.csv`](results_catalog.csv) for a machine-readable index
and full artifact descriptions. The migration checksum audit is recorded in
[`reorganization_result_integrity_audit.json`](reorganization_result_integrity_audit.json).

## Directory map

- `Analysis/`: analysis products and analysis guidance.
- `Hardware/`: physical-robot pilot and trial results.
- `HIL/`: hardware-in-the-loop publications and raw campaign records.
- `Simulation/`: simulation publications, raw campaign records, and validation.
