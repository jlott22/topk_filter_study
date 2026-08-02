# Bayesian Simulation Published Dataset

`bayesian_topk_all_k_trial_level_system_and_timing_results.csv` is the canonical
trial-level Bayesian CLUE-search simulation dataset.

- 27,000 rows and 61 columns
- 6 algorithms and 54 algorithm-by-Top-K conditions
- 500 trial rows per condition
- Top-K levels: K=1, 1%, 3%, 5%, 10%, 25%, 50%, 75%, and 100%
- Includes mission performance, `max_steps_any_robot`, allocator timing,
  candidate-filter timing, allocator call counts, and host trial runtime
- Combines the original 5%-100% campaign with the lower-K supplement

Five source simulation trials reached the 100,000-event safety cap. Their
failure status is retained in the unified rows rather than imputed or dropped.
Integrity checks are in `../../TopKStudyVerification/`.
