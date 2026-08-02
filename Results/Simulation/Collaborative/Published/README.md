# Collaborative Simulation Published Dataset

`collaborative_topk_all_k_trial_level_system_and_timing_results.csv` is the
canonical trial-level collaborative known-target visit simulation dataset.

- 4,800 rows and 60 columns
- 6 algorithms and 48 algorithm-by-Top-K conditions
- 100 trial rows per condition
- Top-K levels: K=1, K=2, 5%, 10%, 25%, 50%, 75%, and 100%
- Includes completion performance, `max_robot_steps`, allocator timing,
  candidate-filter timing, allocator call counts, and target-visit metrics
- Combines the original 5%-100% campaign with the lower-K supplement

Integrity checks are in `../../TopKStudyVerification/`.
