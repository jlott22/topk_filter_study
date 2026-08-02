# Bayesian HIL Top-K Campaign V8

This publication contains the terminal Bayesian portion of the mixed V8 HIL
campaign. Terminal does not mean every condition passed.

## Coverage

- 48 non-K=1 algorithm-by-Top-K conditions reached a terminal state.
- 29 conditions completed all scheduled trials.
- 2 conditions completed with one failed trial each.
- 17 conditions stopped because allocator timing was unusable above the
  30-second per-call cap.
- 793 system trials completed and produced 3,172 robot-level rows.
- 3 failed trials are logged: one hardware-state setup failure and two
  repeated-state deadlocks.
- All six K=1 conditions were explicitly excluded from the HIL publication.
- DGA produced no completed HIL trials because all eight non-K=1 DGA conditions
  hit the timing cap.

## Primary combined CSV

`bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv`
is the analysis-ready equivalent of the Bayesian simulation combined CSV. It
contains one row for each of the 793 successful HIL system trials, with team
steps, maximum robot steps, allocator/filter timings, call classifications,
trial status, and provenance fields.

Because 17 conditions stopped at the timing cap and three individual trials
failed, this CSV is not a complete rectangular trial matrix. Use the condition
aggregate and failure files below when analyzing missing or censored coverage.

## Supporting files

- `bayesian_hil_topk_non_k1_condition_aggregate_metrics.csv`: one row per
  terminal non-K=1 condition, including status, cap reason, trial counts,
  timing distributions, and provenance hashes.
- `bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv`:
  canonical combined successful-trial dataset with system performance,
  maximum-step, allocator, candidate-filter, and provenance metrics.
- `bayesian_hil_topk_non_k1_robot_trial_timing_and_allocator_metrics.csv`: four
  robot rows per successful system trial with device-level timing metrics.
- `bayesian_hil_topk_non_k1_failed_trial_log.csv`: the three failed-trial
  records and watchdog diagnostics.
- `bayesian_hil_topk_non_k1_watchdog_threshold_adjustment_log.csv`: one HIPC
  repeated-state watchdog threshold adjustment recorded during the Bayesian
  segment.
- `bayesian_hil_topk_non_k1_dataset_verification_manifest.json`: source report
  counts, checksums, and arithmetic verification.
- `bayesian_hil_topk_non_k1_publication_manifest.json`: publication filenames,
  checksums, coverage status, and descriptions.

The immutable source report remains at
`../../AllocatorReplay/ActiveCampaigns/pololu_native_persistent_v8/reports/bayesian_final/`.
