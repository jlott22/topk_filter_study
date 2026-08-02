# Cross-Mission Sensitivity Results

This is the completed 324-condition, 18,000-trial sensitivity campaign. It
contains 9,000 Bayesian scale trials, 5,400 Bayesian communication trials,
and 3,600 collaborative known-target visit trials.

## Read results first

- [`reports/bayesian_clue_search/`](reports/bayesian_clue_search/): paired
  step comparisons for Bayesian scale and communication sensitivity tests.
- [`reports/collaborative_known_target_visit/`](reports/collaborative_known_target_visit/): paired step comparisons for the collaborative mission.
- [`collaborative_computational_summary.csv`](collaborative_computational_summary.csv): collaborative allocator, solve-only, and filter timing summary.
- [`reports/all_missions/`](reports/all_missions/): the unfiltered combined
  reports across both mission types.

## Raw outputs by mission

- [`raw/bayesian_clue_search/`](raw/bayesian_clue_search/):
  - `scale/`: grid-size and robot-count sensitivity.
  - `communication/`: Bernoulli, Gilbert-Elliott, and Rayleigh-style
    communication sensitivity.
- [`raw/collaborative_known_target_visit/topk_sensitivity/`](raw/collaborative_known_target_visit/topk_sensitivity/): the 19x19, four-robot, 50-known-target Top-K sensitivity results.

## Supporting records

- [`scenarios/`](scenarios/): deterministic CLUE and known-target input
  scenarios.
- [`campaign_records/`](campaign_records/): condition manifest, run ledger,
  progress, failures, verification summary, and channel calibration.
- [`validation/`](validation/): smoke checks, event-cap retries, diagnostics,
  and redistribution evidence. `validation/smoke/known_visit/` is a small
  pre-campaign simulator check, not a training run and not a headline result.

`_cv100/` is a local, ignored staging area used when the 100-scenario
collaborative results were assembled. It is not part of the published data.
