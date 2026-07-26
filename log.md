# Study Log

## 2026-07-26 — Final 500-trial Top-K simulation campaign

- Completed 18,000 condition-trials: six allocation algorithms (`ACBBA`,
  `CBAA`, `DGA`, `DMCHBA`, `HIPC`, and `PI`) at six Top-K levels (100%, 75%,
  50%, 25%, 10%, and 5%), using the same 500 scenarios in every condition.
- Used a 19 × 19 grid, four robots, ideal communication, commitment horizon
  three, and one single-threaded trial process per worker core.
- Ran structural and arithmetic smoke validation for all 36 conditions before
  the full campaign. The final output validation passed for all conditions.
- First-pass trials used a 15,000-event safety cap. Initial failures were
  retried after the campaign at 20,000 events, then individually at 50,000 and
  100,000 events.
- Five trials remained non-terminating and were excluded from descriptive
  aggregate metrics without imputation:

  | Algorithm | Top-K | Trial IDs | Excluded |
  |---|---:|---|---:|
  | HIPC | 5% | 249 | 1/500 |
  | HIPC | 10% | 81, 435 | 2/500 |
  | HIPC | 50% | 201 | 1/500 |
  | DGA | 50% | 235 | 1/500 |

- The five exclusions represent 0.028% of the 18,000 condition-trials.
  Investigation identified a rare deterministic collision-resolution
  livelock. After peer-prediction mismatches removed peers from HIPC or DGA
  local team planning, incompatible plans caused robots to alternate between
  adjacent cells. Successful sidesteps reset the blocked-goal failure history,
  preventing the normal backoff threshold from breaking the cycle. Raising the
  event cap prolonged the same state without gaining coverage.
- Counterfactual diagnostic runs that retained peers in local planning
  completed all five scenarios below the original 15,000-event cap. These
  diagnostics did not modify the campaign data or production source.
- Final descriptive results and the exclusion statement are consolidated in
  `results/FINAL_RESULTS.md`. Condition-level means are stored in
  `results/topk_500_event15000/final_condition_summary.csv`.

