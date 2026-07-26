# Final Top-K Filtering Simulation Results

## Experimental design

The simulator evaluated six allocation algorithms—ACBBA, CBAA, DGA, DMCHBA,
HIPC, and PI—at Top-K candidate-retention levels of 100%, 75%, 50%, 25%, 10%,
and 5%. The retained candidate limits on the 19 × 19 grid were 361, 271, 181,
90, 36, and 18 cells, respectively. Each of the 36 conditions used the same
500 clue-search scenarios, producing 18,000 condition-trials.

Every trial used four robots in the canonical edge-even starting layout, ideal
communication, a commitment horizon of three, and deterministic scenario
seeding. Trials were single-threaded and partitioned across nine of the
computer's twelve available CPU cores. A serial smoke pass checked all
conditions before the full run. The smoke checks verified completion, output
schema, metric arithmetic, candidate limits, step totals, message totals, and
positive computational timings.

The first pass used a 15,000-event safety cap. Failed trials were retried after
all first-pass work at 20,000 events, and remaining failures were run
individually at 50,000 and 100,000 events. Results below include completed
trials only and do not impute values for non-terminating runs.

## Allocation-cycle time

The primary computational result is the mean duration of one allocator call,
calculated as total allocator time divided by the number of allocator calls
across all completed trials and robots in each condition. Values are
milliseconds per allocation cycle.

| Algorithm | 100% | 75% | 50% | 25% | 10% | 5% |
|---|---:|---:|---:|---:|---:|---:|
| ACBBA | 8.9623 | 10.2503 | 9.6740 | 6.0663 | 3.0746 | 2.0615 |
| CBAA | 2.1573 | 3.6197 | 4.2964 | 2.9572 | 1.6025 | 1.1707 |
| DGA | 843.5533 | 705.7038 | 500.4456 | 265.2355 | 118.4869 | 67.4875 |
| DMCHBA | 755.3705 | 597.4596 | 254.3961 | 37.3588 | 3.4992 | 1.5880 |
| HIPC | 10.6975 | 10.1512 | 7.9966 | 4.7290 | 2.2811 | 1.4509 |
| PI | 8.9302 | 9.5477 | 8.3291 | 4.8550 | 2.1847 | 1.3142 |

From 100% to 5% Top-K, mean allocation-cycle time fell by 77.0% for ACBBA,
45.7% for CBAA, 92.0% for DGA, 99.8% for DMCHBA, 86.4% for HIPC, and 85.3%
for PI. The largest absolute reductions occurred for DGA and DMCHBA, the two
most computationally expensive allocators.

## Search-performance interpretation

The computational savings were not accompanied by a large degradation in
mean post-clue search steps. Comparing 5% with 100% Top-K, the mean changed by
+0.1% for ACBBA, -0.4% for CBAA, -7.9% for DGA, +3.5% for DMCHBA, +1.7% for
HIPC, and +3.2% for PI. Negative values indicate fewer post-clue steps. Thus,
the strongest filtering level produced substantial allocator-time reductions
while leaving the descriptive search-effort mean broadly similar in this
experiment.

These are descriptive results. Formal paired uncertainty estimates or
hypothesis tests are outside this draft and should be added before making
claims about statistical significance.

## Non-terminating edge cases

Five of the 18,000 condition-trials (0.028%) did not terminate after the
100,000-event retry and were excluded from aggregate metrics without
imputation:

| Algorithm | Top-K | Failed trial IDs | Final sample size |
|---|---:|---|---:|
| HIPC | 5% | 249 | 499 |
| HIPC | 10% | 81, 435 | 498 |
| HIPC | 50% | 201 | 499 |
| DGA | 50% | 235 | 499 |

Inspection identified a rare collision-resolution livelock. Peer-prediction
mismatches caused HIPC or DGA to omit peers from local team planning, after
which incompatible plans produced symmetric two-cell oscillations. Each
successful sidestep reset the blocked-goal failure history, so the normal
temporary backoff was never activated. Coverage remained flat while robots
continued generating valid movement events. Targets remained assigned and
reachable, communication was ideal, and increasing the safety cap only
prolonged the cyclic state.

A concise paper disclosure is:

> Of the 18,000 condition-trials, five (0.028%) did not terminate after retries
> with safety limits of up to 100,000 events and were excluded from aggregate
> metrics without imputation. Inspection identified a rare
> collision-resolution livelock in which robots alternated between adjacent
> cells without acquiring new coverage.

## Validation and data files

- `topk_500_event15000/final_condition_summary.csv` contains the condition-level
  descriptive means used in this draft.
- `topk_500_event15000/combined/all_trial_summary.csv` contains all 18,000 trial
  outcome rows.
- `topk_500_event15000/combined/all_system_performance.csv` contains all 18,000
  system-performance rows.
- `topk_500_event15000/combined/all_robot_performance.csv` contains all 72,000
  robot-performance rows.
- `topk_500_event15000/combined/all_computational_performance.csv` contains all
  72,000 computational-performance rows.
- `topk_500_event15000/smoke_validation.json` records the passing pre-campaign
  smoke validation.
- `topk_500_event15000/final_validation.json` records final structural
  validation for all 36 conditions.
- `topk_500_event15000/extended_retry_report.json` records the 50,000- and
  100,000-event retries and the five remaining failures.

