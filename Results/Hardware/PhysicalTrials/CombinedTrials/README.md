# Combined physical-trial metrics

`combined_metrics_per_trial.csv` contains one aggregate row for every matched
four-robot trial. Trial numbers follow the declared handpicked study mapping:
1=`5/9`, 2=`7/2`, 3=`3/11`, 4=`5/4`, and 5=`13/16`.

Matching is conservative: every included group contains exactly one source row
from each of robots 00, 01, 02, and 03. The operator-confirmed failed ACBBA 2A
and HIPC 2A runs are excluded. Their validated former 2B runs are the canonical
`ACBBA 2` and `HIPC 2`. DMCHBA trial 3 uses the unique four-robot duration
cluster with the smallest time range and remains flagged for review. No failed,
incomplete, or non-study-target rows are aggregated.

`matched_robot_rows.csv` is the row-level provenance for included trials.
`excluded_trial_remnants.csv` records every rejected source row and reason.
`review_flags.csv` is a filtered view of aggregate rows needing review.

Average filter and allocator times are weighted per call: total microseconds
divided by total calls across all four robots. Compute time is derived per robot
as `max(0, trial_time_ms - motor_time_ms)` before summing or averaging.
