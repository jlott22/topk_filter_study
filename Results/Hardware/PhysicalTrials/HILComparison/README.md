# Physical versus HIL timing comparison

This analysis compares the 25 validated physical trials with the Bayesian HIL
campaign at the same algorithm-specific Top-K settings. The five physical
source episodes are 4, 53, 232, 394, and 473. None occurs in the 25-scenario
HIL subset, so scenario-by-scenario pairing would be invalid. Comparisons are
therefore unpaired: five physical system trials versus 25 HIL system trials per
algorithm. Target-and-clue content was also compared from the canonical scenario
file and found zero identical scenarios. Two HIL trials share only a target:
HIL 371 with physical episode 4 at `5/9`, and HIL 67 with physical episode 473
at `13/16`; their clue layouts differ, so those comparisons are descriptive only.

The independent unit is a four-robot system trial, not an individual robot.
Each timing value is the call-weighted mean reconstructed from all four robot
rows. Allocation-exclusive is derived in both environments as end-to-end
allocator total minus filter total, divided by timed allocator calls. This is
necessary because physical `allocator_solve_time_us_total` omits a repeatable
algorithm-specific non-filter overhead that HIL allocator-exclusive includes.
Filter is total candidate-filter time divided by actual filter invocations in
both environments; combined is end-to-end allocator time divided by timed
allocator calls. The published HIL filter mean is not used directly because its
denominator is all timed allocator calls, not actual filter invocations.

For each of five algorithms and three timing metrics, the analysis reports an
exact two-sided permutation test of arithmetic means, an exact permutation test
of log means, an exact rank/Mann-Whitney permutation test, Cliff's delta, and a
50,000-resample stratified bootstrap interval for the mean difference and mean
    ratio. Benjamini-Hochberg correction is applied across the 15 log-mean tests.
    At adjusted alpha 0.05, 5 of 15 comparisons show a detectable
    difference. Failure to reject is labeled as no detectable difference, not proof
    that physical and HIL timings are equal. A second algorithm-stratified test
    assesses the overall environment shift for each metric using 200,000 label
    permutations and controls false discovery across the three metrics. A formal
    equivalence test is not run because no practical equivalence margin was specified.

Files:

    - `trial_level_timing_comparison.csv`: all 150 system-trial observations.
    - `statistical_tests_by_algorithm.csv`: detailed tests and effect sizes.
    - `statistical_tests_overall_stratified.csv`: overall tests controlling for algorithm.
- `metric_overview.csv`: metric-level direction summary.
- `matching_audit.csv`: exact condition selection and zero scenario overlap.
- `call_mix_audit.csv`: filter invocation frequency by algorithm and environment.
- `scenario_content_overlap_audit.csv`: target-and-clue identity check for all physical trials.
- `target_only_performance_timing_comparison.csv`: descriptive comparisons for
  same-target HIL trials whose clue layouts differ.
