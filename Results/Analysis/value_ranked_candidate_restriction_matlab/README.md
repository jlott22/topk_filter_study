# MATLAB analysis package: value-ranked candidate restriction

This package builds one analysis dataset and then generates every figure from that dataset. It uses the terminology in the current paper: **Probabilistic Search**, **Collaborative Visit**, and **value-ranked candidate restriction**.

## MATLAB requirements

Your current installation is sufficient:

- MATLAB R2025b (25.2)
- Statistics and Machine Learning Toolbox (25.2)

No additional toolbox is required. The 99,999-permutation multivariate tests are implemented in `build_analysis_tables.m`. Parallel Computing Toolbox is optional and is not used by default.

The serial sensitivity analysis is computationally expensive because it runs 36 omnibus tests before conditional pairwise and mode-level follow-ups. Runtime depends on the computer and on how many omnibus tests are significant. The script caches completed sensitivity results in the Analysis/Cache folder so later figure regeneration does not repeat the permutations.

## Restriction-mode mapping

| Mode | Probabilistic Search | Collaborative Visit |
|---|---:|---:|
| R0 | 100% (K=361) | 100% (K=50) |
| R1 | 75% (K=271) | 75% (K=38) |
| R2 | 50% (K=181) | 50% (K=25) |
| R3 | 25% (K=90) | 25% (K=13) |
| R4 | 10% (K=36) | 10% (K=5) |
| R5 | 5% (K=18) | 5% (K=3) |
| R6 | 3% (K=11) | 4% (K=2) |
| R7 | 1% (K=4) | — |

Absolute K=1 is not assigned a mode. It is excluded from the primary performance, computation, and tradeoff figures but retained in the failure table and failure-rate figure.

## Sensitivity-study interpretation confirmed from the attached file

The attached `paired_step_results.csv` contains the three requested environmental studies for **Probabilistic Search**:

- Grid size: 14x14, **19x19 baseline**, 28x28; independent groups.
- Team size: 2, **4 baseline**, 8 robots; matched trials.
- Communication: **Ideal baseline**, Bernoulli 25%, Gilbert-Elliott 25%, Rayleigh 25%; matched trials.

The file also contains one Collaborative Visit `multitarget` campaign, but it does not contain multiple collaborative environments. The script therefore runs environmental sensitivity tests only for Probabilistic Search. It records this limitation in the source audit rather than creating unsupported collaborative tests.

For maximum robot steps, the five-point curve is normalized from the raw `max_steps_any_robot` values. For total team steps, the source file already provides `total_step_degradation_pct`, which is used directly after checking that all six restriction levels are present.

The independent grid-size omnibus test uses a Euclidean multivariate pseudo-F statistic. The paired team-size and communication omnibus tests center each matched trial across environments and use the squared separation of the environment mean curves. Pairwise curve tests use the squared Euclidean distance between mean five-mode curves, and mode-level tests use the absolute mean degradation difference. The permutation scheme, not a parametric reference distribution, determines every p-value.

## Files

- `analysis_config.m` — all paths, terminology, mode mapping, output settings, and figure limits.
- `verify_inputs.m` — checks MATLAB/toolboxes, every source file, required columns, and the HIL condition-level files needed to identify CAP conditions.
- `build_analysis_tables.m` — creates every summary, validation, failure, claim, and sensitivity table.
- `plot_ps_performance.m` — Probabilistic Search paired maximum-step line graph.
- `plot_cv_performance.m` — Collaborative Visit paired maximum-step line graph.
- `plot_compute_heatmaps.m` — two simulation computation-change panels and two absolute RP2040 latency panels.
- `plot_filter_share_heatmaps.m` — optional RP2040 candidate-filter share figure.
- `plot_tradeoff_trajectories.m` — 2-column x 3-row Probabilistic Search maximum-step versus allocator-time-per-call trajectories; negative y means compute saved.
- `plot_failure_rates.m` — simulation and HIL failure/noncompletion heatmaps, including K=1 as a special condition.
- `run_all.m` — runs the full table build and every figure.
- `restriction_mode_table.tex` — Overleaf-ready methodology table.

## Running the analysis

1. Copy the package folder anywhere on the MATLAB path.
2. Open `analysis_config.m` and verify the root path.
3. Run the preflight alone if desired:

```matlab
cfg = analysis_config();
[cfg, SourceAudit, AnalysisSettings] = verify_inputs(cfg);
```

4. Run everything:

```matlab
run_all
```

Or build tables once and regenerate individual figures later:

```matlab
build_analysis_tables
plot_ps_performance
plot_cv_performance
plot_compute_heatmaps
plot_filter_share_heatmaps
plot_tradeoff_trajectories
plot_failure_rates
```

## Outputs

All outputs are written to:

```text
C:\Users\lottj\Desktop\research\decentralized benchmark\topk_filter_study\Results\Analysis
```

The main files are:

- `value_ranked_candidate_restriction_analysis.mat`
- `value_ranked_candidate_restriction_analysis.xlsx`
- `Figures/*.png`
- `Figures/*.pdf`
- `Figures/*.fig`

The workbook and MAT file contain separate tables:

- `ModeMap`
- `ConditionSummary`
- `PairedChangeSummary`
- `FailureSummary`
- `HILFeasibility`
- `ValidationSummary`
- `ClaimSummary`
- `SensitivityOmnibus`
- `SensitivityPairwise`
- `SensitivityLevelwise`
- `SensitivitySignificantOnly`
- `SensitivityCurveSummary`
- `SourceAudit`
- `AnalysisSettings`

## Figure conventions

- Figure axes use R0-R7 rather than repeated percentages.
- User-facing titles avoid “Top-K.”
- Probabilistic Search and Collaborative Visit performance figures use the same -5% to 40% y-range.
- Computation change is defined relative to R0: **negative values are saved computation; positive values are added computation**.
- Heatmaps use a softened green-yellow-red gradient. HIL absolute latency is transformed only for color assignment; cell labels remain in milliseconds.
- Tradeoff trajectories use paired maximum-step change and allocator-time-per-call change from R0. The lower-left direction is favorable. Stars mark the actual physical-validation modes.
- No validation figure is generated; validation statistics are retained in the workbook.

## HIL condition files

The successful HIL trial CSVs do not by themselves identify zero-completion CAP conditions. `verify_inputs.m` therefore requires condition-aggregate files. It first checks the configured names and then searches the same campaign folder for `*condition*aggregate*metrics*.csv`. If discovery fails, set the exact paths in `analysis_config.m`.
