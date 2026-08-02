# Physical robot trial results

This directory contains only analysis-ready physical results and their audits:

- `Completed/` preserves every consolidated onboard metric row, including
  repeats and error rows, with row-level source checksums.
- `CombinedTrials/` contains validated one-row-per-trial aggregates and the
  inclusion, exclusion, matching, and review audits. It includes five trials
  per observed algorithm; failed ACBBA 2A and HIPC 2A are excluded.
- `HILComparison/` contains the derived physical-versus-HIL timing comparison
  and its statistical and scenario-matching audits.

The raw per-robot filesystem snapshots and recovery artifacts used to build
these tables were intentionally removed from the repository during cleanup.
Their original paths and checksums remain in the result CSVs and manifests as
provenance records. Result-building and comparison scripts are in
`Hardware/Diagnostics/`.
