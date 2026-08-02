# Hardware Results

- `PilotTrials/` — historical pilot inputs and the fixed five-scenario manifest.
- `PhysicalTrials/Completed/` — lossless consolidated onboard metrics across
  all four robots and all observed algorithms.
- `PhysicalTrials/CombinedTrials/` — validated one-row-per-trial results plus
  matching, review, and exclusion audits.
- `PhysicalTrials/HILComparison/` — physical-versus-HIL timing tables,
  condition/scenario audits, confidence intervals, and statistical tests.

Raw robot filesystem backups, flash images, trash folders, recovery payloads,
and transient logs are intentionally not stored here. HIL replay results are
kept separately under `../HIL/`.
