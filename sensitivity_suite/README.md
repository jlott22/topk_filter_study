# Cross-Mission Sensitivity Suite

This package prepares, verifies, and reports the completed Top-K sensitivity
campaign. It orchestrates two mission types without modifying either
simulator:

- Bayesian CLUE search: scale and communication sensitivity.
- Collaborative known-target visit: 50 targets, 100 scenarios per condition.

Run commands from the repository root:

```powershell
python -m sensitivity_suite prepare
python -m sensitivity_suite run --workers 12
python -m sensitivity_suite verify
python -m sensitivity_suite report
```

The default campaign root is `results/sensitivity_suite`.

- `campaign_records/condition_manifest.csv` records all 324 conditions and
  their exact commands.
- `raw/bayesian_clue_search/` holds the 14,400 Bayesian trials.
- `raw/collaborative_known_target_visit/` holds the 3,600 collaborative trials.
- `reports/` contains an all-mission report plus mission-specific report sets.

The default worker count is locked to 12, three-quarters of this computer's
16 physical cores. The standalone `run_collaborative_100.py` utility was used
to replace the collaborative 50-scenario outputs with the finalized
100-scenario outputs.
