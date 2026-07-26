# Read-Only Top-K Sensitivity Suite

This standalone package orchestrates the sensitivity campaign without editing
either simulator architecture.

From the repository root:

```powershell
python -m sensitivity_suite prepare
python -m sensitivity_suite run --workers 12
python -m sensitivity_suite status
python -m sensitivity_suite verify
python -m sensitivity_suite report
```

The default campaign root is `results/sensitivity_suite`. The manifest contains
324 conditions and 16,200 expected trials. Runs resume from the CSV rows already
written by the existing simulator CLIs.

The worker count is intentionally locked to 12: three-quarters of this
computer's 16 physical cores.
