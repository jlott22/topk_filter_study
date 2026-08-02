# Completed physical-trial metrics

`all_robots_all_algorithms.csv` is the lossless consolidated table for every
available onboard metric row from physical robots 00 through 03. It contains
126 rows from 24 source files: 82 rows from the initial recovery and 44 rows
from the completed-trials capture.

No repeated trial or error row was removed. The source order is retained and
every output row includes its capture batch, source path, source file hash,
source line, original field count, structural-repair flag, and source-row hash.

Nine raw lines contained the target location `(-1, -1)` without CSV quotes,
which caused generic readers to see 46 fields. The consolidated files merge
only those two target-location fragments so every derived row has the common
45-field metric schema. The source paths and byte-level checksums remain in
the consolidated rows and `manifest.json`; the bulky robot filesystem
captures themselves are intentionally not retained in this repository.

| Algorithm | Rows |
|---|---:|
| ACBBA | 24 |
| CBAA | 27 |
| DGA | 0 |
| DMCHBA | 23 |
| HIPC | 29 |
| PI | 23 |

No `metrics-log-DGA.txt` source file exists in either capture for any robot.
The header-only `by_algorithm/DGA.csv` and `coverage.csv` make that absence
explicit rather than silently treating DGA as completed data.

Lossless per-algorithm exports are in `by_algorithm/`. `manifest.json` records
source and output hashes. `Hardware/Diagnostics/build_completed_physical_metrics.py`
can regenerate this archive when the external raw captures are restored at
the source paths recorded in the manifest.
