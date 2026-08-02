from __future__ import annotations

import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
IMPORT_ROOTS = (
    REPOSITORY_ROOT / "Simulation" / "Architecture",
    REPOSITORY_ROOT / "Simulation" / "Architecture" / "simulator",
    REPOSITORY_ROOT,
)

for import_root in IMPORT_ROOTS:
    value = str(import_root)
    if value not in sys.path:
        sys.path.insert(0, value)
