from __future__ import annotations

import json
import csv
from pathlib import Path
from typing import Sequence


def write_csv(path: str | Path, rows: Sequence[dict]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for name in row:
            if name not in seen:
                seen.add(name)
                fieldnames.append(name)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_outputs(
    out_dir: str | Path,
    trial_summary_rows: Sequence[dict],
    system_performance_rows: Sequence[dict],
    robot_performance_rows: Sequence[dict],
    config: dict,
    write_parquet: bool = False,
) -> None:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "trial_summary.csv", trial_summary_rows)
    write_csv(out / "system_performance.csv", system_performance_rows)
    write_csv(out / "robot_performance.csv", robot_performance_rows)
    (out / "config_used.json").write_text(json.dumps(config, indent=2, default=str))
