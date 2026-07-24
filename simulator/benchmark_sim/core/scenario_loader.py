from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from .types import Cell, TrialScenario


def _parse_metadata_line(line: str) -> Dict[str, Any]:
    # Lines look like: # condition: distribution=1, clues_per_object=4, grid_size=19
    line = line.strip()[1:].strip()
    meta: Dict[str, Any] = {}
    if ":" in line:
        key, rest = line.split(":", 1)
        key = key.strip()
        rest = rest.strip()
        meta[key] = rest
        for part in rest.split(","):
            if "=" in part:
                k, v = part.split("=", 1)
                meta[k.strip()] = _coerce(v.strip())
    return meta


def _coerce(value: str) -> Any:
    if value.lower() in {"none", "null"}:
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def load_scenarios(path: str | Path, max_trials: Optional[int] = None) -> List[TrialScenario]:
    path = Path(path)
    if path.suffix.lower() == ".json":
        return load_scenarios_json(path, max_trials=max_trials)
    return load_scenarios_csv(path, max_trials=max_trials)


def load_scenarios_csv(path: str | Path, max_trials: Optional[int] = None) -> List[TrialScenario]:
    path = Path(path)
    metadata: Dict[str, Any] = {"scenario_file": str(path)}
    rows: List[str] = []
    with path.open("r", newline="") as f:
        for line in f:
            if line.strip().startswith("#"):
                metadata.update(_parse_metadata_line(line))
            elif line.strip():
                rows.append(line)
    reader = csv.DictReader(rows)
    scenarios: List[TrialScenario] = []
    for idx, row in enumerate(reader):
        if max_trials is not None and len(scenarios) >= max_trials:
            break
        trial_id = int(row.get("episode", row.get("trial_id", idx)))
        target = (int(row["object_x"]), int(row["object_y"]))
        clues: List[Cell] = []
        cnum = 1
        while f"clue{cnum}_x" in row and row.get(f"clue{cnum}_x", "") != "":
            clues.append((int(row[f"clue{cnum}_x"]), int(row[f"clue{cnum}_y"])))
            cnum += 1
        scenarios.append(TrialScenario(trial_id=trial_id, target=target, clues=clues, metadata=dict(metadata)))
    return scenarios


def load_scenarios_json(path: str | Path, max_trials: Optional[int] = None) -> List[TrialScenario]:
    path = Path(path)
    data = json.loads(path.read_text())
    scenarios: List[TrialScenario] = []
    for idx, item in enumerate(data):
        if max_trials is not None and len(scenarios) >= max_trials:
            break
        trial_id = int(item.get("trial_id", item.get("episode", idx)))
        target_raw = item.get("target", item.get("object"))
        if target_raw is None:
            raise ValueError(f"Scenario {idx} missing target/object field")
        target = (int(target_raw[0]), int(target_raw[1]))
        clues = [(int(c[0]), int(c[1])) for c in item.get("clues", [])]
        meta = dict(item.get("metadata", {}))
        meta["scenario_file"] = str(path)
        scenarios.append(TrialScenario(trial_id=trial_id, target=target, clues=clues, metadata=meta))
    return scenarios
