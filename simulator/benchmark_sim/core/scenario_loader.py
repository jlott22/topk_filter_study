from __future__ import annotations

import csv
import json
import re
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from .types import Cell, TrialScenario, in_bounds


_CLUE_FIELD = re.compile(r"^clue(\d+)_(x|y)$")


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
    elif "=" in line:
        key, value = line.split("=", 1)
        meta[key.strip()] = _coerce(value.strip())
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
    if max_trials is not None and max_trials <= 0:
        raise ValueError("max_trials must be positive")
    if path.suffix.lower() == ".json":
        return load_scenarios_json(path, max_trials=max_trials)
    return load_scenarios_csv(path, max_trials=max_trials)


def load_scenarios_csv(path: str | Path, max_trials: Optional[int] = None) -> List[TrialScenario]:
    path = Path(path)
    metadata: Dict[str, Any] = {"scenario_file": str(path)}
    rows: List[str] = []
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        for line in f:
            if line.strip().startswith("#"):
                metadata.update(_parse_metadata_line(line))
            elif line.strip():
                rows.append(line)
    if not rows:
        raise ValueError(f"Scenario file {path} has no CSV header or data rows")

    reader = csv.DictReader(rows)
    fieldnames = [name.strip() for name in (reader.fieldnames or []) if name is not None]
    if not fieldnames:
        raise ValueError(f"Scenario file {path} has no CSV header")

    target_pair = _coordinate_pair(fieldnames, ("object_x", "object_y"), ("target_x", "target_y"))
    if target_pair is None:
        raise ValueError(
            f"Scenario file {path} must contain object_x/object_y or target_x/target_y"
        )

    clue_indices: Dict[int, set[str]] = {}
    for fieldname in fieldnames:
        match = _CLUE_FIELD.fullmatch(fieldname)
        if match:
            clue_indices.setdefault(int(match.group(1)), set()).add(match.group(2))
    for clue_index, axes in sorted(clue_indices.items()):
        if axes != {"x", "y"}:
            raise ValueError(f"Scenario file {path} has an incomplete clue{clue_index} coordinate pair")

    scenarios: List[TrialScenario] = []
    for idx, row in enumerate(reader):
        row_number = idx + 2
        if None in row:
            raise ValueError(f"Scenario file {path} row {row_number} has unexpected extra columns")

        trial_raw = _first_nonblank(row, "episode", "trial_id")
        trial_id = idx if trial_raw is None else _required_int(
            trial_raw,
            f"Scenario file {path} row {row_number} trial ID",
        )
        target = _cell_from_values(
            row.get(target_pair[0]),
            row.get(target_pair[1]),
            f"Scenario file {path} row {row_number} target",
        )

        clues: List[Cell] = []
        saw_blank_clue = False
        for clue_index in sorted(clue_indices):
            x_raw = row.get(f"clue{clue_index}_x")
            y_raw = row.get(f"clue{clue_index}_y")
            x_blank = _is_blank(x_raw)
            y_blank = _is_blank(y_raw)
            if x_blank and y_blank:
                saw_blank_clue = True
                continue
            if x_blank != y_blank:
                raise ValueError(
                    f"Scenario file {path} row {row_number} has an incomplete clue{clue_index}"
                )
            if saw_blank_clue:
                raise ValueError(
                    f"Scenario file {path} row {row_number} has non-contiguous clue coordinates"
                )
            clues.append(
                _cell_from_values(
                    x_raw,
                    y_raw,
                    f"Scenario file {path} row {row_number} clue{clue_index}",
                )
            )

        scenario = TrialScenario(
            trial_id=trial_id,
            target=target,
            clues=clues,
            metadata=dict(metadata),
        )
        _validate_intrinsic_scenario(scenario, f"Scenario file {path} row {row_number}")
        scenarios.append(scenario)
    return _finalize_loaded_scenarios(scenarios, metadata, path, max_trials)


def load_scenarios_json(path: str | Path, max_trials: Optional[int] = None) -> List[TrialScenario]:
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    file_metadata: Dict[str, Any] = {"scenario_file": str(path)}
    if isinstance(data, dict):
        raw_metadata = data.get("metadata", {})
        if raw_metadata is not None and not isinstance(raw_metadata, dict):
            raise ValueError(f"Scenario file {path} metadata must be an object")
        file_metadata.update(raw_metadata or {})
        data = data.get("scenarios")
    if not isinstance(data, list):
        raise ValueError(f"Scenario file {path} must contain a JSON list of scenarios")

    scenarios: List[TrialScenario] = []
    for idx, item in enumerate(data):
        if not isinstance(item, dict):
            raise ValueError(f"Scenario file {path} item {idx} must be an object")
        trial_raw = item.get("trial_id", item.get("episode", idx))
        trial_id = _required_int(trial_raw, f"Scenario file {path} item {idx} trial ID")
        target_raw = item.get("target", item.get("object"))
        if target_raw is None:
            raise ValueError(f"Scenario {idx} missing target/object field")
        target = _cell_from_sequence(target_raw, f"Scenario file {path} item {idx} target")

        raw_clues = item.get("clues", [])
        if not isinstance(raw_clues, list):
            raise ValueError(f"Scenario file {path} item {idx} clues must be a list")
        clues = [
            _cell_from_sequence(clue, f"Scenario file {path} item {idx} clue {clue_index}")
            for clue_index, clue in enumerate(raw_clues, start=1)
        ]

        item_metadata = item.get("metadata", {})
        if item_metadata is not None and not isinstance(item_metadata, dict):
            raise ValueError(f"Scenario file {path} item {idx} metadata must be an object")
        meta = dict(file_metadata)
        meta.update(item_metadata or {})
        scenario = TrialScenario(trial_id=trial_id, target=target, clues=clues, metadata=meta)
        _validate_intrinsic_scenario(scenario, f"Scenario file {path} item {idx}")
        scenarios.append(scenario)
    return _finalize_loaded_scenarios(scenarios, file_metadata, path, max_trials)


def validate_scenario(
    scenario: TrialScenario,
    *,
    grid_size: int,
    start_positions: Mapping[str, Cell],
    trial_mode: str = "clue_search",
) -> None:
    """Validate one scenario against the configured grid and robot starts."""

    if grid_size <= 0:
        raise ValueError("grid_size must be positive")
    if trial_mode not in {"clue_search", "coverage"}:
        raise ValueError(f"unsupported trial mode: {trial_mode}")

    label = f"Scenario trial {scenario.trial_id}"
    _require_runtime_integer(scenario.trial_id, f"{label} trial ID")
    if scenario.target is not None:
        _require_runtime_cell(scenario.target, f"{label} target")
    for clue_index, clue in enumerate(scenario.clues, start=1):
        _require_runtime_cell(clue, f"{label} clue {clue_index}")
    _validate_intrinsic_scenario(scenario, label)
    starts = _validated_start_cells(start_positions, grid_size)

    if trial_mode == "clue_search" and scenario.target is None:
        raise ValueError(f"{label} is missing a target for clue_search mode")
    if scenario.target is not None:
        if not in_bounds(scenario.target, grid_size):
            raise ValueError(f"{label} target {scenario.target} is outside the {grid_size}x{grid_size} grid")
        if trial_mode == "clue_search" and scenario.target in starts:
            raise ValueError(f"{label} target {scenario.target} overlaps a robot start")

    for clue in scenario.clues:
        if not in_bounds(clue, grid_size):
            raise ValueError(f"{label} clue {clue} is outside the {grid_size}x{grid_size} grid")

    declared_grid_size = _optional_metadata_int(scenario.metadata, "grid_size")
    if declared_grid_size is not None and declared_grid_size != grid_size:
        raise ValueError(
            f"{label} declares grid_size={declared_grid_size}, but the run uses {grid_size}"
        )
    declared_robot_count = _optional_metadata_int(scenario.metadata, "num_robots")
    if declared_robot_count is not None and declared_robot_count != len(start_positions):
        raise ValueError(
            f"{label} declares num_robots={declared_robot_count}, "
            f"but the run uses {len(start_positions)}"
        )


def validate_scenarios(
    scenarios: Sequence[TrialScenario],
    *,
    grid_size: int,
    start_positions: Mapping[str, Cell],
    trial_mode: str = "clue_search",
    expected_count: Optional[int] = None,
) -> None:
    """Validate a run's selected scenarios, including IDs and requested count."""

    if not scenarios:
        raise ValueError("scenario input contains no trials")
    if expected_count is not None and len(scenarios) != expected_count:
        raise ValueError(f"expected {expected_count} scenarios, found {len(scenarios)}")

    seen_ids: set[int] = set()
    for scenario in scenarios:
        if scenario.trial_id in seen_ids:
            raise ValueError(f"duplicate scenario trial ID: {scenario.trial_id}")
        seen_ids.add(scenario.trial_id)
        validate_scenario(
            scenario,
            grid_size=grid_size,
            start_positions=start_positions,
            trial_mode=trial_mode,
        )


def _coordinate_pair(fieldnames: Sequence[str], *pairs: tuple[str, str]) -> Optional[tuple[str, str]]:
    fields = set(fieldnames)
    for pair in pairs:
        if pair[0] in fields and pair[1] in fields:
            return pair
    return None


def _first_nonblank(row: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        value = row.get(key)
        if not _is_blank(value):
            return value
    return None


def _is_blank(value: Any) -> bool:
    return value is None or str(value).strip() == ""


def _required_int(value: Any, label: str) -> int:
    if _is_blank(value):
        raise ValueError(f"{label} is missing")
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer, got {value!r}")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        text = value.strip()
        if re.fullmatch(r"[+-]?\d+", text):
            return int(text)
    raise ValueError(f"{label} must be an integer, got {value!r}")


def _cell_from_values(x_raw: Any, y_raw: Any, label: str) -> Cell:
    return _required_int(x_raw, f"{label} x"), _required_int(y_raw, f"{label} y")


def _cell_from_sequence(raw: Any, label: str) -> Cell:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ValueError(f"{label} must contain exactly two coordinates")
    return _cell_from_values(raw[0], raw[1], label)


def _require_runtime_integer(value: Any, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer, got {value!r}")


def _require_runtime_cell(raw: Any, label: str) -> None:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ValueError(f"{label} must contain exactly two integer coordinates")
    _require_runtime_integer(raw[0], f"{label} x")
    _require_runtime_integer(raw[1], f"{label} y")


def _validate_intrinsic_scenario(scenario: TrialScenario, label: str) -> None:
    occupied: set[Cell] = set()
    if scenario.target is not None:
        occupied.add(scenario.target)
    for clue in scenario.clues:
        if clue in occupied:
            raise ValueError(f"{label} has duplicate target/clue coordinate {clue}")
        occupied.add(clue)


def _finalize_loaded_scenarios(
    scenarios: List[TrialScenario],
    metadata: Mapping[str, Any],
    path: Path,
    max_trials: Optional[int],
) -> List[TrialScenario]:
    if not scenarios:
        raise ValueError(f"Scenario file {path} contains no scenario rows")

    seen_ids: set[int] = set()
    for scenario in scenarios:
        if scenario.trial_id in seen_ids:
            raise ValueError(f"Scenario file {path} has duplicate trial ID {scenario.trial_id}")
        seen_ids.add(scenario.trial_id)

    declared_count = _optional_metadata_int(metadata, "num_trials")
    if declared_count is not None and declared_count != len(scenarios):
        raise ValueError(
            f"Scenario file {path} declares num_trials={declared_count}, "
            f"but contains {len(scenarios)} rows"
        )

    declared_clues = _optional_metadata_int(metadata, "num_clues")
    if declared_clues is None:
        declared_clues = _optional_metadata_int(metadata, "clues_per_object")
    if declared_clues is not None:
        for scenario in scenarios:
            if len(scenario.clues) != declared_clues:
                raise ValueError(
                    f"Scenario file {path} trial {scenario.trial_id} declares "
                    f"{declared_clues} clues, but contains {len(scenario.clues)}"
                )

    selected = scenarios if max_trials is None else scenarios[:max_trials]
    if max_trials is not None and len(selected) != max_trials:
        raise ValueError(
            f"Scenario file {path} contains {len(scenarios)} rows, "
            f"fewer than requested max_trials={max_trials}"
        )
    return selected


def _optional_metadata_int(metadata: Mapping[str, Any], key: str) -> Optional[int]:
    value = metadata.get(key)
    if value in (None, ""):
        return None
    return _required_int(value, f"scenario metadata {key}")


def _validated_start_cells(start_positions: Mapping[str, Cell], grid_size: int) -> set[Cell]:
    starts: set[Cell] = set()
    for rid, raw_cell in start_positions.items():
        cell = _cell_from_sequence(raw_cell, f"robot {rid} start")
        if not in_bounds(cell, grid_size):
            raise ValueError(f"robot {rid} start {cell} is outside the {grid_size}x{grid_size} grid")
        if cell in starts:
            raise ValueError(f"duplicate robot start coordinate: {cell}")
        starts.add(cell)
    return starts
