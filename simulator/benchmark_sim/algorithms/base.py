from __future__ import annotations

from dataclasses import dataclass, field
from functools import wraps
from math import isfinite
from time import perf_counter_ns
from typing import Any, Dict, List, Optional, Protocol, Sequence, Set, Tuple

from benchmark_sim.core.types import AllocationDecision, Cell, Observation
from benchmark_sim.comms.message import Message


def timed_candidate_filter(method):
    """Record the full candidate discovery, ranking, and truncation call."""

    @wraps(method)
    def wrapper(self, robot, *args, **kwargs):
        started_ns = perf_counter_ns()
        try:
            return method(self, robot, *args, **kwargs)
        finally:
            counters = getattr(robot, "counters", None)
            samples = getattr(counters, "candidate_filter_time_ns_samples", None)
            if samples is not None:
                samples.append(max(0, perf_counter_ns() - started_ns))

    return wrapper


class RobotAPI(Protocol):
    rid: str
    pos: Cell
    heading: Tuple[int, int]
    grid_size: int

    @property
    def known_clues(self) -> list[Cell]: ...

    @property
    def searched(self) -> Set[Cell]: ...

    @property
    def target_p(self) -> Dict[Cell, float]: ...

    @property
    def peer_positions(self) -> Dict[str, Cell]: ...

    def publish_algorithm_message(self, category: str, payload: Dict[str, Any]) -> None: ...


class AllocatorBase:
    """Base class for task-allocation algorithms.

    The simulator does not implement CBAA, ACBBA, DMCHBA, HIPC, PI, or
    Silent Can-Win here. Add those algorithms by subclassing this class.

    Algorithms should make all task-allocation decisions in `choose_goal` and
    may publish allocation-specific messages with `robot.publish_algorithm_message`.
    """

    name: str = "base"
    PROBABILITY_ALPHA: float = 8.0

    def initialize(self, robot: RobotAPI) -> None:
        pass

    def handle_message(self, robot: RobotAPI, message: Message) -> None:
        """Receive droppable allocation-specific messages.

        Core simulator messages such as state, clue, target, and collision_intent
        are handled by the simulator before this hook. Unknown categories are
        passed through here.
        """
        pass

    def on_observation(self, robot: RobotAPI, observation: Observation) -> None:
        """Called after the robot searches a cell and detects clue/target if present."""
        pass

    def choose_goal(self, robot: RobotAPI) -> AllocationDecision:
        """Return the next task/search cell.

        Pre-clue sweeping behavior belongs in the algorithm implementation, not
        in the simulator.
        """
        raise NotImplementedError

    def debug_state(self) -> Dict[str, Any]:
        return {}

    def _coverage_mode(self, robot: RobotAPI) -> bool:
        cfg = getattr(robot, "cfg", None)
        return getattr(cfg, "trial_mode", "clue_search") == "coverage"

    def _assigned_row_band(self, robot: RobotAPI) -> Tuple[int, int]:
        """Return this robot's deterministic, approximately even row partition."""
        grid_size = int(getattr(robot, "grid_size", 0))
        cfg = getattr(robot, "cfg", None)
        robot_ids = [str(rid) for rid in getattr(cfg, "robot_ids", [])]
        rid = str(robot.rid)
        if grid_size <= 0:
            raise ValueError("grid_size must be positive")
        if not robot_ids or rid not in robot_ids:
            return (0, grid_size - 1)

        robot_count = len(robot_ids)
        index = robot_ids.index(rid)
        rows_per_robot, extra_rows = divmod(grid_size, robot_count)
        start = index * rows_per_robot + min(index, extra_rows)
        height = rows_per_robot + (1 if index < extra_rows else 0)
        if height <= 0:
            raise ValueError("row-band assignment requires robot_count <= grid_size")
        return (start, start + height - 1)

    def _planning_horizon(self, robot: RobotAPI, default: int) -> int:
        cfg = getattr(robot, "cfg", None)
        override = getattr(cfg, "commitment_horizon", None)
        if override is None:
            return int(default)
        horizon = int(override)
        if horizon <= 0:
            raise ValueError("commitment_horizon must be positive")
        return horizon

    def _candidate_limit(self, robot: RobotAPI) -> Optional[int]:
        cfg = getattr(robot, "cfg", None)
        value = getattr(cfg, "max_candidate_cells", None)
        if value is None:
            value = getattr(self, "MAX_CANDIDATE_CELLS", None)
        if value is None:
            return None
        if isinstance(value, str) and value.lower() == "all":
            return None
        limit = int(value)
        if limit <= 0:
            raise ValueError("max_candidate_cells must be positive or 'all'")
        return limit

    def _filter_candidate_cells(self, robot: RobotAPI, candidates: Sequence[Cell]) -> List[Cell]:
        ordered = list(candidates)
        limit = self._candidate_limit(robot)
        setattr(robot, "candidate_count_before_filter", len(ordered))
        setattr(robot, "candidate_count_after_filter", len(ordered) if limit is None else min(len(ordered), limit))
        setattr(robot, "max_candidate_cells", limit)
        if limit is None or limit >= len(ordered):
            return ordered

        origin = self._normalize_filter_cell(getattr(robot, "pos", None)) or (0, 0)

        def ranking(cell: Cell) -> Tuple[float, int, Cell]:
            probability = self._filter_probability(robot, cell)
            distance = self._manhattan_distance(cell, origin)
            return (-probability, distance, cell)

        filtered = sorted(ordered, key=ranking)[:limit]
        setattr(robot, "candidate_count_after_filter", len(filtered))
        return filtered

    def _filter_probability(self, robot: RobotAPI, cell: Cell) -> float:
        target_p = getattr(robot, "target_p", {}) or {}
        try:
            value = target_p.get(cell, 0.0)
        except AttributeError:
            try:
                value = target_p[cell[1]][cell[0]]
            except Exception:
                value = 0.0
        try:
            probability = float(value)
        except Exception:
            return 0.0
        if not isfinite(probability):
            return 0.0
        return max(0.0, probability)

    def _refresh_allocation_probability_normalizer(self, robot: RobotAPI) -> float:
        """Cache max(target_p) for the shared normalized probability objective."""

        target_p = getattr(robot, "target_p", {}) or {}
        values: List[Any]
        if isinstance(target_p, dict):
            values = list(target_p.values())
        else:
            values = []
            try:
                for row in target_p:
                    values.extend(row)
            except Exception:
                values = []

        maximum = 0.0
        for value in values:
            try:
                probability = float(value)
            except Exception:
                continue
            if isfinite(probability) and probability > maximum:
                maximum = probability

        if maximum <= 0.0 or not isfinite(maximum):
            maximum = 1.0

        belief = getattr(robot, "belief", None)
        belief_revision = getattr(belief, "revision", None)
        setattr(robot, "_allocation_probability_source_id", id(target_p))
        setattr(
            robot,
            "_allocation_probability_belief_id",
            id(belief) if belief_revision is not None else None,
        )
        setattr(
            robot,
            "_allocation_probability_belief_revision",
            belief_revision,
        )
        setattr(robot, "_allocation_probability_normalizer", float(maximum))
        return float(maximum)

    def _normalized_allocation_probability(self, robot: RobotAPI, cell: Cell) -> float:
        """Return target_p[cell] / max(target_p), clamped to [0, 1]."""

        target_p = getattr(robot, "target_p", {}) or {}
        source_id = getattr(robot, "_allocation_probability_source_id", None)
        normalizer = getattr(robot, "_allocation_probability_normalizer", None)
        belief = getattr(robot, "belief", None)
        belief_revision = getattr(belief, "revision", None)
        if belief_revision is None:
            cache_is_current = source_id == id(target_p)
        else:
            cache_is_current = (
                getattr(
                    robot,
                    "_allocation_probability_belief_id",
                    None,
                )
                == id(belief)
                and getattr(
                    robot,
                    "_allocation_probability_belief_revision",
                    None,
                )
                == belief_revision
            )
        if not cache_is_current or normalizer is None:
            normalizer = self._refresh_allocation_probability_normalizer(robot)

        try:
            normalizer = float(normalizer)
        except Exception:
            normalizer = 1.0
        if normalizer <= 0.0 or not isfinite(normalizer):
            normalizer = self._refresh_allocation_probability_normalizer(robot)

        probability = self._filter_probability(robot, cell)
        return float(max(0.0, min(1.0, probability / normalizer)))

    def _probability_penalty(self, robot: RobotAPI, cell: Cell) -> float:
        """Return alpha * (1 - normalized probability) with shared alpha=8."""

        try:
            alpha = float(getattr(self, "PROBABILITY_ALPHA", AllocatorBase.PROBABILITY_ALPHA))
        except Exception:
            alpha = AllocatorBase.PROBABILITY_ALPHA
        if alpha < 0.0 or not isfinite(alpha):
            alpha = AllocatorBase.PROBABILITY_ALPHA
        probability = self._normalized_allocation_probability(robot, cell)
        return float(alpha * (1.0 - probability))

    def _probability_adjusted_cost(self, robot: RobotAPI, distance: float, cell: Cell) -> float:
        """Return distance + 8 * (1 - normalized target probability)."""

        try:
            base_distance = float(distance)
        except Exception:
            base_distance = 0.0
        if base_distance < 0.0 or not isfinite(base_distance):
            base_distance = 0.0
        return float(base_distance + self._probability_penalty(robot, cell))

    def _probability_adjusted_score(self, robot: RobotAPI, distance: float, cell: Cell) -> float:
        """Return the higher-is-better negative of the shared adjusted cost."""

        return -self._probability_adjusted_cost(robot, distance, cell)

    def _normalize_filter_cell(self, cell: Any) -> Optional[Cell]:
        try:
            if cell is None or len(cell) != 2:
                return None
            return (int(cell[0]), int(cell[1]))
        except Exception:
            return None

    @staticmethod
    def _manhattan_distance(a: Cell, b: Cell) -> int:
        return abs(int(a[0]) - int(b[0])) + abs(int(a[1]) - int(b[1]))
