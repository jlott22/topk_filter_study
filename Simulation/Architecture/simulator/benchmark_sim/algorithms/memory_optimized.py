from __future__ import annotations

from array import array
from collections.abc import Iterator, MutableMapping
from typing import Any, Dict, Iterable, List, Optional, Tuple

from benchmark_sim.algorithms.base import timed_candidate_filter


Cell = Tuple[int, int]


class CellIndexedMap(MutableMapping):
    """Dense fixed-grid mapping that preserves the cell-keyed mapping API."""

    __slots__ = (
        "grid_size",
        "_active",
        "_count",
        "_numeric",
        "_values",
    )

    def __init__(
        self,
        grid_size: int,
        *,
        numeric: bool = False,
        initial: Optional[Iterable[Tuple[Cell, Any]]] = None,
    ) -> None:
        self.grid_size = int(grid_size)
        cell_count = self.grid_size * self.grid_size
        self._active = bytearray(cell_count)
        self._count = 0
        self._numeric = bool(numeric)
        if self._numeric:
            self._values = array("d", [0.0]) * cell_count
        else:
            self._values = [None] * cell_count
        if initial is not None:
            self.update(initial)

    def _cell_id(self, cell: Cell) -> int:
        try:
            x = int(cell[0])
            y = int(cell[1])
        except (TypeError, ValueError, IndexError) as exc:
            raise KeyError(cell) from exc
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            raise KeyError(cell)
        return y * self.grid_size + x

    def _cell(self, cell_id: int) -> Cell:
        return cell_id % self.grid_size, cell_id // self.grid_size

    def __getitem__(self, cell: Cell) -> Any:
        cell_id = self._cell_id(cell)
        if not self._active[cell_id]:
            raise KeyError(cell)
        return self._values[cell_id]

    def __setitem__(self, cell: Cell, value: Any) -> None:
        cell_id = self._cell_id(cell)
        if not self._active[cell_id]:
            self._active[cell_id] = 1
            self._count += 1
        self._values[cell_id] = float(value) if self._numeric else value

    def __delitem__(self, cell: Cell) -> None:
        cell_id = self._cell_id(cell)
        if not self._active[cell_id]:
            raise KeyError(cell)
        self._active[cell_id] = 0
        self._count -= 1
        if self._numeric:
            self._values[cell_id] = 0.0
        else:
            self._values[cell_id] = None

    def __iter__(self) -> Iterator[Cell]:
        for cell_id, active in enumerate(self._active):
            if active:
                yield self._cell(cell_id)

    def __len__(self) -> int:
        return self._count

    def clear(self) -> None:
        if not self._count:
            return
        for cell_id, active in enumerate(self._active):
            if not active:
                continue
            self._active[cell_id] = 0
            if self._numeric:
                self._values[cell_id] = 0.0
            else:
                self._values[cell_id] = None
        self._count = 0

    def __repr__(self) -> str:
        return repr(dict(self.items()))

    def payload_bytes(self) -> int:
        value_bytes = (
            len(self._values) * self._values.itemsize
            if self._numeric
            else len(self._values) * 8
        )
        return len(self._active) + value_bytes


class DenseCellStateMixin:
    """Convert selected cell-keyed dictionaries to dense mapping-compatible tables."""

    _CELL_MAP_SPECS: Tuple[Tuple[str, bool], ...] = ()

    def _optimize_cell_maps(self, robot: Any) -> None:
        grid_size = int(self._grid_size(robot))
        for attribute, numeric in self._CELL_MAP_SPECS:
            current = getattr(robot, attribute, None)
            if isinstance(current, CellIndexedMap):
                if current.grid_size == grid_size and current._numeric == numeric:
                    continue
                current_items = list(current.items())
            elif current is None:
                current_items = []
            else:
                current_items = list(current.items())
            setattr(
                robot,
                attribute,
                CellIndexedMap(
                    grid_size,
                    numeric=numeric,
                    initial=current_items,
                ),
            )

    def optimized_state_payload_bytes(self, robot: Any) -> int:
        total = 0
        for attribute, _numeric in self._CELL_MAP_SPECS:
            mapping = getattr(robot, attribute, None)
            if isinstance(mapping, CellIndexedMap):
                total += mapping.payload_bytes()
        return total


class PackedCandidateMixin:
    """Behavior-compatible candidate filtering with reusable packed workspaces."""

    def _ensure_candidate_workspace(self, cell_count: int) -> None:
        if len(getattr(self, "_candidate_scan_ids", ())) != cell_count:
            self._candidate_scan_ids = array("H", [0]) * cell_count
            self._candidate_ranked_ids = array("H", [0]) * cell_count
            self._candidate_probabilities = array("d", [0.0]) * cell_count
            self._candidate_distances = array("H", [0]) * cell_count

    def _candidate_precedes(self, left_id: int, right_id: int, grid_size: int) -> bool:
        probabilities = self._candidate_probabilities
        left_probability = probabilities[left_id]
        right_probability = probabilities[right_id]
        if left_probability != right_probability:
            return left_probability > right_probability

        distances = self._candidate_distances
        left_distance = distances[left_id]
        right_distance = distances[right_id]
        if left_distance != right_distance:
            return left_distance < right_distance

        left_x = left_id % grid_size
        right_x = right_id % grid_size
        if left_x != right_x:
            return left_x < right_x
        return left_id // grid_size < right_id // grid_size

    def _rank_packed_candidates(
        self,
        scan_count: int,
        keep_count: int,
        grid_size: int,
    ) -> None:
        ranked = self._candidate_ranked_ids
        scan = self._candidate_scan_ids
        ranked_count = 0
        for scan_index in range(scan_count):
            cell_id = scan[scan_index]
            if ranked_count < keep_count:
                insertion = ranked_count
                while insertion > 0 and self._candidate_precedes(
                    cell_id,
                    ranked[insertion - 1],
                    grid_size,
                ):
                    ranked[insertion] = ranked[insertion - 1]
                    insertion -= 1
                ranked[insertion] = cell_id
                ranked_count += 1
                continue

            if not self._candidate_precedes(
                cell_id,
                ranked[keep_count - 1],
                grid_size,
            ):
                continue
            insertion = keep_count - 1
            while insertion > 0 and self._candidate_precedes(
                cell_id,
                ranked[insertion - 1],
                grid_size,
            ):
                ranked[insertion] = ranked[insertion - 1]
                insertion -= 1
            ranked[insertion] = cell_id

    @timed_candidate_filter
    def _packed_candidate_cells(
        self,
        robot: Any,
        *,
        always_rank: bool = False,
    ) -> list[Cell]:
        grid_size = int(self._grid_size(robot))
        cell_count = grid_size * grid_size
        self._ensure_candidate_workspace(cell_count)
        origin = self._normalize_filter_cell(getattr(robot, "pos", None)) or (0, 0)

        scan_count = 0
        for y in range(grid_size):
            for x in range(grid_size):
                cell = (x, y)
                if not self._valid_task_cell(robot, cell):
                    continue
                cell_id = y * grid_size + x
                self._candidate_scan_ids[scan_count] = cell_id
                self._candidate_probabilities[cell_id] = self._filter_probability(
                    robot,
                    cell,
                )
                self._candidate_distances[cell_id] = self._manhattan_distance(
                    cell,
                    origin,
                )
                scan_count += 1

        limit = self._candidate_limit(robot)
        after_count = scan_count if limit is None else min(scan_count, limit)
        setattr(robot, "candidate_count_before_filter", scan_count)
        setattr(robot, "candidate_count_after_filter", after_count)
        setattr(robot, "max_candidate_cells", limit)

        must_rank = always_rank or (limit is not None and limit < scan_count)
        if must_rank and after_count:
            self._rank_packed_candidates(
                scan_count,
                after_count,
                grid_size,
            )
            source = self._candidate_ranked_ids
        else:
            source = self._candidate_scan_ids

        return [
            (source[index] % grid_size, source[index] // grid_size)
            for index in range(after_count)
        ]

    def candidate_workspace_payload_bytes(self) -> int:
        total = 0
        for attribute in (
            "_candidate_scan_ids",
            "_candidate_ranked_ids",
            "_candidate_probabilities",
            "_candidate_distances",
        ):
            values = getattr(self, attribute, None)
            if values is not None:
                total += len(values) * values.itemsize
        return total


class ACBBAOptimizationMixin(DenseCellStateMixin, PackedCandidateMixin):
    _CELL_MAP_SPECS = (
        ("acbba_winner_by_cell", False),
        ("acbba_winning_bid_by_cell", True),
        ("acbba_bid_time_by_cell", True),
        ("acbba_pending_deltas", False),
        ("acbba_last_sent_signatures", False),
    )

    def _ensure_acbba_state(self, robot: Any) -> None:
        super()._ensure_acbba_state(robot)
        self._optimize_cell_maps(robot)

    def _reset_acbba_state(self, robot: Any, preserve_deltas: bool = False) -> None:
        super()._reset_acbba_state(robot, preserve_deltas=preserve_deltas)
        self._optimize_cell_maps(robot)

    def _candidate_cells(self, robot: Any) -> List[Cell]:
        return self._packed_candidate_cells(robot)

    def _best_insertion_bid(
        self,
        robot: Any,
        path: List[Cell],
        cell: Cell,
    ) -> Tuple[int, float]:
        current_distance = self._route_distance(robot, path)
        best_index = 0
        best_bid = self.NO_BID
        path_length = len(path)

        for insertion_index in range(path_length + 1):
            distance = 0
            previous = self._robot_pos(robot)
            path_index = 0
            for output_index in range(path_length + 1):
                if output_index == insertion_index:
                    next_cell = cell
                else:
                    next_cell = path[path_index]
                    path_index += 1
                distance += self.manhattan(
                    previous[0],
                    previous[1],
                    next_cell[0],
                    next_cell[1],
                )
                previous = next_cell

            marginal_distance = max(0.0, float(distance) - current_distance)
            bid = self._probability_adjusted_score(
                robot,
                marginal_distance,
                cell,
            )
            if bid > best_bid + self.EPS:
                best_index = insertion_index
                best_bid = bid
            elif abs(bid - best_bid) <= self.EPS and insertion_index < best_index:
                best_index = insertion_index
                best_bid = bid

        return best_index, best_bid


class CBAAOptimizationMixin(DenseCellStateMixin, PackedCandidateMixin):
    _CELL_MAP_SPECS = (
        ("cbaa_winner_by_cell", False),
        ("cbaa_winning_bid_by_cell", True),
        ("cbaa_pending_deltas", False),
        ("cbaa_last_sent_signatures", False),
    )

    def _ensure_cbaa_state(self, robot: Any) -> None:
        super()._ensure_cbaa_state(robot)
        self._optimize_cell_maps(robot)

    def _reset_cbaa_state(self, robot: Any) -> None:
        super()._reset_cbaa_state(robot)
        self._optimize_cell_maps(robot)

    def _candidate_cells(self, robot: Any) -> List[Cell]:
        return self._packed_candidate_cells(robot)


class HIPCOptimizationMixin(DenseCellStateMixin, PackedCandidateMixin):
    _CELL_MAP_SPECS = (
        ("hipc_winner_by_cell", False),
        ("hipc_winning_bid_by_cell", True),
        ("hipc_bid_time_by_cell", True),
    )

    def _ensure_hipc_state(self, robot: Any) -> None:
        super()._ensure_hipc_state(robot)
        self._optimize_cell_maps(robot)

    def _reset_path_state(self, robot: Any) -> None:
        super()._reset_path_state(robot)
        self._optimize_cell_maps(robot)

    def _candidate_cells(self, robot: Any) -> List[Cell]:
        return self._packed_candidate_cells(robot, always_rank=True)

    def _run_local_team_taa(
        self,
        robot: Any,
        team_agents: Dict[str, Cell],
        candidates: List[Cell],
    ) -> Dict[str, List[Cell]]:
        winner_by_cell, winning_bid_by_cell = self._consensus_maps(robot)
        team_order = list(team_agents.keys())
        sorted_team_ids = sorted(team_agents.keys())
        team_size = len(team_order)
        bundle_size = self._planning_horizon(robot, self.BUNDLE_SIZE)
        max_assignments = max(1, team_size * bundle_size)

        plan_ids = array("H", [0]) * max(1, team_size * bundle_size)
        plan_counts = bytearray(team_size)
        endpoints = [team_agents[rid] for rid in team_order]
        assigned = bytearray(self._grid_size(robot) ** 2)
        grid_size = self._grid_size(robot)

        for _ in range(max_assignments):
            best_rid: Optional[str] = None
            best_cell: Optional[Cell] = None
            best_score = self.NO_BID

            for rid in sorted_team_ids:
                row = team_order.index(rid)
                if plan_counts[row] >= bundle_size:
                    continue
                reference = endpoints[row]

                for cell in candidates:
                    cell_id = cell[1] * grid_size + cell[0]
                    if assigned[cell_id]:
                        continue
                    known_winner = winner_by_cell.get(cell, self.NO_WINNER)
                    if (
                        known_winner is not self.NO_WINNER
                        and self._rid_key(known_winner) not in team_order
                    ):
                        continue

                    score = self._bid_from_reference(robot, cell, reference)
                    known_bid = float(winning_bid_by_cell.get(cell, self.NO_BID))
                    if known_winner is not self.NO_WINNER:
                        known_key = self._rid_key(known_winner)
                        if known_key != rid and score < known_bid - self.EPS:
                            continue

                    if self._better_team_choice(
                        rid,
                        cell,
                        score,
                        best_rid,
                        best_cell,
                        best_score,
                    ):
                        best_rid = rid
                        best_cell = cell
                        best_score = score

            if best_rid is None or best_cell is None:
                break

            row = team_order.index(best_rid)
            offset = row * bundle_size + plan_counts[row]
            plan_ids[offset] = best_cell[1] * grid_size + best_cell[0]
            plan_counts[row] += 1
            endpoints[row] = best_cell
            assigned[best_cell[1] * grid_size + best_cell[0]] = 1

        result: Dict[str, List[Cell]] = {}
        for row, rid in enumerate(team_order):
            route: List[Cell] = []
            offset = row * bundle_size
            for route_index in range(plan_counts[row]):
                cell_id = plan_ids[offset + route_index]
                route.append((cell_id % grid_size, cell_id // grid_size))
            result[rid] = route
        return result


class PIOptimizationMixin(DenseCellStateMixin, PackedCandidateMixin):
    _CELL_MAP_SPECS = (
        ("pi_owner_by_cell", False),
        ("pi_significance_by_cell", True),
        ("pi_time_by_cell", True),
    )

    def _ensure_pi_state(self, robot: Any) -> None:
        super()._ensure_pi_state(robot)
        self._optimize_cell_maps(robot)

    def _reset_pi_state(self, robot: Any) -> None:
        super()._reset_pi_state(robot)
        self._optimize_cell_maps(robot)

    def _candidate_cells(self, robot: Any) -> List[Cell]:
        cached = getattr(self, "_active_candidate_cache", None)
        if cached is not None:
            return cached
        return self._packed_candidate_cells(robot)

    def _build_bundle(self, robot: Any) -> None:
        self._active_candidate_cache = self._packed_candidate_cells(robot)
        try:
            super()._build_bundle(robot)
        finally:
            self._active_candidate_cache = None

    def _route_cost_with_insertion(
        self,
        robot: Any,
        path: List[Cell],
        cell: Cell,
        insertion_index: int,
    ) -> float:
        current = self._robot_pos(robot)
        total = 0.0
        service_cost = self._finite_nonnegative(
            getattr(self, "TASK_SERVICE_COST", 0.0),
            0.0,
        )
        path_index = 0
        for output_index in range(len(path) + 1):
            if output_index == insertion_index:
                next_cell = cell
            else:
                next_cell = path[path_index]
                path_index += 1
            total += self._effective_move_cost(robot, current, next_cell)
            total += service_cost
            current = next_cell
        return self._finite_nonnegative(total, self.INF_SIGNIFICANCE)

    def _route_cost_without_index(
        self,
        robot: Any,
        path: List[Cell],
        skipped_index: int,
    ) -> float:
        current = self._robot_pos(robot)
        total = 0.0
        service_cost = self._finite_nonnegative(
            getattr(self, "TASK_SERVICE_COST", 0.0),
            0.0,
        )
        for index, next_cell in enumerate(path):
            if index == skipped_index:
                continue
            total += self._effective_move_cost(robot, current, next_cell)
            total += service_cost
            current = next_cell
        return self._finite_nonnegative(total, self.INF_SIGNIFICANCE)

    def _best_insertion(
        self,
        robot: Any,
        path: List[Cell],
        cell: Cell,
    ) -> Tuple[Optional[int], float]:
        if cell in path:
            return None, self.INF_SIGNIFICANCE

        base_cost = self._route_cost(robot, path)
        best_index: Optional[int] = None
        best_delta = self.INF_SIGNIFICANCE
        for index in range(len(path) + 1):
            delta = self._finite_nonnegative(
                self._route_cost_with_insertion(robot, path, cell, index)
                - base_cost,
                self.INF_SIGNIFICANCE,
            )
            if delta < best_delta - self.EPS:
                best_delta = delta
                best_index = index
            elif abs(delta - best_delta) <= self.EPS and best_index is not None:
                if index < best_index:
                    best_index = index
        return best_index, best_delta

    def _refresh_local_path_entries(self, robot: Any) -> None:
        self._ensure_pi_state(robot)
        path = self._get_path(robot)
        owner_by_cell, significance_by_cell = self._consensus_maps(robot)
        time_by_cell = self._time_map(robot)

        full_cost = self._route_cost(robot, path)
        for index, cell in enumerate(path):
            without_cost = self._route_cost_without_index(robot, path, index)
            significance = self._finite_nonnegative(
                full_cost - without_cost,
                0.0,
            )
            old_owner = owner_by_cell.get(cell, self.NO_OWNER)
            old_significance = float(
                significance_by_cell.get(cell, self.INF_SIGNIFICANCE)
            )
            owner_by_cell[cell] = robot.rid
            significance_by_cell[cell] = significance
            if (
                not self._same_robot_id(old_owner, robot.rid)
                or abs(old_significance - significance) > self.EPS
            ):
                time_by_cell[cell] = self._next_time(robot)
            elif (
                cell not in time_by_cell
                or time_by_cell.get(cell, self.NO_TIME) == self.NO_TIME
            ):
                time_by_cell[cell] = self._next_time(robot)
