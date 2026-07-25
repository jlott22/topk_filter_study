from __future__ import annotations

import math
from array import array
from copy import deepcopy
from functools import cmp_to_key
from typing import Any, Dict, List, Optional, Tuple

from benchmark_sim.algorithms.DGA import DGAReferenceAllocator
from benchmark_sim.core.types import Cell


class _PackedPlan:
    """Compact complete team plan used only inside the optimized population."""

    __slots__ = ("cells", "grid_size", "lengths", "team_ids")

    def __init__(
        self,
        cells: array,
        lengths: array,
        team_ids: Tuple[str, ...],
        grid_size: int,
    ) -> None:
        self.cells = cells
        self.lengths = lengths
        self.team_ids = team_ids
        self.grid_size = int(grid_size)

    def clone(self) -> "_PackedPlan":
        return _PackedPlan(
            array("H", self.cells),
            array("H", self.lengths),
            self.team_ids,
            self.grid_size,
        )

    @property
    def payload_bytes(self) -> int:
        return (
            len(self.cells) * self.cells.itemsize
            + len(self.lengths) * self.lengths.itemsize
        )


class _PackedScore:
    __slots__ = ("fitness", "ordinal", "plan")

    def __init__(self, plan: _PackedPlan, fitness: float, ordinal: int) -> None:
        self.plan = plan
        self.fitness = float(fitness)
        self.ordinal = int(ordinal)


class DGAOptimizedAllocator(DGAReferenceAllocator):
    """Memory-oriented DGA with the same search behavior as ``DGAAllocator``.

    The reference allocator remains unchanged. This duplicate stores complete
    populations as packed unsigned cell IDs, ranks compact plans without
    constructing nested signatures, keeps only the best preparation candidates,
    and does not repair canonical plans again during ranking.

    Crossover, mutation, repair, objective calculation, received-solution
    handling, trigger behavior, and messaging remain inherited from the
    reference implementation.
    """

    name = "DGA"

    def __init__(self) -> None:
        self._repair_candidates_ref: Optional[List[Cell]] = None
        self._repair_candidate_mask = bytearray()
        self._repair_seen_mask = bytearray()

    # ------------------------------------------------------------------
    # Packed plan representation
    # ------------------------------------------------------------------

    @staticmethod
    def _pack_cell(cell: Cell, grid_size: int) -> int:
        # x-major encoding preserves the reference (x, y) tuple comparison.
        return int(cell[0]) * int(grid_size) + int(cell[1])

    @staticmethod
    def _unpack_cell(cell_id: int, grid_size: int) -> Cell:
        return int(cell_id) // int(grid_size), int(cell_id) % int(grid_size)

    def _pack_plan(
        self,
        plan: Dict[str, List[Cell]],
        team_ids: Tuple[str, ...],
        grid_size: int,
    ) -> _PackedPlan:
        cells = array("H")
        lengths = array("H")
        for rid in team_ids:
            route = plan.get(rid, []) or []
            lengths.append(len(route))
            cells.extend(
                self._pack_cell(cell, grid_size)
                for cell in route
            )
        return _PackedPlan(cells, lengths, team_ids, grid_size)

    def _unpack_plan(self, plan: _PackedPlan) -> Dict[str, List[Cell]]:
        unpacked: Dict[str, List[Cell]] = {}
        offset = 0
        for rid, length in zip(plan.team_ids, plan.lengths):
            end = offset + int(length)
            unpacked[rid] = [
                self._unpack_cell(plan.cells[index], plan.grid_size)
                for index in range(offset, end)
            ]
            offset = end
        return unpacked

    def _coerce_plan_dict(self, plan: Any) -> Any:
        if isinstance(plan, _PackedPlan):
            return self._unpack_plan(plan)
        return plan

    @staticmethod
    def _compare_packed_plans(left: _PackedPlan, right: _PackedPlan) -> int:
        if left.team_ids != right.team_ids:
            if left.team_ids < right.team_ids:
                return -1
            if left.team_ids > right.team_ids:
                return 1

        left_offset = 0
        right_offset = 0
        for left_length, right_length in zip(left.lengths, right.lengths):
            left_length = int(left_length)
            right_length = int(right_length)
            shared = min(left_length, right_length)
            for route_index in range(shared):
                left_cell = left.cells[left_offset + route_index]
                right_cell = right.cells[right_offset + route_index]
                if left_cell < right_cell:
                    return -1
                if left_cell > right_cell:
                    return 1
            if left_length < right_length:
                return -1
            if left_length > right_length:
                return 1
            left_offset += left_length
            right_offset += right_length
        return 0

    @classmethod
    def _compare_scores(cls, left: _PackedScore, right: _PackedScore) -> int:
        if left.fitness < right.fitness:
            return -1
        if left.fitness > right.fitness:
            return 1
        plan_order = cls._compare_packed_plans(left.plan, right.plan)
        if plan_order:
            return plan_order
        if left.ordinal < right.ordinal:
            return -1
        if left.ordinal > right.ordinal:
            return 1
        return 0

    # ------------------------------------------------------------------
    # Compact objective and ranking
    # ------------------------------------------------------------------

    def _fitness_packed(
        self,
        robot: Any,
        plan: _PackedPlan,
        team_agents: Dict[str, Cell],
    ) -> float:
        route_costs: List[float] = []
        offset = 0
        for rid, route_length in zip(plan.team_ids, plan.lengths):
            previous = team_agents[rid]
            route_cost = 0.0
            end = offset + int(route_length)
            for index in range(offset, end):
                cell = self._unpack_cell(plan.cells[index], plan.grid_size)
                route_cost += self._edge_cost(robot, previous, cell)
                previous = cell
            route_costs.append(
                float(route_cost) if math.isfinite(route_cost) else math.inf
            )
            offset = end

        if not route_costs:
            return math.inf
        fitness = (
            max(route_costs)
            + float(self.MIN_SUM_TIE_WEIGHT) * sum(route_costs)
        )
        return float(fitness) if math.isfinite(fitness) else math.inf

    def _score_packed_plan(
        self,
        robot: Any,
        plan: _PackedPlan,
        team_agents: Dict[str, Cell],
        ordinal: int,
    ) -> _PackedScore:
        return _PackedScore(
            plan,
            self._fitness_packed(robot, plan, team_agents),
            ordinal,
        )

    def _rank_packed_population(
        self,
        robot: Any,
        population: List[_PackedPlan],
        team_agents: Dict[str, Cell],
    ) -> List[_PackedScore]:
        scored = [
            self._score_packed_plan(robot, plan, team_agents, ordinal)
            for ordinal, plan in enumerate(population)
        ]
        scored.sort(key=cmp_to_key(self._compare_scores))
        return scored

    def _insert_prepared_score(
        self,
        ranked: List[_PackedScore],
        score: _PackedScore,
    ) -> None:
        limit = max(1, int(self.POPULATION_SIZE))
        insert_at = len(ranked)
        while (
            insert_at > 0
            and self._compare_scores(score, ranked[insert_at - 1]) < 0
        ):
            insert_at -= 1
        if insert_at >= limit:
            return
        ranked.insert(insert_at, score)
        if len(ranked) > limit:
            ranked.pop()

    # ------------------------------------------------------------------
    # Reusable repair workspace
    # ------------------------------------------------------------------

    def _prepare_repair_masks(
        self,
        robot: Any,
        candidates: List[Cell],
    ) -> None:
        grid_size = self._grid_size(robot)
        required = grid_size * grid_size
        if (
            candidates is self._repair_candidates_ref
            and len(self._repair_candidate_mask) == required
        ):
            return

        if len(self._repair_candidate_mask) != required:
            self._repair_candidate_mask = bytearray(required)
            self._repair_seen_mask = bytearray(required)
        else:
            for index in range(required):
                self._repair_candidate_mask[index] = 0
                self._repair_seen_mask[index] = 0

        for x, y in candidates:
            self._repair_candidate_mask[y * grid_size + x] = 1
        self._repair_candidates_ref = candidates

    def _repair_plan(
        self,
        robot: Any,
        plan: Any,
        team_agents: Dict[str, Cell],
        candidates: List[Cell],
    ) -> Optional[Dict[str, List[Cell]]]:
        if not isinstance(plan, dict):
            return None

        self._prepare_repair_masks(robot, candidates)
        grid_size = self._grid_size(robot)
        seen = self._repair_seen_mask
        for index in range(len(seen)):
            seen[index] = 0

        repaired: Dict[str, List[Cell]] = {
            rid: [] for rid in team_agents
        }
        for rid in sorted(
            team_agents.keys(), key=self._robot_id_key
        ):
            for raw in plan.get(rid, []) or []:
                cell = self._normalize_cell(raw)
                if cell is None:
                    continue
                x, y = cell
                if not (0 <= x < grid_size and 0 <= y < grid_size):
                    continue
                cell_index = y * grid_size + x
                if (
                    seen[cell_index]
                    or not self._repair_candidate_mask[cell_index]
                ):
                    continue
                if not self._valid_task_cell(robot, cell):
                    continue
                repaired[rid].append(cell)
                seen[cell_index] = 1

        for cell in candidates:
            cell_index = cell[1] * grid_size + cell[0]
            if seen[cell_index]:
                continue
            rid = min(
                repaired.keys(),
                key=lambda key: (
                    self._append_cost(
                        robot,
                        team_agents[key],
                        repaired[key],
                        cell,
                    ),
                    len(repaired[key]),
                    self._robot_id_key(key),
                ),
            )
            repaired[rid].append(cell)
            seen[cell_index] = 1

        return repaired

    # ------------------------------------------------------------------
    # Population lifecycle
    # ------------------------------------------------------------------

    def _prepare_packed_population(
        self,
        robot: Any,
        team_agents: Dict[str, Cell],
        candidates: List[Cell],
    ) -> List[_PackedPlan]:
        team_ids = tuple(
            sorted(team_agents.keys(), key=self._robot_id_key)
        )
        grid_size = self._grid_size(robot)
        ranked: List[_PackedScore] = []
        ordinal = 0
        valid_population_count = 0

        def consider(plan: Any, repair: bool) -> None:
            nonlocal ordinal, valid_population_count
            raw = self._coerce_plan_dict(plan)
            if repair:
                raw = super(DGAOptimizedAllocator, self)._repair_plan(
                    robot, raw, team_agents, candidates
                )
            if raw is None:
                return
            packed = self._pack_plan(raw, team_ids, grid_size)
            score = self._score_packed_plan(
                robot, packed, team_agents, ordinal
            )
            self._insert_prepared_score(ranked, score)
            ordinal += 1
            valid_population_count += 1

        current = getattr(robot, "dga_population", []) or []
        received = (getattr(robot, "dga_received_solutions", []) or []) + (
            getattr(robot, "dga_received_solution_pool", []) or []
        )
        for plan in current + received:
            consider(plan, repair=True)

        consider(
            super()._greedy_seed(robot, team_agents, candidates),
            repair=False,
        )
        preserved = super()._current_path_seed(
            robot, team_agents, candidates
        )
        if preserved is not None:
            consider(preserved, repair=False)

        while valid_population_count < max(1, int(self.POPULATION_SIZE)):
            consider(
                super()._random_balanced_seed(
                    robot, team_agents, candidates
                ),
                repair=False,
            )

        return [item.plan for item in ranked]

    def _tournament_packed(
        self,
        robot: Any,
        ranked: List[_PackedScore],
    ) -> Dict[str, List[Cell]]:
        rng = self._rng(robot)
        sample_size = max(1, min(3, len(ranked)))
        contenders = rng.sample(ranked, sample_size)
        rank_by_identity = {
            id(item): rank for rank, item in enumerate(ranked)
        }
        winner = min(
            contenders,
            key=lambda item: rank_by_identity[id(item)],
        )
        return self._unpack_plan(winner.plan)

    def _next_packed_generation(
        self,
        robot: Any,
        population: List[_PackedPlan],
        team_agents: Dict[str, Cell],
        candidates: List[Cell],
    ) -> List[_PackedPlan]:
        ranked = self._rank_packed_population(
            robot, population, team_agents
        )
        elite_count = max(
            0,
            min(
                int(self.ELITE_COUNT),
                len(ranked),
                int(self.POPULATION_SIZE),
            ),
        )
        next_population = [
            item.plan.clone() for item in ranked[:elite_count]
        ]
        rng = self._rng(robot)
        team_ids = tuple(
            sorted(team_agents.keys(), key=self._robot_id_key)
        )
        grid_size = self._grid_size(robot)

        while len(next_population) < max(1, int(self.POPULATION_SIZE)):
            parent_a = self._tournament_packed(robot, ranked)
            parent_b = self._tournament_packed(robot, ranked)
            if rng.random() < float(self.CROSSOVER_RATE):
                child = super()._crossover(
                    robot,
                    parent_a,
                    parent_b,
                    team_agents,
                    candidates,
                )
            else:
                child = parent_a

            if rng.random() < float(self.MUTATION_RATE):
                child = super()._mutate(
                    robot, child, team_agents, candidates
                )

            # Every branch is already canonical: packed parents were repaired
            # at preparation, crossover repairs, and mutation repairs.
            next_population.append(
                self._pack_plan(child, team_ids, grid_size)
            )

        return next_population

    def _run_dga(self, robot: Any, trigger: str) -> None:
        candidates = self._candidate_cells(robot)
        team_agents = self._dga_team_agents(robot)
        setattr(robot, "dga_last_candidate_count", len(candidates))
        setattr(robot, "dga_last_team_size", len(team_agents))

        if not candidates or not team_agents:
            empty_plan = {self._rid_key(robot.rid): []}
            self._commit_best_plan(
                robot, empty_plan, 0.0, trigger
            )
            setattr(robot, "dga_population", [])
            self._repair_candidates_ref = None
            return

        population = self._prepare_packed_population(
            robot, team_agents, candidates
        )
        for _ in range(
            max(0, int(self.DGA_ITERATIONS_PER_TRIGGER))
        ):
            population = self._next_packed_generation(
                robot, population, team_agents, candidates
            )
            setattr(
                robot,
                "dga_generation",
                int(getattr(robot, "dga_generation", 0)) + 1,
            )

        scored = self._rank_packed_population(
            robot, population, team_agents
        )
        best_plan = self._unpack_plan(scored[0].plan)
        best_fitness = scored[0].fitness
        self._commit_best_plan(
            robot, best_plan, best_fitness, trigger
        )
        setattr(
            robot,
            "dga_population",
            [
                item.plan.clone()
                for item in scored[: int(self.POPULATION_SIZE)]
            ],
        )
        self._repair_candidates_ref = None

    def packed_population_payload_bytes(self, robot: Any) -> int:
        """Return numeric population payload, excluding Python object headers."""
        population = getattr(robot, "dga_population", []) or []
        return sum(
            plan.payload_bytes
            for plan in population
            if isinstance(plan, _PackedPlan)
        )

    def unpack_population(
        self,
        robot: Any,
    ) -> List[Dict[str, List[Cell]]]:
        """Testing/debugging view of the compact internal population."""
        return [
            self._unpack_plan(plan)
            if isinstance(plan, _PackedPlan)
            else deepcopy(plan)
            for plan in (getattr(robot, "dga_population", []) or [])
        ]


Allocator = DGAOptimizedAllocator
