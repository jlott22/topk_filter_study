from __future__ import annotations

import random
import unittest

from benchmark_sim.algorithms.DGA import (
    DGAAllocator,
    DGAReferenceAllocator,
)
from benchmark_sim.algorithms.DGA_optimized import DGAOptimizedAllocator
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


class _SmallReferenceDGA(DGAReferenceAllocator):
    POPULATION_SIZE = 10
    DGA_ITERATIONS_PER_TRIGGER = 4


class _SmallOptimizedDGA(DGAOptimizedAllocator):
    POPULATION_SIZE = 10
    DGA_ITERATIONS_PER_TRIGGER = 4


def _state(allocator_cls, grid_size, team_size, max_candidates):
    robot_ids = ["{:02d}".format(index) for index in range(team_size)]
    positions = {
        rid: (index % grid_size, index // grid_size)
        for index, rid in enumerate(robot_ids)
    }
    cfg = SimConfig(
        grid_size=grid_size,
        robot_ids=robot_ids,
        start_positions=positions,
        start_headings={rid: EAST for rid in robot_ids},
        max_candidate_cells=max_candidates,
        async_initial_spread_s=0.0,
        async_step_jitter_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        collision_intent_settle_s=0.0,
        write_parquet=False,
    )
    scenario = TrialScenario(
        trial_id=0,
        target=(grid_size - 1, grid_size - 1),
        clues=[(1, 1)],
    )
    runner = AsyncTrialRunner(
        cfg,
        allocator_cls,
        make_comm_model("ideal", None),
        seed=0,
    )
    state = runner.new_trial(scenario)
    for rid, robot in state.robots.items():
        robot._peer_positions = {
            peer_id: peer_pos
            for peer_id, peer_pos in positions.items()
            if peer_id != rid
        }
    return state


def _prepare_pair(
    reference_cls,
    optimized_cls,
    *,
    grid_size,
    team_size,
    max_candidates,
    seed,
    robot_id,
    uniform=False,
):
    reference_state = _state(
        reference_cls, grid_size, team_size, max_candidates
    )
    optimized_state = _state(
        optimized_cls, grid_size, team_size, max_candidates
    )
    reference = reference_state.robots[robot_id]
    optimized = optimized_state.robots[robot_id]

    rng = random.Random(seed)
    cells = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if (x, y) not in {reference.pos, (1, 1)}
    ]
    rng.shuffle(cells)
    searched = set(cells[: seed % max(1, grid_size)])
    obstacles = set(
        cells[len(searched): len(searched) + seed % 3]
    )

    if uniform:
        probability = 1.0 / (grid_size * grid_size)
        probabilities = {
            (x, y): probability
            for y in range(grid_size)
            for x in range(grid_size)
        }
    else:
        values = {
            (x, y): 0.01 + rng.random()
            for y in range(grid_size)
            for x in range(grid_size)
        }
        total = sum(values.values())
        probabilities = {
            cell: value / total for cell, value in values.items()
        }

    for robot in (reference, optimized):
        robot.belief.add_clue((1, 1))
        robot.belief.searched.update(searched)
        robot.belief.target_p = dict(probabilities)
        robot._temporary_invalid_task_until = {
            cell: float("inf") for cell in obstacles
        }

    return reference, optimized


def _assert_allocator_state_equal(testcase, reference, optimized):
    reference_decision = reference.allocator.choose_goal(reference)
    optimized_decision = optimized.allocator.choose_goal(optimized)

    testcase.assertEqual(reference_decision, optimized_decision)
    testcase.assertEqual(reference.dga_path, optimized.dga_path)
    testcase.assertEqual(reference.dga_best_plan, optimized.dga_best_plan)
    testcase.assertEqual(
        reference.dga_best_fitness, optimized.dga_best_fitness
    )
    testcase.assertEqual(
        reference.dga_generation, optimized.dga_generation
    )
    testcase.assertEqual(
        reference.dga_pending_deltas, optimized.dga_pending_deltas
    )
    testcase.assertEqual(
        reference.dga_population,
        optimized.allocator.unpack_population(optimized),
    )


class DGAOptimizedEquivalenceTests(unittest.TestCase):
    def test_randomized_small_generation_and_goal_parity(self):
        limits = (5, 8, 12, None)
        for seed in range(12):
            grid_size = 5 + seed % 3
            team_size = 1 + seed % 4
            robot_id = "{:02d}".format(
                team_size - 1 if seed % 2 else 0
            )
            reference, optimized = _prepare_pair(
                _SmallReferenceDGA,
                _SmallOptimizedDGA,
                grid_size=grid_size,
                team_size=team_size,
                max_candidates=limits[seed % len(limits)],
                seed=seed + 30,
                robot_id=robot_id,
                uniform=(seed % 4 == 0),
            )
            with self.subTest(
                seed=seed,
                grid_size=grid_size,
                team_size=team_size,
                robot_id=robot_id,
            ):
                _assert_allocator_state_equal(
                    self, reference, optimized
                )

    def test_second_reallocation_reuses_compact_population_with_parity(self):
        reference, optimized = _prepare_pair(
            _SmallReferenceDGA,
            _SmallOptimizedDGA,
            grid_size=7,
            team_size=4,
            max_candidates=18,
            seed=72,
            robot_id="03",
        )
        _assert_allocator_state_equal(self, reference, optimized)

        completed = reference.dga_path[0]
        for robot in (reference, optimized):
            robot.belief.searched.add(completed)
            robot.dga_path = []

        _assert_allocator_state_equal(self, reference, optimized)

    def test_reference_population_and_iterations_match_exactly(self):
        reference, optimized = _prepare_pair(
            DGAReferenceAllocator,
            DGAAllocator,
            grid_size=7,
            team_size=4,
            max_candidates=18,
            seed=91,
            robot_id="03",
        )
        _assert_allocator_state_equal(self, reference, optimized)
        self.assertEqual(len(reference.dga_population), 30)
        self.assertEqual(len(optimized.dga_population), 30)
        self.assertEqual(reference.dga_generation, 25)
        self.assertEqual(optimized.dga_generation, 25)

    def test_production_grid_topk_271_parity(self):
        reference, optimized = _prepare_pair(
            DGAReferenceAllocator,
            DGAAllocator,
            grid_size=19,
            team_size=4,
            max_candidates=271,
            seed=111,
            robot_id="03",
        )
        _assert_allocator_state_equal(self, reference, optimized)
        self.assertEqual(reference.dga_last_candidate_count, 271)
        self.assertEqual(optimized.dga_last_candidate_count, 271)

    def test_packed_population_numeric_payload_is_linear(self):
        _reference, optimized = _prepare_pair(
            _SmallReferenceDGA,
            _SmallOptimizedDGA,
            grid_size=7,
            team_size=4,
            max_candidates=18,
            seed=101,
            robot_id="03",
        )
        optimized.allocator.choose_goal(optimized)
        population = optimized.allocator.unpack_population(optimized)
        candidate_count = optimized.dga_last_candidate_count
        team_size = optimized.dga_last_team_size
        expected_payload = len(population) * (
            candidate_count + team_size
        ) * 2
        self.assertEqual(
            optimized.allocator.packed_population_payload_bytes(
                optimized
            ),
            expected_payload,
        )


if __name__ == "__main__":
    unittest.main()
