from __future__ import annotations

import importlib.util
import math
import random
import unittest
from pathlib import Path

from benchmark_sim.algorithms.DMCHBA import (
    Allocator,
    DMCHBAAllocator,
    DMCHBAOptimizedAllocator,
)
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


def _load_archived_allocator():
    archive_path = (
        Path(__file__).resolve().parents[3]
        / "Simulation"
        / "Archive"
        / "Legacy"
        / "DMCHBA_simulator_unoptimized.py"
    )
    spec = importlib.util.spec_from_file_location(
        "dmchba_simulator_unoptimized", archive_path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load archived DMCHBA allocator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.DMCHBAAllocator


ArchivedDMCHBAAllocator = _load_archived_allocator()


def _state(allocator_cls, grid_size=7, team_size=4, max_candidates=None):
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
        cfg, allocator_cls, make_comm_model("ideal", None), seed=0)
    state = runner.new_trial(scenario)
    for rid, robot in state.robots.items():
        robot._peer_positions = {
            peer_id: peer_pos
            for peer_id, peer_pos in positions.items()
            if peer_id != rid
        }
    return state


def _prepare_robot_pair(
    grid_size,
    team_size,
    max_candidates,
    seed,
    robot_id,
    uniform=False,
):
    original_state = _state(
        ArchivedDMCHBAAllocator, grid_size, team_size, max_candidates)
    optimized_state = _state(
        DMCHBAAllocator, grid_size, team_size, max_candidates)
    original = original_state.robots[robot_id]
    optimized = optimized_state.robots[robot_id]

    rng = random.Random(seed)
    cells = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if (x, y) not in {original.pos, (1, 1)}
    ]
    rng.shuffle(cells)
    searched = set(cells[: min(len(cells), seed % (grid_size + 1))])
    obstacles = set(cells[len(searched): len(searched) + seed % 3])

    for robot in (original, optimized):
        robot.belief.add_clue((1, 1))
        robot.belief.searched.update(searched)
        robot.belief.recompute()
        robot._temporary_invalid_task_until = {
            cell: float("inf") for cell in obstacles
        }

    if uniform:
        valid_probability = 1.0 / (grid_size * grid_size)
        probabilities = {
            (x, y): valid_probability
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
        probabilities = {cell: value / total for cell, value in values.items()}

    original.belief.target_p = dict(probabilities)
    optimized.belief.target_p = dict(probabilities)
    return original, optimized


def _assert_robot_results_equal(testcase, original, optimized):
    original_candidates = original.allocator._candidate_cells(original)
    optimized_candidates = optimized.allocator._candidate_cells(optimized)
    testcase.assertEqual(original_candidates, optimized_candidates)

    original_decision = original.allocator.choose_goal(original)
    optimized_decision = optimized.allocator.choose_goal(optimized)
    testcase.assertEqual(original_decision.goal, optimized_decision.goal)
    testcase.assertEqual(original_decision.debug, optimized_decision.debug)
    testcase.assertEqual(original.dmchba_path, optimized.dmchba_path)
    testcase.assertEqual(
        original.dmchba_last_assignment_signature,
        optimized.dmchba_last_assignment_signature,
    )
    testcase.assertEqual(
        original.dmchba_clones_per_agent,
        optimized.dmchba_clones_per_agent,
    )
    testcase.assertEqual(
        original.dmchba_pseudotask_count,
        optimized.dmchba_pseudotask_count,
    )


class DMCHBAOptimizedEquivalenceTests(unittest.TestCase):
    def test_alias_and_algorithm_name(self):
        self.assertIs(Allocator, DMCHBAAllocator)
        self.assertIs(DMCHBAOptimizedAllocator, DMCHBAAllocator)
        self.assertIs(Allocator, DMCHBAOptimizedAllocator)
        self.assertEqual(
            ArchivedDMCHBAAllocator.name, DMCHBAAllocator.name)

    def test_randomized_candidate_assignment_and_goal_parity(self):
        candidate_limits = (None, 5, 12, 25)
        for seed in range(16):
            grid_size = 5 + seed % 4
            team_size = 1 + seed % 4
            limit = candidate_limits[seed % len(candidate_limits)]
            robot_id = "{:02d}".format(
                team_size - 1 if seed % 2 else 0)
            original, optimized = _prepare_robot_pair(
                grid_size,
                team_size,
                limit,
                seed + 20,
                robot_id,
                uniform=(seed % 4 == 0),
            )
            with self.subTest(
                seed=seed,
                grid_size=grid_size,
                team_size=team_size,
                limit=limit,
                robot_id=robot_id,
            ):
                _assert_robot_results_equal(self, original, optimized)

    def test_virtual_solver_matches_dense_assignment_exactly(self):
        original, optimized = _prepare_robot_pair(
            grid_size=7,
            team_size=4,
            max_candidates=25,
            seed=91,
            robot_id="03",
            uniform=True,
        )
        tasks = original.allocator._candidate_cells(original)
        self.assertEqual(tasks, optimized.allocator._candidate_cells(optimized))
        team = original.allocator._team_agents(original)
        self.assertEqual(team, optimized.allocator._team_agents(optimized))

        agent_ids = sorted(
            team.keys(), key=original.allocator._robot_id_key)
        clones_per_agent = math.ceil(len(tasks) / len(agent_ids))
        clone_rows = [
            (rid, team[rid], clone_index)
            for rid in agent_ids
            for clone_index in range(clones_per_agent)
        ]
        matrix_n = len(clone_rows)
        columns = list(tasks) + [None] * (matrix_n - len(tasks))
        dense = original.allocator._build_cost_matrix(
            original, clone_rows, columns)
        expected = original.allocator._solve_assignment(dense)
        actual = optimized.allocator._solve_virtual_assignment(
            optimized,
            agent_ids,
            team,
            tasks,
            clones_per_agent,
            matrix_n,
        )
        self.assertEqual(expected, list(actual))

    def test_optimized_execution_does_not_call_dense_matrix_builder(self):
        _original, optimized = _prepare_robot_pair(
            grid_size=7,
            team_size=4,
            max_candidates=25,
            seed=101,
            robot_id="00",
        )

        def fail_if_called(*_args, **_kwargs):
            raise AssertionError("optimized execution built the dense matrix")

        optimized.allocator._build_cost_matrix = fail_if_called
        decision = optimized.allocator.choose_goal(optimized)
        self.assertIsNotNone(decision.goal)

    def test_production_grid_topk_271_parity(self):
        original, optimized = _prepare_robot_pair(
            grid_size=19,
            team_size=4,
            max_candidates=271,
            seed=111,
            robot_id="03",
        )
        _assert_robot_results_equal(self, original, optimized)
        self.assertEqual(
            optimized.dmchba_last_candidate_count, 271)
        self.assertEqual(
            optimized.dmchba_last_matrix_n, 272)

    def test_production_grid_all_candidates_parity(self):
        original, optimized = _prepare_robot_pair(
            grid_size=19,
            team_size=4,
            max_candidates=None,
            seed=120,
            robot_id="03",
        )
        _assert_robot_results_equal(self, original, optimized)
        # The local start and known clue cells are searched, leaving 359 valid.
        self.assertEqual(
            optimized.dmchba_last_candidate_count, 359)
        self.assertEqual(
            optimized.dmchba_last_matrix_n, 360)

        allocator = optimized.allocator
        workspace_payload = (
            len(allocator._h_u) * allocator._h_u.itemsize
            + len(allocator._h_v) * allocator._h_v.itemsize
            + len(allocator._h_minv) * allocator._h_minv.itemsize
            + len(allocator._h_p) * allocator._h_p.itemsize
            + len(allocator._h_way) * allocator._h_way.itemsize
            + len(allocator._h_used)
            + len(allocator._h_assignment) * allocator._h_assignment.itemsize
        )
        transient_base_cost_payload = 4 * 359 * 8
        optimized_numeric_payload = (
            workspace_payload + transient_base_cost_payload)
        dense_reference_payload = 360 * 360 * 8
        self.assertLess(optimized_numeric_payload, 32 * 1024)
        self.assertGreater(
            dense_reference_payload, optimized_numeric_payload * 40)

    def test_collision_and_path_exhaustion_sequence_parity(self):
        original, optimized = _prepare_robot_pair(
            grid_size=6,
            team_size=3,
            max_candidates=20,
            seed=121,
            robot_id="00",
        )
        _assert_robot_results_equal(self, original, optimized)

        original.collision_avoidance_active = True
        optimized.collision_avoidance_active = True
        original_collision = original.allocator.choose_goal(original)
        optimized_collision = optimized.allocator.choose_goal(optimized)
        self.assertEqual(original_collision.goal, optimized_collision.goal)
        self.assertEqual(original_collision.debug, optimized_collision.debug)
        self.assertEqual(original.dmchba_path, optimized.dmchba_path)

        for robot in (original, optimized):
            robot.collision_avoidance_active = False
            robot.belief.searched.update(robot.dmchba_path)
            robot.belief.recompute()

        original_exhausted = original.allocator.choose_goal(original)
        optimized_exhausted = optimized.allocator.choose_goal(optimized)
        self.assertEqual(original_exhausted.goal, optimized_exhausted.goal)
        self.assertEqual(original_exhausted.debug, optimized_exhausted.debug)
        self.assertEqual(original.dmchba_path, optimized.dmchba_path)


if __name__ == "__main__":
    unittest.main()
