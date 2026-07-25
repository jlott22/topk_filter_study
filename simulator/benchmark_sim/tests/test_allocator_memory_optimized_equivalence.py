from __future__ import annotations

import importlib.util
import random
import unittest
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from benchmark_sim.algorithms.ACBBA import (
    ACBBAAllocator,
)
from benchmark_sim.algorithms.CBAA import (
    CBAAAllocator,
)
from benchmark_sim.algorithms.HIPC import (
    HIPCAllocator,
)
from benchmark_sim.algorithms.PI import (
    PIAllocator,
)
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


def _load_archived_allocator(filename: str, class_name: str):
    archive_path = Path(__file__).resolve().parents[3] / "archive" / filename
    module_name = "archived_" + filename.replace(".", "_").lower()
    spec = importlib.util.spec_from_file_location(module_name, archive_path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load archived allocator: {}".format(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, class_name)


PAIRS = (
    (
        "acbba",
        _load_archived_allocator(
            "ACBBA_simulator_unoptimized.py",
            "ACBBAAllocator",
        ),
        ACBBAAllocator,
    ),
    (
        "cbaa",
        _load_archived_allocator(
            "CBAA_simulator_unoptimized.py",
            "CBAAAllocator",
        ),
        CBAAAllocator,
    ),
    (
        "hipc",
        _load_archived_allocator(
            "HIPC_simulator_unoptimized.py",
            "HIPCAllocator",
        ),
        HIPCAllocator,
    ),
    (
        "pi",
        _load_archived_allocator(
            "PI_simulator_unoptimized.py",
            "PIAllocator",
        ),
        PIAllocator,
    ),
)

STATE_ATTRIBUTES = {
    "acbba": (
        "acbba_path",
        "acbba_bundle",
        "acbba_winner_by_cell",
        "acbba_winning_bid_by_cell",
        "acbba_bid_time_by_cell",
        "acbba_pending_deltas",
        "acbba_last_sent_signatures",
        "acbba_pending_snapshot",
        "acbba_bid_counter",
    ),
    "cbaa": (
        "cbaa_current_task",
        "cbaa_winner_by_cell",
        "cbaa_winning_bid_by_cell",
        "cbaa_pending_deltas",
        "cbaa_last_sent_signatures",
    ),
    "hipc": (
        "hipc_path",
        "hipc_bundle",
        "hipc_winner_by_cell",
        "hipc_winning_bid_by_cell",
        "hipc_bid_time_by_cell",
        "hipc_pending_snapshot",
        "hipc_bid_counter",
        "hipc_bad_prediction_count",
        "hipc_last_predicted_peer_first_task",
        "hipc_seen_peer_bundle_signature",
        "hipc_dropped_peers",
    ),
    "pi": (
        "pi_path",
        "pi_bundle",
        "pi_owner_by_cell",
        "pi_significance_by_cell",
        "pi_time_by_cell",
        "pi_pending_snapshot",
        "pi_time_counter",
    ),
}


def _state(allocator_cls, grid_size: int, team_size: int, max_candidates):
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
    state = AsyncTrialRunner(
        cfg,
        allocator_cls,
        make_comm_model("ideal", None),
        seed=0,
    ).new_trial(scenario)
    for rid, robot in state.robots.items():
        robot._peer_positions = {
            peer_id: position
            for peer_id, position in positions.items()
            if peer_id != rid
        }
    return state


def _prepare_pair(
    reference_cls,
    optimized_cls,
    *,
    grid_size: int,
    team_size: int,
    max_candidates,
    seed: int,
    robot_id: str,
    uniform: bool,
):
    reference_state = _state(
        reference_cls,
        grid_size,
        team_size,
        max_candidates,
    )
    optimized_state = _state(
        optimized_cls,
        grid_size,
        team_size,
        max_candidates,
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
    obstacles = set(cells[len(searched): len(searched) + seed % 3])

    if uniform:
        probabilities = {
            (x, y): 1.0 / (grid_size * grid_size)
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


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {
            key: _plain(item)
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [_plain(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_plain(item) for item in value)
    if isinstance(value, set):
        return {_plain(item) for item in value}
    return value


def _allocator_state(robot, algorithm: str) -> dict[str, Any]:
    return {
        attribute: _plain(getattr(robot, attribute))
        for attribute in STATE_ATTRIBUTES[algorithm]
    }


class AllocatorMemoryOptimizedEquivalenceTests(unittest.TestCase):
    def test_randomized_candidates_decisions_states_and_messages_match(self):
        limits = (None, 5, 12, 25)
        for algorithm, reference_cls, optimized_cls in PAIRS:
            for seed in range(12):
                grid_size = 5 + seed % 4
                team_size = 1 + seed % 4
                robot_id = "{:02d}".format(
                    team_size - 1 if seed % 2 else 0
                )
                reference, optimized = _prepare_pair(
                    reference_cls,
                    optimized_cls,
                    grid_size=grid_size,
                    team_size=team_size,
                    max_candidates=limits[seed % len(limits)],
                    seed=seed + 40,
                    robot_id=robot_id,
                    uniform=(seed % 4 == 0),
                )
                with self.subTest(
                    algorithm=algorithm,
                    seed=seed,
                    grid_size=grid_size,
                    team_size=team_size,
                ):
                    self.assertEqual(
                        reference.allocator._candidate_cells(reference),
                        optimized.allocator._candidate_cells(optimized),
                    )
                    self.assertEqual(
                        reference.allocator.choose_goal(reference),
                        optimized.allocator.choose_goal(optimized),
                    )
                    self.assertEqual(
                        _allocator_state(reference, algorithm),
                        _allocator_state(optimized, algorithm),
                    )
                    self.assertEqual(
                        reference.allocator.make_messages(reference),
                        optimized.allocator.make_messages(optimized),
                    )
                    self.assertEqual(
                        _allocator_state(reference, algorithm),
                        _allocator_state(optimized, algorithm),
                    )

    def test_second_reallocation_matches_after_first_goal_is_searched(self):
        for algorithm, reference_cls, optimized_cls in PAIRS:
            reference, optimized = _prepare_pair(
                reference_cls,
                optimized_cls,
                grid_size=7,
                team_size=4,
                max_candidates=25,
                seed=91,
                robot_id="03",
                uniform=False,
            )
            with self.subTest(algorithm=algorithm):
                first_reference = reference.allocator.choose_goal(reference)
                first_optimized = optimized.allocator.choose_goal(optimized)
                self.assertEqual(first_reference, first_optimized)
                self.assertIsNotNone(first_reference.goal)

                for robot in (reference, optimized):
                    robot.belief.mark_searched(first_reference.goal)
                    robot.current_goal = None

                self.assertEqual(
                    reference.allocator.choose_goal(reference),
                    optimized.allocator.choose_goal(optimized),
                )
                self.assertEqual(
                    _allocator_state(reference, algorithm),
                    _allocator_state(optimized, algorithm),
                )

    def test_production_grid_topk_271_matches(self):
        for algorithm, reference_cls, optimized_cls in PAIRS:
            reference, optimized = _prepare_pair(
                reference_cls,
                optimized_cls,
                grid_size=19,
                team_size=4,
                max_candidates=271,
                seed=111,
                robot_id="03",
                uniform=False,
            )
            with self.subTest(algorithm=algorithm):
                reference_candidates = reference.allocator._candidate_cells(reference)
                optimized_candidates = optimized.allocator._candidate_cells(optimized)
                self.assertEqual(reference_candidates, optimized_candidates)
                self.assertEqual(len(reference_candidates), 271)
                self.assertEqual(
                    reference.allocator.choose_goal(reference),
                    optimized.allocator.choose_goal(optimized),
                )
                self.assertEqual(
                    _allocator_state(reference, algorithm),
                    _allocator_state(optimized, algorithm),
                )

    def test_multi_robot_event_traces_match(self):
        robot_ids = ["00", "01", "02", "03"]
        positions = {
            "00": (0, 0),
            "01": (6, 0),
            "02": (0, 6),
            "03": (6, 6),
        }
        cfg = SimConfig(
            grid_size=7,
            robot_ids=robot_ids,
            start_positions=positions,
            start_headings={rid: EAST for rid in robot_ids},
            max_candidate_cells=25,
            async_initial_spread_s=0.0,
            async_step_jitter_s=0.0,
            comm_delay_s=0.0,
            comm_delay_jitter_s=0.0,
            collision_intent_settle_s=0.0,
            debug_max_events=5000,
            write_parquet=False,
        )
        scenario = TrialScenario(
            trial_id=7,
            target=(4, 4),
            clues=[(1, 1), (5, 5)],
        )

        for algorithm, reference_cls, optimized_cls in PAIRS:
            traces = []
            states = []
            for allocator_cls in (reference_cls, optimized_cls):
                trace = []

                def record(state, robot, result):
                    trace.append((
                        state.clock_s,
                        robot.rid,
                        robot.pos,
                        robot.current_goal,
                        result.reason,
                        result.found_target,
                    ))

                state = AsyncTrialRunner(
                    cfg,
                    allocator_cls,
                    make_comm_model("ideal", None),
                    seed=123,
                ).run_trial(scenario, on_step=record)
                traces.append(trace)
                states.append(state)

            with self.subTest(algorithm=algorithm):
                self.assertEqual(traces[0], traces[1])
                self.assertEqual(
                    states[0].world.target_found_by,
                    states[1].world.target_found_by,
                )
                self.assertEqual(
                    states[0].events_processed,
                    states[1].events_processed,
                )


if __name__ == "__main__":
    unittest.main()
