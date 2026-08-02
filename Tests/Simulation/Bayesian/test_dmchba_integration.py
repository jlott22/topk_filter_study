from __future__ import annotations

import sys
import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.message import Message, topic_for
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


def _dmchba_cls():
    return load_allocator_class("benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator")


def _cfg(grid_size: int = 19) -> SimConfig:
    return SimConfig(
        grid_size=grid_size,
        robot_ids=["00", "01"],
        start_positions={"00": (0, 0), "01": (0, 1)},
        start_headings={"00": EAST, "01": EAST},
        async_initial_spread_s=0.0,
        async_step_jitter_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        collision_intent_settle_s=0.0,
        write_parquet=False,
    )


def _state(grid_size: int = 19):
    cfg = _cfg(grid_size)
    scenario = TrialScenario(trial_id=0, target=(grid_size - 1, grid_size - 1), clues=[(1, 1)])
    runner = AsyncTrialRunner(cfg, _dmchba_cls(), make_comm_model("ideal", None), seed=0)
    return runner.new_trial(scenario)


class DMCHBAIntegrationTests(unittest.TestCase):
    def test_dynamic_loader_can_import_dmchba_allocator_and_alias(self) -> None:
        checked_modules = (
            "benchmark_sim.algorithms.DMCHBA",
            "benchmark_sim.algorithms.CBAA",
            "benchmark_sim.algorithms.ACBBA",
            "benchmark_sim.algorithms.PI",
            "benchmark_sim.algorithms.HIPC",
            "benchmark_sim.algorithms.Auction_greedy",
        )
        saved_modules = {module_name: sys.modules.get(module_name) for module_name in checked_modules}
        try:
            for module_name in checked_modules:
                sys.modules.pop(module_name, None)

            cls = load_allocator_class("benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator")
            alias = load_allocator_class("benchmark_sim.algorithms.DMCHBA:Allocator")

            self.assertIs(cls, alias)
            self.assertTrue(issubclass(cls, AllocatorBase))
            self.assertNotIn("benchmark_sim.algorithms.CBAA", sys.modules)
            self.assertNotIn("benchmark_sim.algorithms.ACBBA", sys.modules)
            self.assertNotIn("benchmark_sim.algorithms.PI", sys.modules)
            self.assertNotIn("benchmark_sim.algorithms.HIPC", sys.modules)
            self.assertNotIn("benchmark_sim.algorithms.Auction_greedy", sys.modules)
        finally:
            for module_name in checked_modules:
                sys.modules.pop(module_name, None)
                if saved_modules[module_name] is not None:
                    sys.modules[module_name] = saved_modules[module_name]

    def test_pre_clue_serpentine_and_no_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]

        decision = robot.allocator.choose_goal(robot)

        self.assertEqual(decision.debug["alg"], "DMCHBA")
        self.assertEqual(decision.debug["mode"], "serpentine_pre_clue")
        self.assertEqual(decision.goal, (1, 0))
        self.assertEqual(robot.allocator.make_messages(robot), [])
        self.assertEqual(robot.allocator.get_outbound_messages(robot), [])
        self.assertEqual(robot.allocator.build_dmchba_messages(robot), [])
        self.assertEqual(robot.allocator.build_cbaa_messages(robot), [])
        self.assertEqual(robot.allocator.build_acbba_messages(robot), [])

    def test_post_clue_evaluates_all_valid_cells_and_caps_only_committed_path(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot._peer_positions["01"] = (0, 1)
        robot.belief.add_clue((1, 1))
        robot.belief.searched.add((4, 4))
        robot.belief.recompute()

        decision = robot.allocator.choose_goal(robot)
        valid_count = sum(
            1
            for y in range(robot.grid_size)
            for x in range(robot.grid_size)
            if robot.allocator._valid_task_cell(robot, (x, y))
        )

        self.assertEqual(decision.debug["mode"], "dmchba_post_clue")
        self.assertEqual(decision.debug["dmchba_trigger"], "clue_changed")
        self.assertTrue(decision.debug["dmchba_evaluates_all_candidates"])
        self.assertFalse(decision.debug["dmchba_allocator_messages"])
        self.assertEqual(decision.debug["dmchba_candidate_count"], valid_count)
        self.assertEqual(valid_count, 22)
        self.assertGreater(decision.debug["dmchba_candidate_count"], 3)
        self.assertEqual(decision.debug["dmchba_team_size"], 2)
        self.assertEqual(decision.debug["dmchba_matrix_n"], 22)
        self.assertEqual(robot.dmchba_clones_per_agent, 11)
        self.assertEqual(robot.dmchba_pseudotask_count, 0)
        self.assertGreater(decision.debug["dmchba_assigned_count"], 3)
        self.assertEqual(decision.debug["dmchba_commitment_horizon"], 3)
        self.assertEqual(decision.debug["dmchba_committed_count"], len(robot.dmchba_path))
        self.assertLessEqual(len(robot.dmchba_path), 3)
        self.assertEqual(len(robot.dmchba_path), 3)
        self.assertEqual(decision.goal, robot.dmchba_path[0])

        robot.belief.searched.discard((4, 4))
        robot.belief.recompute()
        robot.dmchba_path = []
        robot.dmchba_last_assignment_signature = None
        decision = robot.allocator.choose_goal(robot)

        self.assertEqual(decision.debug["dmchba_candidate_count"], 23)
        self.assertEqual(decision.debug["dmchba_matrix_n"], 24)
        self.assertEqual(robot.dmchba_clones_per_agent, 12)
        self.assertEqual(robot.dmchba_pseudotask_count, 1)
        self.assertGreater(decision.debug["dmchba_assigned_count"], 3)
        self.assertLessEqual(len(robot.dmchba_path), 3)
        self.assertEqual(decision.debug["dmchba_committed_count"], len(robot.dmchba_path))

    def test_ordinary_updates_do_not_reassign_until_path_empty(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot._peer_positions["01"] = (0, 1)
        robot.belief.add_clue((1, 1))
        first = robot.allocator.choose_goal(robot)
        initial_path = list(robot.dmchba_path)
        self.assertEqual(first.debug["dmchba_trigger"], "clue_changed")
        self.assertGreaterEqual(len(initial_path), 2)

        unrelated = next(
            cell
            for y in range(robot.grid_size)
            for x in range(robot.grid_size)
            for cell in [(x, y)]
            if cell not in robot.searched and cell not in initial_path
        )
        robot.belief.searched.add(unrelated)
        robot.belief.recompute()

        second = robot.allocator.choose_goal(robot)
        self.assertIsNone(second.debug["dmchba_trigger"])
        self.assertEqual(robot.dmchba_path, initial_path)

        for cell in list(robot.dmchba_path):
            robot.belief.searched.add(cell)
        robot.belief.recompute()

        third = robot.allocator.choose_goal(robot)
        self.assertEqual(third.debug["dmchba_trigger"], "path_exhausted")
        self.assertNotEqual(robot.dmchba_path, [])

    def test_clue_keeps_current_goal_and_collision_state_triggers_replan(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot.current_goal = (4, 4)

        robot.receive_message(
            Message(
                sender="01",
                topic=topic_for("01", "clue"),
                payload={"loc": [1, 1]},
                created_at_s=0.0,
            )
        )

        self.assertEqual(robot.current_goal, (4, 4))
        decision = robot.allocator.choose_goal(robot)
        self.assertEqual(decision.debug["dmchba_trigger"], "clue_changed")

        robot.current_goal = decision.goal
        robot.collision_avoidance_active = True

        decision = robot.allocator.choose_goal(robot)
        self.assertEqual(decision.debug["dmchba_trigger"], "collision_avoidance")

    def test_multi_task_forward_horizon_matches_other_bounded_algorithms(self) -> None:
        acbba_cls = load_allocator_class("benchmark_sim.algorithms.ACBBA:ACBBAAllocator")
        pi_cls = load_allocator_class("benchmark_sim.algorithms.PI:PIAllocator")
        hipc_cls = load_allocator_class("benchmark_sim.algorithms.HIPC:HIPCAllocator")
        dmchba_cls = load_allocator_class("benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator")

        self.assertEqual(acbba_cls.BUNDLE_SIZE, 3)
        self.assertEqual(pi_cls.BUNDLE_SIZE, 3)
        self.assertEqual(hipc_cls.BUNDLE_SIZE, 3)
        self.assertEqual(dmchba_cls.COMMITMENT_HORIZON, 3)


if __name__ == "__main__":
    unittest.main()
