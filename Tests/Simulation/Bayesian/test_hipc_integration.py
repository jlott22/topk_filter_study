from __future__ import annotations

import sys
import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.algorithms.HIPC import Allocator, HIPCAllocator
from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


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
    runner = AsyncTrialRunner(cfg, HIPCAllocator, make_comm_model("ideal", None), seed=0)
    return runner.new_trial(scenario)


class HIPCIntegrationTests(unittest.TestCase):
    def test_dynamic_loader_can_import_hipc_allocator_and_alias(self) -> None:
        cls = load_allocator_class("benchmark_sim.algorithms.HIPC:HIPCAllocator")
        alias = load_allocator_class("benchmark_sim.algorithms.HIPC:Allocator")

        self.assertIs(cls, HIPCAllocator)
        self.assertIs(alias, HIPCAllocator)
        self.assertIs(Allocator, HIPCAllocator)
        self.assertTrue(issubclass(HIPCAllocator, AllocatorBase))
        self.assertFalse(issubclass(HIPCAllocator, CBAAAllocator))

    def test_hipc_module_does_not_import_acbba(self) -> None:
        sys.modules.pop("benchmark_sim.algorithms.HIPC", None)
        sys.modules.pop("benchmark_sim.algorithms.ACBBA", None)

        load_allocator_class("benchmark_sim.algorithms.HIPC:HIPCAllocator")

        self.assertNotIn("benchmark_sim.algorithms.ACBBA", sys.modules)

    def test_pre_clue_serpentine_and_no_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]

        decision = robot.allocator.choose_goal(robot)
        messages = robot.allocator.make_messages(robot)

        self.assertEqual(decision.debug["alg"], "HIPC")
        self.assertEqual(decision.debug["mode"], "serpentine_pre_clue")
        self.assertEqual(messages, [])

    def test_post_clue_debug_fields_all_valid_candidates_and_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))

        decision = robot.allocator.choose_goal(robot)
        path = list(robot.hipc_path)
        messages = robot.allocator.make_messages(robot)
        valid_count = sum(
            1
            for y in range(robot.grid_size)
            for x in range(robot.grid_size)
            if robot.allocator._valid_task_cell(robot, (x, y))
        )

        self.assertEqual(decision.debug["mode"], "hipc_post_clue")
        self.assertEqual(decision.goal, path[0])
        self.assertEqual(decision.debug["hipc_path"], path)
        self.assertLessEqual(len(path), HIPCAllocator.BUNDLE_SIZE)
        self.assertEqual(decision.debug["hipc_team_size"], 1)
        self.assertEqual(decision.debug["hipc_candidate_count"], valid_count)
        self.assertIn("hipc_bad_prediction_count", decision.debug)
        self.assertGreater(valid_count, HIPCAllocator.BUNDLE_SIZE)
        self.assertLessEqual(len(messages), HIPCAllocator.BUNDLE_SIZE)
        self.assertGreater(len(messages), 0)
        self.assertTrue(all(message["type"] == "hipc_entry" for message in messages))

    def test_unchanged_hipc_snapshot_sends_no_new_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)

        first_messages = robot.allocator.make_messages(robot)
        setattr(robot, "hipc_pending_snapshot", True)
        second_messages = robot.allocator.make_messages(robot)

        self.assertGreater(len(first_messages), 0)
        self.assertEqual(second_messages, [])
        self.assertFalse(robot.hipc_pending_snapshot)

    def test_empty_bundle_sends_one_clear_after_previous_bundle(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)

        first_messages = robot.allocator.make_messages(robot)
        self.assertGreater(len(first_messages), 0)

        robot.hipc_path = []
        robot.hipc_bundle = []
        robot.hipc_pending_snapshot = True
        clear_messages = robot.allocator.make_messages(robot)

        self.assertEqual(len(clear_messages), 1)
        self.assertEqual(clear_messages[0]["type"], "hipc_clear_bundle")
        self.assertEqual(clear_messages[0]["bundle_cells"], [])
        self.assertEqual(clear_messages[0]["bundle_size"], 0)

    def test_clear_bundle_removes_receiver_stale_sender_claims(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        sender.belief.add_clue((1, 1))
        receiver.belief.add_clue((1, 1))
        sender.allocator.choose_goal(sender)
        sender_path = list(sender.hipc_path)

        sender._publish_allocator_messages()
        state.bus.pump(0.0)
        claimed = [cell for cell in sender_path if receiver.hipc_winner_by_cell.get(cell) == "00"]
        self.assertGreater(len(claimed), 0)

        sender.hipc_path = []
        sender.hipc_bundle = []
        sender.hipc_pending_snapshot = True
        sender._publish_allocator_messages()
        state.bus.pump(0.0)

        for cell in claimed:
            self.assertIsNone(receiver.hipc_winner_by_cell.get(cell))
            self.assertEqual(receiver.hipc_winning_bid_by_cell.get(cell), HIPCAllocator.NO_BID)
            self.assertEqual(receiver.hipc_bid_time_by_cell.get(cell), HIPCAllocator.NO_TIME)

    def test_collision_state_triggers_bundle_reevaluation(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first = robot.allocator.choose_goal(robot)
        self.assertGreater(len(robot.hipc_path), 0)

        robot.current_goal = first.goal
        robot.collision_avoidance_active = True

        second = robot.allocator.choose_goal(robot)
        self.assertEqual(second.debug["hipc_trigger"], "collision_avoidance")
        self.assertGreater(len(robot.hipc_path), 0)
        self.assertLessEqual(len(robot.hipc_path), HIPCAllocator.BUNDLE_SIZE)
        self.assertEqual(second.goal, robot.hipc_path[0])

    def test_hipc_entry_routes_and_counts_through_message_bus(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        sender.belief.add_clue((1, 1))
        receiver.belief.add_clue((1, 1))
        sender.allocator.choose_goal(sender)
        sender_path = list(sender.hipc_path)

        sender._publish_allocator_messages()
        state.bus.pump(0.0)

        self.assertEqual(state.bus.counters.sent_by_robot["00"], len(sender_path))
        self.assertEqual(state.bus.counters.sent_total, len(sender_path))
        for cell in sender_path:
            if cell not in receiver.searched:
                self.assertEqual(receiver.hipc_winner_by_cell[cell], "00")


if __name__ == "__main__":
    unittest.main()
