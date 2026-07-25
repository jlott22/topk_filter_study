from __future__ import annotations

import math
import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.algorithms.PI import Allocator, PIAllocator
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
    runner = AsyncTrialRunner(cfg, PIAllocator, make_comm_model("ideal", None), seed=0)
    return runner.new_trial(scenario)


class PIIntegrationTests(unittest.TestCase):
    def test_dynamic_loader_can_import_pi_allocator_and_alias(self) -> None:
        cls = load_allocator_class("benchmark_sim.algorithms.PI:PIAllocator")
        alias = load_allocator_class("benchmark_sim.algorithms.PI:Allocator")

        self.assertIs(cls, PIAllocator)
        self.assertIs(alias, PIAllocator)
        self.assertIs(Allocator, PIAllocator)
        self.assertTrue(issubclass(PIAllocator, AllocatorBase))
        self.assertFalse(issubclass(PIAllocator, CBAAAllocator))

    def test_pre_clue_serpentine_and_no_pi_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]

        decision = robot.allocator.choose_goal(robot)
        messages = robot.allocator.make_messages(robot)

        self.assertEqual(decision.debug["alg"], "PI")
        self.assertEqual(decision.debug["mode"], "serpentine_pre_clue")
        self.assertEqual(messages, [])

    def test_post_clue_path_goal_and_pi_entry_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))

        decision = robot.allocator.choose_goal(robot)
        path = list(robot.pi_path)
        messages = robot.allocator.make_messages(robot)

        self.assertGreater(len(path), 0)
        self.assertLessEqual(len(path), PIAllocator.BUNDLE_SIZE)
        self.assertEqual(decision.goal, path[0])
        self.assertEqual(decision.debug["mode"], "pi_post_clue")
        self.assertTrue(messages)
        self.assertTrue(all(message["type"] == "pi_entry" for message in messages))
        for order, message in enumerate(messages):
            self.assertEqual(message["sender"], "00")
            self.assertEqual(message["owner"], "00")
            self.assertEqual(message["order"], order)
            self.assertEqual(message["path_size"], len(path))
            self.assertTrue(
                {
                    "sender",
                    "owner",
                    "x",
                    "y",
                    "significance",
                    "timestamp",
                    "order",
                    "path_cells",
                    "path_size",
                }
                <= set(message)
            )

    def test_pi_messages_are_counted_individually(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)
        path_len = len(robot.pi_path)

        robot._publish_allocator_messages()

        self.assertEqual(state.bus.counters.sent_by_robot["00"], path_len)
        self.assertEqual(state.bus.counters.sent_total, path_len)

    def test_unchanged_pi_snapshot_sends_no_new_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)

        first_messages = robot.allocator.make_messages(robot)
        setattr(robot, "pi_pending_snapshot", True)
        second_messages = robot.allocator.make_messages(robot)

        self.assertGreater(len(first_messages), 0)
        self.assertEqual(second_messages, [])
        self.assertFalse(robot.pi_pending_snapshot)

    def test_collision_state_triggers_path_reevaluation(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first = robot.allocator.choose_goal(robot)
        self.assertGreater(len(robot.pi_path), 0)

        robot.current_goal = first.goal
        robot.collision_avoidance_active = True

        second = robot.allocator.choose_goal(robot)
        self.assertEqual(second.debug["pi_trigger"], "collision_avoidance")
        self.assertGreater(len(robot.pi_path), 0)
        self.assertLessEqual(len(robot.pi_path), PIAllocator.BUNDLE_SIZE)
        self.assertEqual(second.goal, robot.pi_path[0])

    def test_pi_entry_routes_through_message_bus(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        sender.belief.add_clue((1, 1))
        receiver.belief.add_clue((1, 1))
        sender.allocator.choose_goal(sender)
        sender_path = list(sender.pi_path)

        sender._publish_allocator_messages()
        state.bus.pump(0.0)

        owners = receiver.pi_owner_by_cell
        significances = receiver.pi_significance_by_cell
        delivered_claim_cells = [cell for cell in sender_path if cell not in receiver.searched]
        self.assertGreater(len(delivered_claim_cells), 0)
        for cell in delivered_claim_cells:
            self.assertEqual(owners[cell], "00")
            self.assertGreaterEqual(significances[cell], 0.0)
            self.assertTrue(math.isfinite(significances[cell]))

    def test_lower_significance_claim_removes_only_lost_task(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)
        path = list(robot.pi_path)
        self.assertEqual(len(path), PIAllocator.BUNDLE_SIZE)
        first, second, third = path

        current_sig = robot.pi_significance_by_cell[second]
        robot._deliver_allocator_payload({
            "type": "pi_entry",
            "sender": "01",
            "x": second[0],
            "y": second[1],
            "owner": "01",
            "significance": max(0.0, float(current_sig) - 0.001),
            "timestamp": 10.0,
            "order": 0,
            "path_cells": [{"x": second[0], "y": second[1]}],
            "path_size": 1,
        })

        self.assertEqual(robot.pi_owner_by_cell[second], "01")
        self.assertNotIn(second, robot.pi_path)
        self.assertIn(first, robot.pi_path)
        self.assertIn(third, robot.pi_path)
        self.assertLessEqual(len(robot.pi_path), PIAllocator.BUNDLE_SIZE)

    def test_ownership_message_updates_current_goal_to_repaired_path_head(self) -> None:
        state = _state()
        robot = state.robots["01"]
        robot.belief.add_clue((1, 1))
        decision = robot.allocator.choose_goal(robot)
        old_path = list(robot.pi_path)
        self.assertGreaterEqual(len(old_path), 2)
        old_head = old_path[0]
        robot.current_goal = decision.goal

        current_sig = robot.pi_significance_by_cell[old_head]
        robot._deliver_allocator_payload({
            "type": "pi_entry",
            "sender": "00",
            "x": old_head[0],
            "y": old_head[1],
            "owner": "00",
            "significance": max(0.0, float(current_sig) - 0.001),
            "timestamp": 10.0,
            "order": 0,
            "path_cells": [{"x": old_head[0], "y": old_head[1]}],
            "path_size": 1,
        })

        self.assertNotEqual(robot.pi_path[0], old_head)
        self.assertEqual(robot.current_goal, robot.pi_path[0])

    def test_empty_pi_clear_path_clears_sender_stale_claims(self) -> None:
        state = _state()
        receiver = state.robots["01"]
        receiver.belief.add_clue((1, 1))
        receiver.allocator._ensure_pi_state(receiver)
        receiver.pi_owner_by_cell[(2, 2)] = "00"
        receiver.pi_significance_by_cell[(2, 2)] = 1.0
        receiver.pi_time_by_cell[(2, 2)] = 1.0

        receiver._deliver_allocator_payload({
            "type": "pi_clear_path",
            "sender": "00",
            "timestamp": 2.0,
            "path_cells": [],
            "path_size": 0,
        })

        self.assertIsNone(receiver.pi_owner_by_cell[(2, 2)])
        self.assertEqual(receiver.pi_significance_by_cell[(2, 2)], PIAllocator.INF_SIGNIFICANCE)

    def test_high_target_probability_reduces_effective_move_cost(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.belief.target_p = {(4, 0): 10.0, (0, 4): 1.0}
        robot.allocator._refresh_probability_normalizer(robot)

        high_p_cost = robot.allocator._effective_move_cost(robot, (0, 0), (4, 0))
        low_p_cost = robot.allocator._effective_move_cost(robot, (0, 0), (0, 4))

        self.assertLess(high_p_cost, low_p_cost)
        self.assertGreaterEqual(high_p_cost, 0.0)
        self.assertTrue(math.isfinite(high_p_cost))

    def test_tiny_positive_map_maximum_is_not_replaced_by_eps_fallback(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot.belief.target_p = {(4, 0): 5.0e-10, (0, 4): 2.5e-10}

        robot.allocator._refresh_probability_normalizer(robot)

        self.assertEqual(robot.pi_probability_normalizer, 5.0e-10)
        self.assertEqual(
            robot.allocator._normalized_target_probability(robot, (4, 0)),
            1.0,
        )
        self.assertEqual(
            robot.allocator._normalized_target_probability(robot, (0, 4)),
            0.5,
        )

    def test_route_costs_and_significances_are_finite_nonnegative(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.belief.target_p = {(1, 0): float("nan"), (2, 0): float("inf")}
        robot.allocator.PROBABILITY_ALPHA = float("inf")
        robot.allocator.TASK_SERVICE_COST = float("inf")

        route_cost = robot.allocator._route_cost(robot, [(1, 0), (2, 0)])
        _, marginal = robot.allocator._best_insertion(robot, [(1, 0)], (2, 0))

        self.assertGreaterEqual(route_cost, 0.0)
        self.assertGreaterEqual(marginal, 0.0)
        self.assertTrue(math.isfinite(route_cost))
        self.assertTrue(math.isfinite(marginal))


if __name__ == "__main__":
    unittest.main()
