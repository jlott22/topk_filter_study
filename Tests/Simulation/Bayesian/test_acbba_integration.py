from __future__ import annotations

import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.algorithms.ACBBA import ACBBAAllocator
from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


def _cfg(grid_size: int = 19, robot_ids: list[str] | None = None) -> SimConfig:
    robot_ids = robot_ids or ["00", "01"]
    start_positions = {rid: (0, idx) for idx, rid in enumerate(robot_ids)}
    return SimConfig(
        grid_size=grid_size,
        robot_ids=robot_ids,
        start_positions=start_positions,
        start_headings={rid: EAST for rid in robot_ids},
        async_initial_spread_s=0.0,
        async_step_jitter_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        collision_intent_settle_s=0.0,
        write_parquet=False,
    )


def _state(grid_size: int = 19, robot_ids: list[str] | None = None):
    cfg = _cfg(grid_size, robot_ids)
    scenario = TrialScenario(trial_id=0, target=(grid_size - 1, grid_size - 1), clues=[(1, 1)])
    runner = AsyncTrialRunner(cfg, ACBBAAllocator, make_comm_model("ideal", None), seed=0)
    return runner.new_trial(scenario)


class ACBBAIntegrationTests(unittest.TestCase):
    def test_dynamic_loader_can_import_acbba_allocator(self) -> None:
        cls = load_allocator_class("benchmark_sim.algorithms.ACBBA:ACBBAAllocator")

        self.assertIs(cls, ACBBAAllocator)
        self.assertTrue(issubclass(ACBBAAllocator, AllocatorBase))
        self.assertFalse(issubclass(ACBBAAllocator, CBAAAllocator))

    def test_pre_clue_generates_no_acbba_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]

        messages = robot.allocator.build_acbba_messages(robot)

        self.assertEqual(messages, [])

    def test_first_post_clue_assignment_builds_bundle_and_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))

        decision = robot.allocator.choose_goal(robot)
        path = getattr(robot, "acbba_path")
        messages = robot.allocator.build_acbba_messages(robot)

        self.assertGreater(len(path), 0)
        self.assertLessEqual(len(path), ACBBAAllocator.BUNDLE_SIZE)
        self.assertEqual(decision.goal, path[0])
        self.assertEqual(len(messages), len(path))
        for order, message in enumerate(messages):
            self.assertEqual(message["type"], "acbba_entry")
            self.assertEqual(message["sender"], "00")
            self.assertEqual(message["winner"], "00")
            self.assertEqual(message["order"], order)
            self.assertEqual(message["bundle_size"], len(path))
            self.assertTrue(
                {
                    "sender",
                    "x",
                    "y",
                    "winner",
                    "bid",
                    "timestamp",
                    "order",
                    "bundle_cells",
                    "bundle_size",
                }
                <= set(message)
            )

    def test_unchanged_bundle_sends_no_new_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first_goal = robot.allocator.choose_goal(robot).goal
        self.assertGreater(len(robot.allocator.build_acbba_messages(robot)), 0)

        second_goal = robot.allocator.choose_goal(robot).goal
        messages = robot.allocator.build_acbba_messages(robot)

        self.assertEqual(second_goal, first_goal)
        self.assertEqual(messages, [])

    def test_collision_state_triggers_bundle_reevaluation(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first = robot.allocator.choose_goal(robot)
        self.assertGreater(len(robot.acbba_path), 0)

        robot.current_goal = first.goal
        robot.collision_avoidance_active = True

        second = robot.allocator.choose_goal(robot)
        self.assertEqual(second.debug["acbba_trigger"], "collision_avoidance")
        self.assertGreater(len(robot.acbba_path), 0)
        self.assertLessEqual(len(robot.acbba_path), ACBBAAllocator.BUNDLE_SIZE)
        self.assertEqual(second.goal, robot.acbba_path[0])

    def test_bundle_builder_uses_best_path_insertion(self) -> None:
        state = _state(grid_size=5)
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))

        # Force only one new valid task. From (0, 0), inserting (1, 0)
        # before (4, 0) preserves route length, while appending it adds cost.
        robot.belief.searched = {
            (x, y)
            for y in range(robot.grid_size)
            for x in range(robot.grid_size)
            if (x, y) not in {(1, 0), (4, 0)}
        }
        robot.belief.recompute()

        robot.allocator._ensure_acbba_state(robot)
        robot.acbba_path = [(4, 0)]
        robot.acbba_bundle = [(4, 0)]
        robot.acbba_winner_by_cell[(4, 0)] = "00"
        robot.acbba_winning_bid_by_cell[(4, 0)] = 1.0
        robot.acbba_bid_time_by_cell[(4, 0)] = 1.0

        robot.allocator._build_bundle(robot)

        self.assertEqual(robot.acbba_path, [(1, 0), (4, 0)])
        self.assertLessEqual(len(robot.acbba_path), ACBBAAllocator.BUNDLE_SIZE)

    def test_publish_counts_each_bundle_entry_individually(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)
        path_len = len(getattr(robot, "acbba_path"))

        robot._publish_allocator_messages()

        self.assertEqual(state.bus.counters.sent_by_robot["00"], path_len)
        self.assertEqual(state.bus.counters.sent_total, path_len)

    def test_acbba_entry_routes_through_message_bus(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        sender.belief.add_clue((1, 1))
        receiver.belief.add_clue((1, 1))
        sender.allocator.choose_goal(sender)
        sender_path = list(getattr(sender, "acbba_path"))

        sender._publish_allocator_messages()
        state.bus.pump(0.0)

        winners = getattr(receiver, "acbba_winner_by_cell")
        bids = getattr(receiver, "acbba_winning_bid_by_cell")
        delivered_claim_cells = [cell for cell in sender_path if cell not in receiver.searched]
        self.assertGreater(len(delivered_claim_cells), 0)
        for cell in delivered_claim_cells:
            self.assertEqual(winners[cell], "00")
            self.assertGreater(bids[cell], ACBBAAllocator.NO_BID)

    def test_losing_second_bundle_task_releases_suffix(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)
        path = list(getattr(robot, "acbba_path"))
        self.assertEqual(len(path), ACBBAAllocator.BUNDLE_SIZE)
        first, second, third = path

        bids = getattr(robot, "acbba_winning_bid_by_cell")
        robot._deliver_allocator_payload({
            "type": "acbba_entry",
            "sender": "01",
            "x": second[0],
            "y": second[1],
            "winner": "01",
            "bid": float(bids[second]) + 100.0,
            "timestamp": 10.0,
            "order": 0,
            "bundle_cells": [{"x": second[0], "y": second[1]}],
            "bundle_size": 1,
        })

        repaired_path = list(getattr(robot, "acbba_path"))
        winners = getattr(robot, "acbba_winner_by_cell")
        self.assertEqual(repaired_path, [first])
        self.assertEqual(winners[first], "00")
        self.assertEqual(winners[second], "01")
        self.assertIsNone(winners[third])

        robot.allocator.choose_goal(robot)
        rebuilt_path = list(getattr(robot, "acbba_path"))
        self.assertGreaterEqual(len(rebuilt_path), 1)
        self.assertLessEqual(len(rebuilt_path), ACBBAAllocator.BUNDLE_SIZE)
        self.assertIn(first, rebuilt_path)

    def test_table1_forwards_third_party_claim_without_bundle_metadata(self) -> None:
        state = _state(grid_size=5, robot_ids=["00", "01", "02"])
        sender = state.robots["00"]
        forwarder = state.robots["01"]
        receiver = state.robots["02"]
        for robot in (sender, forwarder, receiver):
            robot.belief.add_clue((1, 1))

        cell = (2, 2)
        sender.allocator._insert_claim(sender, cell, 0, 5.0)
        first_hop = sender.allocator.build_acbba_messages(sender)
        self.assertEqual(len(first_hop), 1)

        forwarder._deliver_allocator_payload(first_hop[0])
        forwarded = forwarder.allocator.build_acbba_messages(forwarder)

        self.assertEqual(len(forwarded), 1)
        self.assertEqual(forwarded[0]["sender"], "01")
        self.assertEqual(forwarded[0]["winner"], "00")
        self.assertNotIn("bundle_cells", forwarded[0])

        receiver._deliver_allocator_payload(forwarded[0])
        self.assertEqual(receiver.acbba_winner_by_cell[cell], "00")
        self.assertEqual(receiver.acbba_winning_bid_by_cell[cell], first_hop[0]["bid"])

    def test_table1_forwarded_duplicate_does_not_rebroadcast_forever(self) -> None:
        state = _state(grid_size=5, robot_ids=["00", "01", "02"])
        sender = state.robots["00"]
        forwarder = state.robots["01"]
        for robot in (sender, forwarder):
            robot.belief.add_clue((1, 1))

        cell = (2, 2)
        sender.allocator._insert_claim(sender, cell, 0, 5.0)
        first_hop = sender.allocator.build_acbba_messages(sender)
        forwarder._deliver_allocator_payload(first_hop[0])

        self.assertEqual(len(forwarder.allocator.build_acbba_messages(forwarder)), 1)
        self.assertEqual(forwarder.allocator.build_acbba_messages(forwarder), [])

    def test_table1_leave_and_rebroadcast_sends_local_belief(self) -> None:
        state = _state(grid_size=5, robot_ids=["00", "01", "02"])
        robot = state.robots["01"]
        robot.belief.add_clue((1, 1))
        cell = (2, 2)
        robot.allocator._ensure_acbba_state(robot)
        robot.acbba_winner_by_cell[cell] = "02"
        robot.acbba_winning_bid_by_cell[cell] = 10.0
        robot.acbba_bid_time_by_cell[cell] = 10.0

        robot._deliver_allocator_payload({
            "type": "acbba_entry",
            "sender": "00",
            "x": cell[0],
            "y": cell[1],
            "winner": "00",
            "bid": 5.0,
            "timestamp": 5.0,
        })
        messages = robot.allocator.build_acbba_messages(robot)

        self.assertEqual(robot.acbba_winner_by_cell[cell], "02")
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["sender"], "01")
        self.assertEqual(messages[0]["winner"], "02")
        self.assertEqual(messages[0]["bid"], 10.0)
        self.assertEqual(messages[0]["timestamp"], 10.0)

    def test_table1_reset_and_rebroadcast_forwards_incoming_entry(self) -> None:
        state = _state(grid_size=5, robot_ids=["00", "01", "02"])
        robot = state.robots["01"]
        robot.belief.add_clue((1, 1))
        cell = (2, 2)
        robot.allocator._ensure_acbba_state(robot)
        robot.acbba_winner_by_cell[cell] = "00"
        robot.acbba_winning_bid_by_cell[cell] = 5.0
        robot.acbba_bid_time_by_cell[cell] = 10.0

        robot._deliver_allocator_payload({
            "type": "acbba_entry",
            "sender": "00",
            "x": cell[0],
            "y": cell[1],
            "winner": "02",
            "bid": 7.0,
            "timestamp": 5.0,
        })
        messages = robot.allocator.build_acbba_messages(robot)

        self.assertIsNone(robot.acbba_winner_by_cell[cell])
        self.assertEqual(robot.acbba_winning_bid_by_cell[cell], ACBBAAllocator.NO_BID)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["sender"], "01")
        self.assertEqual(messages[0]["winner"], "02")
        self.assertEqual(messages[0]["bid"], 7.0)
        self.assertEqual(messages[0]["timestamp"], 5.0)

    def test_table1_update_time_and_rebroadcast_refreshes_self_claim(self) -> None:
        state = _state(grid_size=5, robot_ids=["00", "01"])
        robot = state.robots["01"]
        robot.belief.add_clue((1, 1))
        cell = (2, 2)
        robot.allocator._ensure_acbba_state(robot)
        robot.acbba_path = [cell]
        robot.acbba_bundle = [cell]
        robot.acbba_winner_by_cell[cell] = "01"
        robot.acbba_winning_bid_by_cell[cell] = 10.0
        robot.acbba_bid_time_by_cell[cell] = 1.0
        robot.acbba_bid_counter = 1

        robot._deliver_allocator_payload({
            "type": "acbba_entry",
            "sender": "00",
            "x": cell[0],
            "y": cell[1],
            "winner": "00",
            "bid": 5.0,
            "timestamp": 2.0,
        })
        messages = robot.allocator.build_acbba_messages(robot)

        self.assertEqual(robot.acbba_winner_by_cell[cell], "01")
        self.assertEqual(robot.acbba_winning_bid_by_cell[cell], 10.0)
        self.assertGreater(robot.acbba_bid_time_by_cell[cell], 1.0)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["winner"], "01")
        self.assertEqual(messages[0]["bid"], 10.0)
        self.assertIn("bundle_cells", messages[0])

    def test_table1_higher_bid_releases_suffix_and_clears_first_goal(self) -> None:
        state = _state(grid_size=5, robot_ids=["00", "01"])
        robot = state.robots["01"]
        robot.belief.add_clue((1, 1))
        first = (2, 2)
        second = (3, 2)
        robot.allocator._ensure_acbba_state(robot)
        robot.acbba_path = [first, second]
        robot.acbba_bundle = [first, second]
        robot.current_goal = first
        for idx, cell in enumerate((first, second), start=1):
            robot.acbba_winner_by_cell[cell] = "01"
            robot.acbba_winning_bid_by_cell[cell] = 10.0 - idx
            robot.acbba_bid_time_by_cell[cell] = float(idx)

        robot._deliver_allocator_payload({
            "type": "acbba_entry",
            "sender": "00",
            "x": first[0],
            "y": first[1],
            "winner": "00",
            "bid": 20.0,
            "timestamp": 10.0,
        })
        messages = robot.allocator.build_acbba_messages(robot)

        self.assertEqual(robot.acbba_path, [])
        self.assertIsNone(robot.current_goal)
        self.assertEqual(robot.acbba_winner_by_cell[first], "00")
        self.assertIsNone(robot.acbba_winner_by_cell[second])
        release_messages = [message for message in messages if (message["x"], message["y"]) == second]
        self.assertEqual(len(release_messages), 1)
        self.assertIsNone(release_messages[0]["winner"])


if __name__ == "__main__":
    unittest.main()
