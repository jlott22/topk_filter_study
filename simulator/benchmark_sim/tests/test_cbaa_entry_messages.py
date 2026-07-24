from __future__ import annotations

import unittest

from benchmark_sim.algorithms.CBAA import CBAAAllocator
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
    runner = AsyncTrialRunner(cfg, CBAAAllocator, make_comm_model("ideal", None), seed=0)
    return runner.new_trial(scenario)


class CBAAEntryMessageTests(unittest.TestCase):
    def test_pre_clue_generates_no_cbaa_messages(self) -> None:
        state = _state()
        robot = state.robots["00"]

        messages = robot.allocator.build_cbaa_messages(robot)

        self.assertEqual(messages, [])

    def test_first_post_clue_assignment_generates_one_claim(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)

        messages = robot.allocator.build_cbaa_messages(robot)

        self.assertEqual(len(messages), 1)
        self.assertTrue(all(message["type"] == "cbaa_entry" for message in messages))
        self.assertTrue(all({"sender", "x", "y", "winner", "bid"} <= set(message) for message in messages))
        self.assertEqual(messages[0]["winner"], "00")

    def test_same_task_generates_no_second_claim(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first_goal = robot.allocator.choose_goal(robot).goal
        self.assertIsNotNone(first_goal)
        self.assertEqual(len(robot.allocator.build_cbaa_messages(robot)), 1)

        second_goal = robot.allocator.choose_goal(robot).goal
        messages = robot.allocator.build_cbaa_messages(robot)

        self.assertEqual(second_goal, first_goal)
        self.assertEqual(messages, [])

    def test_collision_state_triggers_current_task_reevaluation(self) -> None:
        state = _state(grid_size=4, robot_ids=["00"])
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        for cell in list(robot.belief.target_p.keys()):
            robot.belief.target_p[cell] = 0.0
        robot.allocator.choose_goal(robot)

        stale_task = (3, 3)
        robot.allocator._claim_cell(robot, stale_task, robot.allocator._bid(robot, stale_task))

        held = robot.allocator.choose_goal(robot)
        self.assertEqual(held.goal, stale_task)
        self.assertIsNone(held.debug["cbaa_trigger"])

        robot.collision_avoidance_active = True
        replanned = robot.allocator.choose_goal(robot)

        self.assertEqual(replanned.debug["cbaa_trigger"], "collision_avoidance")
        self.assertIsNotNone(replanned.goal)
        self.assertNotEqual(replanned.goal, stale_task)
        self.assertEqual(getattr(robot, "cbaa_current_task"), replanned.goal)

    def test_switching_task_generates_release_and_new_claim(self) -> None:
        state = _state(grid_size=3)
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first_goal = robot.allocator.choose_goal(robot).goal
        self.assertIsNotNone(first_goal)
        self.assertEqual(len(robot.allocator.build_cbaa_messages(robot)), 1)
        robot.belief.mark_searched(first_goal)

        second_goal = robot.allocator.choose_goal(robot).goal
        messages = robot.allocator.build_cbaa_messages(robot)

        self.assertIsNotNone(second_goal)
        self.assertNotEqual(second_goal, first_goal)
        self.assertEqual(len(messages), 2)
        by_cell = {(message["x"], message["y"]): message for message in messages}
        self.assertIsNone(by_cell[first_goal]["winner"])
        self.assertEqual(by_cell[first_goal]["released_winner"], "00")
        self.assertEqual(by_cell[(second_goal[0], second_goal[1])]["winner"], "00")

    def test_same_task_retransmits_when_bid_changes(self) -> None:
        state = _state(grid_size=4)
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        claim = (2, 2)
        robot.allocator._claim_cell(robot, claim, robot.allocator._bid(robot, claim))
        first = robot.allocator.build_cbaa_messages(robot)
        self.assertEqual(len(first), 1)

        robot.pos = (1, 2)
        second = robot.allocator.build_cbaa_messages(robot)
        third = robot.allocator.build_cbaa_messages(robot)

        self.assertEqual(len(second), 1)
        self.assertEqual((second[0]["x"], second[0]["y"]), claim)
        self.assertNotEqual(second[0]["bid"], first[0]["bid"])
        self.assertEqual(third, [])

    def test_outbound_entries_are_counted_individually(self) -> None:
        state = _state()
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        robot.allocator.choose_goal(robot)

        robot._publish_allocator_messages()

        self.assertEqual(state.bus.counters.sent_by_robot["00"], 1)
        self.assertEqual(state.bus.counters.sent_total, 1)

    def test_inbound_entry_updates_consensus_map(self) -> None:
        state = _state(grid_size=3)
        receiver = state.robots["01"]
        receiver.belief.add_clue((1, 1))
        payload = {
            "type": "cbaa_entry",
            "sender": "00",
            "x": 2,
            "y": 1,
            "winner": "00",
            "bid": 7.5,
        }

        receiver._deliver_allocator_payload(payload)

        winners = getattr(receiver, "cbaa_winner_by_cell")
        bids = getattr(receiver, "cbaa_winning_bid_by_cell")
        self.assertEqual(winners[(2, 1)], "00")
        self.assertEqual(bids[(2, 1)], 7.5)

    def test_inbound_claim_clears_winner_old_claim(self) -> None:
        state = _state(grid_size=3)
        receiver = state.robots["01"]
        receiver.belief.add_clue((1, 1))
        receiver._deliver_allocator_payload({
            "type": "cbaa_entry",
            "sender": "00",
            "x": 1,
            "y": 0,
            "winner": "00",
            "bid": 5.0,
        })
        receiver._deliver_allocator_payload({
            "type": "cbaa_entry",
            "sender": "00",
            "x": 2,
            "y": 1,
            "winner": "00",
            "bid": 7.5,
        })

        winners = getattr(receiver, "cbaa_winner_by_cell")
        bids = getattr(receiver, "cbaa_winning_bid_by_cell")
        self.assertIsNone(winners[(1, 0)])
        self.assertEqual(bids[(1, 0)], CBAAAllocator.NO_BID)
        self.assertEqual(winners[(2, 1)], "00")
        self.assertEqual(bids[(2, 1)], 7.5)

    def test_receiver_forwards_changed_peer_claim(self) -> None:
        state = _state(grid_size=3, robot_ids=["00", "01", "02"])
        sender = state.robots["00"]
        forwarder = state.robots["01"]
        receiver = state.robots["02"]
        for robot in (sender, forwarder, receiver):
            robot.belief.add_clue((1, 1))

        claim = (2, 2)
        sender.allocator._claim_cell(sender, claim, sender.allocator._bid(sender, claim))
        first_hop = sender.allocator.build_cbaa_messages(sender)
        self.assertEqual(len(first_hop), 1)

        forwarder._deliver_allocator_payload(first_hop[0])
        forwarded = forwarder.allocator.build_cbaa_messages(forwarder)

        self.assertEqual(len(forwarded), 1)
        self.assertEqual(forwarded[0]["sender"], "01")
        self.assertEqual(forwarded[0]["winner"], "00")
        self.assertEqual((forwarded[0]["x"], forwarded[0]["y"]), (first_hop[0]["x"], first_hop[0]["y"]))
        self.assertEqual(forwarded[0]["bid"], first_hop[0]["bid"])

        receiver._deliver_allocator_payload(forwarded[0])
        winners = getattr(receiver, "cbaa_winner_by_cell")
        bids = getattr(receiver, "cbaa_winning_bid_by_cell")
        cell = (forwarded[0]["x"], forwarded[0]["y"])
        self.assertEqual(winners[cell], "00")
        self.assertEqual(bids[cell], forwarded[0]["bid"])

    def test_forwarded_claim_is_not_resent_when_unchanged(self) -> None:
        state = _state(grid_size=3, robot_ids=["00", "01", "02"])
        sender = state.robots["00"]
        forwarder = state.robots["01"]
        for robot in (sender, forwarder):
            robot.belief.add_clue((1, 1))

        claim = (2, 2)
        sender.allocator._claim_cell(sender, claim, sender.allocator._bid(sender, claim))
        first_hop = sender.allocator.build_cbaa_messages(sender)
        forwarder._deliver_allocator_payload(first_hop[0])

        self.assertEqual(len(forwarder.allocator.build_cbaa_messages(forwarder)), 1)
        self.assertEqual(forwarder.allocator.build_cbaa_messages(forwarder), [])

    def test_higher_received_bid_clears_current_task(self) -> None:
        state = _state(grid_size=3)
        robot = state.robots["01"]
        robot.belief.add_clue((1, 1))
        cell = (2, 1)
        robot.allocator._claim_cell(robot, cell, 1.0)
        robot.current_goal = cell
        self.assertEqual(len(robot.allocator.build_cbaa_messages(robot)), 1)

        robot._deliver_allocator_payload({
            "type": "cbaa_entry",
            "sender": "00",
            "x": 2,
            "y": 1,
            "winner": "00",
            "bid": 5.0,
        })

        self.assertIsNone(getattr(robot, "cbaa_current_task"))
        self.assertIsNone(robot.current_goal)
        self.assertEqual(getattr(robot, "cbaa_winner_by_cell")[cell], "00")

    def test_release_clears_only_matching_stale_owner(self) -> None:
        state = _state(grid_size=3, robot_ids=["00", "01", "02"])
        receiver = state.robots["02"]
        receiver.belief.add_clue((1, 1))
        cell = (1, 2)
        receiver.allocator._ensure_cbaa_state(receiver)
        winners = getattr(receiver, "cbaa_winner_by_cell")
        bids = getattr(receiver, "cbaa_winning_bid_by_cell")

        winners[cell] = "02"
        bids[cell] = 10.0
        receiver._deliver_allocator_payload({
            "type": "cbaa_entry",
            "sender": "01",
            "x": cell[0],
            "y": cell[1],
            "winner": None,
            "bid": CBAAAllocator.NO_BID,
            "released_winner": "00",
            "released_bid": 5.0,
        })
        self.assertEqual(winners[cell], "02")
        self.assertEqual(bids[cell], 10.0)

        winners[cell] = "00"
        bids[cell] = 5.0
        receiver._deliver_allocator_payload({
            "type": "cbaa_entry",
            "sender": "01",
            "x": cell[0],
            "y": cell[1],
            "winner": None,
            "bid": CBAAAllocator.NO_BID,
            "released_winner": "00",
            "released_bid": 5.0,
        })
        self.assertIsNone(winners[cell])
        self.assertEqual(bids[cell], CBAAAllocator.NO_BID)


if __name__ == "__main__":
    unittest.main()
