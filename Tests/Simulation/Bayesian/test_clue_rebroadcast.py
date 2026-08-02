from __future__ import annotations

import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.comms.message import Message, topic_for
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import AllocationDecision, Cell, TrialScenario


class _NoGoalAllocator(AllocatorBase):
    def choose_goal(self, robot):
        return AllocationDecision(goal=None)


class _FixedGoalAllocator(AllocatorBase):
    goal: Cell = (1, 0)

    def choose_goal(self, robot):
        return AllocationDecision(goal=self.goal)


def _cfg() -> SimConfig:
    return SimConfig(
        grid_size=5,
        robot_ids=["00", "01", "02", "03"],
        start_positions={
            "00": (0, 0),
            "01": (0, 1),
            "02": (0, 2),
            "03": (0, 3),
        },
        start_headings={
            "00": EAST,
            "01": EAST,
            "02": EAST,
            "03": EAST,
        },
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        write_parquet=False,
    )


def _state(comm_model="ideal", comm_level=None):
    cfg = _cfg()
    scenario = TrialScenario(trial_id=0, target=(4, 4), clues=[(1, 1)])
    runner = AsyncTrialRunner(cfg, _NoGoalAllocator, make_comm_model(comm_model, comm_level), seed=0)
    return runner.new_trial(scenario)


class ClueRebroadcastTests(unittest.TestCase):
    def test_personal_detection_broadcasts_and_triggers_bounded_forwarding(self) -> None:
        cfg = _cfg()
        cfg.async_initial_spread_s = 0.0
        cfg.collision_intent_settle_s = 0.0
        scenario = TrialScenario(trial_id=0, target=(4, 4), clues=[(1, 0)])
        runner = AsyncTrialRunner(cfg, _FixedGoalAllocator, make_comm_model("ideal", None), seed=0)
        state = runner.new_trial(scenario)
        queue = runner.initial_queue(state)
        order = len(queue)
        published = []
        original_publish = state.bus.publish

        def record_publish(sender, topic, payload, now_s, post_clue=False):
            published.append((sender, topic.rstrip("/").split("/")[-1], dict(payload)))
            original_publish(sender, topic, payload, now_s, post_clue=post_clue)

        state.bus.publish = record_publish

        processed, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(processed)
        self.assertEqual(processed.rid, "00")
        self.assertTrue(processed.result.found_clue)

        state.bus.pump(state.clock_s)

        clue_sends = {}
        for sender, category, _payload in published:
            if category == "clue":
                clue_sends[sender] = clue_sends.get(sender, 0) + 1

        self.assertEqual(clue_sends, {
            "00": 1,
            "01": 1,
            "02": 1,
            "03": 1,
        })
        for robot in state.robots.values():
            self.assertIn((1, 0), robot.known_clues)
            self.assertIn((1, 0), robot.forwarded_clues)

    def test_new_clue_is_forwarded_once_by_each_robot(self) -> None:
        state = _state()
        clue = (1, 1)
        origin = state.robots["00"]

        self.assertTrue(origin.belief.add_clue(clue))
        origin.publish_clue(clue)
        state.bus.pump(0.0)

        self.assertEqual(state.bus.counters.sent_total, 4)
        self.assertEqual(state.bus.counters.sent_by_robot, {
            "00": 1,
            "01": 1,
            "02": 1,
            "03": 1,
        })
        for robot in state.robots.values():
            self.assertIn(clue, robot.known_clues)
            self.assertIn(clue, robot.forwarded_clues)

    def test_duplicate_clue_reception_does_not_rebroadcast(self) -> None:
        state = _state()
        clue = (1, 1)
        receiver = state.robots["01"]
        receiver.belief.add_clue(clue)
        receiver.forwarded_clues.add(clue)
        before = state.bus.counters.sent_total

        receiver.receive_message(Message(
            sender="00",
            topic=topic_for("00", "clue"),
            payload={"loc": list(clue)},
            created_at_s=0.0,
        ))

        self.assertEqual(state.bus.counters.sent_total, before)

    def test_clue_rebroadcast_uses_degraded_communication_model(self) -> None:
        state = _state(comm_model="bernoulli", comm_level=1.0)
        clue = (1, 1)
        origin = state.robots["00"]

        self.assertTrue(origin.belief.add_clue(clue))
        origin.publish_clue(clue)
        state.bus.pump(0.0)

        self.assertEqual(state.bus.counters.sent_total, 1)
        self.assertEqual(state.bus.counters.delivered_total, 0)
        self.assertEqual(state.bus.counters.dropped_total, 3)
        self.assertNotIn(clue, state.robots["01"].known_clues)
        self.assertNotIn(clue, state.robots["02"].known_clues)
        self.assertNotIn(clue, state.robots["03"].known_clues)


if __name__ == "__main__":
    unittest.main()
