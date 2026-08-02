from __future__ import annotations

import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.comms.message import topic_for
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import AllocationDecision, TrialScenario
from benchmark_sim.metrics.summary import build_rows


class _NoGoalAllocator(AllocatorBase):
    def choose_goal(self, robot):
        return AllocationDecision(goal=None)


class MessageMetricTests(unittest.TestCase):
    def test_message_summary_counts_by_protection_class_topic_and_phase(self) -> None:
        cfg = SimConfig(
            grid_size=3,
            robot_ids=["00", "01"],
            start_positions={"00": (0, 0), "01": (2, 2)},
            write_parquet=False,
        )
        runner = AsyncTrialRunner(cfg, _NoGoalAllocator, make_comm_model("ideal", None), seed=0)
        state = runner.new_trial(TrialScenario(trial_id=0, target=(2, 0), clues=[]))

        state.bus.publish("00", topic_for("00", "state"), {"loc": [0, 0]}, 0.0)
        state.bus.publish("00", topic_for("00", "collision_intent"), {"loc": [0, 0], "intent": None}, 0.0)
        state.bus.publish("00", topic_for("00", "acbba_entry"), {"type": "acbba_entry"}, 1.0, post_clue=True)
        state.robots["00"].counters.steps_total = 4
        state.robots["01"].counters.steps_total = 1
        state.robots["00"].counters.steps_after_first_clue = 2
        state.robots["01"].counters.steps_after_first_clue = 1

        _trial_row, system_row, robot_rows = build_rows(state, "ACBBA", "ideal", "", "scenario.json")
        robot_row = next(row for row in robot_rows if row["robot_id"] == "00")

        self.assertEqual(system_row["messages_sent_total"], 3)
        self.assertEqual(system_row["protected_messages_sent_total"], 1)
        self.assertEqual(system_row["unprotected_messages_sent_total"], 2)
        self.assertEqual(system_row["core_messages_sent_total"], 2)
        self.assertEqual(system_row["allocation_messages_sent_total"], 1)
        self.assertEqual(system_row["post_clue_messages_sent_total"], 1)
        self.assertEqual(system_row["post_clue_allocation_messages_sent_total"], 1)
        self.assertAlmostEqual(system_row["messages_per_post_clue_step"], 1 / 3)
        self.assertAlmostEqual(system_row["allocation_messages_per_step"], 1 / 5)
        self.assertAlmostEqual(system_row["allocation_messages_per_post_clue_step"], 1 / 3)
        self.assertAlmostEqual(system_row["allocation_messages_per_unique_cell"], 1 / 2)
        self.assertEqual(system_row["messages_sent_by_topic"], "acbba_entry:1;collision_intent:1;state:1")

        self.assertEqual(robot_row["messages_sent"], 3)
        self.assertEqual(robot_row["protected_messages_sent"], 1)
        self.assertEqual(robot_row["unprotected_messages_sent"], 2)
        self.assertEqual(robot_row["core_messages_sent"], 2)
        self.assertEqual(robot_row["allocation_messages_sent"], 1)
        self.assertEqual(robot_row["post_clue_messages_sent"], 1)
        self.assertEqual(robot_row["messages_sent_by_topic"], "acbba_entry:1;collision_intent:1;state:1")

    def test_message_drop_fraction_uses_only_unprotected_attempts(self) -> None:
        cfg = SimConfig(
            grid_size=3,
            robot_ids=["00", "01"],
            start_positions={"00": (0, 0), "01": (2, 2)},
            write_parquet=False,
        )
        runner = AsyncTrialRunner(cfg, _NoGoalAllocator, make_comm_model("ideal", None), seed=0)
        state = runner.new_trial(TrialScenario(trial_id=0, target=(2, 0), clues=[]))

        state.bus.counters.delivered("01", protected=True)
        state.bus.counters.delivered("01", protected=True)
        state.bus.counters.delivered("01", protected=False)
        state.bus.counters.dropped("01")
        state.bus.counters.dropped("01")

        _trial_row, system_row, _robot_rows = build_rows(state, "ACBBA", "bernoulli", "drop_0.5", "scenario.json")

        self.assertEqual(state.bus.counters.delivered_total, 3)
        self.assertEqual(state.bus.counters.protected_delivered_total, 2)
        self.assertEqual(state.bus.counters.unprotected_delivered_total, 1)
        self.assertAlmostEqual(system_row["message_drop_fraction"], 2 / 3)


if __name__ == "__main__":
    unittest.main()
