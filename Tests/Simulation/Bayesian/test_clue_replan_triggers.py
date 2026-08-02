from __future__ import annotations

import unittest

from benchmark_sim.algorithms.ACBBA import ACBBAAllocator
from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.algorithms.DGA import DGAAllocator
from benchmark_sim.algorithms.DMCHBA import DMCHBAAllocator
from benchmark_sim.algorithms.HIPC import HIPCAllocator
from benchmark_sim.algorithms.PI import PIAllocator
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


def _state(allocator_cls):
    cfg = SimConfig(
        grid_size=5,
        robot_ids=["00"],
        start_positions={"00": (0, 0)},
        start_headings={"00": EAST},
        async_initial_spread_s=0.0,
        async_step_jitter_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        collision_intent_settle_s=0.0,
        write_parquet=False,
    )
    scenario = TrialScenario(trial_id=0, target=(4, 4), clues=[(1, 1), (3, 3)])
    runner = AsyncTrialRunner(cfg, allocator_cls, make_comm_model("ideal", None), seed=0)
    return runner.new_trial(scenario)


class ClueReplanTriggerTests(unittest.TestCase):
    def test_later_clue_does_not_reset_allocator_state(self) -> None:
        cases = [
            (CBAAAllocator, "cbaa_current_task"),
            (ACBBAAllocator, "acbba_path"),
            (PIAllocator, "pi_path"),
            (HIPCAllocator, "hipc_path"),
            (DGAAllocator, "dga_path"),
        ]

        for allocator_cls, state_attr in cases:
            with self.subTest(allocator=allocator_cls.__name__):
                state = _state(allocator_cls)
                robot = state.robots["00"]
                robot.belief.add_clue((1, 1))
                first = robot.allocator.choose_goal(robot)
                self.assertIsNotNone(first.goal)

                before = getattr(robot, state_attr)
                before = list(before) if isinstance(before, list) else before
                robot.belief.add_clue((3, 3))

                robot.allocator._reset_if_new_clue_information(robot)

                after = getattr(robot, state_attr)
                after = list(after) if isinstance(after, list) else after
                self.assertEqual(after, before)

    def test_later_clue_does_not_trigger_dmchba_reassignment_while_path_remains(self) -> None:
        state = _state(DMCHBAAllocator)
        robot = state.robots["00"]
        robot.belief.add_clue((1, 1))
        first = robot.allocator.choose_goal(robot)
        self.assertEqual(first.debug["dmchba_trigger"], "clue_changed")
        self.assertGreater(len(robot.dmchba_path), 0)

        before = list(robot.dmchba_path)
        robot.belief.add_clue((3, 3))

        trigger = robot.allocator._post_clue_trigger(robot)

        self.assertIsNone(trigger)
        self.assertEqual(robot.dmchba_path, before)


if __name__ == "__main__":
    unittest.main()
