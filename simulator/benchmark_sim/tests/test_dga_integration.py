from __future__ import annotations

import unittest

from benchmark_sim.algorithms.DGA import DGAAllocator
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


def _state(grid_size: int = 5):
    cfg = SimConfig(
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
    scenario = TrialScenario(trial_id=0, target=(grid_size - 1, grid_size - 1), clues=[(1, 1)])
    runner = AsyncTrialRunner(cfg, DGAAllocator, make_comm_model("ideal", None), seed=0)
    state = runner.new_trial(scenario)
    for robot in state.robots.values():
        robot.belief.add_clue((1, 1))
    return state


class DGAIntegrationTests(unittest.TestCase):
    def test_new_solution_sends_full_prefix_as_cell_level_messages(self) -> None:
        state = _state()
        sender = state.robots["00"]
        allocator = sender.allocator
        allocator._ensure_dga_state(sender)

        old_plan = {
            "00": [(1, 0), (2, 0), (3, 0)],
            "01": [(1, 2)],
        }
        setattr(sender, "dga_generation", 1)
        setattr(sender, "dga_best_plan", old_plan)
        allocator._queue_dga_deltas(sender, old_plan, fitness=10.0)
        old_messages = allocator.build_dga_messages(sender)
        self.assertEqual(
            [(m["order"], (m["x"], m["y"])) for m in old_messages if m["owner"] == "00"],
            [(0, (1, 0)), (1, (2, 0)), (2, (3, 0))],
        )

        new_plan = {
            "00": [(1, 0), (2, 2), (3, 0)],
            "01": [(1, 2)],
        }
        setattr(sender, "dga_generation", 2)
        setattr(sender, "dga_best_plan", new_plan)
        allocator._queue_dga_deltas(sender, new_plan, fitness=9.0)
        new_messages = allocator.build_dga_messages(sender)

        owner_messages = [m for m in new_messages if m["owner"] == "00"]
        self.assertEqual(len(owner_messages), 3)
        self.assertTrue(all(message["type"] == "dga_entry" for message in owner_messages))
        self.assertTrue(all(message["path_size"] == 3 for message in owner_messages))
        self.assertTrue(all(message["removed"] is False for message in owner_messages))
        self.assertEqual(
            [(m["order"], (m["x"], m["y"])) for m in owner_messages],
            [(0, (1, 0)), (1, (2, 2)), (2, (3, 0))],
        )

    def test_received_full_prefix_reconstructs_new_solution_without_old_base(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        allocator = sender.allocator
        allocator._ensure_dga_state(sender)
        receiver.allocator._ensure_dga_state(receiver)

        plan = {
            "00": [(1, 0), (2, 2), (3, 0)],
            "01": [(1, 2)],
        }
        setattr(sender, "dga_generation", 2)
        setattr(sender, "dga_best_plan", plan)
        allocator._queue_dga_deltas(sender, plan, fitness=9.0)
        messages = allocator.build_dga_messages(sender)

        for message in messages:
            receiver.allocator.handle_dga_message(receiver, message)

        solution_id = messages[0]["solution_id"]
        reconstructed = receiver.allocator._reconstruct_received_solution(receiver, "00", solution_id)
        self.assertEqual(reconstructed["00"], [(1, 0), (2, 2), (3, 0)])
        self.assertEqual(reconstructed["01"], [(1, 2)])

    def test_empty_owner_prefix_sends_explicit_clear_message(self) -> None:
        state = _state()
        sender = state.robots["00"]
        allocator = sender.allocator
        allocator._ensure_dga_state(sender)

        plan = {
            "00": [],
            "01": [(1, 2)],
        }
        setattr(sender, "dga_generation", 3)
        setattr(sender, "dga_best_plan", plan)
        allocator._queue_dga_deltas(sender, plan, fitness=8.0)
        messages = allocator.build_dga_messages(sender)

        clear_messages = [m for m in messages if m["owner"] == "00"]
        self.assertEqual(len(clear_messages), 1)
        self.assertEqual(clear_messages[0]["path_size"], 0)
        self.assertTrue(clear_messages[0]["removed"])
        self.assertIsNone(clear_messages[0]["x"])
        self.assertIsNone(clear_messages[0]["y"])


if __name__ == "__main__":
    unittest.main()
