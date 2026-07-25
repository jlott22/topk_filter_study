from __future__ import annotations

import unittest
from unittest.mock import patch

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
    @staticmethod
    def _entry(
        *,
        solution_id: str,
        generation: int,
        owner: str,
        order: int,
        cell: tuple[int, int],
        timestamp: float,
        path_size: int = 2,
    ) -> dict:
        return {
            "type": "dga_entry",
            "sender": "00",
            "solution_id": solution_id,
            "generation": generation,
            "fitness": 10.0,
            "owner": owner,
            "order": order,
            "path_size": path_size,
            "removed": False,
            "x": cell[0],
            "y": cell[1],
            "timestamp": timestamp,
        }

    def test_new_solution_sends_only_changed_full_owner_prefix(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        allocator = sender.allocator
        allocator._ensure_dga_state(sender)
        receiver.allocator._ensure_dga_state(receiver)

        old_plan = {
            "00": [(1, 0), (2, 0), (3, 0)],
            "01": [(1, 2)],
        }
        setattr(sender, "dga_generation", 1)
        setattr(sender, "dga_best_plan", old_plan)
        allocator._queue_dga_deltas(sender, old_plan, fitness=10.0)
        old_messages = allocator.build_dga_messages(sender)
        for message in old_messages:
            receiver.allocator.handle_dga_message(receiver, message)
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

        self.assertEqual({m["owner"] for m in new_messages}, {"00"})
        owner_messages = [m for m in new_messages if m["owner"] == "00"]
        self.assertEqual(len(owner_messages), 3)
        self.assertTrue(all(message["type"] == "dga_entry" for message in owner_messages))
        self.assertTrue(all(message["path_size"] == 3 for message in owner_messages))
        self.assertTrue(all(message["removed"] is False for message in owner_messages))
        self.assertEqual(
            [(m["order"], (m["x"], m["y"])) for m in owner_messages],
            [(0, (1, 0)), (1, (2, 2)), (2, (3, 0))],
        )
        for message in new_messages:
            receiver.allocator.handle_dga_message(receiver, message)
        reconstructed = receiver.allocator._reconstruct_received_solution(
            receiver,
            "00",
            new_messages[0]["solution_id"],
        )
        self.assertEqual(reconstructed["00"], new_plan["00"])
        self.assertEqual(reconstructed["01"], new_plan["01"])

    def test_outbound_delta_sequence_clears_disappeared_owners_once(self) -> None:
        state = _state()
        sender = state.robots["00"]
        allocator = sender.allocator
        allocator._ensure_dga_state(sender)
        sender.dga_last_sent_signatures = {
            "00": ((1, 0), (2, 0), (3, 0)),
            "01": ((4, 0),),
            "02": ((0, 4),),
        }
        plan = {
            "00": [(1, 0), (2, 0), (3, 0), (4, 0)],
            "01": [(4, 1), (4, 2)],
            "03": [],
        }
        sender.dga_generation = 12
        sender.dga_best_plan = plan

        allocator._queue_dga_deltas(sender, plan, fitness=12.345678901234567)
        messages = allocator.build_dga_messages(sender)

        self.assertEqual(
            [
                (
                    message["owner"],
                    message["order"],
                    message["path_size"],
                    message["removed"],
                    (
                        None
                        if message["x"] is None
                        else (message["x"], message["y"])
                    ),
                )
                for message in messages
            ],
            [
                ("01", 0, 2, False, (4, 1)),
                ("01", 1, 2, False, (4, 2)),
                ("02", 0, 0, True, None),
                ("03", 0, 0, True, None),
            ],
        )
        self.assertEqual(len({message["solution_id"] for message in messages}), 1)

        sender.dga_generation = 13
        allocator._queue_dga_deltas(sender, plan, fitness=11.0)
        self.assertEqual(allocator.build_dga_messages(sender), [])

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

    def test_empty_candidate_run_clears_advertised_prefixes_at_receiver(self) -> None:
        state = _state()
        sender = state.robots["00"]
        receiver = state.robots["01"]
        allocator = sender.allocator
        allocator._ensure_dga_state(sender)
        receiver.allocator._ensure_dga_state(receiver)

        old_plan = {
            "00": [(1, 0), (2, 0)],
            "01": [(1, 2)],
        }
        sender.dga_best_plan = old_plan
        sender.dga_generation = 4
        allocator._queue_dga_deltas(sender, old_plan, fitness=9.0)
        for message in allocator.build_dga_messages(sender):
            receiver.allocator.handle_dga_message(receiver, message)
        self.assertEqual(
            receiver.dga_received_latest_owner_prefix["00"]["00"],
            old_plan["00"],
        )
        self.assertEqual(
            receiver.dga_received_latest_owner_prefix["00"]["01"],
            old_plan["01"],
        )

        # Exercise the no-candidate branch with an already-empty stored plan.
        # Last-sent prefixes, rather than the stored best-plan signature, must
        # determine whether clears are needed.
        sender.dga_best_plan = {"00": []}
        with patch.object(allocator, "_candidate_cells", return_value=[]):
            allocator._run_dga(sender, "path_exhausted")
        clear_messages = allocator.build_dga_messages(sender)

        self.assertEqual(
            {message["owner"] for message in clear_messages},
            {"00", "01"},
        )
        self.assertTrue(
            all(message["removed"] for message in clear_messages)
        )
        self.assertTrue(
            all(message["fitness"] == 0.0 for message in clear_messages)
        )
        for message in clear_messages:
            receiver.allocator.handle_dga_message(receiver, message)
        self.assertEqual(
            receiver.dga_received_latest_owner_prefix["00"]["00"],
            [],
        )
        self.assertEqual(
            receiver.dga_received_latest_owner_prefix["00"]["01"],
            [],
        )

    def test_prediction_strikes_use_sender_order_zero_once_and_saturate(self) -> None:
        state = _state()
        receiver = state.robots["01"]
        allocator = receiver.allocator
        allocator._ensure_dga_state(receiver)
        receiver.dga_last_predicted_peer_first_task["00"] = (1, 0)

        allocator.handle_dga_message(
            receiver,
            self._entry(
                solution_id="other-owner",
                generation=1,
                owner="01",
                order=0,
                cell=(4, 0),
                timestamp=1.0,
            ),
        )
        allocator.handle_dga_message(
            receiver,
            self._entry(
                solution_id="later-order",
                generation=1,
                owner="00",
                order=1,
                cell=(4, 0),
                timestamp=2.0,
            ),
        )
        self.assertEqual(receiver.dga_bad_prediction_count, {})

        bad = self._entry(
            solution_id="bad-1",
            generation=2,
            owner="00",
            order=0,
            cell=(4, 0),
            timestamp=3.0,
        )
        allocator.handle_dga_message(receiver, bad)
        allocator.handle_dga_message(receiver, {**bad, "timestamp": 4.0, "x": 3})
        self.assertEqual(receiver.dga_bad_prediction_count["00"], 1)

        allocator.handle_dga_message(
            receiver,
            self._entry(
                solution_id="good",
                generation=3,
                owner="00",
                order=0,
                cell=(1, 0),
                timestamp=5.0,
            ),
        )
        self.assertEqual(receiver.dga_bad_prediction_count["00"], 0)

        for generation in range(4, 8):
            allocator.handle_dga_message(
                receiver,
                self._entry(
                    solution_id=f"bad-{generation}",
                    generation=generation,
                    owner="00",
                    order=0,
                    cell=(4, 0),
                    timestamp=float(generation + 2),
                ),
            )
        self.assertEqual(receiver.dga_bad_prediction_count["00"], allocator.BAD_PRED_LIMIT)

        allocator.handle_dga_message(
            receiver,
            self._entry(
                solution_id="good-after-exclusion",
                generation=8,
                owner="00",
                order=0,
                cell=(1, 0),
                timestamp=10.0,
            ),
        )
        self.assertEqual(receiver.dga_bad_prediction_count["00"], allocator.BAD_PRED_LIMIT)

        assessed = set(receiver.dga_last_assessed_peer_solution)
        allocator._reset_path_state(receiver)
        self.assertEqual(receiver.dga_bad_prediction_count["00"], allocator.BAD_PRED_LIMIT)
        self.assertEqual(receiver.dga_last_assessed_peer_solution, assessed)

        fresh_receiver = _state().robots["01"]
        fresh_receiver.allocator._ensure_dga_state(fresh_receiver)
        self.assertEqual(fresh_receiver.dga_bad_prediction_count, {})
        self.assertEqual(fresh_receiver.dga_last_assessed_peer_solution, set())

    def test_distinct_solution_ids_in_same_generation_are_each_assessed(self) -> None:
        state = _state()
        receiver = state.robots["01"]
        allocator = receiver.allocator
        allocator._ensure_dga_state(receiver)
        receiver.dga_last_predicted_peer_first_task["00"] = (1, 0)

        allocator.handle_dga_message(
            receiver,
            self._entry(
                solution_id="same-generation-bad",
                generation=1,
                owner="00",
                order=0,
                cell=(4, 0),
                timestamp=1.0,
            ),
        )
        self.assertEqual(receiver.dga_bad_prediction_count["00"], 1)

        allocator.handle_dga_message(
            receiver,
            self._entry(
                solution_id="same-generation-good",
                generation=1,
                owner="00",
                order=0,
                cell=(1, 0),
                timestamp=2.0,
            ),
        )
        self.assertEqual(receiver.dga_bad_prediction_count["00"], 0)
        self.assertEqual(
            receiver.dga_last_assessed_peer_solution,
            {
                ("00", 1, "same-generation-bad"),
                ("00", 1, "same-generation-good"),
            },
        )


if __name__ == "__main__":
    unittest.main()
