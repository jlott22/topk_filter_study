"""Focused desktop regressions for DMCHBA/HIPC/PI hardware parity."""

from __future__ import annotations

import ast
import threading
import unittest
from array import array
from pathlib import Path


HARDWARE_DIR = Path(__file__).resolve().parent


def _extract(path, names, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    nodes = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    missing = names - {node.name for node in nodes}
    if missing:
        raise AssertionError("{} lacks {}".format(path.name, sorted(missing)))
    module = ast.Module(body=nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(path), "exec"), namespace)
    return namespace


class DHPSimulatorParityTests(unittest.TestCase):
    def test_probability_normalizer_preserves_tiny_finite_positive_maximum(self):
        cases = (
            ([0.0, 0.5e-9, 0.0, 0.0], 0.5e-9),
            ([0.0, 0.0, 0.0, 0.0], 1.0),
            ([float("inf"), 0.5, 0.0, 0.0], 1.0),
            ([float("nan")] * 4, 1.0),
        )
        for filename in (
            "Pololu_DMCHBA.py",
            "Pololu_HIPC.py",
            "Pololu_PI.py",
        ):
            for values, expected in cases:
                with self.subTest(file=filename, values=values):
                    namespace = {
                        "GRID_SIZE": 2,
                        "EPS": 1.0e-9,
                        "target_p": array("d", values),
                        "prob_map": array("d", [0.0] * 4),
                        "allocation_probability_normalizer": 1.0,
                    }
                    _extract(
                        HARDWARE_DIR / filename,
                        {"recompute_value_map"},
                        namespace,
                    )

                    namespace["recompute_value_map"]()

                    self.assertEqual(
                        namespace["allocation_probability_normalizer"],
                        expected,
                    )

    def test_first_protected_clear_carries_current_cell_and_dedupes_by_signature(self):
        for filename in (
            "Pololu_DMCHBA.py",
            "Pololu_HIPC.py",
            "Pololu_PI.py",
        ):
            with self.subTest(file=filename):
                tx_buf = bytearray(64)
                sent = []

                def uart_send(topic, payload_length):
                    sent.append(
                        (topic, bytes(tx_buf[2 : 2 + payload_length]))
                    )

                namespace = {
                    "pos": [2, 3],
                    "communicated_intent": None,
                    "topic_2_sent": 0,
                    "metrics_frozen": False,
                    "tx_buf": tx_buf,
                    "uart_tx_lock": threading.Lock(),
                    "uart_send": uart_send,
                }
                _extract(
                    HARDWARE_DIR / filename,
                    {"_write_int", "publish_intent"},
                    namespace,
                )

                self.assertTrue(namespace["publish_intent"]())
                self.assertEqual(sent, [("2", b"2,3,X,X")])
                self.assertEqual(namespace["topic_2_sent"], 1)
                self.assertEqual(
                    namespace["communicated_intent"],
                    ((2, 3), None),
                )

                self.assertFalse(namespace["publish_intent"]())
                self.assertEqual(len(sent), 1)

                namespace["pos"][1] = 4
                self.assertTrue(namespace["publish_intent"]())
                self.assertEqual(sent[-1], ("2", b"2,4,X,X"))
                self.assertEqual(namespace["topic_2_sent"], 2)

    def test_hipc_empty_clear_does_not_reset_prediction_deduplication(self):
        peer = "01"
        advertised = (2, 2)
        namespace = {
            "ROBOT_ID": "03",
            "HIPC_NO_BID": -1.0e18,
            "HIPC_NO_TIME": -1.0e18,
            "HIPC_PREDICTION_TOLERANCE": 0,
            "hipc_seen_peer_bundle_signature": {},
            "hipc_last_predicted_peer_first_task": {
                peer: (0, 0)
            },
            "hipc_bad_prediction_count": {},
            "hipc_winner_by_cell": {advertised: peer},
            "hipc_winning_bid_by_cell": {advertised: -2.0},
            "hipc_bid_time_by_cell": {advertised: 1.0},
        }
        _extract(
            HARDWARE_DIR / "Pololu_HIPC.py",
            {
                "manhattan",
                "_same_robot_id",
                "_hipc_update_prediction",
                "_hipc_clear_sender_claims_not_in_bundle",
            },
            namespace,
        )

        namespace["_hipc_update_prediction"](peer, [advertised])
        self.assertEqual(namespace["hipc_bad_prediction_count"][peer], 1)
        self.assertEqual(
            namespace["hipc_seen_peer_bundle_signature"][peer],
            (advertised,),
        )

        namespace["_hipc_update_prediction"](peer, [])
        namespace["_hipc_clear_sender_claims_not_in_bundle"](
            peer, []
        )
        self.assertIsNone(
            namespace["hipc_winner_by_cell"][advertised]
        )
        self.assertEqual(
            namespace["hipc_seen_peer_bundle_signature"][peer],
            (advertised,),
        )

        namespace["_hipc_update_prediction"](peer, [advertised])
        self.assertEqual(namespace["hipc_bad_prediction_count"][peer], 1)

    def test_dropped_position_cannot_update_protected_peer_position(self):
        class RandomStub:
            @staticmethod
            def random():
                return 0.0

        for filename in (
            "Pololu_DMCHBA.py",
            "Pololu_HIPC.py",
            "Pololu_PI.py",
        ):
            with self.subTest(file=filename):
                grid_size = 5
                namespace = {
                    "ROBOT_ID": "01",
                    "GRID_SIZE": grid_size,
                    "CELL_SEARCHED": 1,
                    "grid": bytearray(grid_size * grid_size),
                    "pos": [0, 0],
                    "pre_start_signal": False,
                    "peer_intent": {},
                    "peer_pos": {},
                    "peer_pos_yield": {},
                    "current_task_cell": None,
                    "first_clue_seen": False,
                    "target_location": None,
                    "start_signal": True,
                    "found_target": False,
                    "move_forward_flag": False,
                    "communicated_intent": None,
                    "metrics_frozen": False,
                    "msg_drop_rate": 1.0,
                    "topic_1_rec": 0,
                    "random": RandomStub,
                    "idx": lambda x, y: y * grid_size + x,
                    "update_target_on_miss": lambda _cell_index: None,
                }
                _extract(
                    HARDWARE_DIR / filename,
                    {"_trial_traffic_enabled", "handle_msg"},
                    namespace,
                )

                namespace["handle_msg"]("021.2,3")
                self.assertEqual(namespace["peer_pos"], {})
                self.assertEqual(namespace["peer_pos_yield"], {})
                self.assertEqual(namespace["topic_1_rec"], 0)

                namespace["msg_drop_rate"] = 0.0
                namespace["handle_msg"]("021.2,3")
                self.assertEqual(namespace["peer_pos"], {"02": (2, 3)})
                self.assertEqual(namespace["peer_pos_yield"], {})
                self.assertEqual(namespace["topic_1_rec"], 1)

    def test_calibration_does_not_publish_start_cell_as_normal_state(self):
        for filename in (
            "Pololu_DMCHBA.py",
            "Pololu_HIPC.py",
            "Pololu_PI.py",
        ):
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                calibrate = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "calibrate"
                )
                calls = {
                    node.func.id
                    for node in ast.walk(calibrate)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertNotIn("publish_position", calls)

    def test_dmchba_eps_score_tie_prefers_route_distance_before_cell(self):
        """Mirror DMCHBAAllocator._order_assigned_cells tie ordering."""

        path = HARDWARE_DIR / "Pololu_DMCHBA.py"
        grid_size = 5
        far_cell = (2, 0)
        near_cell = (1, 0)
        far_id = far_cell[1] * grid_size + far_cell[0]
        near_id = near_cell[1] * grid_size + near_cell[0]
        costs = {
            far_cell: 1.0,
            # The nearer cell is slightly worse, but still inside the EPS tie.
            near_cell: 1.0 + 0.5e-9,
        }
        namespace = {
            "GRID_SIZE": grid_size,
            "DMCHBA_COMMITMENT_HORIZON": 3,
            "DMCHBA_TIE_EPS": 1.0e-9,
            "pos": [0, 0],
            "dmchba_assigned_ids": array("H", [far_id, near_id, 0]),
            "_dmchba_cost": lambda _reference, cell: costs[cell],
        }
        _extract(
            path,
            {"manhattan", "_dmchba_order_assigned_ids"},
            namespace,
        )

        ordered = namespace["_dmchba_order_assigned_ids"](2)

        self.assertEqual(ordered[0], near_cell)
        self.assertEqual(ordered[1], far_cell)

    def test_second_same_goal_conflict_blocks_reallocation_through_backoff(self):
        class Clock:
            now = 100

            @classmethod
            def ticks_ms(cls):
                return cls.now

            @staticmethod
            def ticks_add(value, delta):
                return value + delta

            @staticmethod
            def ticks_diff(left, right):
                return left - right

        cases = {
            "Pololu_DMCHBA.py": "_dmchba_valid_task",
            "Pololu_HIPC.py": "_hipc_valid_task",
            "Pololu_PI.py": "_pi_valid_task",
        }
        for filename, valid_name in cases.items():
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                run_active = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "run_active_trial"
                )
                called = {
                    node.func.id
                    for node in ast.walk(run_active)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertIn("_register_goal_conflict", called)
                self.assertIn("_temporarily_invalidate_task", called)

                grid_size = 3
                cell = (1, 1)
                namespace = {
                    "time": Clock,
                    "GRID_SIZE": grid_size,
                    "CELL_UNSEARCHED": 0,
                    "grid": bytearray(grid_size * grid_size),
                    "temporary_invalid_task_until": {},
                    "blocked_goal_cell": None,
                    "blocked_goal_conflicts": 0,
                    "safe_assert": lambda condition, message: (
                        None
                        if condition
                        else (_ for _ in ()).throw(AssertionError(message))
                    ),
                }
                _extract(
                    path,
                    {
                        "idx",
                        valid_name,
                        "_register_goal_conflict",
                        "_temporarily_invalidate_task",
                        "_task_temporarily_invalid",
                    },
                    namespace,
                )

                self.assertEqual(
                    namespace["_register_goal_conflict"](cell), 1
                )
                self.assertEqual(
                    namespace["_register_goal_conflict"](cell), 2
                )
                namespace["_temporarily_invalidate_task"](cell, 0)
                self.assertEqual(
                    namespace["temporary_invalid_task_until"][cell], 101
                )
                self.assertFalse(namespace[valid_name](cell))

                Clock.now = 101
                self.assertTrue(namespace[valid_name](cell))
                self.assertNotIn(
                    cell, namespace["temporary_invalid_task_until"]
                )
                Clock.now = 100


class PICompletionCadenceTests(unittest.TestCase):
    @staticmethod
    def _pi_namespace():
        from hardware.test_allocator_memory_optimized_equivalence import (
            _namespace,
        )

        namespace = _namespace("pi", True, 5, 1.0, 23, 4)
        namespace["candidate_filter_time_us_total"] = 0
        grid_size = namespace["GRID_SIZE"]
        for index in range(grid_size * grid_size):
            namespace["grid"][index] = namespace["CELL_UNSEARCHED"]
            namespace["target_p"][index] = 1.0 / (
                grid_size * grid_size
            )
            namespace["prob_map"][index] = namespace["target_p"][index]
        namespace["pi_clue_signature"] = tuple(
            sorted(set(namespace["clues"]))
        )
        return namespace

    @staticmethod
    def _seed_path(namespace, path):
        namespace["pi_path"][:] = path
        namespace["pi_bundle"][:] = path
        for index, cell in enumerate(path):
            namespace["pi_owner_by_cell"][cell] = namespace["ROBOT_ID"]
            namespace["pi_significance_by_cell"][cell] = 999.0 + index
            namespace["pi_time_by_cell"][cell] = index + 1.0

    def test_completed_head_repairs_and_refills_after_belief_update(self):
        namespace = self._pi_namespace()
        head = (1, 0)
        suffix = (2, 0)
        self._seed_path(namespace, [head, suffix])
        namespace["pos"][:] = head
        namespace["current_task_cell"] = head
        events = []

        def update_belief(cell_index):
            events.append(("belief", cell_index))
            grid_size = namespace["GRID_SIZE"]
            idx = namespace["idx"]
            for y in range(grid_size):
                for x in range(grid_size):
                    index = idx(x, y)
                    namespace["target_p"][index] = (
                        0.0
                        if namespace["grid"][index]
                        == namespace["CELL_SEARCHED"]
                        else float(1 + x + 2 * y)
                    )
            total = sum(namespace["target_p"])
            for index in range(grid_size * grid_size):
                namespace["target_p"][index] /= total
            namespace["allocation_probability_normalizer"] = max(
                namespace["target_p"]
            )

        original_refresh = namespace["_pi_refresh_local_entries"]

        def refresh_significance(normalizer=None):
            self.assertTrue(events)
            self.assertEqual(events[0][0], "belief")
            events.append(("significance", tuple(namespace["pi_path"])))
            return original_refresh(normalizer)

        namespace["update_target_on_miss"] = update_belief
        namespace["_pi_refresh_local_entries"] = refresh_significance

        completed = namespace["_pi_record_arrival"](head)

        self.assertTrue(completed)
        self.assertIsNone(namespace["current_task_cell"])
        self.assertEqual(namespace["pi_path"], [head, suffix])
        self.assertEqual(namespace["pi_bundle"], [head, suffix])
        self.assertEqual(
            namespace["grid"][namespace["idx"](*head)],
            namespace["CELL_SEARCHED"],
        )
        self.assertEqual([event[0] for event in events], ["belief"])

        selected = namespace["_pick_task_cell_impl"]()

        self.assertIsNotNone(selected)
        self.assertEqual(selected, namespace["pi_path"][0])
        self.assertNotIn(head, namespace["pi_path"])
        self.assertIn(suffix, namespace["pi_path"])
        self.assertEqual(
            len(namespace["pi_path"]), namespace["PI_BUNDLE_SIZE"]
        )
        self.assertEqual(namespace["pi_bundle"], namespace["pi_path"])
        self.assertIn("significance", [event[0] for event in events])

        normalizer = namespace["_pi_probability_normalizer"]()
        full_cost = namespace["_pi_route_cost"](
            namespace["pi_path"], normalizer
        )
        for index, cell in enumerate(namespace["pi_path"]):
            expected = max(
                0.0,
                full_cost
                - namespace["_pi_route_cost_without_index"](
                    namespace["pi_path"], index, normalizer
                ),
            )
            self.assertAlmostEqual(
                namespace["pi_significance_by_cell"][cell],
                expected,
                places=14,
            )

    def test_crossing_later_task_does_not_mutate_allocator_state(self):
        namespace = self._pi_namespace()
        head = (4, 4)
        crossed_later_task = (1, 0)
        third = (3, 3)
        path = [head, crossed_later_task, third]
        self._seed_path(namespace, path)
        namespace["current_task_cell"] = head
        namespace["pi_pending_snapshot"] = False
        namespace["pi_last_sent_signature"] = ("stable",)
        belief_updates = []
        namespace["update_target_on_miss"] = belief_updates.append

        before = {
            "path": list(namespace["pi_path"]),
            "bundle": list(namespace["pi_bundle"]),
            "owners": dict(namespace["pi_owner_by_cell"].items()),
            "significance": dict(
                namespace["pi_significance_by_cell"].items()
            ),
            "times": dict(namespace["pi_time_by_cell"].items()),
            "pending": namespace["pi_pending_snapshot"],
            "last_sent": namespace["pi_last_sent_signature"],
            "messages": list(namespace["_sent"]),
        }

        completed = namespace["_pi_record_arrival"](
            crossed_later_task
        )

        self.assertFalse(completed)
        self.assertEqual(namespace["current_task_cell"], head)
        self.assertEqual(namespace["pi_path"], before["path"])
        self.assertEqual(namespace["pi_bundle"], before["bundle"])
        self.assertEqual(
            dict(namespace["pi_owner_by_cell"].items()),
            before["owners"],
        )
        self.assertEqual(
            dict(namespace["pi_significance_by_cell"].items()),
            before["significance"],
        )
        self.assertEqual(
            dict(namespace["pi_time_by_cell"].items()),
            before["times"],
        )
        self.assertEqual(namespace["pi_pending_snapshot"], before["pending"])
        self.assertEqual(
            namespace["pi_last_sent_signature"], before["last_sent"]
        )
        self.assertEqual(namespace["_sent"], before["messages"])
        self.assertEqual(
            belief_updates,
            [namespace["idx"](*crossed_later_task)],
        )

        tree = ast.parse(
            (HARDWARE_DIR / "Pololu_PI.py").read_text(encoding="utf-8")
        )
        run_active = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "run_active_trial"
        )
        called = {
            node.func.id
            for node in ast.walk(run_active)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
        }
        self.assertIn("_pi_complete_cell_arrival", called)
        self.assertNotIn("_pi_clear_invalid_or_completed_cells", called)

        complete_arrival = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_pi_complete_cell_arrival"
        )
        arrival_calls = {
            node.func.id
            for node in ast.walk(complete_arrival)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
        }
        self.assertIn("_pi_record_arrival", arrival_calls)
        self.assertNotIn("publish_intent", arrival_calls)

    def test_consensus_head_loss_still_synchronizes_wrapper_goal(self):
        namespace = self._pi_namespace()
        old_head = (1, 0)
        retained_suffix = (2, 0)
        self._seed_path(namespace, [old_head, retained_suffix])
        namespace["current_task_cell"] = old_head
        namespace["pi_owner_by_cell"][old_head] = "01"

        namespace["_pi_repair_after_consensus"]()

        self.assertEqual(namespace["pi_path"], [retained_suffix])
        self.assertEqual(namespace["current_task_cell"], retained_suffix)


if __name__ == "__main__":
    unittest.main()
