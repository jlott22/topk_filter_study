"""Focused desktop regressions for ACBBA, CBAA, and DGA hardware parity.

The mission programs initialize physical hardware at module scope, so these
tests execute only AST-extracted pure functions and classes.
"""

from __future__ import annotations

import ast
import random
import unittest
from array import array
from pathlib import Path
from types import SimpleNamespace

from hardware.allocator_memory import PackedCandidateWorkspace
from hardware.test_dga_simulator_equivalence import (
    CANDIDATES,
    TEAM,
    _hardware_namespace,
)


HARDWARE_DIR = Path(__file__).resolve().parent
ACBBA_PATH = HARDWARE_DIR / "Pololu_ACBBA.py"
CBAA_PATH = HARDWARE_DIR / "Pololu_CBAA.py"
DGA_PATH = HARDWARE_DIR / "Pololu_DGA.py"


def _extract(path, function_names=(), class_names=(), namespace=None):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function_names = set(function_names)
    class_names = set(class_names)
    nodes = [
        node
        for node in tree.body
        if (
            isinstance(node, ast.FunctionDef)
            and node.name in function_names
        )
        or (
            isinstance(node, ast.ClassDef)
            and node.name in class_names
        )
    ]
    found_functions = {
        node.name for node in nodes if isinstance(node, ast.FunctionDef)
    }
    found_classes = {
        node.name for node in nodes if isinstance(node, ast.ClassDef)
    }
    missing = (
        (function_names - found_functions)
        | (class_names - found_classes)
    )
    if missing:
        raise AssertionError(
            "{} lacks {}".format(path.name, sorted(missing))
        )
    module = ast.Module(body=nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    result = {} if namespace is None else namespace
    exec(compile(module, str(path), "exec"), result)
    return result


def _embedded_rng_class():
    namespace = _extract(
        DGA_PATH,
        class_names={"_DgaCpythonRandom"},
        namespace={"array": array},
    )
    return namespace["_DgaCpythonRandom"]


class DeltaSignatureParityTests(unittest.TestCase):
    def test_acbba_bundle_metadata_only_change_does_not_resend(self):
        cell = (3, 4)
        namespace = {
            "ACBBA_NO_WINNER_CODE": "99",
            "ACBBA_NO_BID": -1.0e18,
            "ACBBA_NO_TIME": -1.0e18,
            "ACBBA_EPS_BID": 1.0e-9,
            "ACBBA_EPS_TIME": 1.0e-9,
            "ROBOT_ID": "00",
            "first_clue_seen": True,
            "acbba_path": [cell, (5, 4)],
            "acbba_pending_deltas": {},
            "acbba_last_sent_signatures": {},
        }
        functions = {
            "_rid_sort_key",
            "_acbba_normalize_winner",
            "_acbba_encode_winner",
            "_same_robot_id",
            "_bid_eq",
            "_time_eq",
            "_acbba_signature",
            "_acbba_same_signature",
            "_acbba_queue_delta",
        }
        _extract(ACBBA_PATH, functions, namespace=namespace)

        old_entry = (
            cell, "00", -2.125, 7.0, [cell])
        namespace["acbba_last_sent_signatures"][cell] = (
            namespace["_acbba_signature"](old_entry)
        )
        namespace["_acbba_queue_delta"](
            cell, "00", -2.125, 7.0,
            include_bundle_metadata=True,
        )

        self.assertEqual(
            namespace["_acbba_signature"](old_entry),
            (cell, "00", -2.125, 7.0),
        )
        self.assertNotIn(cell, namespace["acbba_pending_deltas"])

    def test_cbaa_release_metadata_only_change_does_not_resend(self):
        cell = (8, 9)
        namespace = {
            "CBAA_NO_BID": -1.0e18,
            "CBAA_EPS_BID": 1.0e-9,
            "cbaa_pending_deltas": {},
            "cbaa_last_sent_signatures": {},
        }
        _extract(
            CBAA_PATH,
            {
                "_same_robot_id",
                "_cbaa_signature",
                "_cbaa_same_signature",
                "_cbaa_queue_delta",
                "_cbaa_release_matches",
            },
            namespace=namespace,
        )

        first = (None, -1.0e18, "01", -4.5)
        namespace["cbaa_last_sent_signatures"][cell] = (
            namespace["_cbaa_signature"](first)
        )
        namespace["_cbaa_queue_delta"](
            cell, None, -1.0e18, "02", -9.0)
        self.assertEqual(
            namespace["_cbaa_signature"](first),
            (None, -1.0e18),
        )
        self.assertNotIn(cell, namespace["cbaa_pending_deltas"])

    def test_cbaa_release_requires_exact_winner_and_bid_guard(self):
        namespace = {
            "CBAA_NO_BID": -1.0e18,
            "CBAA_EPS_BID": 1.0e-9,
        }
        _extract(
            CBAA_PATH,
            {"_same_robot_id", "_cbaa_release_matches"},
            namespace=namespace,
        )
        matches = namespace["_cbaa_release_matches"]
        self.assertFalse(matches("01", -5.0, "01", -1.0e18))
        self.assertFalse(matches("01", -5.0, "02", -4.0))
        self.assertFalse(matches("01", -5.0, "01", -6.0))
        self.assertTrue(matches("01", -5.0, "01", -4.0))

    def test_dga_queues_only_changed_owner_prefixes_and_empty_clears(self):
        namespace = {
            "ROBOT_ID": "03",
            "DGA_COMMITMENT_HORIZON": 3,
            "dga_generation": 25,
            "dga_delta_counter": 0,
            "dga_pending_deltas": [],
            "dga_last_sent_signatures": {
                "00": ((1, 0), (2, 0), (3, 0)),
                "01": ((4, 0),),
                "02": ((5, 0),),
            },
        }
        _extract(
            DGA_PATH,
            {"_rid_sort_key", "_dga_queue_plan"},
            namespace=namespace,
        )
        namespace["_dga_queue_plan"](
            {
                "00": [(1, 0), (2, 0), (3, 0), (9, 9)],
                "01": [(4, 1), (4, 2)],
                "03": [],
            },
            12.345678901234567,
        )

        pending = namespace["dga_pending_deltas"]
        owners = [entry[3] for entry in pending]
        self.assertNotIn("00", owners)
        self.assertEqual(owners.count("01"), 2)
        self.assertEqual(owners.count("02"), 1)
        self.assertEqual(owners.count("03"), 1)
        clears = {
            entry[3]
            for entry in pending
            if entry[6] == 0 and entry[7] == 1
        }
        self.assertEqual(clears, {"02", "03"})

    def test_dga_empty_run_queues_finite_clears_for_advertised_prefixes(self):
        namespace = {
            "ROBOT_ID": "03",
            "DGA_COMMITMENT_HORIZON": 3,
            "dga_generation": 25,
            "dga_delta_counter": 0,
            "dga_pending_deltas": [],
            "dga_last_sent_signatures": {
                "00": ((1, 0),),
                "01": ((4, 0),),
            },
            "dga_best_plan": {"03": []},
            "dga_best_fitness": 99.0,
            "dga_path": [],
            "dga_population": ["stale"],
            "dga_last_predicted_peer_first_task": {
                "00": (1, 0),
            },
            "_dga_refresh_probability_normalizer": lambda: 1.0,
            "_dga_candidates": lambda: [],
            "_dga_team_agents": lambda: {"03": (0, 0)},
        }
        _extract(
            DGA_PATH,
            {
                "_rid_sort_key",
                "_dga_copy_plan",
                "_dga_queue_plan",
                "_dga_commit",
                "_dga_run_impl",
            },
            namespace=namespace,
        )

        namespace["_dga_run_impl"]()

        self.assertEqual(namespace["dga_best_plan"], {"03": []})
        self.assertEqual(namespace["dga_best_fitness"], 0.0)
        self.assertEqual(namespace["dga_population"], [])
        clear_entries = {
            entry[3]: entry
            for entry in namespace["dga_pending_deltas"]
            if entry[6] == 0 and entry[7] == 1
        }
        self.assertTrue({"00", "01"}.issubset(clear_entries))
        self.assertTrue(
            all(entry[2] == 0.0 for entry in clear_entries.values())
        )


class DgaPredictionAndRandomnessTests(unittest.TestCase):
    def test_prediction_assessment_is_once_per_solution_for_entire_trial(self):
        namespace = {
            "ROBOT_ID": "03",
            "DGA_BAD_PRED_LIMIT": 3,
            "DGA_PREDICTION_TOLERANCE_CELLS": 0,
            "dga_last_assessed_peer_solution": set(),
            "dga_last_predicted_peer_first_task": {"01": (2, 2)},
            "dga_bad_prediction_count": {},
            "manhattan": (
                lambda x1, y1, x2, y2:
                abs(x1 - x2) + abs(y1 - y2)
            ),
        }
        _extract(
            DGA_PATH,
            {"_dga_assess_peer_prediction"},
            namespace=namespace,
        )
        assess = namespace["_dga_assess_peer_prediction"]

        assess("01", "A", 1, "01", 0, (9, 9), False)
        assess("01", "B", 2, "01", 0, (9, 9), False)
        assess("01", "A", 1, "01", 0, (9, 9), False)
        self.assertEqual(namespace["dga_bad_prediction_count"]["01"], 2)
        self.assertEqual(
            len(namespace["dga_last_assessed_peer_solution"]), 2)

        assess("01", "C", 3, "00", 0, (9, 9), False)
        self.assertEqual(namespace["dga_bad_prediction_count"]["01"], 2)
        assess("01", "D", 4, "01", 0, (2, 2), False)
        self.assertEqual(namespace["dga_bad_prediction_count"]["01"], 1)
        assess("01", "E", 5, "01", 0, (9, 9), False)
        assess("01", "F", 6, "01", 0, (9, 9), False)
        self.assertEqual(namespace["dga_bad_prediction_count"]["01"], 3)
        assess("01", "G", 7, "01", 0, (2, 2), False)
        self.assertEqual(namespace["dga_bad_prediction_count"]["01"], 3)

    def test_embedded_mt19937_subset_matches_cpython(self):
        rng_class = _embedded_rng_class()
        for seed in (0, 1, 7, 1012, (1 << 70) + 123):
            with self.subTest(seed=seed):
                actual = rng_class(seed)
                expected = random.Random(seed)
                self.assertEqual(
                    [actual.random() for _ in range(20)],
                    [expected.random() for _ in range(20)],
                )

                actual = rng_class(seed)
                expected = random.Random(seed)
                left = list(range(50))
                right = list(range(50))
                actual.shuffle(left)
                expected.shuffle(right)
                self.assertEqual(left, right)
                self.assertEqual(
                    actual.sample(list(range(30)), 3),
                    expected.sample(list(range(30)), 3),
                )

    def test_packet_and_backoff_draws_do_not_change_25_generation_ga(self):
        rng_class = _embedded_rng_class()
        left = _hardware_namespace(90210)
        right = _hardware_namespace(90210)
        left["dga_rng"] = rng_class(1012)
        right["dga_rng"] = rng_class(1012)
        left["dga_backoff_rng"] = rng_class(123456)
        right["dga_backoff_rng"] = rng_class(123456)

        for _ in range(73):
            right["random"].random()
        for _ in range(41):
            right["dga_backoff_rng"].random()

        left_population = left["_dga_prepare_population"](
            TEAM, CANDIDATES)
        right_population = right["_dga_prepare_population"](
            TEAM, CANDIDATES)
        for generation in range(25):
            for _ in range((generation * 11) % 29):
                right["random"].random()
            for _ in range((generation * 7) % 17):
                right["dga_backoff_rng"].random()
            left_population = left["_dga_next_generation"](
                left_population, TEAM, CANDIDATES)
            right_population = right["_dga_next_generation"](
                right_population, TEAM, CANDIDATES)

        self.assertEqual(left_population, right_population)
        self.assertEqual(
            left["dga_rng"].random(), right["dga_rng"].random())


class HandlerAndStartupParityTests(unittest.TestCase):
    def test_self_echo_is_ignored_before_any_state_or_counter_changes(self):
        frames = (
            "001.8,9",
            "002.4,5,6,7",
            "003.echoed-allocation",
            "004.1,2",
            "005.3,4",
            "007.2",
        )
        for path in (ACBBA_PATH, CBAA_PATH, DGA_PATH):
            with self.subTest(program=path.name):
                calls = []
                counters = {
                    "topic_1_rec": 11,
                    "topic_2_rec": 12,
                    "topic_3_rec": 13,
                    "topic_4_rec": 14,
                    "topic_5_rec": 15,
                }
                namespace = {
                    "ROBOT_ID": "00",
                    **counters,
                    "peer_pos": {"01": (1, 1)},
                    "peer_pos_yield": {"01": (1, 1)},
                    "peer_intent": {"01": (2, 1)},
                    "target_location": None,
                    "first_clue_seen": False,
                    "found_target": False,
                    "start_signal": True,
                    "pre_start_signal": False,
                    "published_intent": ((0, 0), (1, 0)),
                    "mark_cell_searched_miss": (
                        lambda *_args: calls.append("belief")
                    ),
                    "_acbba_receive_payload": (
                        lambda *_args: calls.append("acbba")
                    ),
                    "_cbaa_receive_payload": (
                        lambda *_args: calls.append("cbaa")
                    ),
                    "_dga_receive_payload": (
                        lambda *_args: calls.append("dga")
                    ),
                    "add_clue_if_new": (
                        lambda *_args: calls.append("clue")
                    ),
                    "_handle_config_command": (
                        lambda *_args: calls.append("config")
                    ),
                }
                _extract(
                    path,
                    {"_trial_traffic_enabled", "handle_msg"},
                    namespace=namespace,
                )

                for frame in frames:
                    namespace["handle_msg"](frame)

                self.assertEqual(
                    tuple(namespace[key] for key in counters),
                    tuple(counters.values()),
                )
                self.assertEqual(namespace["peer_pos"], {"01": (1, 1)})
                self.assertEqual(
                    namespace["peer_pos_yield"], {"01": (1, 1)})
                self.assertEqual(
                    namespace["peer_intent"], {"01": (2, 1)})
                self.assertIsNone(namespace["target_location"])
                self.assertFalse(namespace["first_clue_seen"])
                self.assertFalse(namespace["found_target"])
                self.assertTrue(namespace["start_signal"])
                self.assertFalse(namespace["pre_start_signal"])
                self.assertEqual(
                    namespace["published_intent"],
                    ((0, 0), (1, 0)),
                )
                self.assertEqual(calls, [])

    def test_peer_target_alert_freezes_before_inflight_arrival(self):
        for path in (ACBBA_PATH, CBAA_PATH, DGA_PATH):
            with self.subTest(program=path.name):
                finalized = []
                namespace = {
                    "ROBOT_ID": "00",
                    "GRID_SIZE": 19,
                    "topic_5_rec": 4,
                    "target_location": None,
                    "trial_active": True,
                    "start_signal": True,
                    "returning_home": False,
                    "found_target": False,
                    "metrics_frozen": False,
                    "metric_freeze_time_ms": None,
                    "busy_ms": 8,
                    "intersection_count": 6,
                    "pos": [5, 5],
                    "time": SimpleNamespace(ticks_ms=lambda: 4321),
                    "finalize_motor_time": finalized.append,
                    "busy_timer_value_ms": lambda: 3,
                    "safe_assert": (
                        lambda condition, message:
                        None
                        if condition
                        else (_ for _ in ()).throw(
                            AssertionError(message))
                    ),
                }
                _extract(
                    path,
                    {
                        "record_intersection",
                        "freeze_trial_metrics",
                        "_trial_traffic_enabled",
                        "handle_msg",
                    },
                    namespace=namespace,
                )
                namespace["handle_msg"]("015.6,7")

                self.assertTrue(namespace["found_target"])
                self.assertEqual(namespace["target_location"], (6, 7))
                self.assertEqual(namespace["topic_5_rec"], 5)
                self.assertTrue(namespace["metrics_frozen"])
                self.assertEqual(namespace["metric_freeze_time_ms"], 4321)
                self.assertEqual(namespace["busy_ms"], 11)
                self.assertEqual(finalized, [4321])
                self.assertEqual(namespace["pos"], [5, 5])

                self.assertFalse(
                    namespace["record_intersection"](6, 5))
                self.assertEqual(namespace["intersection_count"], 6)

    def test_calibration_does_not_publish_start_cell_state(self):
        for path in (ACBBA_PATH, CBAA_PATH, DGA_PATH):
            with self.subTest(program=path.name):
                tree = ast.parse(
                    path.read_text(encoding="utf-8"),
                    filename=str(path),
                )
                calibrate = next(
                    node
                    for node in tree.body
                    if (
                        isinstance(node, ast.FunctionDef)
                        and node.name == "calibrate"
                    )
                )
                publish_calls = [
                    node
                    for node in ast.walk(calibrate)
                    if (
                        isinstance(node, ast.Call)
                        and isinstance(node.func, ast.Name)
                        and node.func.id == "publish_position"
                    )
                ]
                self.assertEqual(publish_calls, [])


class ProbabilityNormalizerParityTests(unittest.TestCase):
    def test_tiny_finite_positive_wins_over_nan_and_infinity(self):
        tiny = 5.0e-324
        for path in (ACBBA_PATH, CBAA_PATH, DGA_PATH):
            with self.subTest(program=path.name):
                namespace = {
                    "target_p": [
                        float("nan"),
                        float("inf"),
                        -float("inf"),
                        tiny,
                        0.0,
                        -3.0,
                    ],
                    "allocation_probability_normalizer": 99.0,
                }
                _extract(
                    path,
                    {"refresh_probability_normalizer"},
                    namespace=namespace,
                )
                self.assertEqual(
                    namespace["refresh_probability_normalizer"](),
                    tiny,
                )
                self.assertEqual(
                    namespace["allocation_probability_normalizer"],
                    tiny,
                )

                namespace["target_p"] = [
                    float("nan"),
                    float("inf"),
                    -float("inf"),
                    0.0,
                    -1.0,
                ]
                self.assertEqual(
                    namespace["refresh_probability_normalizer"](),
                    1.0,
                )

    def test_dga_allocator_normalizer_ignores_nonfinite_values(self):
        namespace = {
            "target_p": [
                float("nan"),
                float("inf"),
                -float("inf"),
                0.25,
                0.0,
            ],
            "dga_probability_normalizer": 99.0,
        }
        _extract(
            DGA_PATH,
            {"_dga_refresh_probability_normalizer"},
            namespace=namespace,
        )
        self.assertEqual(
            namespace["_dga_refresh_probability_normalizer"](),
            0.25,
        )
        namespace["target_p"] = [
            float("nan"),
            float("inf"),
            -float("inf"),
            0.0,
            -1.0,
        ]
        self.assertEqual(
            namespace["_dga_refresh_probability_normalizer"](),
            1.0,
        )


class PackedCandidateAndMetricTests(unittest.TestCase):
    def test_dga_all_361_candidates_stay_in_packed_workspace(self):
        grid_size = 19
        probability = array(
            "d",
            [
                ((x * 13 + y * 7) % 29) / 29.0
                for y in range(grid_size - 1, -1, -1)
                for x in range(grid_size)
            ],
        )
        workspace = PackedCandidateWorkspace(grid_size, 361)
        namespace = {
            "time": SimpleNamespace(ticks_us=lambda: 0),
            "GRID_SIZE": grid_size,
            "CELL_UNSEARCHED": 0,
            "grid": bytearray(grid_size * grid_size),
            "target_p": probability,
            "pos": [7, 11],
            "dga_candidate_workspace": workspace,
            "_task_temporarily_invalid": lambda _cell: False,
            "record_candidate_filter_time": lambda _started: None,
            "safe_assert": (
                lambda condition, message:
                None
                if condition
                else (_ for _ in ()).throw(AssertionError(message))
            ),
        }
        _extract(
            DGA_PATH,
            {"idx", "_dga_valid_task", "_dga_candidates"},
            namespace=namespace,
        )
        result = namespace["_dga_candidates"]()
        expected = sorted(
            (
                (x, y)
                for y in range(grid_size)
                for x in range(grid_size)
            ),
            key=lambda cell: (
                -probability[
                    (grid_size - 1 - cell[1]) * grid_size + cell[0]
                ],
                abs(7 - cell[0]) + abs(11 - cell[1]),
                cell,
            ),
        )
        self.assertIs(result, workspace)
        self.assertEqual(list(result), expected)

    def test_local_target_bump_counts_logical_step_without_moving_pose(self):
        for path in (ACBBA_PATH, CBAA_PATH, DGA_PATH):
            with self.subTest(program=path.name):
                grid_size = 19
                finalized = []
                published = []
                namespace = {
                    "GRID_SIZE": grid_size,
                    "CELL_SEARCHED": 2,
                    "grid": bytearray(grid_size * grid_size),
                    "pos": [5, 6],
                    "heading": (1, 0),
                    "intersection_count": 4,
                    "trial_active": True,
                    "metrics_frozen": False,
                    "metric_freeze_time_ms": None,
                    "terminal_target_step_counted": False,
                    "busy_ms": 10,
                    "target_location": None,
                    "found_target": False,
                    "move_forward_flag": True,
                    "target_bump_stop": False,
                    "time": SimpleNamespace(ticks_ms=lambda: 1234),
                    "finalize_motor_time": finalized.append,
                    "busy_timer_value_ms": lambda: 7,
                    "safe_assert": (
                        lambda condition, message:
                        None
                        if condition
                        else (_ for _ in ()).throw(
                            AssertionError(message))
                    ),
                    "publish_target": (
                        lambda x, y: published.append((x, y))
                    ),
                    "motors_off": lambda: None,
                    "buzz": lambda _event: None,
                    "flash_LEDS": lambda *_args: None,
                    "BLUE": 1,
                }
                _extract(
                    path,
                    {
                        "idx",
                        "record_intersection",
                        "freeze_trial_metrics",
                        "stop_and_alert_target",
                    },
                    namespace=namespace,
                )
                namespace["stop_and_alert_target"]()

                self.assertEqual(namespace["pos"], [5, 6])
                self.assertEqual(namespace["target_location"], (6, 6))
                self.assertEqual(namespace["intersection_count"], 5)
                self.assertEqual(
                    namespace["grid"][
                        namespace["idx"](6, 6)
                    ],
                    2,
                )
                self.assertTrue(namespace["terminal_target_step_counted"])
                self.assertTrue(namespace["metrics_frozen"])
                self.assertEqual(namespace["metric_freeze_time_ms"], 1234)
                self.assertEqual(namespace["busy_ms"], 17)
                self.assertEqual(finalized, [1234])
                self.assertEqual(published, [(6, 6)])
                self.assertFalse(
                    namespace["record_intersection"](7, 6))
                self.assertEqual(namespace["intersection_count"], 5)


if __name__ == "__main__":
    unittest.main()
