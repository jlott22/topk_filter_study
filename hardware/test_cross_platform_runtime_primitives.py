"""Cross-platform checks for runtime behavior shared by all Pololu programs.

The robot programs cannot be imported on CPython because their module bodies
initialize hardware and start the mission.  These tests extract only pure
functions from their AST and execute them against controlled state.
"""

from __future__ import annotations

import ast
import heapq
import threading
import unittest
from array import array
from pathlib import Path
from types import SimpleNamespace

from hardware.allocator_memory import PackedCandidateWorkspace
from hardware.test_allocator_memory_optimized_equivalence import (
    _namespace as _allocator_namespace,
)
from simulator.benchmark_sim.core.belief import BeliefMap
from simulator.benchmark_sim.core.planner import AStarPlanner


HARDWARE_DIR = Path(__file__).resolve().parent
POLULU_FILES = tuple(sorted(HARDWARE_DIR.glob("Pololu_*.py")))
GRID_SIZE = 19
TOP_K_LIMITS = (361, 271, 181, 90, 36, 18)
ROBOT_BANDS = {
    "00": ((0, 0), 0, 4),
    "01": ((0, 6), 5, 9),
    "02": ((0, 12), 10, 14),
    "03": ((0, 18), 15, 18),
}


def _extract(path: Path, function_names: set[str], namespace: dict):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    nodes = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in function_names
    ]
    missing = function_names - {node.name for node in nodes}
    if missing:
        raise AssertionError(
            "{} lacks {}".format(path.name, sorted(missing))
        )
    module = ast.Module(body=nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(path), "exec"), namespace)
    return namespace


def _idx(grid_size: int, x: int, y: int) -> int:
    return (grid_size - 1 - y) * grid_size + x


def _expected_sweep(start, low, high):
    ordered = []
    for y in range(low, high + 1):
        xs = range(GRID_SIZE) if (y - low) % 2 == 0 else range(
            GRID_SIZE - 1, -1, -1
        )
        ordered.extend((x, y) for x in xs)
    start_index = ordered.index(start)
    return ordered[start_index + 1 :] + ordered[:start_index]


class _TimeStub:
    @staticmethod
    def ticks_us():
        return 0


class CrossPlatformRuntimePrimitiveTests(unittest.TestCase):
    def test_candidate_order_contract_at_all_six_topk_limits(self):
        searched = {
            (0, 0),
            (5, 2),
            (12, 3),
            (7, 9),
            (4, 11),
            (18, 16),
            (3, 18),
        }
        origin = [4, 11]
        probability = {
            (x, y): ((x * 13 + y * 7) % 17) / 17.0
            for y in range(GRID_SIZE)
            for x in range(GRID_SIZE)
        }
        valid_y_major = [
            (x, y)
            for y in range(GRID_SIZE)
            for x in range(GRID_SIZE)
            if (x, y) not in searched
        ]
        ranked = sorted(
            valid_y_major,
            key=lambda cell: (
                -probability[cell],
                abs(origin[0] - cell[0]) + abs(origin[1] - cell[1]),
                cell,
            ),
        )

        cases = {
            "Pololu_ACBBA.py": (
                {"idx", "manhattan", "_acbba_valid_task", "_acbba_candidate_cells"},
                "_acbba_candidate_cells",
                "acbba_candidate_workspace",
                False,
            ),
            "Pololu_CBAA.py": (
                {"idx", "manhattan", "_cbaa_valid_task", "_cbaa_candidate_cells"},
                "_cbaa_candidate_cells",
                "cbaa_candidate_workspace",
                False,
            ),
            "Pololu_DGA.py": (
                {"idx", "manhattan", "_dga_valid_task", "_dga_candidates"},
                "_dga_candidates",
                "dga_candidate_workspace",
                True,
            ),
            "Pololu_HIPC.py": (
                {"idx", "_hipc_candidates"},
                "_hipc_candidates",
                "hipc_candidate_workspace",
                True,
            ),
            "Pololu_PI.py": (
                {"idx", "_pi_candidates"},
                "_pi_candidates",
                "pi_candidate_workspace",
                False,
            ),
        }

        for filename, (
            functions,
            candidate_function,
            workspace_name,
            rank_always,
        ) in cases.items():
            path = HARDWARE_DIR / filename
            for limit in TOP_K_LIMITS:
                with self.subTest(file=filename, top_k_limit=limit):
                    grid = bytearray(GRID_SIZE * GRID_SIZE)
                    target_p = array(
                        "d", [0.0] * (GRID_SIZE * GRID_SIZE)
                    )
                    for cell in searched:
                        grid[_idx(GRID_SIZE, *cell)] = 2
                    for cell, value in probability.items():
                        target_p[_idx(GRID_SIZE, *cell)] = value
                    namespace = {
                        "time": _TimeStub,
                        "GRID_SIZE": GRID_SIZE,
                        "TOP_K_MAX_CELLS": limit,
                        "CELL_UNSEARCHED": 0,
                        "grid": grid,
                        "target_p": target_p,
                        "pos": list(origin),
                        "_task_temporarily_invalid": lambda _cell: False,
                        "record_candidate_filter_time": lambda _started: None,
                        "safe_assert": lambda condition, message: (
                            None
                            if condition
                            else (_ for _ in ()).throw(
                                AssertionError(message)
                            )
                        ),
                    }
                    if workspace_name is not None:
                        namespace[workspace_name] = PackedCandidateWorkspace(
                            GRID_SIZE, limit
                        )
                    _extract(path, functions, namespace)
                    actual = list(namespace[candidate_function]())
                    expected = (
                        ranked[:limit]
                        if rank_always or len(valid_y_major) > limit
                        else valid_y_major
                    )
                    self.assertEqual(actual, expected)

        dmchba_path = HARDWARE_DIR / "Pololu_DMCHBA.py"
        dmchba_functions = {
            "idx",
            "manhattan",
            "_dmchba_pack_cell",
            "_dmchba_unpack_cell",
            "_dmchba_candidate_precedes",
            "_dmchba_candidate_indices",
        }
        for limit in TOP_K_LIMITS:
            with self.subTest(file=dmchba_path.name, top_k_limit=limit):
                grid = bytearray(GRID_SIZE * GRID_SIZE)
                target_p = array(
                    "d", [0.0] * (GRID_SIZE * GRID_SIZE)
                )
                for cell in searched:
                    grid[_idx(GRID_SIZE, *cell)] = 2
                for cell, value in probability.items():
                    target_p[_idx(GRID_SIZE, *cell)] = value
                namespace = {
                    "time": _TimeStub,
                    "GRID_SIZE": GRID_SIZE,
                    "TOP_K_MAX_CELLS": limit,
                    "CELL_UNSEARCHED": 0,
                    "grid": grid,
                    "target_p": target_p,
                    "pos": list(origin),
                    "dmchba_candidate_ids": array(
                        "H", [0] * limit
                    ),
                    "record_candidate_filter_time": lambda _started: None,
                    "safe_assert": lambda condition, message: (
                        None
                        if condition
                        else (_ for _ in ()).throw(
                            AssertionError(message)
                        )
                    ),
                }
                _extract(dmchba_path, dmchba_functions, namespace)
                count = namespace["_dmchba_candidate_indices"]()
                actual = [
                    namespace["_dmchba_unpack_cell"](
                        namespace["dmchba_candidate_ids"][index]
                    )
                    for index in range(count)
                ]
                expected = (
                    ranked[:limit]
                    if len(valid_y_major) > limit
                    else valid_y_major
                    )
                self.assertEqual(actual, expected)

    def test_multi_task_allocators_defer_goal_cleanup_to_next_choose(self):
        active_goal = (5, 0)
        crossed_later_task = (2, 0)
        cases = (
            (
                "Pololu_ACBBA.py",
                "_acbba_handle_allocator_goal_arrival",
                "_acbba_complete_cell_arrival",
                "_acbba_clear_invalid_or_completed_cells",
                "acbba_flush_messages",
            ),
            (
                "Pololu_DGA.py",
                "_dga_handle_allocator_goal_arrival",
                "_dga_complete_cell_arrival",
                "_dga_clear_invalid_or_completed_cells",
                "dga_flush_messages",
            ),
            (
                "Pololu_HIPC.py",
                "_hipc_handle_allocator_goal_arrival",
                "_hipc_complete_cell_arrival",
                "_hipc_clear_invalid_or_completed_cells",
                "hipc_flush_messages",
            ),
        )

        for (
            filename,
            helper_name,
            complete_name,
            cleanup_name,
            flush_name,
        ) in cases:
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                searched = {crossed_later_task}
                committed = [active_goal, crossed_later_task]
                cleanup_calls = []
                flush_calls = []
                outbound_frames = []
                pending_allocator_state = [False]

                def cleanup():
                    cleanup_calls.append(tuple(sorted(searched)))
                    committed[:] = [
                        cell for cell in committed if cell not in searched
                    ]
                    pending_allocator_state[0] = True

                def flush():
                    flush_calls.append(True)
                    if pending_allocator_state[0]:
                        outbound_frames.append(tuple(committed))
                        pending_allocator_state[0] = False

                namespace = {
                    "current_task_cell": active_goal,
                    cleanup_name: cleanup,
                    flush_name: flush,
                    "first_clue_seen": True,
                    "found_target": False,
                    "pos": [
                        crossed_later_task[0],
                        crossed_later_task[1],
                    ],
                    "clues": [(9, 9)],
                    "grid": bytearray(GRID_SIZE * GRID_SIZE),
                    "CELL_SEARCHED": 2,
                    "publish_position": lambda: None,
                    "publish_intent": lambda: None,
                    "update_target_on_miss": lambda _cell_i: None,
                    "at_intersection_and_white": lambda: False,
                }
                _extract(path, {helper_name, complete_name}, namespace)
                complete_arrival = namespace[complete_name]

                self.assertFalse(complete_arrival(17))
                self.assertEqual(
                    committed,
                    [active_goal, crossed_later_task],
                )
                self.assertEqual(cleanup_calls, [])
                self.assertEqual(len(flush_calls), 1)
                self.assertEqual(outbound_frames, [])

                searched.add(active_goal)
                namespace["pos"][:] = active_goal
                self.assertTrue(complete_arrival(18))
                self.assertEqual(
                    committed,
                    [active_goal, crossed_later_task],
                )
                self.assertEqual(cleanup_calls, [])
                self.assertEqual(len(flush_calls), 2)
                self.assertEqual(outbound_frames, [])
                self.assertIsNone(namespace["current_task_cell"])

                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                pick_impl = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "_pick_task_cell_impl"
                )
                pick_calls = {
                    node.func.id
                    for node in ast.walk(pick_impl)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertIn(cleanup_name, pick_calls)
                run_trial = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "run_active_trial"
                )
                direct_calls = {
                    node.func.id
                    for node in ast.walk(run_trial)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertIn(complete_name, direct_calls)
                self.assertNotIn(cleanup_name, direct_calls)

    def test_cbaa_claims_are_untouched_until_next_choose_boundary(self):
        path = HARDWARE_DIR / "Pololu_CBAA.py"
        active_goal = (5, 0)
        crossed_peer_claim = (2, 0)
        grid = bytearray(GRID_SIZE * GRID_SIZE)
        grid[_idx(GRID_SIZE, *crossed_peer_claim)] = 2
        winners = {
            active_goal: "03",
            crossed_peer_claim: "01",
        }
        bids = {
            active_goal: -2.0,
            crossed_peer_claim: -1.0,
        }
        pending_deltas = {}
        flush_calls = []
        published_topics = []
        namespace = {
            "GRID_SIZE": GRID_SIZE,
            "CELL_UNSEARCHED": 0,
            "grid": grid,
            "CBAA_NO_BID": -1.0e18,
            "CBAA_EPS_BID": 1.0e-9,
            "cbaa_winner_by_cell": winners,
            "cbaa_winning_bid_by_cell": bids,
            "cbaa_current_task": active_goal,
            "cbaa_pending_deltas": pending_deltas,
            "cbaa_last_sent_signatures": {},
            "current_task_cell": active_goal,
            "temporary_invalid_task_until": {},
            "_task_temporarily_invalid": lambda _cell: False,
            "safe_assert": lambda condition, message: (
                None
                if condition
                else (_ for _ in ()).throw(AssertionError(message))
            ),
            "cbaa_flush_messages": lambda *_args: flush_calls.append(True),
            "first_clue_seen": True,
            "found_target": False,
            "pos": [crossed_peer_claim[0], crossed_peer_claim[1]],
            "clues": [(9, 9)],
            "publish_position": lambda: published_topics.append("state"),
            "publish_intent": lambda: published_topics.append("intent"),
            "update_target_on_miss": lambda _cell_i: None,
            "at_intersection_and_white": lambda: False,
        }
        _extract(
            path,
            {
                "idx",
                "_same_robot_id",
                "_cbaa_valid_task",
                "_cbaa_signature",
                "_cbaa_same_signature",
                "_cbaa_queue_delta",
                "_cbaa_set_table_entry",
                "_cbaa_clear_invalid_or_completed_cells",
                "_cbaa_handle_allocator_goal_arrival",
                "_cbaa_complete_cell_arrival",
            },
            namespace,
        )

        complete_arrival = namespace["_cbaa_complete_cell_arrival"]
        self.assertFalse(
            complete_arrival(_idx(GRID_SIZE, *crossed_peer_claim))
        )
        self.assertEqual(winners[crossed_peer_claim], "01")
        self.assertEqual(bids[crossed_peer_claim], -1.0)
        self.assertEqual(pending_deltas, {})
        self.assertEqual(len(flush_calls), 1)

        grid[_idx(GRID_SIZE, *active_goal)] = 2
        namespace["pos"][:] = active_goal
        self.assertTrue(
            complete_arrival(_idx(GRID_SIZE, *active_goal))
        )
        self.assertEqual(winners[crossed_peer_claim], "01")
        self.assertEqual(winners[active_goal], "03")
        self.assertEqual(pending_deltas, {})
        self.assertEqual(len(flush_calls), 2)
        self.assertIsNone(namespace["current_task_cell"])

        tree = ast.parse(
            path.read_text(encoding="utf-8"), filename=str(path)
        )
        run_trial = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "run_active_trial"
        )
        direct_calls = {
            node.func.id
            for node in ast.walk(run_trial)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
        }
        self.assertIn(
            "_cbaa_complete_cell_arrival",
            direct_calls,
        )
        self.assertNotIn(
            "_cbaa_clear_invalid_or_completed_cells",
            direct_calls,
        )

    def test_cbaa_clue_observation_precedes_allocator_outbound(self):
        path = HARDWARE_DIR / "Pololu_CBAA.py"
        active_goal = (5, 0)
        events = []
        bid = [7.0]
        clues = [(0, 0)]

        def add_clue(x, y):
            clues.append((x, y))
            bid[0] = 11.0
            events.append(("clue-observed", bid[0]))
            return True

        namespace = {
            "current_task_cell": active_goal,
            "first_clue_seen": True,
            "found_target": False,
            "pos": [active_goal[0], active_goal[1]],
            "clues": clues,
            "publish_position": lambda: events.append(("topic-1", bid[0])),
            "update_target_on_miss": lambda _cell_i: events.append(
                ("miss", bid[0])
            ),
            "_cbaa_clear_invalid_or_completed_cells": lambda: events.append(
                ("cleanup", bid[0])
            ),
            "at_intersection_and_white": lambda: True,
            "add_clue_if_new": add_clue,
            "publish_clue": lambda _x, _y: events.append(
                ("topic-4", bid[0])
            ),
            "update_mem_headroom": lambda: None,
            "gc": SimpleNamespace(collect=lambda: None),
            "cbaa_flush_messages": lambda *_args: events.append(
                ("topic-3", bid[0])
            ),
        }
        _extract(
            path,
            {
                "_cbaa_handle_allocator_goal_arrival",
                "_cbaa_complete_cell_arrival",
            },
            namespace,
        )

        self.assertTrue(namespace["_cbaa_complete_cell_arrival"](17))
        self.assertEqual(
            events,
            [
                ("topic-1", 7.0),
                ("miss", 7.0),
                ("clue-observed", 11.0),
                ("topic-4", 11.0),
                ("topic-3", 11.0),
            ],
        )

    def test_cbaa_goal_arrival_sends_release_before_next_claim(self):
        namespace = _allocator_namespace(
            "cbaa", True, 5, 1.0, 31, 4
        )
        goal = (1, 0)
        namespace["pos"][:] = goal
        namespace["current_task_cell"] = goal
        namespace["cbaa_current_task"] = goal
        namespace["cbaa_winner_by_cell"][goal] = namespace["ROBOT_ID"]
        namespace["cbaa_winning_bid_by_cell"][goal] = -2.0
        namespace["cbaa_last_sent_signatures"][goal] = (
            goal,
            namespace["ROBOT_ID"],
            -2.0,
        )
        namespace["cbaa_clue_signature"] = (
            namespace["_cbaa_clue_signature"]()
        )
        namespace["candidate_filter_time_us_total"] = 0
        namespace["publish_position"] = lambda: None
        namespace["found_target"] = False
        namespace["at_intersection_and_white"] = lambda: False

        def apply_miss(cell_i):
            namespace["grid"][cell_i] = namespace["CELL_SEARCHED"]

        namespace["update_target_on_miss"] = apply_miss
        _extract(
            HARDWARE_DIR / "Pololu_CBAA.py",
            {"_pick_task_cell_impl"},
            namespace,
        )
        self.assertTrue(
            namespace["_cbaa_complete_cell_arrival"](
                namespace["idx"](*goal)
            )
        )
        self.assertIsNone(namespace["current_task_cell"])
        self.assertIsNone(namespace["cbaa_current_task"])
        self.assertEqual(len(namespace["_sent"]), 1)
        release = namespace["_sent"][0].split(",")
        self.assertEqual(release[:3], ["1", "0", "99"])

        next_goal = namespace["_pick_task_cell_impl"]()
        self.assertIsNotNone(next_goal)
        self.assertNotEqual(next_goal, goal)
        # Selection queues the replacement but does not retroactively merge it
        # with the already-published arrival release.
        self.assertEqual(len(namespace["_sent"]), 1)
        namespace["cbaa_flush_messages"]()
        self.assertEqual(len(namespace["_sent"]), 2)
        claim = namespace["_sent"][1].split(",")
        self.assertEqual(
            (int(claim[0]), int(claim[1])),
            next_goal,
        )
        self.assertEqual(claim[2], namespace["ROBOT_ID"])

    def test_cbaa_intermediate_clue_refreshes_bid_after_clue(self):
        path = HARDWARE_DIR / "Pololu_CBAA.py"
        active_goal = (5, 0)
        intermediate = (2, 0)
        events = []
        bid = [7.0]
        clues = [(0, 0)]

        def add_clue(x, y):
            clues.append((x, y))
            bid[0] = 11.0
            events.append(("clue-observed", bid[0]))
            return True

        namespace = {
            "current_task_cell": active_goal,
            "first_clue_seen": True,
            "found_target": False,
            "pos": [intermediate[0], intermediate[1]],
            "clues": clues,
            "publish_position": lambda: events.append(("topic-1", bid[0])),
            "update_target_on_miss": lambda _cell_i: events.append(
                ("miss", bid[0])
            ),
            "_cbaa_clear_invalid_or_completed_cells": lambda: (
                _ for _ in ()
            ).throw(AssertionError("intermediate arrival cleaned CBAA")),
            "at_intersection_and_white": lambda: True,
            "add_clue_if_new": add_clue,
            "publish_clue": lambda _x, _y: events.append(
                ("topic-4", bid[0])
            ),
            "update_mem_headroom": lambda: None,
            "gc": SimpleNamespace(collect=lambda: None),
            "cbaa_flush_messages": lambda *_args: events.append(
                ("topic-3", bid[0])
            ),
        }
        _extract(
            path,
            {
                "_cbaa_handle_allocator_goal_arrival",
                "_cbaa_complete_cell_arrival",
            },
            namespace,
        )

        self.assertFalse(namespace["_cbaa_complete_cell_arrival"](17))
        self.assertEqual(
            events,
            [
                ("topic-1", 7.0),
                ("miss", 7.0),
                ("clue-observed", 11.0),
                ("topic-4", 11.0),
                ("topic-3", 11.0),
            ],
        )

    def test_other_clue_observations_precede_allocator_outbound(self):
        active_goal = (5, 0)
        cases = (
            (
                "Pololu_ACBBA.py",
                "_acbba_handle_allocator_goal_arrival",
                "_acbba_complete_cell_arrival",
                "_acbba_clear_invalid_or_completed_cells",
                "acbba_flush_messages",
                True,
            ),
            (
                "Pololu_DGA.py",
                "_dga_handle_allocator_goal_arrival",
                "_dga_complete_cell_arrival",
                "_dga_clear_invalid_or_completed_cells",
                "dga_flush_messages",
                True,
            ),
            (
                "Pololu_HIPC.py",
                "_hipc_handle_allocator_goal_arrival",
                "_hipc_complete_cell_arrival",
                "_hipc_clear_invalid_or_completed_cells",
                "hipc_flush_messages",
                False,
            ),
        )

        for (
            filename,
            helper_name,
            complete_name,
            cleanup_name,
            flush_name,
            uses_add_clue_helper,
        ) in cases:
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                events = []
                bid = [7.0]
                clues = [(0, 0)]

                def observe_clue(x=None, y=None):
                    if uses_add_clue_helper:
                        clues.append((x, y))
                    bid[0] = 11.0
                    events.append(("clue-observed", bid[0]))
                    return True

                namespace = {
                    "current_task_cell": active_goal,
                    "first_clue_seen": True,
                    "found_target": False,
                    "pos": [active_goal[0], active_goal[1]],
                    "clues": clues,
                    "grid": bytearray(GRID_SIZE * GRID_SIZE),
                    "CELL_SEARCHED": 2,
                    "publish_position": lambda: events.append(
                        ("topic-1", bid[0])
                    ),
                    "update_target_on_miss": lambda _cell_i: events.append(
                        ("miss", bid[0])
                    ),
                    cleanup_name: lambda: events.append(
                        ("cleanup", bid[0])
                    ),
                    "at_intersection_and_white": lambda: True,
                    "publish_clue": lambda _x, _y: events.append(
                        ("topic-4", bid[0])
                    ),
                    "update_mem_headroom": lambda: None,
                    "gc": SimpleNamespace(collect=lambda: None),
                    flush_name: lambda: events.append(
                        ("topic-3", bid[0])
                    ),
                }
                if uses_add_clue_helper:
                    namespace["add_clue_if_new"] = observe_clue
                else:
                    namespace["update_prob_map"] = observe_clue

                _extract(
                    path,
                    {helper_name, complete_name},
                    namespace,
                )
                self.assertTrue(namespace[complete_name](17))
                self.assertEqual(
                    events,
                    [
                        ("topic-1", 7.0),
                        ("miss", 7.0),
                        ("clue-observed", 11.0),
                        ("topic-4", 11.0),
                        ("topic-3", 11.0),
                    ],
                )

    def test_pi_clue_observation_precedes_pending_snapshot(self):
        path = HARDWARE_DIR / "Pololu_PI.py"
        active_goal = (5, 0)
        events = []
        bid = [7.0]
        clues = [(0, 0)]

        def record_arrival(_cell):
            events.append(("miss", bid[0]))
            return True

        def observe_clue():
            bid[0] = 11.0
            events.append(("clue-observed", bid[0]))

        namespace = {
            "first_clue_seen": True,
            "found_target": False,
            "pos": [active_goal[0], active_goal[1]],
            "clues": clues,
            "publish_position": lambda: events.append(("topic-1", bid[0])),
            "_pi_record_arrival": record_arrival,
            "at_intersection_and_white": lambda: True,
            "update_prob_map": observe_clue,
            "publish_clue": lambda _x, _y: events.append(
                ("topic-4", bid[0])
            ),
            "update_mem_headroom": lambda: None,
            "gc": SimpleNamespace(collect=lambda: None),
            "pi_flush_messages": lambda: events.append(
                ("topic-3", bid[0])
            ),
        }
        _extract(path, {"_pi_complete_cell_arrival"}, namespace)

        self.assertTrue(
            namespace["_pi_complete_cell_arrival"](active_goal)
        )
        self.assertEqual(
            events,
            [
                ("topic-1", 7.0),
                ("miss", 7.0),
                ("clue-observed", 11.0),
                ("topic-4", 11.0),
                ("topic-3", 11.0),
            ],
        )

    def test_dmchba_arrival_has_no_allocator_outbound(self):
        path = HARDWARE_DIR / "Pololu_DMCHBA.py"
        active_goal = (5, 0)
        events = []
        clues = [(0, 0)]
        path_state = [active_goal]
        namespace = {
            "first_clue_seen": True,
            "found_target": False,
            "current_task_cell": active_goal,
            "pos": [active_goal[0], active_goal[1]],
            "clues": clues,
            "dmchba_path": path_state,
            "grid": bytearray(GRID_SIZE * GRID_SIZE),
            "CELL_SEARCHED": 2,
            "publish_position": lambda: events.append("topic-1"),
            "update_target_on_miss": lambda _cell_i: events.append("miss"),
            "at_intersection_and_white": lambda: True,
            "update_prob_map": lambda: events.append("clue-observed"),
            "publish_clue": lambda _x, _y: events.append("topic-4"),
            "update_mem_headroom": lambda: None,
            "gc": SimpleNamespace(collect=lambda: None),
        }
        _extract(path, {"_dmchba_complete_cell_arrival"}, namespace)

        self.assertTrue(
            namespace["_dmchba_complete_cell_arrival"](17)
        )
        self.assertEqual(path_state, [active_goal])
        self.assertIsNone(namespace["current_task_cell"])
        self.assertEqual(
            events,
            ["topic-1", "miss", "clue-observed", "topic-4"],
        )

    def test_straight_multicell_intents_have_no_arrival_clears(self):
        completion_functions = {
            "Pololu_ACBBA.py": "_acbba_complete_cell_arrival",
            "Pololu_CBAA.py": "_cbaa_complete_cell_arrival",
            "Pololu_DGA.py": "_dga_complete_cell_arrival",
            "Pololu_DMCHBA.py": "_dmchba_complete_cell_arrival",
            "Pololu_HIPC.py": "_hipc_complete_cell_arrival",
            "Pololu_PI.py": "_pi_complete_cell_arrival",
        }

        for filename, completion_name in completion_functions.items():
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                tx_buf = bytearray(64)
                sent = []

                def uart_send(topic, payload_len):
                    sent.append(
                        (
                            topic,
                            bytes(tx_buf[2 : 2 + payload_len]).decode(
                                "ascii"
                            ),
                        )
                    )

                namespace = {
                    "pos": [0, 0],
                    "tx_buf": tx_buf,
                    "topic_2_sent": 0,
                    "metrics_frozen": False,
                    "published_intent": None,
                    "communicated_intent": None,
                    "uart_tx_lock": threading.Lock(),
                    "uart_send": uart_send,
                }
                _extract(
                    path,
                    {"_write_int", "publish_intent"},
                    namespace,
                )
                publish_intent = namespace["publish_intent"]

                for x in range(3):
                    namespace["pos"][:] = (x, 0)
                    self.assertTrue(publish_intent(x + 1, 0))

                self.assertEqual(
                    sent,
                    [
                        ("2", "0,0,1,0"),
                        ("2", "1,0,2,0"),
                        ("2", "2,0,3,0"),
                    ],
                )
                self.assertEqual(namespace["topic_2_sent"], 3)
                self.assertFalse(
                    any("X" in payload for _topic, payload in sent)
                )

                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                completion = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == completion_name
                )
                completion_calls = {
                    node.func.id
                    for node in ast.walk(completion)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertNotIn("publish_intent", completion_calls)

                run_trial = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "run_active_trial"
                )
                run_calls = {
                    node.func.id
                    for node in ast.walk(run_trial)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertIn(completion_name, run_calls)

    def test_collision_backoff_defers_one_reallocation_to_next_choose(self):
        old_goal = (5, 0)
        new_goal = (7, 0)
        cases = (
            ("Pololu_ACBBA.py", "acbba"),
            ("Pololu_CBAA.py", "cbaa"),
            ("Pololu_DGA.py", "dga"),
            ("Pololu_DMCHBA.py", "dmchba"),
            ("Pololu_HIPC.py", "hipc"),
            ("Pololu_PI.py", "pi"),
        )

        for filename, algorithm in cases:
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                release_calls = []
                solve_calls = []
                signature_calls = []
                namespace = {
                    "current_task_cell": old_goal,
                    "pending_collision_reallocation": False,
                }

                if algorithm == "acbba":
                    namespace.update(
                        {
                            "acbba_path": [old_goal],
                            "_acbba_reset_if_new_clue_information": lambda: None,
                            "_acbba_clear_invalid_or_completed_cells": lambda: None,
                            "_acbba_repair_bundle_after_consensus": lambda: None,
                        }
                    )

                    def release():
                        release_calls.append("release")
                        namespace["acbba_path"] = []

                    def solve():
                        if not namespace["acbba_path"]:
                            solve_calls.append("solve")
                            namespace["acbba_path"] = [new_goal]

                    namespace["_acbba_release_own_bundle_for_replan"] = release
                    namespace["_acbba_build_bundle"] = solve
                elif algorithm == "cbaa":
                    namespace.update(
                        {
                            "cbaa_current_task": old_goal,
                            "candidate_filter_time_us_total": 0,
                            "time": _TimeStub,
                            "record_allocator_solve_time": lambda *_args: None,
                            "_cbaa_reset_if_new_clue_information": lambda: None,
                            "_cbaa_clear_invalid_or_completed_cells": lambda: None,
                            "_cbaa_resolve_current_task": (
                                lambda: namespace["cbaa_current_task"]
                            ),
                        }
                    )

                    def release():
                        release_calls.append("release")
                        namespace["cbaa_current_task"] = None

                    def solve():
                        solve_calls.append("solve")
                        namespace["cbaa_current_task"] = new_goal
                        return new_goal

                    namespace["_cbaa_release_current_task_for_replan"] = release
                    namespace["_cbaa_select_new_task"] = solve
                elif algorithm == "dga":
                    namespace.update(
                        {
                            "dga_path": [old_goal],
                            "dga_received_better_solution": False,
                            "_dga_reset_if_new_clue_information": lambda: None,
                            "_dga_clear_invalid_or_completed_cells": lambda: None,
                        }
                    )

                    def release():
                        release_calls.append("release")
                        namespace["dga_path"] = []

                    def solve():
                        solve_calls.append("solve")
                        namespace["dga_path"] = [new_goal]

                    namespace["_dga_release_current_task_for_replan"] = release
                    namespace["_dga_run"] = solve
                elif algorithm == "dmchba":
                    def should_reassign():
                        signature_calls.append("signature/filter")
                        return None

                    namespace.update(
                        {
                            "dmchba_path": [old_goal],
                            "_dmchba_should_reassign": should_reassign,
                            "_dmchba_drop_invalid_path_cells": lambda: None,
                        }
                    )

                    def solve(reason):
                        solve_calls.append(reason)
                        namespace["dmchba_path"] = [new_goal]

                    namespace["_dmchba_run_assignment"] = solve
                elif algorithm == "hipc":
                    namespace.update(
                        {
                            "hipc_path": [old_goal],
                            "_hipc_reset_if_new_clue_information": lambda: None,
                            "_hipc_clear_invalid_or_completed_cells": lambda: None,
                            "_hipc_repair_after_consensus": lambda: None,
                        }
                    )

                    def release():
                        release_calls.append("release")
                        namespace["hipc_path"] = []

                    def solve():
                        if not namespace["hipc_path"]:
                            solve_calls.append("solve")
                            namespace["hipc_path"] = [new_goal]

                    namespace["_hipc_release_own_bundle_for_replan"] = release
                    namespace["_hipc_build_bundle"] = solve
                else:
                    namespace.update(
                        {
                            "pi_path": [old_goal],
                            "_pi_reset_if_new_clue_information": lambda: None,
                            "_pi_clear_invalid_or_completed_cells": lambda: None,
                            "_pi_repair_after_consensus": lambda: None,
                        }
                    )

                    def release():
                        release_calls.append("release")
                        namespace["pi_path"] = []

                    def solve():
                        if not namespace["pi_path"]:
                            solve_calls.append("solve")
                            namespace["pi_path"] = [new_goal]

                    namespace["_pi_release_own_path_for_replan"] = release
                    namespace["_pi_build_bundle"] = solve

                helper_name = "_{}_defer_collision_reallocation".format(
                    algorithm
                )
                _extract(
                    path,
                    {helper_name, "_pick_task_cell_impl"},
                    namespace,
                )
                allocator_state_name = (
                    "cbaa_current_task"
                    if algorithm == "cbaa"
                    else algorithm + "_path"
                )
                allocator_before = (
                    list(namespace[allocator_state_name])
                    if isinstance(namespace[allocator_state_name], list)
                    else namespace[allocator_state_name]
                )

                namespace[helper_name]()
                self.assertIsNone(namespace["current_task_cell"])
                self.assertTrue(
                    namespace["pending_collision_reallocation"]
                )
                self.assertEqual(
                    namespace[allocator_state_name],
                    allocator_before,
                )
                self.assertEqual(release_calls, [])
                self.assertEqual(solve_calls, [])

                self.assertEqual(
                    namespace["_pick_task_cell_impl"](), new_goal
                )
                self.assertFalse(
                    namespace["pending_collision_reallocation"]
                )
                self.assertEqual(
                    len(release_calls),
                    0 if algorithm == "dmchba" else 1,
                )
                self.assertEqual(len(solve_calls), 1)
                if algorithm == "dmchba":
                    self.assertEqual(signature_calls, [])

                self.assertEqual(
                    namespace["_pick_task_cell_impl"](), new_goal
                )
                self.assertEqual(
                    len(release_calls),
                    0 if algorithm == "dmchba" else 1,
                )
                self.assertEqual(len(solve_calls), 1)
                if algorithm == "dmchba":
                    self.assertEqual(
                        signature_calls,
                        ["signature/filter"],
                    )

    def test_empty_consensus_path_during_backoff_emits_no_extra_clear(self):
        cases = (
            (
                "Pololu_HIPC.py",
                {
                    "hipc_path": [],
                    "hipc_bundle": [],
                    "hipc_pending_snapshot": False,
                    "hipc_winner_by_cell": {},
                    "hipc_winning_bid_by_cell": {},
                    "hipc_bid_time_by_cell": {},
                    "ROBOT_ID": "03",
                    "HIPC_NO_BID": -1.0e18,
                    "HIPC_NO_TIME": -1.0e18,
                    "_hipc_reset_if_new_clue_information": lambda: None,
                    "_hipc_clear_invalid_or_completed_cells": lambda: None,
                    "_hipc_repair_after_consensus": lambda: None,
                    "_hipc_build_bundle": lambda: None,
                },
                {
                    "_same_robot_id",
                    "_hipc_release_local_path",
                    "_hipc_release_own_bundle_for_replan",
                    "_pick_task_cell_impl",
                },
                "hipc_pending_snapshot",
            ),
            (
                "Pololu_PI.py",
                {
                    "pi_path": [],
                    "pi_bundle": [],
                    "pi_pending_snapshot": False,
                    "_pi_clear_removed_local_entries": lambda _cells: None,
                    "_pi_reset_if_new_clue_information": lambda: None,
                    "_pi_clear_invalid_or_completed_cells": lambda: None,
                    "_pi_repair_after_consensus": lambda: None,
                    "_pi_build_bundle": lambda: None,
                },
                {
                    "_pi_release_own_path_for_replan",
                    "_pick_task_cell_impl",
                },
                "pi_pending_snapshot",
            ),
        )
        for filename, namespace, functions, pending_name in cases:
            with self.subTest(file=filename):
                namespace.update(
                    {
                        "current_task_cell": None,
                        "pending_collision_reallocation": True,
                    }
                )
                _extract(
                    HARDWARE_DIR / filename,
                    functions,
                    namespace,
                )
                self.assertIsNone(
                    namespace["_pick_task_cell_impl"]()
                )
                self.assertFalse(
                    namespace["pending_collision_reallocation"]
                )
                self.assertFalse(namespace[pending_name])

    def test_allocator_messages_flush_only_at_choose_and_arrival_boundaries(self):
        messaging = {
            "Pololu_ACBBA.py": (
                "acbba_flush_messages",
                "_acbba_complete_cell_arrival",
            ),
            "Pololu_CBAA.py": (
                "cbaa_flush_messages",
                "_cbaa_complete_cell_arrival",
            ),
            "Pololu_DGA.py": (
                "dga_flush_messages",
                "_dga_complete_cell_arrival",
            ),
            "Pololu_HIPC.py": (
                "hipc_flush_messages",
                "_hipc_complete_cell_arrival",
            ),
            "Pololu_PI.py": (
                "pi_flush_messages",
                "_pi_complete_cell_arrival",
            ),
        }
        for filename, (flush_name, completion_name) in messaging.items():
            with self.subTest(file=filename):
                path = HARDWARE_DIR / filename
                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                functions = {
                    node.name: node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                }
                run_flushes = [
                    node
                    for node in ast.walk(functions["run_active_trial"])
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == flush_name
                ]
                arrival_flushes = [
                    node
                    for node in ast.walk(functions[completion_name])
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == flush_name
                ]
                self.assertEqual(len(run_flushes), 1)
                self.assertEqual(len(arrival_flushes), 1)

                pending = []
                outbound = []
                namespace = {
                    "current_task_cell": (5, 0),
                    "first_clue_seen": True,
                    "found_target": False,
                    "pos": [2, 0],
                    "clues": [(9, 9)],
                    "grid": bytearray(GRID_SIZE * GRID_SIZE),
                    "CELL_SEARCHED": 2,
                    "publish_position": lambda: None,
                    "update_target_on_miss": lambda _cell_i: None,
                    "at_intersection_and_white": lambda: False,
                    flush_name: lambda *_args: (
                        outbound.append(tuple(pending)),
                        pending.clear(),
                    ),
                }
                if filename == "Pololu_PI.py":
                    namespace["_pi_record_arrival"] = lambda _cell: False
                else:
                    handler_name = completion_name.replace(
                        "_complete_cell_arrival",
                        "_handle_allocator_goal_arrival",
                    )
                    namespace[handler_name] = lambda _cell: False

                _extract(path, {completion_name}, namespace)
                # Two inbound consensus updates overwrite the same pending
                # owner/cell delta before a permitted publication boundary.
                pending[:] = [("owner-00", "first")]
                pending[:] = [("owner-00", "final")]
                argument = (2, 0) if filename == "Pololu_PI.py" else 17
                namespace[completion_name](argument)
                self.assertEqual(
                    outbound,
                    [(("owner-00", "final"),)],
                )

        dmchba_tree = ast.parse(
            (HARDWARE_DIR / "Pololu_DMCHBA.py").read_text(
                encoding="utf-8"
            )
        )
        self.assertFalse(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and "flush_messages" in node.func.id
                for node in ast.walk(dmchba_tree)
            )
        )

    def test_first_collision_replan_does_not_clear_protected_intent(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                run_trial = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "run_active_trial"
                )
                defer_name = "_{}_defer_collision_reallocation".format(
                    path.stem.removeprefix("Pololu_").lower()
                )
                conflict = next(
                    node
                    for node in ast.walk(run_trial)
                    if isinstance(node, ast.If)
                    and (
                        (
                            isinstance(node.test, ast.Call)
                            and isinstance(node.test.func, ast.Name)
                            and node.test.func.id == "i_should_yield"
                        )
                        or (
                            isinstance(node.test, ast.Name)
                            and node.test.id == "collision_blocked"
                        )
                    )
                )
                direct_clears = [
                    statement
                    for statement in conflict.body
                    if isinstance(statement, ast.Expr)
                    and isinstance(statement.value, ast.Call)
                    and isinstance(statement.value.func, ast.Name)
                    and statement.value.func.id == "publish_intent"
                    and not statement.value.args
                ]
                self.assertEqual(direct_clears, [])

                backoff_branch = next(
                    node
                    for node in ast.walk(conflict)
                    if isinstance(node, ast.If)
                    and any(
                        isinstance(child, ast.Call)
                        and isinstance(child.func, ast.Name)
                        and child.func.id == defer_name
                        for child in ast.walk(node)
                    )
                    and node is not conflict
                )
                backoff_clears = [
                    node
                    for node in ast.walk(backoff_branch)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "publish_intent"
                    and not node.args
                ]
                self.assertEqual(len(backoff_clears), 1)
                run_calls = {
                    node.func.id
                    for node in ast.walk(run_trial)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                }
                self.assertIn(
                    "_retry_original_goal_after_failed_alternate",
                    run_calls,
                )

                tx_buf = bytearray(64)
                sent = []

                def uart_send(topic, payload_len):
                    sent.append(
                        (
                            topic,
                            bytes(
                                tx_buf[2 : 2 + payload_len]
                            ).decode("ascii"),
                        )
                    )

                goal = (5, 0)
                intent_namespace = {
                    "pos": [0, 0],
                    "tx_buf": tx_buf,
                    "topic_2_sent": 0,
                    "metrics_frozen": False,
                    "published_intent": None,
                    "communicated_intent": None,
                    "uart_tx_lock": threading.Lock(),
                    "first_clue_seen": True,
                    "current_task_cell": goal,
                    "uart_send": uart_send,
                }
                uses_failure_map = path.name in {
                    "Pololu_ACBBA.py",
                    "Pololu_CBAA.py",
                    "Pololu_DGA.py",
                }
                if uses_failure_map:
                    intent_namespace["blocked_goal_failures"] = {goal: 1}
                else:
                    intent_namespace["blocked_goal_cell"] = goal
                    intent_namespace["blocked_goal_conflicts"] = 1

                _extract(
                    path,
                    {
                        "_write_int",
                        "publish_intent",
                        "_retry_original_goal_after_failed_alternate",
                    },
                    intent_namespace,
                )
                blocked_retry_cells = {(1, 0)}

                intent_namespace["publish_intent"](1, 0)
                self.assertTrue(
                    intent_namespace[
                        "_retry_original_goal_after_failed_alternate"
                    ](blocked_retry_cells)
                )
                self.assertEqual(blocked_retry_cells, set())
                self.assertEqual(intent_namespace["current_task_cell"], goal)
                if uses_failure_map:
                    self.assertEqual(
                        intent_namespace["blocked_goal_failures"],
                        {goal: 1},
                    )
                else:
                    self.assertEqual(
                        intent_namespace["blocked_goal_cell"],
                        goal,
                    )
                    self.assertEqual(
                        intent_namespace["blocked_goal_conflicts"],
                        1,
                    )

                # With no alternate A* route, retry the original protected
                # route. A second conflict then enters canonical backoff.
                intent_namespace["publish_intent"](1, 0)
                intent_namespace["publish_intent"]()
                self.assertEqual(
                    sent,
                    [
                        ("2", "0,0,1,0"),
                        ("2", "0,0,X,X"),
                        ("2", "0,0,1,0"),
                        ("2", "0,0,X,X"),
                    ],
                )

    def test_terminal_target_is_idempotent_and_sends_no_intent_clear(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                def make_namespace(trial_active):
                    events = []
                    namespace = {
                        "time": SimpleNamespace(ticks_ms=lambda: 123),
                        "pos": [4, 4],
                        "heading": (1, 0),
                        "GRID_SIZE": GRID_SIZE,
                        "CELL_SEARCHED": 2,
                        "grid": bytearray(GRID_SIZE * GRID_SIZE),
                        "idx": lambda x, y: _idx(GRID_SIZE, x, y),
                        "target_location": None,
                        "found_target": False,
                        "move_forward_flag": True,
                        "target_bump_stop": False,
                        "terminal_target_step_counted": False,
                        "trial_active": trial_active,
                        "metrics_frozen": False,
                        "BLUE": 1,
                        "record_intersection": (
                            lambda x, y: events.append(
                                ("terminal-step", x, y)
                            )
                        ),
                        "finalize_motor_time": (
                            lambda _when: events.append(("motor-time",))
                        ),
                        "motors_off": lambda: events.append(("motors-off",)),
                        "publish_intent": (
                            lambda *_args: events.append(("intent",))
                        ),
                        "publish_target": (
                            lambda x, y: events.append(("target", x, y))
                        ),
                        "freeze_trial_metrics": (
                            lambda _when: events.append(("freeze",))
                        ),
                        "buzz": lambda _name: events.append(("buzz",)),
                        "flash_LEDS": (
                            lambda _color, _count: events.append(("flash",))
                        ),
                    }
                    _extract(
                        path,
                        {"stop_and_alert_target"},
                        namespace,
                    )
                    return namespace, events

                namespace, events = make_namespace(True)
                namespace["stop_and_alert_target"]()
                namespace["stop_and_alert_target"]()
                self.assertTrue(namespace["found_target"])
                self.assertFalse(namespace["move_forward_flag"])
                self.assertEqual(namespace["target_location"], (5, 4))
                self.assertTrue(
                    namespace["terminal_target_step_counted"]
                )
                self.assertEqual(events.count(("target", 5, 4)), 1)
                self.assertEqual(
                    events.count(("terminal-step", 5, 4)), 1
                )
                self.assertNotIn(("intent",), events)

                prestart, prestart_events = make_namespace(False)
                prestart["stop_and_alert_target"]()
                self.assertFalse(
                    prestart["terminal_target_step_counted"]
                )
                self.assertNotIn(
                    ("terminal-step", 5, 4), prestart_events
                )
                self.assertEqual(
                    prestart["grid"][_idx(GRID_SIZE, 5, 4)], 0
                )

                source_tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                run_trial = next(
                    node
                    for node in source_tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "run_active_trial"
                )
                for try_node in (
                    node
                    for node in ast.walk(run_trial)
                    if isinstance(node, ast.Try)
                ):
                    self.assertFalse(
                        any(
                            isinstance(child, ast.Call)
                            and isinstance(child.func, ast.Name)
                            and child.func.id == "publish_intent"
                            for statement in try_node.finalbody
                            for child in ast.walk(statement)
                        )
                    )

    def test_complete_belief_rebuild_matches_simulator_after_each_mutation(self):
        grid_size = 7
        mutation_stages = (
            ([(0, 0), (6, 6)], []),
            ([(0, 0), (6, 6), (3, 3)], [(3, 3)]),
            (
                [(0, 0), (6, 6), (3, 3), (6, 1), (2, 4)],
                [(3, 3), (6, 1)],
            ),
        )

        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                available = {
                    node.name
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                }
                functions = {
                    "idx",
                    "manhattan",
                    "renorm",
                    "recompute_value_map",
                    "update_prob_map",
                }
                if "refresh_probability_normalizer" in available:
                    functions.add("refresh_probability_normalizer")

                grid = bytearray(grid_size * grid_size)
                uniform = 1.0 / (grid_size * grid_size)
                target_p = array(
                    "d", [uniform] * (grid_size * grid_size)
                )
                prob_map = array(
                    "d", [uniform] * (grid_size * grid_size)
                )
                namespace = {
                    "GRID_SIZE": grid_size,
                    "CELL_SEARCHED": 2,
                    "grid": grid,
                    "target_p": target_p,
                    "prob_map": prob_map,
                    "clues": [],
                    "TARGET_DECAY_EXP": 1.0,
                    "EPS": 1.0e-9,
                    "allocation_probability_normalizer": uniform,
                    "safe_assert": lambda condition, message: (
                        None
                        if condition
                        else (_ for _ in ()).throw(
                            AssertionError(message)
                        )
                    ),
                }
                _extract(path, functions, namespace)

                for searched, clues in mutation_stages:
                    grid[:] = bytearray(grid_size * grid_size)
                    for cell in searched:
                        grid[_idx(grid_size, *cell)] = 2
                    namespace["clues"][:] = clues
                    namespace["update_prob_map"]()

                    belief = BeliefMap(grid_size)
                    belief.searched = set(searched)
                    belief.known_clues = list(clues)
                    belief.recompute()
                    for cell, expected in belief.target_p.items():
                        actual = target_p[_idx(grid_size, *cell)]
                        self.assertAlmostEqual(
                            actual,
                            expected,
                            places=14,
                            msg="{} at {}".format(path.name, cell),
                        )
                        self.assertEqual(
                            actual,
                            prob_map[_idx(grid_size, *cell)],
                        )
                    self.assertAlmostEqual(
                        namespace["allocation_probability_normalizer"],
                        max(belief.target_p.values()),
                        places=14,
                    )

    def test_complete_preclue_sweep_sequences_match_simulator_contract(self):
        for path in POLULU_FILES:
            for rid, (start, low, high) in ROBOT_BANDS.items():
                with self.subTest(file=path.name, robot=rid):
                    grid = bytearray(GRID_SIZE * GRID_SIZE)
                    grid[_idx(GRID_SIZE, *start)] = 2
                    namespace = {
                        "GRID_SIZE": GRID_SIZE,
                        "BAND_Y_MIN": low,
                        "BAND_Y_MAX": high,
                        "CELL_UNSEARCHED": 0,
                        "grid": grid,
                        "pos": [start[0], start[1]],
                        "safe_assert": lambda condition, message: (
                            None
                            if condition
                            else (_ for _ in ()).throw(
                                AssertionError(message)
                            )
                        ),
                    }
                    _extract(
                        path,
                        {"idx", "next_serpentine_task_cell_in_band"},
                        namespace,
                    )

                    actual = []
                    while True:
                        goal = namespace[
                            "next_serpentine_task_cell_in_band"
                        ]()
                        if goal is None:
                            break
                        actual.append(goal)
                        namespace["pos"][:] = goal
                        grid[_idx(GRID_SIZE, *goal)] = 2

                    self.assertEqual(
                        actual,
                        _expected_sweep(start, low, high),
                    )

    def test_a_star_path_matches_simulator_in_all_six_programs(self):
        grid_size = 7
        start = (0, 6)
        goal = (6, 0)
        heading = (1, 0)
        searched = {(0, 5), (1, 5), (3, 3), (5, 1)}
        obstacles = {(2, 5), (2, 4), (4, 2)}
        peer_positions = {"00": (1, 6), "01": (5, 0)}
        # Protected intent location is intentionally newer than the last
        # accepted droppable state. It is used only by the immediate safety
        # recheck, not by route-wide A* planning.
        protected_peer_positions = dict(peer_positions)
        protected_peer_positions["02"] = (0, 5)
        probabilities = {
            (x, y): (
                ((x + 2) * (y + 3)) / 1000.0
                if (x, y) not in searched
                else 0.0
            )
            for y in range(grid_size)
            for x in range(grid_size)
        }
        blocked = set(obstacles) | set(peer_positions.values())
        expected = AStarPlanner(grid_size=grid_size).plan(
            start=start,
            heading=heading,
            goal=goal,
            target_p=probabilities,
            searched=searched,
            blocked=blocked,
        )

        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                grid = bytearray(grid_size * grid_size)
                dense_probability = array(
                    "d", [0.0] * (grid_size * grid_size)
                )
                for cell in searched:
                    grid[_idx(grid_size, *cell)] = 2
                for cell in obstacles:
                    grid[_idx(grid_size, *cell)] = 1
                for cell, value in probabilities.items():
                    dense_probability[_idx(grid_size, *cell)] = value

                namespace = {
                    "heapq": heapq,
                    "GRID_SIZE": grid_size,
                    "CELL_OBSTACLE": 1,
                    "CELL_SEARCHED": 2,
                    "DIRS4": ((0, 1), (1, 0), (0, -1), (-1, 0)),
                    "TURN_COST": 0.3,
                    "REWARD_FACTOR": 5,
                    "grid": grid,
                    "prob_map": dense_probability,
                    "target_p": dense_probability,
                    "peer_pos": dict(peer_positions),
                    "peer_pos_yield": protected_peer_positions,
                    "heading": heading,
                    "running": True,
                    "found_target": False,
                    "frontier": [],
                    "came_from": array(
                        "i", [-1] * (grid_size * grid_size)
                    ),
                    "cost_so_far": array(
                        "d", [0.0] * (grid_size * grid_size)
                    ),
                    "cfg": SimpleNamespace(VISITED_STEP_PENALTY=4),
                    "safe_assert": lambda condition, message: (
                        None
                        if condition
                        else (_ for _ in ()).throw(
                            AssertionError(message)
                        )
                    ),
                }
                _extract(
                    path,
                    {"idx", "quarter_turns", "a_star"},
                    namespace,
                )
                self.assertEqual(
                    namespace["a_star"](start, goal),
                    expected,
                )


if __name__ == "__main__":
    unittest.main()
