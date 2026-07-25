"""Simulator-versus-onboard allocation oracle for ACBBA/CBAA/HIPC/PI.

Each case creates a real study-simulator ``RobotShell`` and replays the same
belief, position, peer-position, clue, consensus, completion, collision, and
Top-K state through AST-extracted functions from the deployed Pololu program.
It compares initial and stateful allocation transitions plus decoded logical
outbound traffic at every study K.

DGA is intentionally not duplicated here: ``test_dga_simulator_equivalence``
already compares its complete 25-generation search and packed payloads, and
``test_acd_simulation_parity`` verifies its isolated CPython RNG stream.
DMCHBA is covered by ``test_dmchba_optimized_equivalence``, including every
virtual-versus-dense matrix cost, pseudotasks, ties, and production-size runs.
"""

from __future__ import annotations

import ast
import math
import sys
import unittest
from collections.abc import Mapping
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HARDWARE_DIR = ROOT / "hardware"
SIMULATOR_DIR = ROOT / "simulator"
if str(SIMULATOR_DIR) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_DIR))

from benchmark_sim.algorithms.ACBBA import ACBBAAllocator  # noqa: E402
from benchmark_sim.algorithms.CBAA import CBAAAllocator  # noqa: E402
from benchmark_sim.algorithms.HIPC import HIPCAllocator  # noqa: E402
from benchmark_sim.algorithms.PI import PIAllocator  # noqa: E402
from benchmark_sim.comms.message import Message, topic_for  # noqa: E402
from benchmark_sim.tests.test_reference_repository_topk_parity import (  # noqa: E402
    _make_robot,
)
from hardware.test_allocator_memory_optimized_equivalence import (  # noqa: E402
    _namespace,
)


GRID_SIZE = 19
TOP_K_LIMITS = (361, 271, 181, 90, 36, 18)
FLOAT_ABS_TOL = 1.0e-12
FLOAT_REL_TOL = 1.0e-12

CASES = {
    "acbba": {
        "class": ACBBAAllocator,
        "file": "Pololu_ACBBA.py",
        "candidate": "_acbba_candidate_cells",
        "flush": "acbba_flush_messages",
        "discrete": ("acbba_path", "acbba_bundle"),
        "owner_maps": ("acbba_winner_by_cell",),
        "numeric_maps": (
            "acbba_winning_bid_by_cell",
            "acbba_bid_time_by_cell",
        ),
        "scalars": ("acbba_bid_counter",),
    },
    "cbaa": {
        "class": CBAAAllocator,
        "file": "Pololu_CBAA.py",
        "candidate": "_cbaa_candidate_cells",
        "flush": "cbaa_flush_messages",
        "discrete": ("cbaa_current_task",),
        "owner_maps": ("cbaa_winner_by_cell",),
        "numeric_maps": ("cbaa_winning_bid_by_cell",),
        "scalars": (),
    },
    "hipc": {
        "class": HIPCAllocator,
        "file": "Pololu_HIPC.py",
        "candidate": "_hipc_candidates",
        "flush": "hipc_flush_messages",
        "discrete": (
            "hipc_path",
            "hipc_bundle",
            "hipc_bad_prediction_count",
            "hipc_dropped_peers",
            "hipc_last_predicted_peer_first_task",
        ),
        "owner_maps": ("hipc_winner_by_cell",),
        "numeric_maps": (
            "hipc_winning_bid_by_cell",
            "hipc_bid_time_by_cell",
        ),
        "scalars": ("hipc_bid_counter",),
    },
    "pi": {
        "class": PIAllocator,
        "file": "Pololu_PI.py",
        "candidate": "_pi_candidates",
        "flush": "pi_flush_messages",
        "discrete": ("pi_path", "pi_bundle"),
        "owner_maps": ("pi_owner_by_cell",),
        "numeric_maps": (
            "pi_significance_by_cell",
            "pi_time_by_cell",
        ),
        "scalars": ("pi_time_counter",),
    },
}


def _compile_pick_entry_point(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_pick_task_cell_impl"
    )
    module = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(module)
    return compile(module, str(path), "exec")


PICK_ENTRY_POINTS = {
    algorithm: _compile_pick_entry_point(HARDWARE_DIR / case["file"])
    for algorithm, case in CASES.items()
}


def _compile_named_functions(path, predicate):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and predicate(node.name)
    ]
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    return compile(module, str(path), "exec")


RECEIVE_SUPPORT = {
    "acbba": _compile_named_functions(
        HARDWARE_DIR / CASES["acbba"]["file"],
        lambda name: name.startswith("_action_"),
    )
}


def _plain(value):
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_plain(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_plain(item) for item in value)
    if isinstance(value, set):
        return {_plain(item) for item in value}
    return value


def _hardware_map(namespace, name):
    value = namespace[name]
    return dict(value.items()) if hasattr(value, "items") else dict(value)


def _prepare_pair(algorithm, top_k_limit):
    case = CASES[algorithm]
    robot = _make_robot(case["class"], top_k_limit)
    hardware = _namespace(
        algorithm,
        True,
        GRID_SIZE,
        top_k_limit / float(GRID_SIZE * GRID_SIZE),
        77,
        4,
    )
    exec(PICK_ENTRY_POINTS[algorithm], hardware)
    if algorithm in RECEIVE_SUPPORT:
        exec(RECEIVE_SUPPORT[algorithm], hardware)

    hardware["ROBOT_ID"] = str(robot.rid)
    hardware["pos"][:] = list(robot.pos)
    hardware["peer_pos"] = dict(robot._peer_positions)
    hardware["clues"] = list(robot.belief.known_clues)
    hardware["first_clue_seen"] = True
    hardware["current_task_cell"] = None
    hardware["pending_collision_reallocation"] = False
    hardware["candidate_filter_time_us_total"] = 0
    hardware["temporary_invalid_task_until"].clear()
    hardware["_sent"].clear()

    for index in range(GRID_SIZE * GRID_SIZE):
        hardware["grid"][index] = hardware["CELL_UNSEARCHED"]
        hardware["target_p"][index] = 0.0
        hardware["prob_map"][index] = 0.0
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell = (x, y)
            index = hardware["idx"](x, y)
            probability = robot.belief.probability(cell)
            hardware["target_p"][index] = probability
            hardware["prob_map"][index] = probability
            if cell in robot.belief.searched:
                hardware["grid"][index] = hardware["CELL_SEARCHED"]
    hardware["allocation_probability_normalizer"] = max(
        hardware["target_p"]
    )
    hardware[algorithm + "_clue_signature"] = None

    if algorithm == "hipc":
        hardware.update(
            {
                "hipc_seen_peer_bundle_signature": {},
                "hipc_last_team_size": 0,
                "hipc_last_candidate_count": 0,
            }
        )

    if hardware["TOP_K_MAX_CELLS"] != top_k_limit:
        raise AssertionError(
            "hardware Top-K mismatch: {} != {}".format(
                hardware["TOP_K_MAX_CELLS"], top_k_limit
            )
        )
    return robot, hardware


def _decode_number(value):
    return float(value.replace("N", "-"))


def _decode_cells(fields):
    cells = []
    for index in range(0, len(fields), 2):
        x, y = fields[index : index + 2]
        if x == "X" and y == "X":
            continue
        cells.append((int(x), int(y)))
    return tuple(cells)


def _simulator_logical_messages(algorithm, messages):
    logical = []
    for wire_order, message in enumerate(messages):
        if algorithm == "cbaa":
            logical.append(
                {
                    "kind": "cbaa_entry",
                    "sender": str(message["sender"]),
                    "cell": (int(message["x"]), int(message["y"])),
                    "claimant": (
                        None
                        if message["winner"] is None
                        else str(message["winner"])
                    ),
                    "score": float(message["bid"]),
                }
            )
            continue
        path_key = "path_cells" if algorithm == "pi" else "bundle_cells"
        claimant_key = "owner" if algorithm == "pi" else "winner"
        score_key = "significance" if algorithm == "pi" else "bid"
        logical.append(
            {
                "kind": str(message["type"]),
                "sender": str(message["sender"]),
                "cell": (int(message["x"]), int(message["y"])),
                "claimant": (
                    None
                    if message[claimant_key] is None
                    else str(message[claimant_key])
                ),
                "score": float(message[score_key]),
                "timestamp": float(message["timestamp"]),
                "order": int(message.get("order", wire_order)),
                "path": tuple(
                    (int(cell["x"]), int(cell["y"]))
                    for cell in message.get(path_key, [])
                ),
            }
        )
    return logical


def _hardware_logical_messages(algorithm, robot_id, payloads):
    logical = []
    for order, payload in enumerate(payloads):
        fields = payload.split(",")
        if algorithm == "cbaa":
            if len(fields) != 6:
                raise AssertionError("invalid CBAA payload: {}".format(payload))
            logical.append(
                {
                    "kind": "cbaa_entry",
                    "sender": str(robot_id),
                    "cell": (int(fields[0]), int(fields[1])),
                    "claimant": (
                        None if fields[2] == "99" else str(fields[2])
                    ),
                    "score": _decode_number(fields[3]),
                }
            )
            continue
        if len(fields) != 11:
            raise AssertionError(
                "invalid {} payload: {}".format(algorithm, payload)
            )
        logical.append(
            {
                "kind": (
                    "pi_entry"
                    if algorithm == "pi"
                    else "hipc_entry"
                    if algorithm == "hipc"
                    else "acbba_entry"
                ),
                "sender": str(robot_id),
                "cell": (int(fields[0]), int(fields[1])),
                "claimant": (
                    None if fields[2] == "99" else str(fields[2])
                ),
                "score": (
                    -1.0e18
                    if fields[3] == "X" and algorithm == "acbba"
                    else 1.0e18
                    if fields[3] == "X" and algorithm == "pi"
                    else -1.0e18
                    if fields[3] == "X"
                    else _decode_number(fields[3])
                ),
                "timestamp": (
                    -1.0e18
                    if fields[4] == "X"
                    else float(fields[4])
                ),
                "order": order,
                "path": _decode_cells(fields[5:]),
            }
        )
    return logical


def _sync_hardware_belief(robot, hardware):
    """Copy one canonical belief snapshot into the hardware dense storage."""

    hardware["clues"] = list(robot.belief.known_clues)
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell = (x, y)
            index = hardware["idx"](x, y)
            probability = robot.belief.probability(cell)
            hardware["target_p"][index] = probability
            hardware["prob_map"][index] = probability
            hardware["grid"][index] = (
                hardware["CELL_SEARCHED"]
                if cell in robot.belief.searched
                else hardware["CELL_UNSEARCHED"]
            )
    hardware["allocation_probability_normalizer"] = max(
        hardware["target_p"]
    )


def _next_unsearched_cell(robot, excluded=()):
    excluded = set(excluded)
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell = (x, y)
            if cell not in robot.belief.searched and cell not in excluded:
                return cell
    raise AssertionError("no unsearched cell remains")


def _intermediate_cell(robot, goal):
    """Choose an unsearched non-goal neighbor to model a retained-goal move."""

    x, y = robot.pos
    neighbors = (
        (x, y - 1),
        (x + 1, y),
        (x, y + 1),
        (x - 1, y),
    )
    ranked = sorted(
        (
            cell
            for cell in neighbors
            if (
                0 <= cell[0] < GRID_SIZE
                and 0 <= cell[1] < GRID_SIZE
                and cell != goal
                and cell not in robot.belief.searched
            )
        ),
        key=lambda cell: (
            abs(cell[0] - goal[0]) + abs(cell[1] - goal[1]),
            cell,
        ),
    )
    if ranked:
        return ranked[0]
    return _next_unsearched_cell(robot, excluded=(goal,))


def _forced_lower_id_claim(algorithm, robot, hardware):
    """Build one valid peer frame that wins an exact-score robot-ID tie."""

    sender = "01"
    if algorithm == "cbaa":
        cell = robot.cbaa_current_task
        score = float(robot.cbaa_winning_bid_by_cell[cell])
        message = {
            "type": "cbaa_entry",
            "sender": sender,
            "x": cell[0],
            "y": cell[1],
            "winner": sender,
            "bid": score,
            "released_winner": None,
            "released_bid": robot.allocator.NO_BID,
        }
        payload = ",".join(
            (
                str(cell[0]),
                str(cell[1]),
                hardware["_cbaa_encode_winner"](sender),
                hardware["_cbaa_encode_number"](score),
                hardware["_cbaa_encode_winner"](None),
                hardware["_cbaa_encode_number"](
                    hardware["CBAA_NO_BID"]
                ),
            )
        )
        return message, payload

    if algorithm == "acbba":
        cell = robot.acbba_path[0]
        score = float(robot.acbba_winning_bid_by_cell[cell])
        timestamp = float(robot.acbba_bid_time_by_cell[cell]) + 1.0
        message = {
            "type": "acbba_entry",
            "sender": sender,
            "x": cell[0],
            "y": cell[1],
            "winner": sender,
            "bid": score,
            "timestamp": timestamp,
            "order": 0,
            "bundle_cells": [{"x": cell[0], "y": cell[1]}],
        }
        payload = hardware["_acbba_payload_from_entry"](
            (cell, sender, score, timestamp, [cell])
        )
        return message, payload

    if algorithm == "hipc":
        cell = robot.hipc_path[0]
        score = float(robot.hipc_winning_bid_by_cell[cell])
        timestamp = float(robot.hipc_bid_time_by_cell[cell]) + 1.0
        message = {
            "type": "hipc_entry",
            "sender": sender,
            "x": cell[0],
            "y": cell[1],
            "winner": sender,
            "bid": score,
            "timestamp": timestamp,
            "order": 0,
            "bundle_cells": [{"x": cell[0], "y": cell[1]}],
        }
        payload = ",".join(
            [
                str(cell[0]),
                str(cell[1]),
                sender,
                hardware["_hipc_encode_signed"](
                    score, hardware["HIPC_NO_BID"]
                ),
                str(timestamp),
            ]
            + hardware["_hipc_bundle_fields"]([cell])
        )
        return message, payload

    cell = robot.pi_path[0]
    score = float(robot.pi_significance_by_cell[cell])
    timestamp = float(robot.pi_time_by_cell[cell]) + 1.0
    message = {
        "type": "pi_entry",
        "sender": sender,
        "x": cell[0],
        "y": cell[1],
        "owner": sender,
        "significance": score,
        "timestamp": timestamp,
        "order": 0,
        "path_cells": [{"x": cell[0], "y": cell[1]}],
    }
    payload = ",".join(
        [
            str(cell[0]),
            str(cell[1]),
            sender,
            hardware["_pi_encode_significance"](score),
            str(timestamp),
        ]
        + hardware["_pi_path_fields"]([cell])
    )
    return message, payload


class CrossImplementationAllocatorOracleTests(unittest.TestCase):
    def assert_close_structure(self, expected, actual, context="root"):
        if (
            isinstance(expected, (int, float))
            and not isinstance(expected, bool)
            and isinstance(actual, (int, float))
            and not isinstance(actual, bool)
        ):
            self.assertTrue(
                math.isclose(
                    float(expected),
                    float(actual),
                    rel_tol=FLOAT_REL_TOL,
                    abs_tol=FLOAT_ABS_TOL,
                ),
                "{}: {} != {}".format(context, expected, actual),
            )
            return
        if isinstance(expected, Mapping):
            self.assertEqual(
                set(expected), set(actual), "{} keys".format(context)
            )
            for key in expected:
                self.assert_close_structure(
                    expected[key],
                    actual[key],
                    "{}[{!r}]".format(context, key),
                )
            return
        if isinstance(expected, (list, tuple)):
            self.assertEqual(
                len(expected), len(actual), "{} length".format(context)
            )
            for index, (left, right) in enumerate(zip(expected, actual)):
                self.assert_close_structure(
                    left,
                    right,
                    "{}[{}]".format(context, index),
                )
            return
        self.assertEqual(expected, actual, context)

    def assert_pair_state(
        self,
        algorithm,
        robot,
        hardware,
        *,
        context,
        compare_candidates=True,
    ):
        case = CASES[algorithm]
        if compare_candidates:
            self.assertEqual(
                list(robot.allocator._candidate_cells(robot)),
                list(hardware[case["candidate"]]()),
                "{} candidate order".format(context),
            )
        self.assertEqual(
            list(robot.belief.known_clues),
            list(hardware["clues"]),
            "{} clues".format(context),
        )
        self.assertEqual(
            robot.current_goal,
            hardware["current_task_cell"],
            "{} wrapper goal".format(context),
        )
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                cell = (x, y)
                index = hardware["idx"](x, y)
                self.assertEqual(
                    cell in robot.belief.searched,
                    hardware["grid"][index]
                    == hardware["CELL_SEARCHED"],
                    "{} searched {!r}".format(context, cell),
                )
                self.assert_close_structure(
                    robot.belief.probability(cell),
                    hardware["target_p"][index],
                    "{} probability {!r}".format(context, cell),
                )
        for name in case["discrete"]:
            self.assertEqual(
                _plain(getattr(robot, name)),
                _plain(hardware[name]),
                "{} {}".format(context, name),
            )
        for name in case["owner_maps"]:
            self.assertEqual(
                _plain(getattr(robot, name)),
                _hardware_map(hardware, name),
                "{} {}".format(context, name),
            )
        for name in case["numeric_maps"]:
            self.assert_close_structure(
                _plain(getattr(robot, name)),
                _hardware_map(hardware, name),
                "{} {}".format(context, name),
            )
        for name in case["scalars"]:
            self.assert_close_structure(
                getattr(robot, name),
                hardware[name],
                "{} {}".format(context, name),
            )

    def assert_outbound_matches(
        self, algorithm, robot, hardware, *, context
    ):
        case = CASES[algorithm]
        start = len(hardware["_sent"])
        simulator_messages = robot.allocator.make_messages(robot)
        hardware[case["flush"]]()
        expected = _simulator_logical_messages(
            algorithm, simulator_messages
        )
        actual = _hardware_logical_messages(
            algorithm,
            hardware["ROBOT_ID"],
            hardware["_sent"][start:],
        )
        self.assertEqual(
            len(expected),
            len(actual),
            "{} outbound message count".format(context),
        )
        self.assert_close_structure(
            expected,
            actual,
            "{} outbound messages".format(context),
        )

    def choose_and_set_wrapper_goal(
        self, algorithm, robot, hardware, *, context
    ):
        simulator_goal = robot.allocator.choose_goal(robot).goal
        hardware_goal = hardware["_pick_task_cell_impl"]()
        self.assertEqual(
            simulator_goal,
            hardware_goal,
            "{} selected goal".format(context),
        )
        robot.current_goal = simulator_goal
        hardware["current_task_cell"] = hardware_goal
        return simulator_goal

    def test_initial_allocation_matches_at_every_study_topk(self):
        for algorithm, case in CASES.items():
            for top_k_limit in TOP_K_LIMITS:
                with self.subTest(
                    algorithm=algorithm, top_k_limit=top_k_limit
                ):
                    robot, hardware = _prepare_pair(
                        algorithm, top_k_limit
                    )
                    simulator_candidates = list(
                        robot.allocator._candidate_cells(robot)
                    )
                    hardware_candidates = list(
                        hardware[case["candidate"]]()
                    )
                    self.assertEqual(
                        simulator_candidates,
                        hardware_candidates,
                        "candidate order",
                    )

                    if algorithm == "hipc":
                        simulator_team = (
                            robot.allocator._hipc_team_agents(robot)
                        )
                        hardware_team = hardware["_hipc_team_agents"]()
                        self.assertEqual(simulator_team, hardware_team)
                        simulator_plan = (
                            robot.allocator._run_local_team_taa(
                                robot,
                                simulator_team,
                                simulator_candidates,
                            )
                        )
                        hardware_plan = hardware[
                            "_hipc_run_local_team_taa"
                        ](
                            hardware_team,
                            hardware_candidates,
                            hardware[
                                "allocation_probability_normalizer"
                            ],
                        )
                        self.assertEqual(simulator_plan, hardware_plan)

                    simulator_decision = robot.allocator.choose_goal(robot)
                    hardware_goal = hardware[
                        "_pick_task_cell_impl"
                    ]()
                    self.assertEqual(
                        simulator_decision.goal,
                        hardware_goal,
                        "selected goal",
                    )

                    for name in case["discrete"]:
                        self.assertEqual(
                            _plain(getattr(robot, name)),
                            _plain(hardware[name]),
                            name,
                        )
                    for name in case["owner_maps"]:
                        self.assertEqual(
                            _plain(getattr(robot, name)),
                            _hardware_map(hardware, name),
                            name,
                        )
                    for name in case["numeric_maps"]:
                        self.assert_close_structure(
                            _plain(getattr(robot, name)),
                            _hardware_map(hardware, name),
                            name,
                        )
                    for name in case["scalars"]:
                        self.assert_close_structure(
                            getattr(robot, name),
                            hardware[name],
                            name,
                        )

                    if algorithm == "hipc":
                        self.assertEqual(
                            robot.hipc_last_predicted_team_plan,
                            hardware_plan,
                        )
                        self.assertEqual(
                            robot.hipc_last_team_size,
                            len(hardware_team),
                        )
                        self.assertEqual(
                            robot.hipc_last_candidate_count,
                            len(hardware_candidates),
                        )

                    simulator_messages = (
                        robot.allocator.make_messages(robot)
                    )
                    hardware[case["flush"]]()
                    expected_messages = _simulator_logical_messages(
                        algorithm, simulator_messages
                    )
                    actual_messages = _hardware_logical_messages(
                        algorithm,
                        hardware["ROBOT_ID"],
                        hardware["_sent"],
                    )
                    self.assertEqual(
                        len(expected_messages),
                        len(actual_messages),
                        "outbound message count",
                    )
                    self.assert_close_structure(
                        expected_messages,
                        actual_messages,
                        "outbound messages",
                    )

    def test_stateful_event_replay_matches_at_every_study_topk(self):
        """Replay retained-goal, consensus, completion, and collision events."""

        for algorithm in CASES:
            for top_k_limit in TOP_K_LIMITS:
                with self.subTest(
                    algorithm=algorithm, top_k_limit=top_k_limit
                ):
                    robot, hardware = _prepare_pair(
                        algorithm, top_k_limit
                    )
                    initial_goal = self.choose_and_set_wrapper_goal(
                        algorithm,
                        robot,
                        hardware,
                        context="initial",
                    )
                    self.assertIsNotNone(initial_goal)
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="initial",
                    )
                    self.assert_outbound_matches(
                        algorithm,
                        robot,
                        hardware,
                        context="initial",
                    )

                    # A normal non-goal arrival updates belief and position but
                    # must retain the active allocator task and consensus state.
                    retained_state = tuple(
                        _plain(getattr(robot, name))
                        for name in CASES[algorithm]["discrete"]
                    )
                    intermediate = _intermediate_cell(robot, initial_goal)
                    robot.pos = intermediate
                    robot.belief.mark_searched(intermediate)
                    hardware["pos"][:] = list(intermediate)
                    _sync_hardware_belief(robot, hardware)
                    self.assertEqual(
                        retained_state,
                        tuple(
                            _plain(getattr(robot, name))
                            for name in CASES[algorithm]["discrete"]
                        ),
                    )
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="intermediate miss",
                    )
                    # The shell services allocator output after an arrival.
                    # CBAA may refresh its retained bid; snapshot allocators
                    # remain silent when their state did not change.
                    self.assert_outbound_matches(
                        algorithm,
                        robot,
                        hardware,
                        context="intermediate miss",
                    )
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="post-miss flush",
                    )

                    # A later forwarded clue reshapes probability but does not
                    # itself discard the active task or allocator state.
                    before_clue_state = tuple(
                        _plain(getattr(robot, name))
                        for name in CASES[algorithm]["discrete"]
                    )
                    later_clue = _next_unsearched_cell(
                        robot,
                        excluded=(
                            robot.current_goal,
                            *getattr(robot, algorithm + "_path", []),
                        ),
                    )
                    robot.receive_message(
                        Message(
                            sender="01",
                            topic=topic_for("01", "clue"),
                            payload={"loc": list(later_clue)},
                            created_at_s=1.0,
                            delivered_at_s=1.0,
                        )
                    )
                    _sync_hardware_belief(robot, hardware)
                    self.assertEqual(
                        before_clue_state,
                        tuple(
                            _plain(getattr(robot, name))
                            for name in CASES[algorithm]["discrete"]
                        ),
                    )
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="later clue",
                    )

                    # A newly delivered peer position is one canonical miss.
                    # It changes belief/peer knowledge without touching safety
                    # intent or forcing allocation while the goal is valid.
                    peer_cell = _next_unsearched_cell(
                        robot,
                        excluded=(
                            robot.current_goal,
                            *getattr(robot, algorithm + "_path", []),
                        ),
                    )
                    robot.receive_message(
                        Message(
                            sender="01",
                            topic=topic_for("01", "state"),
                            payload={"loc": list(peer_cell)},
                            created_at_s=2.0,
                            delivered_at_s=2.0,
                        )
                    )
                    hardware["peer_pos"]["01"] = peer_cell
                    _sync_hardware_belief(robot, hardware)
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="peer state miss",
                    )

                    # Deliver an exact-score claim from a lower robot ID for
                    # the current head. This crosses message parsing, EPS tie
                    # resolution, repair, wrapper-goal synchronization, and
                    # deduplicated forwarding.
                    message, payload = _forced_lower_id_claim(
                        algorithm, robot, hardware
                    )
                    robot.allocator.on_message(robot, message)
                    hardware[
                        "_" + algorithm + "_receive_payload"
                    ]("01", payload)
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="peer allocator message",
                    )

                    if robot.current_goal is None:
                        self.choose_and_set_wrapper_goal(
                            algorithm,
                            robot,
                            hardware,
                            context="ownership-change reallocation",
                        )
                    self.assert_outbound_matches(
                        algorithm,
                        robot,
                        hardware,
                        context="ownership-change service",
                    )
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="ownership-change service",
                    )

                    # Complete the retained/reheaded goal. Allocator cleanup is
                    # intentionally deferred until this choose boundary.
                    completed = robot.current_goal
                    self.assertIsNotNone(completed)
                    robot.pos = completed
                    robot.belief.mark_searched(completed)
                    robot.current_goal = None
                    hardware["pos"][:] = list(completed)
                    _sync_hardware_belief(robot, hardware)
                    hardware["current_task_cell"] = None
                    self.choose_and_set_wrapper_goal(
                        algorithm,
                        robot,
                        hardware,
                        context="goal completion",
                    )
                    self.assert_outbound_matches(
                        algorithm,
                        robot,
                        hardware,
                        context="goal completion",
                    )
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="goal completion",
                    )

                    # Canonical two-conflict handling reaches the allocator as
                    # a collision rising edge after the wrapper goal is
                    # cleared. Hardware represents that edge with its pending
                    # collision flag.
                    robot.current_goal = None
                    robot.collision_avoidance_active = True
                    hardware[
                        "_" + algorithm
                        + "_defer_collision_reallocation"
                    ]()
                    self.choose_and_set_wrapper_goal(
                        algorithm,
                        robot,
                        hardware,
                        context="collision rising edge",
                    )
                    self.assert_outbound_matches(
                        algorithm,
                        robot,
                        hardware,
                        context="collision rising edge",
                    )
                    self.assert_pair_state(
                        algorithm,
                        robot,
                        hardware,
                        context="collision rising edge",
                    )

    def test_claim_decisions_match_around_eps(self):
        eps = 1.0e-9
        advantages = (-1.5 * eps, 0.5 * eps, 1.5 * eps)
        for algorithm, case in CASES.items():
            for claimant, expected in (
                ("03", (False, False, True)),
                ("01", (False, True, True)),
            ):
                with self.subTest(
                    algorithm=algorithm, claimant=claimant
                ):
                    robot, hardware = _prepare_pair(algorithm, 18)
                    robot.rid = claimant
                    hardware["ROBOT_ID"] = claimant
                    owner = "02"
                    if algorithm == "pi":
                        known = 1.0
                        simulator_results = [
                            robot.allocator._can_include(
                                robot,
                                owner,
                                known,
                                known - advantage,
                            )
                            for advantage in advantages
                        ]
                        hardware_results = [
                            hardware["_pi_can_include"](
                                owner,
                                known,
                                known - advantage,
                            )
                            for advantage in advantages
                        ]
                    else:
                        cell = (5, 5)
                        winner_name = {
                            "acbba": "acbba_winner_by_cell",
                            "cbaa": "cbaa_winner_by_cell",
                            "hipc": "hipc_winner_by_cell",
                        }[algorithm]
                        bid_name = {
                            "acbba": "acbba_winning_bid_by_cell",
                            "cbaa": "cbaa_winning_bid_by_cell",
                            "hipc": "hipc_winning_bid_by_cell",
                        }[algorithm]
                        setattr(robot, winner_name, {cell: owner})
                        setattr(robot, bid_name, {cell: 0.0})
                        hardware[winner_name][cell] = owner
                        hardware[bid_name][cell] = 0.0
                        simulator_results = [
                            robot.allocator._can_claim(
                                robot, cell, advantage
                            )
                            for advantage in advantages
                        ]
                        hardware_results = [
                            hardware[
                                "_" + algorithm + "_can_claim"
                            ](cell, advantage)
                            for advantage in advantages
                        ]
                    self.assertEqual(simulator_results, hardware_results)
                    self.assertEqual(tuple(simulator_results), expected)


if __name__ == "__main__":
    unittest.main()
