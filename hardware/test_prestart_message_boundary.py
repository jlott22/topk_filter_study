"""Trial-boundary regressions for the six onboard message handlers.

The Pololu programs initialize hardware at module scope, so these tests execute
only AST-extracted message and belief helpers.
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

from hardware.allocator_memory import require_binary64


HARDWARE_DIR = Path(__file__).resolve().parent
PROGRAMS = (
    "Pololu_ACBBA.py",
    "Pololu_CBAA.py",
    "Pololu_DGA.py",
    "Pololu_DMCHBA.py",
    "Pololu_HIPC.py",
    "Pololu_PI.py",
)
ACD_PROGRAMS = {
    "Pololu_ACBBA.py",
    "Pololu_CBAA.py",
    "Pololu_DGA.py",
}
ALLOCATOR_RECEIVERS = {
    "Pololu_ACBBA.py": "_acbba_receive_payload",
    "Pololu_CBAA.py": "_cbaa_receive_payload",
    "Pololu_DGA.py": "_dga_receive_payload",
    "Pololu_DMCHBA.py": None,
    "Pololu_HIPC.py": "_hipc_receive_payload",
    "Pololu_PI.py": "_pi_receive_payload",
}


class _RandomStub:
    def __init__(self, value=0.5):
        self.value = value
        self.calls = 0

    def random(self):
        self.calls += 1
        return self.value


def _extract(path, names, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names = set(names)
    if "handle_msg" in names:
        names.add("_trial_traffic_enabled")
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


def _base_namespace():
    return {
        "ROBOT_ID": "03",
        "GRID_SIZE": 5,
        "CELL_SEARCHED": 1,
        "grid": bytearray(25),
        "pos": [0, 0],
        "pre_start_signal": False,
        "start_signal": False,
        "control_state": "BOOT",
        "trial_active": False,
        "returning_home": False,
        "metrics_frozen": False,
        "peer_pos": {"01": (4, 4)},
        "peer_pos_yield": {"01": (4, 4)},
        "peer_intent": {"01": (3, 4)},
        "current_task_cell": (2, 3),
        "first_clue_seen": False,
        "target_location": None,
        "found_target": False,
        "move_forward_flag": False,
        "published_intent": None,
        "communicated_intent": None,
        "msg_drop_rate": 0.0,
        "topic_1_rec": 11,
        "topic_2_rec": 12,
        "topic_3_rec": 13,
        "topic_4_rec": 14,
        "topic_5_rec": 15,
        "clues": [],
        "idx": lambda x, y: y * 5 + x,
        "random": _RandomStub(),
        "gc": SimpleNamespace(collect=lambda: None),
        "publish_clue": lambda *_args: None,
        "freeze_trial_metrics": lambda: None,
    }


class PrestartMessageBoundaryTests(unittest.TestCase):
    def test_stale_prestart_state_allocator_clue_and_target_are_isolated(self):
        frames = (
            "021.2,3",
            "023.stale-allocation",
            "024.1,2",
            "025.3,4",
        )
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                calls = []
                namespace = _base_namespace()
                namespace.update(
                    {
                        "mark_cell_searched_miss": (
                            lambda *_args: calls.append("miss")
                        ),
                        "update_target_on_miss": (
                            lambda *_args: calls.append("miss")
                        ),
                        "update_prob_map": (
                            lambda: calls.append("belief")
                        ),
                        "add_clue_if_new": (
                            lambda *_args: calls.append("clue")
                        ),
                        "_acbba_receive_payload": (
                            lambda *_args: calls.append("allocation")
                        ),
                        "_cbaa_receive_payload": (
                            lambda *_args: calls.append("allocation")
                        ),
                        "_dga_receive_payload": (
                            lambda *_args: calls.append("allocation")
                        ),
                        "_hipc_receive_payload": (
                            lambda *_args: calls.append("allocation")
                        ),
                        "_pi_receive_payload": (
                            lambda *_args: calls.append("allocation")
                        ),
                    }
                )
                initial_counters = tuple(
                    namespace["topic_{}_rec".format(topic)]
                    for topic in range(1, 6)
                )
                initial_grid = bytes(namespace["grid"])
                initial_peer_pos = dict(namespace["peer_pos"])
                initial_goal = namespace["current_task_cell"]
                _extract(path, {"handle_msg"}, namespace)

                for frame in frames:
                    namespace["handle_msg"](frame)

                self.assertEqual(namespace["peer_pos"], initial_peer_pos)
                self.assertEqual(bytes(namespace["grid"]), initial_grid)
                self.assertEqual(
                    namespace["current_task_cell"], initial_goal
                )
                self.assertFalse(namespace["first_clue_seen"])
                self.assertEqual(namespace["clues"], [])
                self.assertIsNone(namespace["target_location"])
                self.assertFalse(namespace["found_target"])
                self.assertEqual(
                    tuple(
                        namespace["topic_{}_rec".format(topic)]
                        for topic in range(1, 6)
                    ),
                    initial_counters,
                )
                self.assertEqual(calls, [])
                self.assertEqual(namespace["random"].calls, 0)

    def test_return_home_state_updates_routing_only(self):
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                calls = []
                namespace = _base_namespace()
                namespace["returning_home"] = True
                namespace["mark_cell_searched_miss"] = (
                    lambda *_args: calls.append("belief")
                )
                namespace["update_target_on_miss"] = (
                    lambda *_args: calls.append("belief")
                )
                _extract(path, {"handle_msg"}, namespace)

                namespace["handle_msg"]("021.2,3")

                self.assertEqual(namespace["peer_pos"]["02"], (2, 3))
                self.assertEqual(namespace["grid"][17], 0)
                self.assertEqual(namespace["current_task_cell"], (2, 3))
                self.assertEqual(namespace["topic_1_rec"], 11)
                self.assertEqual(calls, [])
                self.assertEqual(namespace["random"].calls, 0)

    def test_armed_run_window_marks_active_miss_once(self):
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                belief_calls = []
                namespace = _base_namespace()
                namespace.update(
                    {
                        "start_signal": False,
                        "control_state": "STARTED",
                        # Deliberately model a late RUN delivery after another
                        # robot has already started publishing trial traffic.
                        "trial_active": False,
                        "topic_1_rec": 0,
                        "peer_pos": {},
                        "update_prob_map": (
                            lambda: belief_calls.append("recomputed")
                        ),
                        "update_target_on_miss": (
                            lambda _index: belief_calls.append("recomputed")
                        ),
                    }
                )
                names = {"handle_msg"}
                if filename in ACD_PROGRAMS:
                    names.add("mark_cell_searched_miss")
                _extract(path, names, namespace)

                namespace["handle_msg"]("021.2,3")
                namespace["handle_msg"]("021.2,3")

                self.assertEqual(namespace["peer_pos"], {"02": (2, 3)})
                self.assertEqual(namespace["grid"][17], 1)
                self.assertIsNone(namespace["current_task_cell"])
                self.assertEqual(namespace["topic_1_rec"], 2)
                self.assertEqual(belief_calls, ["recomputed"])
                self.assertEqual(namespace["random"].calls, 2)

    def test_armed_run_window_accepts_t0_clue(self):
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                belief_calls = []
                published = []
                namespace = _base_namespace()
                namespace.update(
                    {
                        "start_signal": False,
                        "control_state": "STARTED",
                        "trial_active": False,
                        "topic_4_rec": 0,
                        "update_prob_map": (
                            lambda: belief_calls.append("recomputed")
                        ),
                        "publish_clue": (
                            lambda x, y: published.append((x, y))
                        ),
                    }
                )
                names = {"handle_msg"}
                if filename in ACD_PROGRAMS:
                    names.add("add_clue_if_new")
                _extract(path, names, namespace)

                namespace["handle_msg"]("024.1,2")

                self.assertEqual(namespace["clues"], [(1, 2)])
                self.assertEqual(namespace["grid"][11], 1)
                self.assertTrue(namespace["first_clue_seen"])
                self.assertEqual(namespace["topic_4_rec"], 1)
                self.assertEqual(belief_calls, ["recomputed"])
                self.assertEqual(published, [(1, 2)])

    def test_armed_run_window_accepts_allocator_messages(self):
        for filename, receiver_name in ALLOCATOR_RECEIVERS.items():
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                received = []
                namespace = _base_namespace()
                namespace.update(
                    {
                        "start_signal": False,
                        "control_state": "STARTED",
                        "trial_active": False,
                        "topic_3_rec": 0,
                        "_acbba_receive_payload": (
                            lambda *args: received.append(args)
                        ),
                        "_cbaa_receive_payload": (
                            lambda *args: received.append(args)
                        ),
                        "_dga_receive_payload": (
                            lambda *args: received.append(args)
                        ),
                        "_hipc_receive_payload": (
                            lambda *args: received.append(args)
                        ),
                        "_pi_receive_payload": (
                            lambda *args: received.append(args)
                        ),
                    }
                )
                _extract(path, {"handle_msg"}, namespace)

                namespace["handle_msg"]("023.t0-allocation")

                if receiver_name is None:
                    self.assertEqual(received, [])
                    self.assertEqual(namespace["topic_3_rec"], 0)
                else:
                    self.assertEqual(
                        received, [("02", "t0-allocation")]
                    )
                    self.assertEqual(namespace["topic_3_rec"], 1)

    def test_armed_target_is_retained_without_starting_motion(self):
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                freezes = []
                namespace = _base_namespace()
                namespace.update(
                    {
                        "start_signal": False,
                        "control_state": "STARTED",
                        "trial_active": False,
                        "topic_5_rec": 0,
                        "move_forward_flag": True,
                        "freeze_trial_metrics": (
                            lambda: freezes.append(True)
                        ),
                    }
                )
                _extract(path, {"handle_msg"}, namespace)

                namespace["handle_msg"]("025.3,4")

                self.assertEqual(namespace["target_location"], (3, 4))
                self.assertTrue(namespace["found_target"])
                self.assertFalse(namespace["move_forward_flag"])
                self.assertEqual(namespace["topic_5_rec"], 1)
                self.assertEqual(freezes, [])

    def test_protected_intent_remains_available_before_start(self):
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                namespace = _base_namespace()
                namespace.update(
                    {
                        "topic_2_rec": 0,
                        "peer_pos_yield": {},
                        "peer_intent": {},
                    }
                )
                _extract(path, {"handle_msg"}, namespace)

                namespace["handle_msg"]("022.1,1,2,2")

                self.assertEqual(namespace["peer_pos_yield"], {"02": (1, 1)})
                self.assertEqual(namespace["peer_intent"], {"02": (2, 2)})
                self.assertEqual(namespace["topic_2_rec"], 1)


class Binary64StartupProbeTests(unittest.TestCase):
    def test_probe_checks_arithmetic_and_array_round_trip(self):
        self.assertTrue(require_binary64())

        def collapsed_storage(_typecode, _values):
            return [1.0]

        with self.assertRaisesRegex(RuntimeError, "round-trip"):
            require_binary64(collapsed_storage)

        helper_path = HARDWARE_DIR / "allocator_memory.py"
        tree = ast.parse(
            helper_path.read_text(encoding="utf-8"),
            filename=str(helper_path),
        )
        probe = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "require_binary64"
        )
        constants = {
            node.value
            for node in ast.walk(probe)
            if isinstance(node, ast.Constant)
            and isinstance(node.value, float)
        }
        self.assertIn(2.220446049250313e-16, constants)
        self.assertTrue(
            any(
                isinstance(node, ast.BinOp)
                and isinstance(node.op, ast.Add)
                for node in ast.walk(probe)
            )
        )

    def test_every_program_runs_probe_at_module_startup(self):
        for filename in PROGRAMS:
            with self.subTest(program=filename):
                path = HARDWARE_DIR / filename
                tree = ast.parse(
                    path.read_text(encoding="utf-8"), filename=str(path)
                )
                imported = any(
                    isinstance(node, ast.ImportFrom)
                    and node.module == "allocator_memory"
                    and any(
                        alias.name == "require_binary64"
                        for alias in node.names
                    )
                    for node in tree.body
                )
                calls = [
                    node
                    for node in tree.body
                    if isinstance(node, ast.Expr)
                    and isinstance(node.value, ast.Call)
                    and isinstance(node.value.func, ast.Name)
                    and node.value.func.id == "require_binary64"
                ]
                self.assertTrue(imported)
                self.assertEqual(len(calls), 1)


class DelimiterSafeScoreEncodingTests(unittest.TestCase):
    def test_hipc_scientific_notation_round_trips_without_delimiter(self):
        namespace = {
            "HIPC_EMPTY_FIELD": "X",
            "HIPC_NO_BID": -1.0e18,
        }
        _extract(
            HARDWARE_DIR / "Pololu_HIPC.py",
            {"_hipc_encode_signed", "_hipc_decode_signed"},
            namespace,
        )
        encode = namespace["_hipc_encode_signed"]
        decode = namespace["_hipc_decode_signed"]
        for value in (
            5.0e-10,
            -5.0e-10,
            1.2345678901234567,
            -1.2345678901234567,
        ):
            with self.subTest(value=value):
                encoded = encode(value, namespace["HIPC_NO_BID"])
                self.assertNotIn("-", encoded)
                self.assertEqual(
                    encoded.count("N"),
                    "{:.17g}".format(value).count("-"),
                )
                self.assertEqual(
                    decode(encoded, namespace["HIPC_NO_BID"]), value
                )
                self.assertEqual((encoded + "-").count("-"), 1)

    def test_pi_scientific_notation_round_trips_without_delimiter(self):
        namespace = {
            "PI_EMPTY_FIELD": "X",
            "PI_INF_SIGNIFICANCE": 1.0e18,
        }
        _extract(
            HARDWARE_DIR / "Pololu_PI.py",
            {"_pi_encode_significance", "_pi_decode_significance"},
            namespace,
        )
        encode = namespace["_pi_encode_significance"]
        decode = namespace["_pi_decode_significance"]
        for value in (
            5.0e-10,
            -5.0e-10,
            1.2345678901234567,
            -1.2345678901234567,
        ):
            with self.subTest(value=value):
                encoded = encode(value)
                self.assertNotIn("-", encoded)
                self.assertEqual(
                    encoded.count("N"),
                    "{:.17g}".format(value).count("-"),
                )
                self.assertEqual(decode(encoded), value)
                self.assertEqual((encoded + "-").count("-"), 1)


if __name__ == "__main__":
    unittest.main()
