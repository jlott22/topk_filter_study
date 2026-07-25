from __future__ import annotations

import ast
import gc
import io
import threading
import unittest
from array import array
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from hardware import metrics_hub
from hardware.allocator_memory import PackedCandidateWorkspace


HARDWARE_DIR = Path(__file__).resolve().parent
POLULU_FILES = {
    "Pololu_ACBBA.py": "ACBBA",
    "Pololu_CBAA.py": "CBAA",
    "Pololu_DGA.py": "DGA",
    "Pololu_DMCHBA.py": "DMCHBA",
    "Pololu_HIPC.py": "HIPC",
    "Pololu_PI.py": "PI",
}


class _UART:
    def __init__(self):
        self.messages = []

    def write(self, message):
        self.messages.append(message)
        return len(message)


def _configuration_namespace(filename, algorithm, apply_capacity=None):
    path = HARDWARE_DIR / filename
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name
        in {
            "_send_config_ack",
            "_valid_scenario_sha256",
            "_handle_config_command",
        }
    ]
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    calls = []

    def apply(value):
        calls.append(value)
        if apply_capacity is not None:
            apply_capacity(value)

    namespace = {
        "ALGORITHM_NAME": algorithm,
        "GRID_SIZE": 19,
        "CONFIG_RATE_SCALE": 1_000_000,
        "TRIAL_MODE": "clue_search",
        "COMMITMENT_HORIZON": 1 if algorithm == "CBAA" else 3,
        "LOGIC_REVISION": "dcta_parity_v1",
        "TOP_K_PERCENT": 1.0,
        "TOP_K_MAX_CELLS": 361,
        "msg_drop_rate": 0.0,
        "applied_config_sequence": 0,
        "applied_top_k_ppm": 1_000_000,
        "applied_drop_ppm": 0,
        "applied_trial_mode": "clue_search",
        "applied_commitment_horizon": (
            1 if algorithm == "CBAA" else 3
        ),
        "applied_logic_revision": "dcta_parity_v1",
        "applied_scenario_sha256": "",
        "last_config_request": None,
        "last_config_status": "OK",
        "control_state": "BOOT",
        "trial_active": False,
        "start_signal": False,
        "pre_start_signal": False,
        "returning_home": False,
        "_apply_top_k_capacity": apply,
        "uart": _UART(),
    }
    namespace["_uart_send_text"] = lambda topic, payload, count_bytes=True: (
        namespace["uart"].write("{}.{}-".format(topic, payload))
    )
    exec(compile(module, str(path), "exec"), namespace)
    namespace["_capacity_calls"] = calls
    return namespace


def _extract_function(filename, function_name, namespace):
    path = HARDWARE_DIR / filename
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == function_name
    )
    module = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[function_name]


class RateConversionTests(unittest.TestCase):
    def test_default_parser_values(self):
        args = metrics_hub.parser().parse_args([])
        self.assertEqual(args.top_k_rate, 1.0)
        self.assertEqual(args.drop_rate, 0.0)

    def test_study_rates_resolve_with_round_half_up(self):
        expected = {
            "1": 361,
            ".75": 271,
            ".50": 181,
            ".25": 90,
            ".10": 36,
            ".05": 18,
        }
        for value, cells in expected.items():
            with self.subTest(value=value):
                ppm = metrics_hub.rate_to_ppm(value, allow_zero=False)
                self.assertEqual(metrics_hub.top_k_cells(19, ppm), cells)

    def test_drop_rate_accepts_zero_and_rejects_out_of_range(self):
        self.assertEqual(
            metrics_hub.rate_to_ppm("0", allow_zero=True), 0
        )
        self.assertEqual(
            metrics_hub.rate_to_ppm("0.125", allow_zero=True), 125000
        )
        with self.assertRaises(ValueError):
            metrics_hub.rate_to_ppm("0", allow_zero=False)
        with self.assertRaises(ValueError):
            metrics_hub.rate_to_ppm("1.01", allow_zero=True)

    def test_config_ack_parser(self):
        manifest = "a" * 64
        acknowledgment = metrics_hub.parse_config_ack(
            "CFGACK,7,DGA,750000,271,250000,clue_search,3,"
            "dcta_parity_v1,{},OK".format(manifest)
        )
        self.assertEqual(acknowledgment["sequence"], 7)
        self.assertEqual(acknowledgment["algorithm"], "DGA")
        self.assertEqual(acknowledgment["top_k_max_cells"], 271)
        self.assertEqual(acknowledgment["trial_mode"], "clue_search")
        self.assertEqual(acknowledgment["commitment_horizon"], 3)
        self.assertEqual(acknowledgment["logic_revision"], "dcta_parity_v1")
        self.assertEqual(acknowledgment["scenario_sha256"], manifest)
        self.assertIsNone(metrics_hub.parse_config_ack("bad"))


class PololuConfigurationTests(unittest.TestCase):
    def test_every_algorithm_applies_and_acknowledges_configuration(self):
        manifest = "a" * 64
        for filename, algorithm in POLULU_FILES.items():
            with self.subTest(filename=filename):
                namespace = _configuration_namespace(filename, algorithm)
                horizon = 1 if algorithm == "CBAA" else 3
                namespace["_handle_config_command"](
                    "CFG,7,750000,271,250000,clue_search,{},"
                    "dcta_parity_v1,{}".format(horizon, manifest)
                )
                self.assertEqual(namespace["TOP_K_PERCENT"], 0.75)
                self.assertEqual(namespace["TOP_K_MAX_CELLS"], 271)
                self.assertEqual(namespace["msg_drop_rate"], 0.25)
                self.assertEqual(namespace["applied_config_sequence"], 7)
                self.assertEqual(
                    namespace["applied_scenario_sha256"], manifest
                )
                self.assertEqual(namespace["_capacity_calls"], [271])
                self.assertEqual(
                    namespace["uart"].messages[-1],
                    (
                        "6.CFGACK,7,{},750000,271,250000,clue_search,{},"
                        "dcta_parity_v1,{},OK-"
                    ).format(
                        algorithm, horizon, manifest
                    ),
                )

                namespace["_handle_config_command"](
                    "CFG,7,750000,271,250000,clue_search,{},"
                    "dcta_parity_v1,{}".format(horizon, manifest)
                )
                self.assertEqual(namespace["_capacity_calls"], [271])
                self.assertEqual(len(namespace["uart"].messages), 2)

    def test_delayed_older_sequence_cannot_roll_back_configuration(self):
        manifest = "a" * 64
        for filename, algorithm in POLULU_FILES.items():
            with self.subTest(filename=filename):
                namespace = _configuration_namespace(
                    filename, algorithm
                )
                horizon = 1 if algorithm == "CBAA" else 3
                namespace["_handle_config_command"](
                    "CFG,2,750000,271,250000,clue_search,{},"
                    "dcta_parity_v1,{}".format(horizon, manifest)
                )
                applied = (
                    namespace["applied_config_sequence"],
                    namespace["TOP_K_PERCENT"],
                    namespace["TOP_K_MAX_CELLS"],
                    namespace["msg_drop_rate"],
                    namespace["applied_scenario_sha256"],
                )

                namespace["_handle_config_command"](
                    "CFG,1,500000,181,100000,clue_search,{},"
                    "dcta_parity_v1,{}".format(horizon, manifest)
                )

                self.assertTrue(
                    namespace["uart"].messages[-1].endswith("INVALID-")
                )
                self.assertEqual(
                    (
                        namespace["applied_config_sequence"],
                        namespace["TOP_K_PERCENT"],
                        namespace["TOP_K_MAX_CELLS"],
                        namespace["msg_drop_rate"],
                        namespace["applied_scenario_sha256"],
                    ),
                    applied,
                )
                self.assertEqual(namespace["_capacity_calls"], [271])

                # A retry of the still-applied request remains idempotently OK
                # even though the intervening stale frame replaced the
                # last-request cache.
                namespace["_handle_config_command"](
                    "CFG,2,750000,271,250000,clue_search,{},"
                    "dcta_parity_v1,{}".format(horizon, manifest)
                )
                self.assertTrue(
                    namespace["uart"].messages[-1].endswith("OK-")
                )
                self.assertEqual(namespace["_capacity_calls"], [271])

    def test_invalid_or_active_configuration_is_rejected(self):
        manifest = "a" * 64
        namespace = _configuration_namespace("Pololu_DGA.py", "DGA")
        namespace["_handle_config_command"](
            "CFG,1,750000,270,100000,clue_search,3,"
            "dcta_parity_v1,{}".format(manifest)
        )
        self.assertTrue(namespace["uart"].messages[-1].endswith("INVALID-"))
        self.assertEqual(namespace["_capacity_calls"], [])

        namespace["trial_active"] = True
        namespace["_handle_config_command"](
            "CFG,2,750000,271,100000,clue_search,3,"
            "dcta_parity_v1,{}".format(manifest)
        )
        self.assertTrue(namespace["uart"].messages[-1].endswith("INVALID-"))
        self.assertEqual(namespace["_capacity_calls"], [])

    def test_mode_horizon_revision_and_manifest_must_match(self):
        manifest = "a" * 64
        bad_payloads = (
            "CFG,4,750000,271,0,coverage,3,dcta_parity_v1,{}".format(
                manifest
            ),
            "CFG,4,750000,271,0,clue_search,2,dcta_parity_v1,{}".format(
                manifest
            ),
            "CFG,4,750000,271,0,clue_search,3,stale-build,{}".format(
                manifest
            ),
            "CFG,4,750000,271,0,clue_search,3,dcta_parity_v1,not-a-hash",
        )
        for payload in bad_payloads:
            with self.subTest(payload=payload):
                namespace = _configuration_namespace(
                    "Pololu_DGA.py", "DGA"
                )
                namespace["_handle_config_command"](payload)
                self.assertTrue(
                    namespace["uart"].messages[-1].endswith("INVALID-")
                )
                self.assertEqual(namespace["_capacity_calls"], [])

    def test_memory_failure_is_reported_without_applying_values(self):
        def fail(_capacity):
            raise MemoryError

        namespace = _configuration_namespace(
            "Pololu_DMCHBA.py", "DMCHBA", fail
        )
        namespace["_handle_config_command"](
            "CFG,3,500000,181,500000,clue_search,3,"
            "dcta_parity_v1,{}".format("a" * 64)
        )
        self.assertEqual(namespace["TOP_K_PERCENT"], 1.0)
        self.assertEqual(namespace["TOP_K_MAX_CELLS"], 361)
        self.assertEqual(namespace["msg_drop_rate"], 0.0)
        self.assertTrue(
            namespace["uart"].messages[-1].endswith("MEMORY_ERROR-")
        )

    def test_every_file_defaults_to_full_grid_and_logs_runtime_values(self):
        for filename, algorithm in POLULU_FILES.items():
            with self.subTest(filename=filename):
                tree = ast.parse(
                    (HARDWARE_DIR / filename).read_text(encoding="utf-8")
                )
                assignments = {}
                for node in tree.body:
                    if isinstance(node, ast.Assign):
                        for target in node.targets:
                            if isinstance(target, ast.Name):
                                assignments[target.id] = node.value
                self.assertEqual(
                    ast.literal_eval(assignments["TOP_K_PERCENT"]), 1.0
                )
                self.assertEqual(
                    ast.literal_eval(assignments["ALGORITHM_NAME"]),
                    algorithm,
                )
                logic_revision = ast.literal_eval(
                    assignments["LOGIC_REVISION"]
                )
                self.assertEqual(logic_revision, "dcta_parity_v1")
                self.assertNotIn(
                    "-",
                    logic_revision,
                    "hyphen terminates an ESP32-to-Pololu UART frame",
                )

    def test_packed_candidate_workspaces_resize_to_exact_capacity(self):
        workspace_names = {
            "Pololu_ACBBA.py": "acbba_candidate_workspace",
            "Pololu_CBAA.py": "cbaa_candidate_workspace",
            "Pololu_DGA.py": "dga_candidate_workspace",
            "Pololu_HIPC.py": "hipc_candidate_workspace",
            "Pololu_PI.py": "pi_candidate_workspace",
        }
        for filename, workspace_name in workspace_names.items():
            with self.subTest(filename=filename):
                namespace = {
                    "gc": gc,
                    "GRID_SIZE": 19,
                    "PackedCandidateWorkspace": PackedCandidateWorkspace,
                    workspace_name: PackedCandidateWorkspace(19, 361),
                }
                resize = _extract_function(
                    filename, "_apply_top_k_capacity", namespace
                )
                resize(18)
                self.assertEqual(namespace[workspace_name].capacity, 18)
                resize(271)
                self.assertEqual(namespace[workspace_name].capacity, 271)

    def test_dmchba_rebuilds_every_top_k_dependent_workspace(self):
        namespace = {
            "gc": gc,
            "array": array,
            "NUM_ROBOTS": 4,
            "DMCHBA_MAX_MATRIX_N": 364,
            "dmchba_candidate_ids": array("H", [0] * 361),
            "dmchba_agent_task_costs": [
                array("d", [0.0] * 361) for _ in range(4)
            ],
            "dmchba_h_u": array("d", [0.0] * 365),
            "dmchba_h_v": array("d", [0.0] * 365),
            "dmchba_h_minv": array("d", [0.0] * 365),
            "dmchba_h_p": array("H", [0] * 365),
            "dmchba_h_way": array("H", [0] * 365),
            "dmchba_h_used": bytearray(365),
            "dmchba_h_assignment": array("h", [-1] * 364),
            "dmchba_assigned_ids": array("H", [0] * 361),
        }
        resize = _extract_function(
            "Pololu_DMCHBA.py", "_apply_top_k_capacity", namespace
        )
        resize(18)
        self.assertEqual(namespace["DMCHBA_MAX_MATRIX_N"], 21)
        self.assertEqual(len(namespace["dmchba_candidate_ids"]), 18)
        self.assertEqual(len(namespace["dmchba_agent_task_costs"]), 4)
        self.assertTrue(
            all(
                len(costs) == 18
                for costs in namespace["dmchba_agent_task_costs"]
            )
        )
        self.assertEqual(len(namespace["dmchba_h_u"]), 22)
        self.assertEqual(len(namespace["dmchba_h_assignment"]), 21)
        self.assertEqual(len(namespace["dmchba_assigned_ids"]), 18)


class HubHandshakeTests(unittest.TestCase):
    def _hub(self, algorithm="DGA"):
        hub = object.__new__(metrics_hub.Hub)
        hub.args = SimpleNamespace(
            algorithm=algorithm,
            grid_size=19,
            config_timeout=0.02,
            config_retry_seconds=0.005,
            auto=True,
            trial_mode=metrics_hub.TRIAL_MODE,
            commitment_horizon=3,
            logic_revision=metrics_hub.LOGIC_REVISION,
        )
        hub.ids = ["00", "01"]
        hub.condition = threading.Condition()
        hub.config_sequence = 0
        hub.config_acks = {}
        hub.commands = []
        hub.scenario_sha256 = "b" * 64
        hub.wait_home = lambda: None
        return hub

    def _trial(self):
        scenario = metrics_hub.Scenario("scenario-1", (1, 1), [])
        return metrics_hub.Trial(
            "run-1",
            scenario,
            {"00": metrics_hub.RobotState(), "01": metrics_hub.RobotState()},
        )

    def test_all_matching_acknowledgments_allow_configuration(self):
        hub = self._hub()

        def publish(_topic, payload, _kind, _trial):
            if not payload.startswith("CFG,"):
                return
            sequence = int(payload.split(",")[1])
            with hub.condition:
                for rid in hub.ids:
                    hub.config_acks[rid] = {
                        "sequence": sequence,
                        "algorithm": "DGA",
                        "top_k_ppm": 750000,
                        "top_k_max_cells": 271,
                        "drop_ppm": 250000,
                        "trial_mode": "clue_search",
                        "commitment_horizon": 3,
                        "logic_revision": "dcta_parity_v1",
                        "scenario_sha256": "b" * 64,
                        "status": "OK",
                    }
                hub.condition.notify_all()

        hub.publish = publish
        trial = self._trial()
        hub.configure_robots(trial, 750000, 250000)
        self.assertTrue(trial.algorithm_verified)
        self.assertEqual(trial.top_k_rate, 0.75)
        self.assertEqual(trial.top_k_max_cells, 271)
        self.assertEqual(trial.drop_rate, 0.25)

    def test_algorithm_mismatch_prevents_configuration(self):
        hub = self._hub()

        def publish(_topic, payload, _kind, _trial):
            if not payload.startswith("CFG,"):
                return
            sequence = int(payload.split(",")[1])
            with hub.condition:
                for rid in hub.ids:
                    hub.config_acks[rid] = {
                        "sequence": sequence,
                        "algorithm": "CBAA",
                        "top_k_ppm": 1000000,
                        "top_k_max_cells": 361,
                        "drop_ppm": 0,
                        "trial_mode": "clue_search",
                        "commitment_horizon": 3,
                        "logic_revision": "dcta_parity_v1",
                        "scenario_sha256": "b" * 64,
                        "status": "OK",
                    }
                hub.condition.notify_all()

        hub.publish = publish
        with self.assertRaises(metrics_hub.ConfigurationError):
            hub.configure_robots(self._trial(), 1000000, 0)

    def test_logic_revision_or_manifest_mismatch_prevents_configuration(self):
        for field, wrong_value in (
            ("logic_revision", "stale-build"),
            ("scenario_sha256", "c" * 64),
            ("commitment_horizon", 2),
        ):
            with self.subTest(field=field):
                hub = self._hub()

                def publish(_topic, payload, _kind, _trial):
                    if not payload.startswith("CFG,"):
                        return
                    sequence = int(payload.split(",")[1])
                    acknowledgment = {
                        "sequence": sequence,
                        "algorithm": "DGA",
                        "top_k_ppm": 1000000,
                        "top_k_max_cells": 361,
                        "drop_ppm": 0,
                        "trial_mode": "clue_search",
                        "commitment_horizon": 3,
                        "logic_revision": "dcta_parity_v1",
                        "scenario_sha256": "b" * 64,
                        "status": "OK",
                    }
                    acknowledgment[field] = wrong_value
                    with hub.condition:
                        for rid in hub.ids:
                            hub.config_acks[rid] = dict(acknowledgment)
                        hub.condition.notify_all()

                hub.publish = publish
                with self.assertRaises(metrics_hub.ConfigurationError):
                    hub.configure_robots(self._trial(), 1000000, 0)

    def test_missing_robot_prevents_configuration(self):
        hub = self._hub()

        def publish(_topic, payload, _kind, _trial):
            if not payload.startswith("CFG,"):
                return
            sequence = int(payload.split(",")[1])
            with hub.condition:
                hub.config_acks["00"] = {
                    "sequence": sequence,
                    "algorithm": "DGA",
                    "top_k_ppm": 1000000,
                    "top_k_max_cells": 361,
                    "drop_ppm": 0,
                    "trial_mode": "clue_search",
                    "commitment_horizon": 3,
                    "logic_revision": "dcta_parity_v1",
                    "scenario_sha256": "b" * 64,
                    "status": "OK",
                }
                hub.condition.notify_all()

        hub.publish = publish
        with self.assertRaises(metrics_hub.ConfigurationError):
            hub.configure_robots(self._trial(), 1000000, 0)

    def test_stale_acknowledgment_is_audited_but_not_accepted(self):
        hub = object.__new__(metrics_hub.Hub)
        hub.ids = ["00"]
        hub.condition = threading.Condition()
        hub.last_message = 0.0
        hub.config_sequence = 8
        hub.config_acks = {}
        hub.config_ack_rows = []
        hub.connected_robots = set()
        hub.printed_config_acks = set()
        hub.trial = None
        message = SimpleNamespace(
            topic="006",
            payload=(
                "CFGACK,7,DGA,1000000,361,0,clue_search,3,"
                "dcta_parity_v1,{},OK".format("b" * 64)
            ).encode(),
        )
        hub.on_message(None, None, message)
        self.assertEqual(hub.config_acks, {})
        self.assertEqual(len(hub.config_ack_rows), 1)

    def test_console_reports_pretrial_confirmation_and_silences_state_during_trial(self):
        hub = object.__new__(metrics_hub.Hub)
        hub.ids = ["00", "01"]
        hub.condition = threading.Condition()
        hub.last_message = 0.0
        hub.config_sequence = 8
        hub.config_acks = {}
        hub.config_ack_rows = []
        hub.connected_robots = set()
        hub.printed_config_acks = set()
        hub.positions = {}
        trial = self._trial()
        hub.trial = trial

        acknowledgment = SimpleNamespace(
            topic="006",
            payload=(
                "CFGACK,8,DGA,750000,271,250000,clue_search,3,"
                "dcta_parity_v1,{},OK".format("b" * 64)
            ).encode(),
        )
        output = io.StringIO()
        with redirect_stdout(output):
            hub.on_message(None, None, acknowledgment)
        text = output.getvalue()
        self.assertIn("[CONNECTED] robot=00", text)
        self.assertIn(
            "[CONFIRMED] robot=00 algorithm=DGA top_k=0.75 "
            "(271 cells) drop_rate=0.25 mode=clue_search horizon=3 "
            "revision=dcta_parity_v1 manifest=bbbbbbbbbbbb status=OK",
            text,
        )

        trial.active = True
        trial.t0 = 1.0
        state_message = SimpleNamespace(topic="001", payload=b"0,0")
        output = io.StringIO()
        with redirect_stdout(output):
            hub.on_message(None, None, state_message)
        self.assertEqual(output.getvalue(), "")

    def test_interactive_prompts_reuse_rate_and_require_memory_answer(self):
        hub = object.__new__(metrics_hub.Hub)
        with patch("builtins.input", return_value=""):
            ppm, rate = hub.prompt_rate(
                "Top-K rate", 0.75, allow_zero=False
            )
        self.assertEqual(ppm, 750000)
        self.assertEqual(rate, 0.75)

        with patch("builtins.input", side_effect=["", "maybe", "yes"]):
            self.assertTrue(hub.prompt_memory_error())


if __name__ == "__main__":
    unittest.main()
