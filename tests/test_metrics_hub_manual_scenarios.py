from __future__ import annotations

import csv
import io
import json
import tempfile
import threading
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from hardware import metrics_hub


def cohort():
    return metrics_hub.handpicked_scenarios(
        metrics_hub.load_scenarios(
            metrics_hub.SCENARIO_FILE,
            starts=metrics_hub.HOME.values(),
            expected_clues=4,
        )
    )


class ManualScenarioTests(unittest.TestCase):
    def test_fixed_mapping_and_hash(self):
        scenarios = cohort()
        self.assertEqual(
            [
                (item.trial_id, item.source_trial_id, item.target)
                for item in scenarios
            ],
            [
                ("1", "4", (5, 9)),
                ("2", "53", (7, 2)),
                ("3", "232", (3, 11)),
                ("4", "394", (5, 4)),
                ("5", "473", (13, 16)),
            ],
        )
        self.assertTrue(all(len(item.clues) == 4 for item in scenarios))
        self.assertEqual(
            metrics_hub.scenario_manifest_sha256(scenarios),
            metrics_hub.HANDPICKED_COHORT_SHA256,
        )

    def test_manifest_records_study_and_source_ids(self):
        scenarios = cohort()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "manifest.json"
            metrics_hub.enforce_scenario_manifest_lock(
                str(path),
                scenarios,
                grid_size=19,
            )
            record = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(record["schema"], 2)
        self.assertEqual(record["trial_ids"], ["1", "2", "3", "4", "5"])
        self.assertEqual(
            record["source_trial_ids"],
            ["4", "53", "232", "394", "473"],
        )

    def test_manual_prompt_rejects_invalid_and_reuses_previous(self):
        hub = object.__new__(metrics_hub.Hub)
        hub.scenarios = cohort()
        hub.scenario_by_id = {
            item.trial_id: item for item in hub.scenarios
        }
        with patch("builtins.input", side_effect=["bad", "3"]):
            with redirect_stdout(io.StringIO()):
                selected = hub.prompt_scenario(None)
        self.assertEqual(selected.trial_id, "3")

        with patch("builtins.input", return_value=""):
            with redirect_stdout(io.StringIO()):
                repeated = hub.prompt_scenario(selected.trial_id)
        self.assertIs(repeated, selected)

    def test_removed_unattended_options_and_positive_trial_count(self):
        parser = metrics_hub.parser()
        for arguments in (
            ["--auto"],
            ["--memory-error-default", "unknown"],
            ["--start-index", "1"],
            ["--trials", "0"],
        ):
            with self.subTest(arguments=arguments):
                with redirect_stdout(io.StringIO()), redirect_stderr(
                    io.StringIO()
                ):
                    with self.assertRaises(SystemExit):
                        parser.parse_args(arguments)
        self.assertEqual(parser.parse_args(["--trials", "6"]).trials, 6)

    def test_three_manual_trials_allow_repeated_scenario_ids(self):
        class Client:
            def connect(self, *_args, **_kwargs):
                return None

            def loop_start(self):
                return None

            def loop_stop(self):
                return None

            def disconnect(self):
                return None

        selected = [cohort()[0], cohort()[0], cohort()[2]]
        previous_values = []
        saved = []
        tasks = []
        transitions = []
        hub = object.__new__(metrics_hub.Hub)
        hub.args = SimpleNamespace(
            out_dir="unused",
            broker="unused",
            port=1883,
            connect_timeout=1,
            top_k_rate=1.0,
            drop_rate=0.0,
            trials=3,
            grid_size=19,
            algorithm="DGA",
            drain_seconds=0,
        )
        hub.client = Client()
        hub.connected = threading.Event()
        hub.connected.set()
        hub.condition = threading.Condition()
        hub.ids = ["00"]
        hub.positions = {"00": metrics_hub.HOME["00"]}
        hub.trial = None
        hub.wait_home = lambda: None

        def choose(previous):
            previous_values.append(previous)
            return selected[len(previous_values) - 1]

        hub.prompt_scenario = choose
        hub.prompt_rate = lambda _label, current, allow_zero: (
            metrics_hub.rate_to_ppm(current, allow_zero=allow_zero),
            current,
        )

        def configure(trial, _top_k_ppm, _drop_ppm):
            trial.algorithm_verified = True
            trial.config_sequence = 7
            trial.scenario_sha256 = metrics_hub.HANDPICKED_COHORT_SHA256

        hub.configure_robots = configure
        hub.onboard_counts = lambda: {}
        hub.publish = lambda _topic, payload, kind, _trial: tasks.append(
            (kind, json.loads(payload)) if kind == "trial_task" else (kind, payload)
        )
        hub.transition_robots = (
            lambda _trial, command, expected: transitions.append(
                (command, expected)
            )
        )
        hub.wait_quiet = lambda: None
        hub.activate_after_run_quorum = lambda trial: setattr(
            trial, "active", True
        )

        def finish(trial):
            trial.active = False
            trial.end_time = 1.0

        hub.wait_target = finish
        hub.prompt_memory_error = lambda: False
        hub.write_trial = lambda trial: saved.append(
            (trial.scenario.trial_id, trial.status)
        )
        hub.import_onboard = lambda _trial: None
        hub.write_commands = lambda: None
        hub.write_config_acks = lambda: None
        hub.write_control_acks = lambda: None

        with tempfile.TemporaryDirectory() as temporary:
            hub.args.out_dir = temporary
            with patch("builtins.input", side_effect=["", "", ""]):
                with redirect_stdout(io.StringIO()):
                    hub.run()

        self.assertEqual(previous_values, [None, "1", "1"])
        self.assertEqual(
            [item[1]["trial_id"] for item in tasks],
            ["1", "1", "3"],
        )
        self.assertEqual(
            [item[1]["source_trial_id"] for item in tasks],
            ["4", "4", "232"],
        )
        self.assertEqual(
            saved,
            [("1", "completed"), ("1", "completed"), ("3", "completed")],
        )
        self.assertEqual(
            transitions,
            [
                ("PRESTART", "READY"),
                ("START", "STARTED"),
                ("RUN", "RUNNING"),
            ]
            * 3,
        )

    def test_task_payload_carries_both_ids_and_selected_layout(self):
        scenario = cohort()[3]
        trial = metrics_hub.Trial(
            "run-1",
            scenario,
            {"00": metrics_hub.RobotState()},
        )
        trial.algorithm_verified = True
        trial.top_k_rate = 0.25
        trial.top_k_max_cells = 90
        trial.drop_rate = 0.0
        trial.config_sequence = 9
        trial.scenario_sha256 = metrics_hub.HANDPICKED_COHORT_SHA256
        payload = json.loads(metrics_hub.trial_task_payload(trial, "DGA"))
        self.assertEqual(payload["trial_id"], "4")
        self.assertEqual(payload["source_trial_id"], "394")
        self.assertEqual(payload["target"], [5, 4])
        self.assertEqual(payload["clues"], [[4, 3], [4, 4], [2, 6], [6, 4]])

    def test_location_mismatches_warn_without_changing_status(self):
        hub = object.__new__(metrics_hub.Hub)
        scenario = cohort()[0]
        robot = metrics_hub.RobotState(last_pos=(0, 0))
        trial = metrics_hub.Trial("run-1", scenario, {"00": robot})
        trial.active = True
        trial.status = "completed"
        output = io.StringIO()
        with redirect_stdout(output):
            hub.collect(trial, "00", "4", (18, 18), 1.0)
            hub.collect(trial, "00", "5", (18, 17), 2.0)
        self.assertEqual(trial.status, "completed")
        self.assertEqual(trial.unexpected_clues, [(18, 18)])
        self.assertEqual(len(trial.location_warnings), 2)
        self.assertIn("[LOCATION WARNING]", output.getvalue())

    def test_system_robot_and_event_csvs_include_source_id_and_warnings(self):
        scenario = cohort()[0]
        robots = {
            robot_id: metrics_hub.RobotState(
                last_pos=metrics_hub.HOME[robot_id]
            )
            for robot_id in metrics_hub.ROBOT_IDS
        }
        trial = metrics_hub.Trial("run-1", scenario, robots)
        trial.status = "completed"
        trial.reported_target = scenario.target
        trial.clues = [scenario.clues[0], (18, 18)]
        trial.unexpected_clues = [(18, 18)]
        trial.location_warnings = ["unexpected clue (18, 18) for trial 1"]
        trial.events = [[
            trial.run_id,
            scenario.trial_id,
            scenario.source_trial_id,
            1.0,
            0.5,
            "trial",
            "00",
            "4",
            "clue",
            "18,18",
            18,
            18,
        ]]
        with tempfile.TemporaryDirectory() as temporary:
            hub = object.__new__(metrics_hub.Hub)
            hub.args = SimpleNamespace(out_dir=temporary, algorithm="DGA")
            hub.ids = list(metrics_hub.ROBOT_IDS)
            hub.write_trial(trial)
            with open(
                Path(temporary) / "DGA_sys.csv",
                newline="",
            ) as stream:
                system_rows = list(csv.DictReader(stream))
            with open(
                Path(temporary) / "DGA_robots.csv",
                newline="",
            ) as stream:
                robot_rows = list(csv.DictReader(stream))
            with open(
                Path(temporary) / "DGA_events.csv",
                newline="",
            ) as stream:
                event = next(csv.DictReader(stream))
        self.assertEqual(len(system_rows), 1)
        self.assertEqual(len(robot_rows), 4)
        system = system_rows[0]
        for row in (system, *robot_rows, event):
            self.assertEqual(row["trial_id"], "1")
            self.assertEqual(row["source_trial_id"], "4")
        self.assertEqual(system["unexpected_clues"], "18/18")
        self.assertIn("unexpected clue", system["location_warning"])

    def test_commands_acknowledgments_and_onboard_csvs_include_both_ids(self):
        scenario = cohort()[1]
        trial = metrics_hub.Trial(
            "run-2",
            scenario,
            {"00": metrics_hub.RobotState()},
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            onboard = root / "mounted" / "00"
            onboard.mkdir(parents=True)
            (onboard / "metrics-log-DGA.txt").write_text(
                "steps,top_k_rate\n7,0.25\n",
                encoding="utf-8",
            )

            hub = object.__new__(metrics_hub.Hub)
            hub.args = SimpleNamespace(
                out_dir=temporary,
                algorithm="DGA",
                robot_metrics_root=str(root / "mounted"),
                onboard_wait=0.1,
            )
            hub.ids = ["00"]
            hub.commands = [[
                1.0, "run-2", "2", "53", "trial_task",
                metrics_hub.HUB_TASK_TOPIC, "{}",
            ]]
            hub.config_ack_rows = [[
                1.0, "run-2", "2", "53", "00", 7, "DGA",
                250000, 90, 0, "clue_search", 3,
                metrics_hub.LOGIC_REVISION,
                metrics_hub.HANDPICKED_COHORT_SHA256, "OK",
            ]]
            hub.control_ack_rows = [[
                1.0, "run-2", "2", "53", "00", 7, "00",
                "RUNNING", True,
            ]]

            hub.write_commands()
            hub.write_config_acks()
            hub.write_control_acks()
            hub.import_onboard(trial)

            for name in (
                "DGA_commands.csv",
                "DGA_configuration_acks.csv",
                "DGA_control_acks.csv",
                "DGA_onboard.csv",
            ):
                with (root / name).open(newline="") as stream:
                    row = next(csv.DictReader(stream))
                self.assertEqual(row["trial_id"], "2", name)
                self.assertEqual(row["source_trial_id"], "53", name)

    def test_control_boundaries_and_pending_metric_replay(self):
        hub = object.__new__(metrics_hub.Hub)
        hub.args = SimpleNamespace(
            config_timeout=0.1,
            config_retry_seconds=0.1,
            control_timeout=0.1,
            control_retry_seconds=0.1,
        )
        hub.ids = []
        hub.condition = threading.Condition()
        hub.control_acks = {}
        hub.control_expected_state = ""
        hub.control_fault = ""
        published = []
        hub.publish = lambda topic, payload, kind, trial: published.append(
            (topic, payload, kind, trial.control_phase)
        )

        scenario = cohort()[4]
        trial = metrics_hub.Trial("run-3", scenario, {})
        trial.config_sequence = 11
        hub.transition_robots(trial, "PRESTART", "READY")
        self.assertEqual(trial.control_phase, "preparing")
        hub.transition_robots(trial, "START", "STARTED")
        self.assertEqual(trial.control_phase, "arming")
        hub.transition_robots(trial, "RUN", "RUNNING")
        self.assertEqual(trial.control_phase, "starting")
        self.assertGreater(trial.t0, 0)
        hub.activate_after_run_quorum(trial)
        self.assertTrue(trial.active)
        self.assertEqual(trial.control_phase, "active")
        self.assertEqual(
            [item[1] for item in published],
            ["CMD,PRESTART,11", "CMD,START,11", "CMD,RUN,11"],
        )

        replay_hub = object.__new__(metrics_hub.Hub)
        replay_hub.ids = ["00", "01"]
        replay_hub.condition = threading.Condition()
        replay_hub.control_acks = {
            rid: {
                "sequence": 11,
                "robot_id": rid,
                "state": "RUNNING",
            }
            for rid in replay_hub.ids
        }
        replay_trial = metrics_hub.Trial(
            "run-4",
            scenario,
            {
                "00": metrics_hub.RobotState(last_pos=(0, 0)),
                "01": metrics_hub.RobotState(last_pos=(0, 6)),
            },
        )
        replay_trial.config_sequence = 11
        replay_trial.pending_start_events = [
            ("00", "1", (1, 0), 0.01),
            ("01", "4", scenario.clues[0], 0.02),
        ]
        replay_hub.activate_after_run_quorum(replay_trial)
        self.assertEqual(replay_trial.robots["00"].steps, 1)
        self.assertEqual(replay_trial.clues, [scenario.clues[0]])
        self.assertEqual(replay_trial.first_clue, 0.02)
        self.assertEqual(replay_trial.pending_start_events, [])


if __name__ == "__main__":
    unittest.main()
