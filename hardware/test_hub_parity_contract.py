from __future__ import annotations

import hashlib
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from hardware import metrics_hub


class HubParityContractTests(unittest.TestCase):
    def _write_csv(self, directory: str, rows: str) -> Path:
        path = Path(directory) / "scenarios.csv"
        path.write_text(
            "trial_id,target_x,target_y,clue1_x,clue1_y\n" + rows,
            encoding="utf-8",
        )
        return path

    def test_hardware_home_cells_are_canonical_simulator_starts(self):
        self.assertEqual(
            metrics_hub.HOME,
            {
                "00": (0, 0),
                "01": (0, 6),
                "02": (0, 12),
                "03": (0, 18),
            },
        )
        self.assertTrue(Path(metrics_hub.SCENARIO_FILE).is_file())

    def test_initial_starts_count_toward_each_robots_unique_workload(self):
        robots = {
            rid: metrics_hub.RobotState(last_pos=position)
            for rid, position in metrics_hub.HOME.items()
        }
        trial = metrics_hub.Trial(
            "run",
            metrics_hub.Scenario("1", (18, 18), [(9, 9)]),
            robots,
        )

        metrics_hub.record_initial_robot_visits(trial, metrics_hub.HOME)

        self.assertEqual(len(trial.visits), 4)
        self.assertEqual(
            trial.visits,
            {position: 1 for position in metrics_hub.HOME.values()},
        )
        self.assertEqual(
            [robots[rid].unique for rid in sorted(robots)],
            [1, 1, 1, 1],
        )
        self.assertEqual(
            [robots[rid].revisits for rid in sorted(robots)],
            [0, 0, 0, 0],
        )

    def test_nonfinite_rates_are_rejected(self):
        for value in ("nan", "inf", "-inf"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    metrics_hub.rate_to_ppm(value, allow_zero=True)

    def test_cfg_wire_shape_contains_all_parity_fields(self):
        hub = object.__new__(metrics_hub.Hub)
        hub.args = SimpleNamespace(
            algorithm="CBAA",
            grid_size=19,
            commitment_horizon=3,
            trial_mode=metrics_hub.TRIAL_MODE,
            logic_revision=metrics_hub.LOGIC_REVISION,
            config_timeout=0.05,
            config_retry_seconds=1.0,
            auto=True,
        )
        hub.ids = ["00"]
        hub.condition = threading.Condition()
        hub.config_sequence = 0
        hub.config_acks = {}
        hub.commands = []
        hub.scenario_sha256 = "a" * 64
        published = []

        def publish(_topic, payload, _kind, _trial):
            published.append(payload)
            fields = payload.split(",")
            with hub.condition:
                hub.config_acks["00"] = {
                    "sequence": int(fields[1]),
                    "algorithm": "CBAA",
                    "top_k_ppm": int(fields[2]),
                    "top_k_max_cells": int(fields[3]),
                    "drop_ppm": int(fields[4]),
                    "trial_mode": fields[5],
                    "commitment_horizon": int(fields[6]),
                    "logic_revision": fields[7],
                    "scenario_sha256": fields[8],
                    "status": "OK",
                }
                hub.condition.notify_all()

        hub.publish = publish
        trial = metrics_hub.Trial(
            "run",
            metrics_hub.Scenario("1", (1, 1), []),
            {"00": metrics_hub.RobotState()},
        )

        hub.configure_robots(trial, 750000, 250000)

        self.assertEqual(
            published,
            [
                "CFG,1,750000,271,250000,clue_search,1,"
                "dcta_parity_v1,{}".format("a" * 64)
            ],
        )
        self.assertEqual(len(published[0].split(",")), 9)
        self.assertNotIn(
            "-",
            published[0],
            "the ESP32 uses '-' as the Pololu UART frame terminator",
        )
        framed = "997." + published[0] + "-"
        self.assertEqual(framed.split("-", 1)[0], "997." + published[0])
        self.assertEqual(trial.commitment_horizon, 1)

    def test_start_clue_is_allowed_but_start_target_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_csv(tmp, "1,3,3,0,0\n")
            scenarios = metrics_hub.load_scenarios(
                str(path),
                grid_size=19,
                starts=metrics_hub.HOME.values(),
                expected_clues=1,
            )
            self.assertEqual(scenarios[0].clues, [(0, 0)])

            path = self._write_csv(tmp, "1,0,6,2,2\n")
            with self.assertRaisesRegex(ValueError, "robot start"):
                metrics_hub.load_scenarios(
                    str(path),
                    grid_size=19,
                    starts=metrics_hub.HOME.values(),
                )

    def test_scenario_loader_fails_on_partial_duplicate_and_invalid_rows(self):
        cases = {
            "partial clue": "1,3,3,2,\n",
            "duplicate clues": (
                "trial_id,target_x,target_y,clue1_x,clue1_y,"
                "clue2_x,clue2_y\n1,3,3,2,2,2,2\n"
            ),
            "outside": "1,19,3,2,2\n",
            "target is also a clue": "1,3,3,3,3\n",
        }
        with tempfile.TemporaryDirectory() as tmp:
            for name, body in cases.items():
                with self.subTest(case=name):
                    path = Path(tmp) / (name.replace(" ", "_") + ".csv")
                    if body.startswith("trial_id"):
                        path.write_text(body, encoding="utf-8")
                    else:
                        path.write_text(
                            "trial_id,target_x,target_y,clue1_x,clue1_y\n"
                            + body,
                            encoding="utf-8",
                        )
                    with self.assertRaises(ValueError):
                        metrics_hub.load_scenarios(
                            str(path),
                            grid_size=19,
                            starts=metrics_hub.HOME.values(),
                        )

    def test_scenario_loader_normalizes_integer_ids_and_rejects_gaps(self):
        with tempfile.TemporaryDirectory() as tmp:
            normalized = Path(tmp) / "normalized.csv"
            normalized.write_text(
                "trial_id,target_x,target_y,clue1_x,clue1_y\n"
                "001,3,3,2,2\n",
                encoding="utf-8",
            )
            scenarios = metrics_hub.load_scenarios(str(normalized))
            self.assertEqual(scenarios[0].trial_id, "1")

            noninteger = Path(tmp) / "noninteger.csv"
            noninteger.write_text(
                "trial_id,target_x,target_y,clue1_x,clue1_y\n"
                "trial-a,3,3,2,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "non-integer trial ID"):
                metrics_hub.load_scenarios(str(noninteger))

            gap = Path(tmp) / "gap.csv"
            gap.write_text(
                "trial_id,target_x,target_y,clue1_x,clue1_y,"
                "clue2_x,clue2_y\n"
                "1,3,3,,,2,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "non-contiguous"):
                metrics_hub.load_scenarios(str(gap))

    def test_manifest_hash_covers_selected_ids_targets_and_clues(self):
        first = [
            metrics_hub.Scenario("1", (3, 4), [(0, 0)]),
            metrics_hub.Scenario("2", (5, 6), [(0, 6)]),
        ]
        same = [
            metrics_hub.Scenario("1", (3, 4), [(0, 0)]),
            metrics_hub.Scenario("2", (5, 6), [(0, 6)]),
        ]
        reversed_order = list(reversed(same))
        self.assertEqual(
            metrics_hub.scenario_manifest_sha256(first),
            metrics_hub.scenario_manifest_sha256(same),
        )
        self.assertNotEqual(
            metrics_hub.scenario_manifest_sha256(first),
            metrics_hub.scenario_manifest_sha256(reversed_order),
        )

    def test_manifest_lock_rejects_a_different_cross_condition_selection(self):
        first = [
            metrics_hub.Scenario("1", (3, 4), [(0, 0)]),
            metrics_hub.Scenario("2", (5, 6), [(0, 6)]),
        ]
        changed = [
            metrics_hub.Scenario("1", (3, 4), [(0, 0)]),
            metrics_hub.Scenario("3", (5, 6), [(0, 6)]),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            lock = Path(tmp) / "study_manifest.json"
            expected_hash = metrics_hub.scenario_manifest_sha256(first)
            self.assertEqual(
                metrics_hub.enforce_scenario_manifest_lock(
                    str(lock), first, grid_size=19
                ),
                expected_hash,
            )
            record = json.loads(lock.read_text(encoding="utf-8"))
            self.assertEqual(record["scenario_sha256"], expected_hash)
            self.assertEqual(record["trial_ids"], ["1", "2"])

            # Reusing exactly the same selection is idempotent.
            self.assertEqual(
                metrics_hub.enforce_scenario_manifest_lock(
                    str(lock), list(first), grid_size=19
                ),
                expected_hash,
            )
            with self.assertRaisesRegex(ValueError, "manifest lock"):
                metrics_hub.enforce_scenario_manifest_lock(
                    str(lock), changed, grid_size=19
                )

    def test_final_500_manifest_has_no_target_on_canonical_start(self):
        root = Path(__file__).resolve().parents[1]
        study_path = root / "simulator" / "scenarios" / "final_trial_500.csv"
        scenarios = metrics_hub.load_scenarios(
            str(study_path),
            grid_size=19,
            starts=metrics_hub.HOME.values(),
            expected_clues=4,
        )
        self.assertEqual(len(scenarios), 500)

        canonical_path = (
            root.parent / "dcta_benchmark_sim" / "scenarios"
            / "final_trial_500.csv"
        )
        if canonical_path.exists():
            self.assertEqual(
                hashlib.sha256(study_path.read_bytes()).hexdigest(),
                hashlib.sha256(canonical_path.read_bytes()).hexdigest(),
            )

    def test_target_bump_counts_terminal_logical_step_without_moving_pose(self):
        hub = object.__new__(metrics_hub.Hub)
        robot = metrics_hub.RobotState(last_pos=(4, 4))
        scenario = metrics_hub.Scenario("1", (5, 4), [(1, 1)])
        trial = metrics_hub.Trial("run", scenario, {"03": robot})
        trial.active = True
        trial.first_clue = 0.25
        trial.visits[(4, 4)] = 1

        hub.collect(trial, "03", "5", (5, 4), 10.0)

        self.assertFalse(trial.active)
        self.assertEqual(trial.reported_target, (5, 4))
        self.assertEqual(robot.last_pos, (4, 4))
        self.assertEqual(robot.steps, 1)
        self.assertEqual(robot.post_steps, 1)
        self.assertEqual(robot.unique, 1)
        self.assertEqual(trial.visits[(5, 4)], 1)

    def test_messages_after_target_do_not_change_frozen_trial_metrics(self):
        hub = object.__new__(metrics_hub.Hub)
        hub.ids = ["00", "03"]
        hub.condition = threading.Condition()
        hub.last_message = 0.0
        hub.connected_robots = set()
        hub.printed_config_acks = set()
        hub.config_ack_rows = []
        hub.config_acks = {}
        hub.config_sequence = 0
        hub.positions = {}

        finder = metrics_hub.RobotState(last_pos=(4, 4))
        peer = metrics_hub.RobotState(last_pos=(0, 0))
        scenario = metrics_hub.Scenario("1", (5, 4), [(1, 1)])
        trial = metrics_hub.Trial(
            "run",
            scenario,
            {"00": peer, "03": finder},
        )
        trial.t0 = time.monotonic() - 1.0
        trial.active = True
        trial.first_clue = 0.25
        hub.trial = trial

        hub.on_message(
            None,
            None,
            SimpleNamespace(topic="035", payload=b"5,4"),
        )
        frozen_messages = dict(trial.messages)
        frozen_visits = dict(trial.visits)

        hub.on_message(
            None,
            None,
            SimpleNamespace(topic="001", payload=b"1,0"),
        )

        self.assertFalse(trial.active)
        self.assertEqual(trial.messages, frozen_messages)
        self.assertEqual(trial.visits, frozen_visits)
        self.assertEqual(peer.steps, 0)
        self.assertEqual(hub.positions["00"], (1, 0))


if __name__ == "__main__":
    unittest.main()
