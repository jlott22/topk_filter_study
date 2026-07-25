from __future__ import annotations

import ast
import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from hardware import metrics_hub


HARDWARE_DIR = Path(__file__).resolve().parent
POLULU_FILES = (
    "Pololu_ACBBA.py",
    "Pololu_CBAA.py",
    "Pololu_DGA.py",
    "Pololu_DMCHBA.py",
    "Pololu_HIPC.py",
    "Pololu_PI.py",
)


class TopKMetricsLoggingTests(unittest.TestCase):
    def test_every_pololu_metrics_csv_logs_rate_and_candidate_limit(self) -> None:
        for filename in POLULU_FILES:
            with self.subTest(filename=filename):
                tree = ast.parse((HARDWARE_DIR / filename).read_text(encoding="utf-8"))
                metrics_log = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "metrics_log"
                )

                metric_dicts = [
                    node
                    for node in ast.walk(metrics_log)
                    if isinstance(node, ast.Dict)
                ]
                logged_values = {}
                for metric_dict in metric_dicts:
                    for key, value in zip(metric_dict.keys, metric_dict.values):
                        if isinstance(key, ast.Constant) and isinstance(key.value, str):
                            logged_values[key.value] = value

                self.assertIsInstance(logged_values.get("top_k_rate"), ast.Name)
                self.assertEqual(logged_values["top_k_rate"].id, "TOP_K_PERCENT")
                self.assertIsInstance(logged_values.get("top_k_max_cells"), ast.Name)
                self.assertEqual(logged_values["top_k_max_cells"].id, "TOP_K_MAX_CELLS")
                self.assertIsInstance(logged_values.get("drop_rate"), ast.Name)
                self.assertEqual(logged_values["drop_rate"].id, "msg_drop_rate")
                self.assertIsInstance(
                    logged_values.get("config_sequence"), ast.Name
                )
                self.assertEqual(
                    logged_values["config_sequence"].id,
                    "applied_config_sequence",
                )
                expected_sources = {
                    "trial_mode": {"TRIAL_MODE", "applied_trial_mode"},
                    "commitment_horizon": {
                        "COMMITMENT_HORIZON",
                        "applied_commitment_horizon",
                    },
                    "logic_revision": {
                        "LOGIC_REVISION",
                        "applied_logic_revision",
                    },
                    "scenario_sha256": {"applied_scenario_sha256"},
                }
                for field, allowed_sources in expected_sources.items():
                    self.assertIsInstance(logged_values.get(field), ast.Name)
                    self.assertIn(logged_values[field].id, allowed_sources)

                fieldnames = next(
                    node.value
                    for node in ast.walk(metrics_log)
                    if isinstance(node, ast.Assign)
                    and any(
                        isinstance(target, ast.Name) and target.id == "fieldnames"
                        for target in node.targets
                    )
                )
                columns = [
                    element.value
                    for element in fieldnames.elts
                    if isinstance(element, ast.Constant) and isinstance(element.value, str)
                ]
                self.assertIn("top_k_rate", columns)
                self.assertIn("top_k_max_cells", columns)
                for field in expected_sources:
                    self.assertIn(field, columns)

                for metric in (
                    "allocator_calls",
                    "allocator_time_us_total",
                    "allocator_time_us_mean",
                    "allocator_time_us_max",
                    "allocator_time_pct",
                    "mean_step_time_ms",
                    "trial_time_ms",
                    "candidate_filter_calls",
                    "candidate_filter_time_us_total",
                    "candidate_filter_time_us_mean",
                    "candidate_filter_time_us_max",
                    "allocator_solve_time_us_total",
                ):
                    self.assertIn(metric, logged_values)
                    self.assertIn(metric, columns)

                record_allocator_time = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "record_allocator_time"
                )
                self.assertTrue(
                    any(
                        isinstance(node, ast.AugAssign)
                        and isinstance(node.target, ast.Name)
                        and node.target.id == "allocator_calls"
                        and isinstance(node.op, ast.Add)
                        and isinstance(node.value, ast.Constant)
                        and node.value.value == 1
                        for node in ast.walk(record_allocator_time)
                    ),
                    filename,
                )

                reset_trial_metrics = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "reset_trial_metrics"
                )
                self.assertTrue(
                    any(
                        isinstance(node, ast.Assign)
                        and any(
                            isinstance(target, ast.Name)
                            and target.id == "allocator_calls"
                            for target in node.targets
                        )
                        and isinstance(node.value, ast.Constant)
                        and node.value.value == 0
                        for node in ast.walk(reset_trial_metrics)
                    ),
                    filename,
                )
                self.assertIn("drop_rate", columns)
                self.assertIn("config_sequence", columns)

    def test_hub_system_and_robot_csvs_log_top_k_setting(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            hub = object.__new__(metrics_hub.Hub)
            hub.args = SimpleNamespace(
                out_dir=tmp,
                algorithm="CBAA",
                comm_level=0.0,
                drop_rate=0.25,
                top_k_rate=0.75,
                top_k_max_cells=271,
            )
            hub.ids = ["00"]
            scenario = metrics_hub.Scenario("1", (2, 2), [(1, 1)])
            robot = metrics_hub.RobotState(last_pos=(0, 0))
            trial = metrics_hub.Trial("run-1", scenario, {"00": robot})
            trial.end_time = 12.5
            trial.reported_target = scenario.target
            trial.reporter = "00"
            trial.algorithm_verified = True
            trial.top_k_rate = 0.75
            trial.top_k_max_cells = 271
            trial.drop_rate = 0.25
            trial.memory_error = False
            trial.status = "completed"
            trial.config_sequence = 4
            trial.trial_mode = "clue_search"
            trial.commitment_horizon = 1
            trial.logic_revision = "dcta_parity_v1"
            trial.scenario_sha256 = "c" * 64

            hub.write_trial(trial)

            for filename in ("CBAA_sys.csv", "CBAA_robots.csv"):
                with (Path(tmp) / filename).open(newline="") as stream:
                    row = next(csv.DictReader(stream))
                self.assertNotIn(None, row, filename)
                self.assertFalse(
                    any(value is None for value in row.values()), filename
                )
                self.assertEqual(row["top_k_rate"], "0.75", filename)
                self.assertEqual(row["top_k_max_cells"], "271", filename)
                self.assertEqual(row["algorithm"], "CBAA", filename)
                self.assertEqual(row["algorithm_verified"], "1", filename)
                self.assertEqual(row["drop_rate"], "0.25", filename)
                self.assertEqual(row["comm_level"], "0.25", filename)
                self.assertEqual(row["memory_error"], "0", filename)
                self.assertEqual(row["trial_status"], "completed", filename)
                self.assertEqual(row["config_sequence"], "4", filename)
                self.assertEqual(row["trial_mode"], "clue_search", filename)
                self.assertEqual(row["commitment_horizon"], "1", filename)
                self.assertEqual(
                    row["logic_revision"], "dcta_parity_v1", filename
                )
                self.assertEqual(row["scenario_sha256"], "c" * 64, filename)

    def test_hub_writes_configuration_acknowledgment_audit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            hub = object.__new__(metrics_hub.Hub)
            hub.args = SimpleNamespace(out_dir=tmp, algorithm="DGA")
            hub.config_ack_rows = [[
                123.5, "run-1", "trial-1", "03", 7, "DGA",
                750000, 271, 250000, "clue_search", 3,
                "dcta_parity_v1", "d" * 64, "OK",
            ]]
            hub.write_config_acks()

            path = Path(tmp) / "DGA_configuration_acks.csv"
            with path.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["algorithm"], "DGA")
            self.assertEqual(row["top_k_max_cells"], "271")
            self.assertEqual(row["drop_ppm"], "250000")
            self.assertEqual(row["trial_mode"], "clue_search")
            self.assertEqual(row["commitment_horizon"], "3")
            self.assertEqual(row["logic_revision"], "dcta_parity_v1")
            self.assertEqual(row["scenario_sha256"], "d" * 64)
            self.assertEqual(row["status"], "OK")


if __name__ == "__main__":
    unittest.main()
