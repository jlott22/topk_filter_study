from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from benchmark_sim.algorithms.base import AllocatorBase, timed_candidate_filter
from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import AllocationDecision, TrialScenario
from benchmark_sim.metrics.export import write_outputs
from benchmark_sim.metrics.summary import build_computational_performance_rows, build_rows
from benchmark_sim.run_trials import resolve_top_k_settings


class _NoGoalAllocator(AllocatorBase):
    def choose_goal(self, robot):
        return AllocationDecision(goal=None)

    @timed_candidate_filter
    def _candidate_cells(self, robot):
        return self._filter_candidate_cells(
            robot,
            [(0, 0), (1, 0), (2, 0)],
        )


class ComputationalPerformanceTests(unittest.TestCase):
    def _state(self):
        cfg = SimConfig(
            grid_size=3,
            robot_ids=["00"],
            start_positions={"00": (0, 0)},
            write_parquet=False,
        )
        runner = AsyncTrialRunner(cfg, _NoGoalAllocator, make_comm_model("ideal", None), seed=0)
        return runner.new_trial(TrialScenario(trial_id=7, target=(2, 2), clues=[]))

    def test_choose_goal_call_records_host_runtime(self) -> None:
        state = self._state()
        robot = state.robots["00"]

        robot.step(0.0, state.planner)

        self.assertEqual(len(robot.counters.allocator_time_ns_samples), 1)
        self.assertEqual(len(robot.counters.allocator_solve_time_ns_samples), 1)
        self.assertEqual(len(robot.counters.allocator_time_ns_pre_clue), 1)
        self.assertGreaterEqual(robot.counters.allocator_time_ns_samples[0], 0)
        self.assertGreaterEqual(robot.counters.allocator_solve_time_ns_samples[0], 0)
        self.assertLessEqual(
            robot.counters.allocator_solve_time_ns_samples[0],
            robot.counters.allocator_time_ns_samples[0],
        )

    def test_study_top_k_rates_resolve_to_expected_19_by_19_limits(self) -> None:
        expected = {
            1.0: 361,
            0.75: 271,
            0.5: 181,
            0.25: 90,
            0.10: 36,
            0.05: 18,
        }
        for rate, limit in expected.items():
            with self.subTest(rate=rate):
                self.assertEqual(resolve_top_k_settings(19, None, rate), (limit, rate))

    def test_rows_report_aggregate_host_runtime_statistics(self) -> None:
        state = self._state()
        counters = state.robots["00"].counters
        counters.allocator_time_ns_samples = [1_000_000, 2_000_000, 10_000_000]
        counters.allocator_solve_time_ns_samples = [500_000, 1_000_000, 7_000_000]
        counters.allocator_time_ns_pre_clue = [1_000_000]
        counters.allocator_time_ns_post_clue = [2_000_000, 10_000_000]
        counters.candidate_filter_time_ns_samples = [1_000_000, 3_000_000]
        state.host_runtime_ns = 20_000_000

        rows = build_computational_performance_rows(
            state,
            algorithm_name="test",
            comm_model="ideal",
            comm_level="ideal",
            scenario_file="scenario.json",
        )

        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["allocator_calls"], 3)
        self.assertEqual(row["allocator_calls_pre_clue"], 1)
        self.assertEqual(row["allocator_calls_post_clue"], 2)
        self.assertAlmostEqual(row["allocator_time_ms_total"], 13.0)
        self.assertAlmostEqual(row["allocator_time_ms_mean"], 13.0 / 3.0)
        self.assertAlmostEqual(row["allocator_time_ms_median"], 2.0)
        self.assertAlmostEqual(row["allocator_time_ms_p95"], 10.0)
        self.assertAlmostEqual(row["allocator_time_ms_max"], 10.0)
        self.assertAlmostEqual(row["allocator_solve_time_ms_total"], 8.5)
        self.assertAlmostEqual(row["allocator_solve_time_ms_mean"], 8.5 / 3.0)
        self.assertAlmostEqual(row["allocator_solve_time_ms_median"], 1.0)
        self.assertAlmostEqual(row["allocator_solve_time_ms_p95"], 7.0)
        self.assertAlmostEqual(row["allocator_solve_time_ms_max"], 7.0)
        self.assertAlmostEqual(row["allocator_time_pct"], 65.0)
        self.assertAlmostEqual(row["allocator_host_runtime_pct"], 65.0)
        self.assertEqual(row["candidate_filter_calls"], 2)
        self.assertAlmostEqual(row["candidate_filter_time_ms_total"], 4.0)
        self.assertAlmostEqual(row["candidate_filter_time_ms_mean"], 2.0)
        self.assertAlmostEqual(row["candidate_filter_time_ms_max"], 3.0)

    def test_shared_candidate_filter_records_each_call(self) -> None:
        state = self._state()
        robot = state.robots["00"]

        filtered = robot.allocator._candidate_cells(robot)

        self.assertEqual(filtered, [(0, 0), (1, 0), (2, 0)])
        self.assertEqual(len(robot.counters.candidate_filter_time_ns_samples), 1)
        self.assertGreaterEqual(robot.counters.candidate_filter_time_ns_samples[0], 0)

    def test_all_six_active_allocators_record_full_candidate_filter_calls(self) -> None:
        algorithm_specs = (
            "benchmark_sim.algorithms.ACBBA:ACBBAAllocator",
            "benchmark_sim.algorithms.CBAA:CBAAAllocator",
            "benchmark_sim.algorithms.DGA:DGAAllocator",
            "benchmark_sim.algorithms.DMCHBA:DMCHBAAllocator",
            "benchmark_sim.algorithms.HIPC:HIPCAllocator",
            "benchmark_sim.algorithms.PI:PIAllocator",
        )
        for index, spec in enumerate(algorithm_specs):
            with self.subTest(algorithm=spec):
                cfg = SimConfig(
                    grid_size=3,
                    robot_ids=["00"],
                    start_positions={"00": (0, 0)},
                    max_candidate_cells=7,
                    top_k_rate=0.75,
                    write_parquet=False,
                )
                allocator_cls = load_allocator_class(spec)
                runner = AsyncTrialRunner(
                    cfg,
                    allocator_cls,
                    make_comm_model("ideal", None),
                    seed=index,
                )
                state = runner.new_trial(
                    TrialScenario(trial_id=index, target=(2, 2), clues=[])
                )
                robot = state.robots["00"]

                robot.allocator._candidate_cells(robot)

                self.assertEqual(
                    len(robot.counters.candidate_filter_time_ns_samples),
                    1,
                )
                self.assertGreaterEqual(
                    robot.counters.candidate_filter_time_ns_samples[0],
                    0,
                )

    def test_export_writes_separate_computational_performance_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            write_outputs(
                out_dir=tmp,
                trial_summary_rows=[],
                system_performance_rows=[],
                robot_performance_rows=[],
                computational_performance_rows=[{"trial_id": 1, "allocator_calls": 2}],
                config={},
            )

            path = Path(tmp) / "computational_performance.csv"
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows, [{"trial_id": "1", "allocator_calls": "2"}])

    def test_top_k_setting_is_written_to_every_metric_csv(self) -> None:
        cfg = SimConfig(
            grid_size=3,
            robot_ids=["00"],
            start_positions={"00": (0, 0)},
            max_candidate_cells=7,
            top_k_rate=0.75,
            write_parquet=False,
        )
        runner = AsyncTrialRunner(cfg, _NoGoalAllocator, make_comm_model("ideal", None), seed=0)
        state = runner.new_trial(TrialScenario(trial_id=8, target=(2, 2), clues=[]))
        trial_row, system_row, robot_rows = build_rows(
            state,
            algorithm_name="test",
            comm_model="ideal",
            comm_level="ideal",
            scenario_file="scenario.json",
        )
        computational_rows = build_computational_performance_rows(
            state,
            algorithm_name="test",
            comm_model="ideal",
            comm_level="ideal",
            scenario_file="scenario.json",
        )

        with tempfile.TemporaryDirectory() as tmp:
            write_outputs(
                out_dir=tmp,
                trial_summary_rows=[trial_row],
                system_performance_rows=[system_row],
                robot_performance_rows=robot_rows,
                computational_performance_rows=computational_rows,
                config=cfg.to_dict(),
            )

            for filename in (
                "trial_summary.csv",
                "system_performance.csv",
                "robot_performance.csv",
                "computational_performance.csv",
            ):
                with (Path(tmp) / filename).open(newline="") as stream:
                    row = next(csv.DictReader(stream))
                self.assertEqual(row["top_k_rate"], "0.75", filename)
                self.assertEqual(row["top_k_max_cells"], "7", filename)


if __name__ == "__main__":
    unittest.main()
