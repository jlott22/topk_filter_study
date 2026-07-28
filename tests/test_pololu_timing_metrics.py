from __future__ import annotations

import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPECS = {
    "Pololu_ACBBA.py": "_acbba_candidate_cells",
    "Pololu_CBAA.py": "_cbaa_candidate_cells",
    "Pololu_DGA.py": "_dga_candidates",
    "Pololu_DMCHBA.py": "_dmchba_candidate_cells",
    "Pololu_HIPC.py": "_hipc_candidates",
    "Pololu_PI.py": "_pi_candidates",
}
TIMING_FIELDS = {
    "trial_time_ms",
    "candidate_filter_calls",
    "candidate_filter_time_us_total",
    "candidate_filter_time_us_mean",
    "candidate_filter_time_us_max",
    "allocator_calls",
    "allocator_solve_time_us_total",
    "allocator_solve_time_us_mean",
    "allocator_solve_time_us_max",
    "allocator_time_us_total",
    "allocator_time_us_mean",
    "allocator_time_us_max",
    "allocator_time_pct",
}
RESET_COUNTERS = {
    "candidate_filter_calls",
    "candidate_filter_time_us_total",
    "candidate_filter_time_us_max",
    "allocator_calls",
    "allocator_solve_time_us_total",
    "allocator_solve_time_us_max",
    "allocator_time_us_total",
    "allocator_time_us_max",
}


def _called_functions(function: ast.FunctionDef) -> set[str]:
    return {
        node.func.id
        for node in ast.walk(function)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }


class PololuTimingMetricsTests(unittest.TestCase):
    def test_all_six_programs_wrap_candidate_and_allocator_calls(self) -> None:
        for filename, candidate_name in SPECS.items():
            with self.subTest(filename=filename):
                tree = ast.parse(
                    (ROOT / "hardware" / filename).read_text(encoding="utf-8")
                )
                functions = {
                    node.name: node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                }

                self.assertIn(candidate_name + "_impl", functions)
                self.assertIn("_pick_task_cell_impl", functions)
                self.assertIn(
                    "record_candidate_filter_time",
                    _called_functions(functions[candidate_name]),
                )
                self.assertIn(
                    "record_allocator_time",
                    _called_functions(functions["pick_task_cell"]),
                )

    def test_all_six_programs_export_and_reset_timing_fields(self) -> None:
        for filename in SPECS:
            with self.subTest(filename=filename):
                tree = ast.parse(
                    (ROOT / "hardware" / filename).read_text(encoding="utf-8")
                )
                strings = {
                    node.value
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Constant)
                    and isinstance(node.value, str)
                }
                self.assertFalse(TIMING_FIELDS - strings)

                reset = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "reset_trial_metrics"
                )
                reset_names = {
                    target.id
                    for node in ast.walk(reset)
                    if isinstance(node, ast.Assign)
                    for target in node.targets
                    if isinstance(target, ast.Name)
                }
                self.assertFalse(RESET_COUNTERS - reset_names)


if __name__ == "__main__":
    unittest.main()
