from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from sensitivity_suite.suite import (
    EXPECTED_CONDITIONS,
    EXPECTED_TRIALS,
    MULTITARGET_TRIALS_PER_CONDITION,
    TARGET_COUNT,
    TOP_K_RATES,
    build_manifest_rows,
    edge_even_starts,
    read_csv_rows,
    top_k_limit,
    validate_scenario_files,
    write_clue_scenarios,
    write_known_target_scenarios,
)


class SensitivitySuiteTests(unittest.TestCase):
    def test_top_k_limits(self) -> None:
        self.assertEqual(
            [top_k_limit(14, rate, "clue") for rate in TOP_K_RATES],
            [196, 147, 98, 49, 20, 10],
        )
        self.assertEqual(
            [top_k_limit(19, rate, "clue") for rate in TOP_K_RATES],
            [361, 271, 181, 90, 36, 18],
        )
        self.assertEqual(
            [top_k_limit(28, rate, "clue") for rate in TOP_K_RATES],
            [784, 588, 392, 196, 78, 39],
        )
        self.assertEqual(
            [top_k_limit(19, rate, "known_visit") for rate in TOP_K_RATES],
            [50, 38, 25, 13, 5, 3],
        )

    def test_scenarios_are_reproducible_and_exclude_all_starts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = root / "first.csv"
            second = root / "second.csv"
            write_clue_scenarios(first, 19, (2, 4, 8), 123)
            write_clue_scenarios(second, 19, (2, 4, 8), 123)
            self.assertEqual(first.read_bytes(), second.read_bytes())
            rows = read_csv_rows(first)
            starts = set().union(
                *(edge_even_starts(19, count) for count in (2, 4, 8))
            )
            self.assertEqual(len(rows), 50)
            for row in rows:
                cells = {
                    (int(row["target_x"]), int(row["target_y"])),
                    *[
                        (int(row[f"clue{i}_x"]), int(row[f"clue{i}_y"]))
                        for i in range(1, 5)
                    ],
                }
                self.assertTrue(cells.isdisjoint(starts))
                self.assertEqual(len(cells), 5)

    def test_known_target_scenarios_have_100_trials_with_50_unique_targets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "known.csv"
            write_known_target_scenarios(path, 456)
            rows = read_csv_rows(path)
            self.assertEqual(len(rows), MULTITARGET_TRIALS_PER_CONDITION)
            starts = edge_even_starts(19, 4)
            for row in rows:
                targets = {
                    (int(row[f"target{i}_x"]), int(row[f"target{i}_y"]))
                    for i in range(1, TARGET_COUNT + 1)
                }
                self.assertEqual(len(targets), TARGET_COUNT)
                self.assertTrue(targets.isdisjoint(starts))

    def test_manifest_has_expected_condition_and_trial_counts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scenario_dir = root / "scenarios"
            write_clue_scenarios(
                scenario_dir / "clue_g19_n50.csv", 19, (2, 4, 8), 1
            )
            write_clue_scenarios(
                scenario_dir / "clue_g14_n50.csv", 14, (4,), 2
            )
            write_clue_scenarios(
                scenario_dir / "clue_g28_n50.csv", 28, (4,), 3
            )
            write_known_target_scenarios(
                scenario_dir / "known_targets_g19_t50_n100.csv", 4
            )
            validate_scenario_files(scenario_dir)
            rows = build_manifest_rows(root)
            self.assertEqual(len(rows), EXPECTED_CONDITIONS)
            self.assertEqual(
                sum(int(row["num_trials"]) for row in rows), EXPECTED_TRIALS
            )
            self.assertEqual(len({row["condition_id"] for row in rows}), len(rows))


if __name__ == "__main__":
    unittest.main()
