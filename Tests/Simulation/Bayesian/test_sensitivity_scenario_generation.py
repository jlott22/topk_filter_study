from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from benchmark_sim.config import edge_even_start_positions, generate_robot_ids
from benchmark_sim.core.scenario_loader import load_scenarios
from Simulation.Workflows.Bayesian.generate_clue_sensitivity_scenarios import (
    generate_scenario_file,
)


class SensitivityScenarioGenerationTests(unittest.TestCase):
    def test_generation_is_reproducible_and_avoids_robot_starts(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            first = Path(temp_dir) / "first.csv"
            second = Path(temp_dir) / "second.csv"
            kwargs = {
                "grid_size": 7,
                "num_trials": 12,
                "num_clues": 4,
                "num_robots": 4,
                "seed": 20260702,
                "target_decay_exp": 1.0,
            }

            generate_scenario_file(first, **kwargs)
            generate_scenario_file(second, **kwargs)

            self.assertEqual(first.read_text(encoding="utf-8"), second.read_text(encoding="utf-8"))
            scenarios = load_scenarios(first)
            starts = set(edge_even_start_positions(7, generate_robot_ids(4)).values())
            self.assertEqual(len(scenarios), 12)
            for scenario in scenarios:
                self.assertNotIn(scenario.target, starts)
                self.assertEqual(len(scenario.clues), 4)
                self.assertEqual(len(set(scenario.clues)), 4)
                self.assertNotIn(scenario.target, scenario.clues)
                self.assertTrue(starts.isdisjoint(scenario.clues))


if __name__ == "__main__":
    unittest.main()
