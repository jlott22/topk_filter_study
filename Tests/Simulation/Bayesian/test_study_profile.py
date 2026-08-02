from __future__ import annotations

import argparse
import unittest

from benchmark_sim.config import SimConfig
from benchmark_sim.core.types import TrialScenario
from benchmark_sim.run_trials import apply_study_profile, select_scenario_shard


def _args(**overrides) -> argparse.Namespace:
    values = {
        "study_profile": "topk_filter",
        "trial_mode": "clue_search",
        "grid_size": 19,
        "num_robots": 4,
        "robot_start_layout": "edge_even",
        "commitment_horizon": None,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class StudyProfileTests(unittest.TestCase):
    def test_scenario_shards_partition_without_changing_order(self) -> None:
        scenarios = [
            TrialScenario(trial_id=index, target=(index, 0), clues=[])
            for index in range(7)
        ]

        shards = [
            select_scenario_shard(scenarios, shard_count=3, shard_index=index)
            for index in range(3)
        ]

        self.assertEqual(
            [[scenario.trial_id for scenario in shard] for shard in shards],
            [[0, 3, 6], [1, 4], [2, 5]],
        )
        self.assertEqual(
            sorted(scenario.trial_id for shard in shards for scenario in shard),
            list(range(7)),
        )

    def test_scenario_shards_reject_invalid_index(self) -> None:
        with self.assertRaisesRegex(ValueError, "trial-shard-index"):
            select_scenario_shard([], shard_count=2, shard_index=2)

    def test_programmatic_config_is_custom_unless_profile_is_explicit(self) -> None:
        self.assertEqual(SimConfig().study_profile, "custom")

    def test_programmatic_topk_profile_enforces_starts_headings_and_horizon(self) -> None:
        cfg = SimConfig(study_profile="topk_filter", commitment_horizon=3)
        self.assertEqual(cfg.start_positions["03"], (0, 18))

        with self.assertRaisesRegex(ValueError, "canonical controls"):
            SimConfig(
                study_profile="topk_filter",
                commitment_horizon=3,
                start_positions={
                    "00": (0, 0),
                    "01": (0, 5),
                    "02": (0, 10),
                    "03": (0, 15),
                },
            )

    def test_topk_profile_locks_effective_horizon_to_three(self) -> None:
        args = _args()

        apply_study_profile(args)

        self.assertEqual(args.commitment_horizon, 3)

    def test_topk_profile_rejects_noncanonical_clue_search_controls(self) -> None:
        cases = [
            {"trial_mode": "coverage"},
            {"grid_size": 20},
            {"num_robots": 3},
            {"commitment_horizon": 5},
        ]
        for overrides in cases:
            with self.subTest(overrides=overrides):
                with self.assertRaisesRegex(ValueError, "canonical controls"):
                    apply_study_profile(_args(**overrides))

    def test_custom_profile_explicitly_allows_legacy_controls(self) -> None:
        args = _args(
            study_profile="custom",
            trial_mode="coverage",
            grid_size=7,
            num_robots=2,
            commitment_horizon=5,
        )

        apply_study_profile(args)

        self.assertEqual(args.commitment_horizon, 5)


if __name__ == "__main__":
    unittest.main()
