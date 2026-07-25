from __future__ import annotations

import argparse
import unittest

from benchmark_sim.config import SimConfig
from benchmark_sim.run_trials import apply_study_profile


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
