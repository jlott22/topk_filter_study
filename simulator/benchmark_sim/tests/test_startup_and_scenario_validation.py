from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.belief import BeliefMap
from benchmark_sim.core.scenario_loader import load_scenarios, validate_scenario
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import AllocationDecision, TrialScenario
from benchmark_sim.run_trials import (
    DEFAULT_TOPK_SCENARIO_MANIFEST_LOCK,
    enforce_scenario_manifest_lock,
    enforce_expected_scenario_sha256,
    resolve_scenario_manifest_lock,
    scenario_selection_sha256,
)


class _ObservationAllocator(AllocatorBase):
    def initialize(self, robot) -> None:
        self.observations = []
        self.clue_messages_seen_during_observation = []

    def on_observation(self, robot, observation) -> None:
        self.observations.append(observation)
        self.clue_messages_seen_during_observation.append(
            robot.bus.counters.sent_by_topic.get("clue", 0)
        )

    def choose_goal(self, robot):
        return AllocationDecision(goal=None)


def _config() -> SimConfig:
    return SimConfig(
        grid_size=3,
        robot_ids=["00", "01"],
        start_positions={"00": (0, 0), "01": (0, 2)},
        start_headings={"00": EAST, "01": EAST},
        async_initial_spread_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        write_parquet=False,
    )


class StartupObservationTests(unittest.TestCase):
    def test_start_clue_is_observed_after_registration_without_extra_visit_or_step(self) -> None:
        cfg = _config()
        runner = AsyncTrialRunner(
            cfg,
            _ObservationAllocator,
            make_comm_model("ideal", None),
            seed=0,
        )
        state = runner.new_trial(
            TrialScenario(trial_id=0, target=(2, 1), clues=[(0, 0)])
        )
        finder = state.robots["00"]
        peer = state.robots["01"]

        self.assertEqual(set(state.bus.receivers), {"00", "01"})
        self.assertEqual(state.world.first_clue_time_s, 0.0)
        self.assertEqual(state.world.first_clue_robot, "00")
        self.assertEqual(state.world.first_clue_cell, (0, 0))
        self.assertIn((0, 0), finder.known_clues)
        self.assertNotIn((0, 0), peer.known_clues)
        self.assertEqual(state.bus.counters.sent_by_topic["clue"], 1)

        self.assertEqual(finder.counters.steps_total, 0)
        self.assertEqual(peer.counters.steps_total, 0)
        self.assertEqual(state.world.visits[(0, 0)].total_visits, 1)
        self.assertEqual(state.world.visits[(0, 2)].total_visits, 1)
        self.assertEqual(state.world.unique_cells_searched(), 2)
        self.assertEqual(state.world.system_revisits(), 0)
        self.assertEqual(len(finder.allocator.observations), 1)
        self.assertTrue(finder.allocator.observations[0].clue_detected)
        self.assertEqual(finder.allocator.clue_messages_seen_during_observation, [0])
        self.assertEqual(len(peer.allocator.observations), 1)
        self.assertFalse(peer.allocator.observations[0].clue_detected)

        state.bus.pump(0.0)
        self.assertIn((0, 0), peer.known_clues)

    def test_duplicate_searched_cell_does_not_recompute_belief(self) -> None:
        belief = BeliefMap(3)
        recompute_calls = 0
        original_recompute = belief.recompute

        def counted_recompute() -> None:
            nonlocal recompute_calls
            recompute_calls += 1
            original_recompute()

        belief.recompute = counted_recompute
        self.assertTrue(belief.mark_searched((1, 1)))
        probabilities_after_first_report = belief.target_p
        self.assertFalse(belief.mark_searched((1, 1)))

        self.assertEqual(recompute_calls, 1)
        self.assertIs(belief.target_p, probabilities_after_first_report)


class ScenarioValidationTests(unittest.TestCase):
    def test_target_on_start_is_rejected_but_clue_on_start_is_allowed(self) -> None:
        cfg = _config()
        with self.assertRaisesRegex(ValueError, "overlaps a robot start"):
            validate_scenario(
                TrialScenario(trial_id=1, target=(0, 0), clues=[(1, 1)]),
                grid_size=cfg.grid_size,
                start_positions=cfg.start_positions,
            )

        validate_scenario(
            TrialScenario(trial_id=2, target=(2, 1), clues=[(0, 0)]),
            grid_size=cfg.grid_size,
            start_positions=cfg.start_positions,
        )

    def test_direct_scenario_validation_rejects_fractional_coordinates(self) -> None:
        cfg = _config()
        with self.assertRaisesRegex(ValueError, "must be an integer"):
            validate_scenario(
                TrialScenario(
                    trial_id=3,
                    target=(1.5, 1),
                    clues=[(0, 0)],
                ),
                grid_size=cfg.grid_size,
                start_positions=cfg.start_positions,
            )
        with self.assertRaisesRegex(ValueError, "must be an integer"):
            validate_scenario(
                TrialScenario(
                    trial_id=4,
                    target=(2, 1),
                    clues=[(0, 0.5)],
                ),
                grid_size=cfg.grid_size,
                start_positions=cfg.start_positions,
            )

    def test_loader_rejects_duplicate_ids_coordinates_and_short_requested_input(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            duplicate_ids = root / "duplicate_ids.csv"
            duplicate_ids.write_text(
                "episode,object_x,object_y,clue1_x,clue1_y\n"
                "7,1,1,2,2\n"
                "7,2,1,1,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate trial ID"):
                load_scenarios(duplicate_ids)

            duplicate_coordinates = root / "duplicate_coordinates.csv"
            duplicate_coordinates.write_text(
                "episode,object_x,object_y,clue1_x,clue1_y\n"
                "0,1,1,1,1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate target/clue coordinate"):
                load_scenarios(duplicate_coordinates)

            short_input = root / "short.csv"
            short_input.write_text(
                "episode,object_x,object_y,clue1_x,clue1_y\n"
                "0,1,1,2,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "fewer than requested"):
                load_scenarios(short_input, max_trials=2)

    def test_loader_rejects_incomplete_clue_columns_and_metadata_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            incomplete = root / "incomplete.csv"
            incomplete.write_text(
                "episode,object_x,object_y,clue1_x\n"
                "0,1,1,2\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "incomplete clue1 coordinate pair"):
                load_scenarios(incomplete)

            metadata = root / "metadata.csv"
            metadata.write_text(
                "# grid_size=4\n"
                "episode,object_x,object_y,clue1_x,clue1_y\n"
                "0,1,1,2,2\n",
                encoding="utf-8",
            )
            scenario = load_scenarios(metadata)[0]
            with self.assertRaisesRegex(ValueError, "declares grid_size=4"):
                validate_scenario(
                    scenario,
                    grid_size=3,
                    start_positions={"00": (0, 0)},
                )

    def test_json_loader_rejects_fractional_coordinates(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "fractional.json"
            path.write_text(
                json.dumps(
                    [
                        {
                            "trial_id": 1,
                            "target": [1.5, 1],
                            "clues": [[2, 2]],
                        }
                    ]
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "must be an integer"):
                load_scenarios(path)

    def test_selection_hash_uses_selected_order_and_canonical_payload(self) -> None:
        scenarios = [
            TrialScenario(trial_id=2, target=(2, 1), clues=[(0, 0), (1, 1)]),
            TrialScenario(trial_id=5, target=(1, 2), clues=[(2, 0)]),
        ]
        payload = json.dumps(
            [
                {
                    "trial_id": "2",
                    "target": [2, 1],
                    "clues": [[0, 0], [1, 1]],
                },
                {
                    "trial_id": "5",
                    "target": [1, 2],
                    "clues": [[2, 0]],
                },
            ],
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")

        expected = hashlib.sha256(payload).hexdigest()
        self.assertEqual(scenario_selection_sha256(scenarios), expected)
        self.assertNotEqual(
            scenario_selection_sha256(scenarios),
            scenario_selection_sha256(list(reversed(scenarios))),
        )

    def test_expected_selection_hash_fails_before_a_mismatched_run(self) -> None:
        actual = "a" * 64
        enforce_expected_scenario_sha256(actual, actual.upper())
        enforce_expected_scenario_sha256(actual, None)

        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            enforce_expected_scenario_sha256(actual, "b" * 64)
        with self.assertRaisesRegex(ValueError, "invalid expected"):
            enforce_expected_scenario_sha256(actual, "not-a-hash")

    def test_study_manifest_lock_rejects_a_different_ordered_selection(self) -> None:
        first = [
            TrialScenario(trial_id=1, target=(2, 1), clues=[(0, 0)]),
            TrialScenario(trial_id=2, target=(1, 2), clues=[(0, 2)]),
        ]
        changed = [
            TrialScenario(trial_id=1, target=(2, 1), clues=[(0, 0)]),
            TrialScenario(trial_id=3, target=(1, 2), clues=[(0, 2)]),
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            lock = Path(temp_dir) / "selection.json"
            expected_hash = scenario_selection_sha256(first)

            self.assertEqual(
                enforce_scenario_manifest_lock(
                    lock,
                    first,
                    grid_size=19,
                    logic_revision="dcta_parity_v1",
                ),
                expected_hash,
            )
            record = json.loads(lock.read_text(encoding="utf-8"))
            self.assertEqual(record["trial_ids"], ["1", "2"])
            self.assertEqual(record["scenario_sha256"], expected_hash)

            self.assertEqual(
                enforce_scenario_manifest_lock(
                    lock,
                    list(first),
                    grid_size=19,
                    logic_revision="dcta_parity_v1",
                ),
                expected_hash,
            )
            with self.assertRaisesRegex(ValueError, "manifest lock"):
                enforce_scenario_manifest_lock(
                    lock,
                    changed,
                    grid_size=19,
                    logic_revision="dcta_parity_v1",
                )

    def test_manifest_lock_is_automatic_only_for_the_study_profile(self) -> None:
        self.assertEqual(
            resolve_scenario_manifest_lock("topk_filter", None, False),
            DEFAULT_TOPK_SCENARIO_MANIFEST_LOCK,
        )
        self.assertIsNone(
            resolve_scenario_manifest_lock("custom", None, False)
        )
        self.assertIsNone(
            resolve_scenario_manifest_lock("topk_filter", None, True)
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            override = Path(temp_dir) / "override.json"
            self.assertEqual(
                resolve_scenario_manifest_lock("custom", override, False),
                override.resolve(),
            )


if __name__ == "__main__":
    unittest.main()
