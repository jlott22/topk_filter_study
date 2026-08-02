from __future__ import annotations

import importlib.util
import unittest
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from benchmark_sim.algorithms.ACBBA import ACBBAAllocator
from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.algorithms.DGA import DGAAllocator
from benchmark_sim.algorithms.DMCHBA import DMCHBAAllocator
from benchmark_sim.algorithms.HIPC import HIPCAllocator
from benchmark_sim.algorithms.PI import PIAllocator
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


GRID_SIZE = 19
ROBOT_IDS = ("00", "01", "02", "03")
STARTS = {
    "00": (0, 0),
    "01": (0, 6),
    "02": (0, 12),
    "03": (0, 18),
}
TOP_K_LIMITS = (361, 271, 181, 90, 36, 18)

HERE = Path(__file__).resolve()
REFERENCE_ALGORITHM_DIR = (
    HERE.parents[3]
    / "Simulation"
    / "dcta_benchmark_sim"
    / "benchmark_sim"
    / "algorithms"
)


def _load_reference_class(filename: str, class_name: str):
    path = REFERENCE_ALGORITHM_DIR / filename
    if not path.exists():
        raise unittest.SkipTest(
            "reference repository is not available at {}".format(path)
        )
    module_name = "dcta_reference_{}".format(filename[:-3].lower())
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load reference allocator {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, class_name)


ALGORITHMS = (
    (
        "acbba",
        _load_reference_class("ACBBA.py", "ACBBAAllocator"),
        ACBBAAllocator,
        (
            "acbba_path",
            "acbba_bundle",
            "acbba_winner_by_cell",
            "acbba_winning_bid_by_cell",
            "acbba_bid_time_by_cell",
        ),
    ),
    (
        "cbaa",
        _load_reference_class("CBAA.py", "CBAAAllocator"),
        CBAAAllocator,
        (
            "cbaa_current_task",
            "cbaa_winner_by_cell",
            "cbaa_winning_bid_by_cell",
        ),
    ),
    (
        "dga",
        _load_reference_class("DGA.py", "DGAAllocator"),
        DGAAllocator,
        (
            "dga_path",
            "dga_best_plan",
            "dga_best_fitness",
            "dga_generation",
        ),
    ),
    (
        "dmchba",
        _load_reference_class("DMCHBA.py", "DMCHBAAllocator"),
        DMCHBAAllocator,
        (
            "dmchba_path",
            "dmchba_last_assignment_signature",
            "dmchba_clones_per_agent",
            "dmchba_pseudotask_count",
        ),
    ),
    (
        "hipc",
        _load_reference_class("HIPC.py", "HIPCAllocator"),
        HIPCAllocator,
        (
            "hipc_path",
            "hipc_bundle",
            "hipc_winner_by_cell",
            "hipc_winning_bid_by_cell",
            "hipc_bad_prediction_count",
        ),
    ),
    (
        "pi",
        _load_reference_class("PI.py", "PIAllocator"),
        PIAllocator,
        (
            "pi_path",
            "pi_bundle",
            "pi_owner_by_cell",
            "pi_significance_by_cell",
            "pi_time_by_cell",
        ),
    ),
)


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_plain(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_plain(item) for item in value)
    if isinstance(value, set):
        return {_plain(item) for item in value}
    return value


def _make_robot(allocator_class, top_k_limit: int):
    cfg = SimConfig(
        grid_size=GRID_SIZE,
        robot_ids=list(ROBOT_IDS),
        start_positions=dict(STARTS),
        start_headings={rid: EAST for rid in ROBOT_IDS},
        trial_mode="clue_search",
        commitment_horizon=3,
        max_candidate_cells=top_k_limit,
        async_initial_spread_s=0.0,
        async_step_jitter_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        collision_intent_settle_s=0.0,
        write_parquet=False,
    )
    state = AsyncTrialRunner(
        cfg,
        allocator_class,
        make_comm_model("ideal", None),
        seed=77,
    ).new_trial(
        TrialScenario(
            trial_id=77,
            target=(18, 17),
            clues=[(9, 9)],
        )
    )
    robot = state.robots["03"]
    robot.belief.add_clue((9, 9))
    robot._peer_positions = {
        rid: cell for rid, cell in STARTS.items() if rid != robot.rid
    }
    # A controlled non-uniform searched set exercises source-order preservation
    # at K=361 and probability ranking at every truncating K.
    for cell in ((2, 2), (3, 7), (12, 4), (17, 16)):
        robot.belief.mark_searched(cell)
    return robot


class ReferenceRepositoryTopKParityTests(unittest.TestCase):
    def test_all_algorithms_match_reference_at_all_six_topk_limits(self):
        for (
            algorithm,
            reference_class,
            study_class,
            state_attributes,
        ) in ALGORITHMS:
            for top_k_limit in TOP_K_LIMITS:
                with self.subTest(
                    algorithm=algorithm,
                    top_k_limit=top_k_limit,
                ):
                    reference = _make_robot(reference_class, top_k_limit)
                    study = _make_robot(study_class, top_k_limit)

                    reference_candidates = (
                        reference.allocator._candidate_cells(reference)
                    )
                    study_candidates = study.allocator._candidate_cells(study)
                    self.assertEqual(reference_candidates, study_candidates)

                    reference_decision = reference.allocator.choose_goal(reference)
                    study_decision = study.allocator.choose_goal(study)
                    self.assertEqual(reference_decision, study_decision)

                    for attribute in state_attributes:
                        self.assertEqual(
                            _plain(getattr(reference, attribute)),
                            _plain(getattr(study, attribute)),
                            "{} differs at K={} for {}".format(
                                attribute,
                                top_k_limit,
                                algorithm,
                            ),
                        )

                    self.assertEqual(
                        reference.allocator.make_messages(reference),
                        study.allocator.make_messages(study),
                    )


if __name__ == "__main__":
    unittest.main()
