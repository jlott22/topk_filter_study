from __future__ import annotations

import unittest
from types import SimpleNamespace

from benchmark_sim.algorithms.ACBBA import ACBBAAllocator
from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.algorithms.DGA import DGAAllocator
from benchmark_sim.algorithms.DMCHBA import DMCHBAAllocator
from benchmark_sim.algorithms.HIPC import HIPCAllocator
from benchmark_sim.algorithms.PI import PIAllocator


class ProbabilityCostConsistencyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.high = (4, 0)
        self.low = (0, 4)
        self.robot = SimpleNamespace(
            pos=(0, 0),
            grid_size=5,
            target_p={self.high: 10.0, self.low: 2.0},
            cfg=SimpleNamespace(trial_mode="clue_search"),
        )

    def test_all_retained_algorithms_use_alpha_eight(self) -> None:
        allocators = [
            CBAAAllocator(),
            ACBBAAllocator(),
            PIAllocator(),
            HIPCAllocator(),
            DMCHBAAllocator(),
            DGAAllocator(),
        ]
        self.assertTrue(all(allocator.PROBABILITY_ALPHA == 8.0 for allocator in allocators))

    def test_algorithm_objectives_match_shared_adjusted_cost(self) -> None:
        expected_high_cost = 4.0
        expected_low_cost = 4.0 + 8.0 * (1.0 - 0.2)

        cbaa = CBAAAllocator()
        self.assertAlmostEqual(cbaa._bid(self.robot, self.high), -expected_high_cost)
        self.assertAlmostEqual(cbaa._bid(self.robot, self.low), -expected_low_cost)

        acbba = ACBBAAllocator()
        self.assertAlmostEqual(
            acbba._bid_from_reference(self.robot, self.high, self.robot.pos),
            -expected_high_cost,
        )
        self.assertAlmostEqual(
            acbba._bid_from_reference(self.robot, self.low, self.robot.pos),
            -expected_low_cost,
        )

        pi = PIAllocator()
        self.assertAlmostEqual(
            pi._effective_move_cost(self.robot, self.robot.pos, self.high),
            expected_high_cost,
        )
        self.assertAlmostEqual(
            pi._effective_move_cost(self.robot, self.robot.pos, self.low),
            expected_low_cost,
        )

        hipc = HIPCAllocator()
        self.assertAlmostEqual(
            hipc._bid_from_reference(self.robot, self.high, self.robot.pos),
            -expected_high_cost,
        )
        self.assertAlmostEqual(
            hipc._bid_from_reference(self.robot, self.low, self.robot.pos),
            -expected_low_cost,
        )

        dmchba = DMCHBAAllocator()
        matrix = dmchba._build_cost_matrix(
            self.robot,
            [("00", self.robot.pos, 0)],
            [self.high, self.low],
        )
        self.assertAlmostEqual(matrix[0][0], expected_high_cost, places=6)
        self.assertAlmostEqual(matrix[0][1], expected_low_cost, places=6)

        dga = DGAAllocator()
        self.assertAlmostEqual(
            dga._edge_cost(self.robot, self.robot.pos, self.high),
            expected_high_cost,
        )
        self.assertAlmostEqual(
            dga._edge_cost(self.robot, self.robot.pos, self.low),
            expected_low_cost,
        )


if __name__ == "__main__":
    unittest.main()
