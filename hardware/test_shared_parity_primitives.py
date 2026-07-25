from __future__ import annotations

import unittest

from hardware.allocator_memory import CellIndexedMap, PackedCandidateWorkspace


class SharedParityPrimitiveTests(unittest.TestCase):
    def test_numeric_cell_map_round_trips_binary64_values(self):
        values = CellIndexedMap(19, numeric=True)
        expected = 1.0 / 7.0
        values[(18, 18)] = expected
        self.assertEqual(values._values.typecode, "d")
        self.assertEqual(values[(18, 18)], expected)

    def test_candidate_workspace_preserves_scan_order_without_truncation(self):
        grid_size = 3
        grid = bytearray([0] * 9)
        probabilities = [
            0.1, 0.9, 0.2,
            0.8, 0.3, 0.7,
            0.4, 0.6, 0.5,
        ]
        workspace = PackedCandidateWorkspace(grid_size, 9)
        workspace.fill(
            grid,
            probabilities,
            lambda x, y: y * grid_size + x,
            (0, 0),
            0,
        )
        self.assertEqual(
            list(workspace),
            [(x, y) for y in range(grid_size) for x in range(grid_size)],
        )

    def test_candidate_workspace_ranks_only_after_real_overflow(self):
        grid_size = 3
        grid = bytearray([0] * 9)
        probabilities = [
            0.1, 0.9, 0.2,
            0.8, 0.3, 0.7,
            0.4, 0.6, 0.5,
        ]
        workspace = PackedCandidateWorkspace(grid_size, 4)
        workspace.fill(
            grid,
            probabilities,
            lambda x, y: y * grid_size + x,
            (0, 0),
            0,
        )
        self.assertEqual(
            list(workspace),
            [(1, 0), (0, 1), (2, 1), (1, 2)],
        )

    def test_rank_always_matches_dga_hipc_source_contract(self):
        grid_size = 2
        grid = bytearray([0] * 4)
        probabilities = [0.1, 0.4, 0.3, 0.2]
        workspace = PackedCandidateWorkspace(grid_size, 4)
        workspace.fill(
            grid,
            probabilities,
            lambda x, y: y * grid_size + x,
            (0, 0),
            0,
            rank_always=True,
        )
        self.assertEqual(
            list(workspace),
            [(1, 0), (0, 1), (1, 1), (0, 0)],
        )


if __name__ == "__main__":
    unittest.main()
