from __future__ import annotations

import unittest

from benchmark_sim.algorithms.CBAA import CBAAAllocator
from benchmark_sim.config import EAST, SimConfig, edge_even_start_positions, generate_robot_ids


class _Robot:
    def __init__(self, rid: str, grid_size: int, robot_ids: list[str]) -> None:
        self.rid = rid
        self.grid_size = grid_size
        self.cfg = type("Config", (), {"robot_ids": robot_ids})()


class GridDensityScalingTests(unittest.TestCase):
    def test_edge_even_single_robot(self) -> None:
        robot_ids = generate_robot_ids(1)
        self.assertEqual(robot_ids, ["00"])
        self.assertEqual(edge_even_start_positions(14, robot_ids), {"00": (0, 6)})

    def test_edge_even_46_robots(self) -> None:
        robot_ids = generate_robot_ids(46)
        starts = edge_even_start_positions(48, robot_ids)
        self.assertEqual(starts["00"], (0, 0))
        self.assertEqual(starts["45"], (0, 47))
        self.assertEqual(len(set(starts.values())), 46)

    def test_config_generates_starts_and_east_headings(self) -> None:
        robot_ids = generate_robot_ids(46)
        cfg = SimConfig(grid_size=48, robot_ids=robot_ids)
        self.assertEqual(cfg.start_positions, edge_even_start_positions(48, robot_ids))
        self.assertTrue(all(cfg.start_headings[rid] == EAST for rid in robot_ids))

    def test_dynamic_bands_cover_each_row_once(self) -> None:
        robot_ids = generate_robot_ids(46)
        allocator = CBAAAllocator()
        bands = [allocator._assigned_row_band(_Robot(rid, 48, robot_ids)) for rid in robot_ids]
        rows = [row for start, end in bands for row in range(start, end + 1)]
        self.assertEqual(rows, list(range(48)))
        self.assertEqual(bands[:3], [(0, 1), (2, 3), (4, 4)])

    def test_ids_remain_at_least_two_digits(self) -> None:
        robot_ids = generate_robot_ids(101)
        self.assertEqual(robot_ids[0], "00")
        self.assertEqual(robot_ids[99], "99")
        self.assertEqual(robot_ids[100], "100")


if __name__ == "__main__":
    unittest.main()
