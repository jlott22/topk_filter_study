"""Desktop parity checks for the simulator and RP2040 DGA operators.

The Pololu mission script cannot be imported on CPython because import starts
hardware initialization. This test extracts only its DGA functions and supplies
deterministic state snapshots.
"""

import ast
import random
import sys
import unittest
from array import array
from pathlib import Path
from types import SimpleNamespace


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
HARDWARE_DGA = HERE / "Pololu_DGA.py"
sys.path.insert(0, str(ROOT / "simulator"))

from benchmark_sim.algorithms.DGA import DGAAllocator  # noqa: E402


FUNCTIONS = {
    "_rid_sort_key",
    "_dga_valid_task",
    "_dga_refresh_probability_normalizer",
    "_dga_normalized_probability",
    "_dga_edge_cost",
    "_dga_route_cost",
    "_dga_fitness",
    "_dga_copy_plan",
    "_dga_plan_signature",
    "_dga_pack_cell",
    "_dga_unpack_cell",
    "_dga_is_packed_plan",
    "_dga_pack_plan",
    "_dga_clone_packed_plan",
    "_dga_unpack_plan",
    "_dga_compare_packed_plans",
    "_dga_compare_packed_scores",
    "_dga_insert_packed_score",
    "_dga_fitness_packed",
    "_dga_rank_packed_population",
    "_dga_append_cost",
    "_dga_prepare_repair_masks",
    "_dga_repair_plan",
    "_dga_nearest_neighbor_order",
    "_dga_greedy_seed",
    "_dga_random_seed",
    "_dga_current_path_seed",
    "_dga_rank_population",
    "_dga_tournament",
    "_dga_crossover",
    "_dga_mutate",
    "_dga_prepare_population",
    "_dga_tournament_packed",
    "_dga_next_generation",
    "_dga_owner_path_from_solution",
    "_dga_reconstruct_received",
    "_dga_receive_payload",
}


def _extract_functions():
    tree = ast.parse(HARDWARE_DGA.read_text(encoding="utf-8"), filename=str(HARDWARE_DGA))
    selected = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS
    ]
    missing = FUNCTIONS - {node.name for node in selected}
    if missing:
        raise AssertionError("missing hardware DGA functions: {}".format(sorted(missing)))
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    return compile(module, str(HARDWARE_DGA), "exec")


HARDWARE_CODE = _extract_functions()
PROBABILITIES = {
    (0, 0): 0.125,
    (1, 0): 0.5,
    (2, 0): 0.25,
    (3, 0): 0.375,
    (0, 1): 0.1875,
    (1, 1): 0.4375,
    (2, 1): 0.3125,
    (3, 1): 0.0625,
}
CANDIDATES = list(PROBABILITIES)
TEAM = {"00": (0, 0), "01": (3, 0), "02": (0, 1), "03": (3, 1)}
STUDY_TOP_K_LIMITS = (361, 271, 181, 90, 36, 18)
PARENT_A = {
    "00": [(0, 0), (1, 0)],
    "01": [(2, 0), (3, 0)],
    "02": [(0, 1), (1, 1)],
    "03": [(2, 1), (3, 1)],
}
PARENT_B = {
    "00": [(3, 1), (2, 1)],
    "01": [(1, 1), (0, 1)],
    "02": [(3, 0), (2, 0)],
    "03": [(1, 0), (0, 0)],
}


def _hardware_namespace(seed, probabilities=None):
    grid_size = 19
    index = lambda x, y: (grid_size - 1 - y) * grid_size + x
    probability_by_cell = (
        PROBABILITIES if probabilities is None else probabilities
    )
    probability_array = array(
        "d", [0.0] * (grid_size * grid_size)
    )
    for cell, probability in probability_by_cell.items():
        probability_array[index(*cell)] = probability
    namespace = {
        "random": random.Random(seed),
        "array": array,
        "GRID_SIZE": grid_size,
        "ROBOT_ID": "03",
        "CELL_UNSEARCHED": 0,
        "DGA_POPULATION_SIZE": 30,
        "DGA_ITERATIONS_PER_TRIGGER": 25,
        "DGA_COMMITMENT_HORIZON": 3,
        "DGA_MIN_SUM_TIE_WEIGHT": 0.05,
        "DGA_CROSSOVER_RATE": 0.7,
        "DGA_MUTATION_RATE": 0.3,
        "DGA_ELITE_COUNT": 2,
        "DGA_FITNESS_EPS": 1.0e-9,
        "DGA_FITNESS_SCALE": 100000,
        "DGA_EMPTY_CELL": "X",
        "grid": bytearray(grid_size * grid_size),
        "target_p": probability_array,
        "idx": index,
        "manhattan": lambda x1, y1, x2, y2: abs(x1 - x2) + abs(y1 - y2),
        "dga_probability_normalizer": 0.0,
        "dga_candidate_mask": bytearray(grid_size * grid_size),
        "dga_seen_mask": bytearray(grid_size * grid_size),
        "dga_population": [],
        "dga_received_solutions": [],
        "dga_received_solution_pool": [],
        "dga_received_latest_owner_prefix": {},
        "dga_received_entries": {},
        "dga_received_better_solution": False,
        "dga_best_plan": {},
        "dga_best_fitness": 1000000000000,
        "dga_generation": 0,
        "dga_path": [],
        "current_task_cell": None,
        "pending_collision_reallocation": False,
    }
    exec(HARDWARE_CODE, namespace)
    return namespace


class _FakeRobot:
    pass


def _simulator_robot(seed, probabilities=None, max_candidate_cells=None):
    robot = _FakeRobot()
    robot.rid = "03"
    robot.pos = (0, 0)
    robot.target_p = dict(
        PROBABILITIES if probabilities is None else probabilities
    )
    robot.searched = set()
    robot.known_obstacles = set()
    robot.dga_rng = random.Random(seed)
    robot.dga_population = []
    robot.dga_received_solutions = []
    robot.dga_received_solution_pool = []
    robot.dga_path = []
    robot.cfg = SimpleNamespace(
        max_candidate_cells=max_candidate_cells,
        commitment_horizon=None,
    )
    return robot


def _study_candidates(top_k_limit):
    """Return the canonical DGA-ranked candidate prefix for one study K."""

    weights = {}
    for y in range(19):
        for x in range(19):
            cell = (x, y)
            if cell == (0, 0):
                continue
            weights[cell] = (
                1.0 / (1.0 + abs(x - 9) + abs(y - 9))
                + 0.5 / (1.0 + abs(x - 16) + abs(y - 3))
            )
    total = sum(weights.values())
    probabilities = {
        cell: value / total for cell, value in weights.items()
    }
    probabilities[(0, 0)] = 0.0
    candidates = sorted(
        weights,
        key=lambda cell: (
            -probabilities[cell],
            abs(cell[0]) + abs(cell[1]),
            cell,
        ),
    )[:top_k_limit]
    return probabilities, candidates


class DGASimulatorEquivalenceTests(unittest.TestCase):
    def setUp(self):
        self.allocator = DGAAllocator()

    def test_search_parameters_match(self):
        source = HARDWARE_DGA.read_text(encoding="utf-8")
        self.assertIn("DGA_POPULATION_SIZE = 30", source)
        self.assertIn("DGA_ITERATIONS_PER_TRIGGER = 25", source)
        self.assertIn("DGA_CROSSOVER_RATE = 0.7", source)
        self.assertIn("DGA_MUTATION_RATE = 0.3", source)
        self.assertIn("DGA_ELITE_COUNT = 2", source)

    def test_normalized_edge_cost_matches(self):
        hardware = _hardware_namespace(7)
        robot = _simulator_robot(7)
        for previous, cell in (
            ((0, 0), (1, 0)),
            ((3, 1), (0, 0)),
            ((2, 0), (1, 1)),
        ):
            self.assertEqual(
                self.allocator._edge_cost(robot, previous, cell),
                hardware["_dga_edge_cost"](previous, cell),
            )

    def test_random_segment_crossover_matches(self):
        for seed in range(20):
            with self.subTest(seed=seed):
                hardware = _hardware_namespace(seed)
                robot = _simulator_robot(seed)
                self.assertEqual(
                    self.allocator._crossover(
                        robot, PARENT_A, PARENT_B, TEAM, CANDIDATES
                    ),
                    hardware["_dga_crossover"](
                        PARENT_A, PARENT_B, TEAM, CANDIDATES
                    ),
                )

    def test_all_five_mutation_operators_match(self):
        for seed in range(100):
            with self.subTest(seed=seed):
                hardware = _hardware_namespace(seed)
                robot = _simulator_robot(seed)
                self.assertEqual(
                    self.allocator._mutate(robot, PARENT_A, TEAM, CANDIDATES),
                    hardware["_dga_mutate"](PARENT_A, TEAM, CANDIDATES),
                )

    def test_population_preparation_matches(self):
        received = [{"00": [(0, 0)], "01": [(1, 0)], "02": [], "03": []}]
        for seed in range(5):
            with self.subTest(seed=seed):
                hardware = _hardware_namespace(seed)
                hardware["dga_population"] = [PARENT_A, PARENT_B]
                hardware["dga_received_solution_pool"] = received
                hardware["dga_received_solutions"] = list(received)
                hardware["dga_path"] = [(3, 1), (2, 1)]

                robot = _simulator_robot(seed)
                robot.dga_population = [PARENT_A, PARENT_B]
                robot.dga_received_solution_pool = received
                robot.dga_received_solutions = list(received)
                robot.dga_path = [(3, 1), (2, 1)]

                expected = self.allocator._prepare_population(
                    robot, TEAM, CANDIDATES
                )
                actual = hardware["_dga_prepare_population"](TEAM, CANDIDATES)
                unpacked = [
                    hardware["_dga_unpack_plan"](plan)
                    for plan in actual
                ]
                self.assertEqual(expected, unpacked)
                self.assertEqual(len(actual), 30)

    def test_next_generation_matches(self):
        for seed in range(5):
            with self.subTest(seed=seed):
                hardware = _hardware_namespace(seed)
                robot = _simulator_robot(seed)
                expected_population = self.allocator._prepare_packed_population(
                    robot, TEAM, CANDIDATES
                )
                actual_population = hardware["_dga_prepare_population"](
                    TEAM, CANDIDATES
                )
                self.assertEqual(
                    [
                        self.allocator._unpack_plan(plan)
                        for plan in expected_population
                    ],
                    [
                        hardware["_dga_unpack_plan"](plan)
                        for plan in actual_population
                    ],
                )

                expected = self.allocator._next_packed_generation(
                    robot, expected_population, TEAM, CANDIDATES
                )
                actual = hardware["_dga_next_generation"](
                    actual_population, TEAM, CANDIDATES
                )
                self.assertEqual(
                    [self.allocator._unpack_plan(plan) for plan in expected],
                    [hardware["_dga_unpack_plan"](plan) for plan in actual],
                )

    def test_full_25_generation_search_and_packed_payload_match(self):
        hardware = _hardware_namespace(37)
        robot = _simulator_robot(37)
        expected = self.allocator._prepare_packed_population(
            robot, TEAM, CANDIDATES
        )
        actual = hardware["_dga_prepare_population"](TEAM, CANDIDATES)

        for _ in range(25):
            expected = self.allocator._next_packed_generation(
                robot, expected, TEAM, CANDIDATES
            )
            actual = hardware["_dga_next_generation"](
                actual, TEAM, CANDIDATES
            )

        expected_ranked = self.allocator._rank_packed_population(
            robot, expected, TEAM
        )
        actual_ranked = hardware["_dga_rank_packed_population"](
            actual, TEAM
        )
        self.assertEqual(
            [
                self.allocator._unpack_plan(score.plan)
                for score in expected_ranked
            ],
            [
                hardware["_dga_unpack_plan"](score[0])
                for score in actual_ranked
            ],
        )
        self.assertEqual(
            [score.fitness for score in expected_ranked],
            [score[1] for score in actual_ranked],
        )
        payload_bytes = sum(
            len(plan[0]) * plan[0].itemsize
            + len(plan[1]) * plan[1].itemsize
            for plan in actual
        )
        self.assertEqual(
            payload_bytes,
            len(actual) * (len(CANDIDATES) + len(TEAM)) * 2,
        )

    def test_all_study_topk_25_generations_multiple_seeds_and_rng_isolation(self):
        """Match exact GA output at every study K with unrelated RNG draws."""

        for top_k_limit in STUDY_TOP_K_LIMITS:
            probabilities, candidates = _study_candidates(top_k_limit)
            for seed in (7, 37):
                with self.subTest(top_k_limit=top_k_limit, seed=seed):
                    hardware = _hardware_namespace(
                        9000 + seed, probabilities
                    )
                    robot = _simulator_robot(
                        seed, probabilities, top_k_limit
                    )
                    hardware["dga_rng"] = random.Random(seed)
                    hardware["dga_backoff_rng"] = random.Random(
                        seed + 100000
                    )

                    expected = self.allocator._prepare_packed_population(
                        robot, TEAM, candidates
                    )
                    actual = hardware["_dga_prepare_population"](
                        TEAM, candidates
                    )
                    for generation in range(25):
                        # Packet-loss and backoff draws must not perturb the
                        # dedicated simulator-compatible GA stream.
                        for _ in range((generation * 11) % 7):
                            hardware["random"].random()
                        for _ in range((generation * 5) % 9):
                            hardware["dga_backoff_rng"].random()
                        expected = self.allocator._next_packed_generation(
                            robot, expected, TEAM, candidates
                        )
                        actual = hardware["_dga_next_generation"](
                            actual, TEAM, candidates
                        )

                    expected_ranked = (
                        self.allocator._rank_packed_population(
                            robot, expected, TEAM
                        )
                    )
                    actual_ranked = hardware[
                        "_dga_rank_packed_population"
                    ](actual, TEAM)
                    self.assertEqual(
                        [
                            self.allocator._unpack_plan(score.plan)
                            for score in expected_ranked
                        ],
                        [
                            hardware["_dga_unpack_plan"](score[0])
                            for score in actual_ranked
                        ],
                    )
                    self.assertEqual(
                        [score.fitness for score in expected_ranked],
                        [score[1] for score in actual_ranked],
                    )
                    self.assertEqual(
                        robot.dga_rng.getstate(),
                        hardware["dga_rng"].getstate(),
                    )

    def test_received_solutions_retain_latest_owner_prefixes(self):
        hardware = _hardware_namespace(1)
        hardware["_dga_receive_payload"](
            "01", "s1,1,1000000,00,0,1,0,1,0,1"
        )
        hardware["_dga_receive_payload"](
            "01", "s2,2,2000000,01,0,2,0,1,0,2"
        )

        reconstructed = hardware["_dga_reconstruct_received"]("01", "s2")
        self.assertEqual(reconstructed["00"], [(1, 0)])
        self.assertEqual(reconstructed["01"], [(2, 0)])
        self.assertEqual(
            hardware["dga_received_solutions"],
            hardware["dga_received_solution_pool"],
        )
        self.assertEqual(len(hardware["dga_received_solution_pool"]), 2)


if __name__ == "__main__":
    unittest.main()
