"""Desktop parity tests for RP2040 allocator memory optimizations."""

import ast
import random
import unittest
from array import array
from pathlib import Path

from hardware.allocator_memory import (
    CellIndexedMap,
    PackedCandidateWorkspace,
)


ROOT = Path(__file__).resolve().parents[1]
CASES = {
    "acbba": (
        ROOT / "hardware" / "Pololu_ACBBA.py",
        ROOT / "hardware" / "Pololu_ACBBA.py",
        "_acbba_build_bundle_impl",
        "acbba_flush_messages",
    ),
    "cbaa": (
        ROOT / "hardware" / "Pololu_CBAA.py",
        ROOT / "hardware" / "Pololu_CBAA.py",
        "_cbaa_select_new_task",
        "cbaa_flush_messages",
    ),
    "hipc": (
        ROOT / "hardware" / "Pololu_HIPC.py",
        ROOT / "hardware" / "Pololu_HIPC.py",
        "_hipc_build_bundle_impl",
        "hipc_flush_messages",
    ),
    "pi": (
        ROOT / "hardware" / "Pololu_PI.py",
        ROOT / "hardware" / "Pololu_PI.py",
        "_pi_build_bundle_impl",
        "pi_flush_messages",
    ),
}

SHARED_FUNCTIONS = {
    "idx",
    "manhattan",
    "_rid_sort_key",
    "_cell_sort_key",
    "_same_robot_id",
    "_robot_id_less",
    "_bid_gt",
    "_bid_lt",
    "_bid_eq",
    "_time_gt",
    "_time_lt",
    "_time_eq",
    "_time_gte",
    "_time_lte",
    "_trial_traffic_enabled",
}


class _TimeStub:
    @staticmethod
    def ticks_us():
        return 0


def _allocator_code(path, prefix, extra_names):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = []
    for node in tree.body:
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        if (
            node.name.startswith(prefix)
            or node.name in SHARED_FUNCTIONS
            or node.name in extra_names
        ):
            selected.append(node)
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    return compile(module, str(path), "exec")


COMPILED = {}
for _algorithm, (_old, _new, _solve, _flush) in CASES.items():
    _prefix = "_" + _algorithm
    _extra = {_flush}
    COMPILED[(_algorithm, False)] = _allocator_code(_old, _prefix, _extra)
    COMPILED[(_algorithm, True)] = _allocator_code(_new, _prefix, _extra)


def _probabilities(grid_size, seed, uniform=False):
    rng = random.Random(seed)
    count = grid_size * grid_size
    values = [1.0] * count if uniform else [
        0.01 + rng.random() for _ in range(count)
    ]
    total = sum(values)
    return array("d", [value / total for value in values])


def _snapshot(grid_size, seed, team_size):
    rng = random.Random(seed)
    grid = bytearray(grid_size * grid_size)
    position = [rng.randrange(grid_size), rng.randrange(grid_size)]
    cells = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if (x, y) != tuple(position)
    ]
    rng.shuffle(cells)
    for x, y in cells[: seed % max(1, grid_size)]:
        grid[(grid_size - 1 - y) * grid_size + x] = 2
    for x, y in cells[grid_size: grid_size + seed % 3]:
        grid[(grid_size - 1 - y) * grid_size + x] = 1

    peer_positions = {}
    occupied = {tuple(position)}
    for peer_index in range(1, team_size):
        while True:
            cell = (rng.randrange(grid_size), rng.randrange(grid_size))
            if cell in occupied:
                continue
            occupied.add(cell)
            peer_positions["{:02d}".format(peer_index)] = cell
            break
    return grid, position, peer_positions


class _FloatValueDict(dict):
    """Plain mapping reference with the production binary64 value contract."""

    def __setitem__(self, key, value):
        super().__setitem__(key, float(value))


def _cell_map(grid_size, numeric, optimized):
    if optimized:
        return CellIndexedMap(grid_size, numeric=numeric)
    return _FloatValueDict() if numeric else {}


def _namespace(algorithm, optimized, grid_size, topk_fraction, seed, team_size):
    grid, position, peer_positions = _snapshot(
        grid_size,
        seed,
        team_size,
    )
    topk = max(1, int(grid_size * grid_size * topk_fraction + 0.5))
    probabilities = _probabilities(
        grid_size,
        seed + 1000,
        uniform=(seed % 4 == 0),
    )
    sent = []
    namespace = {
        "array": array,
        "time": _TimeStub,
        "safe_assert": lambda condition, message: (
            None if condition else (_ for _ in ()).throw(AssertionError(message))
        ),
        "record_candidate_filter_time": lambda _started: None,
        "record_allocator_solve_time": lambda *_args: None,
        "GRID_SIZE": grid_size,
        "TOP_K_MAX_CELLS": topk,
        "CELL_UNSEARCHED": 0,
        "CELL_OBSTACLE": 1,
        "CELL_SEARCHED": 2,
        "ROBOT_ID": "00",
        "NUM_ROBOTS": 4,
        "REWARD_FACTOR": 5,
        "grid": bytearray(grid),
        "target_p": array("d", probabilities),
        "prob_map": array("d", probabilities),
        "pos": list(position),
        "peer_pos": dict(peer_positions),
        "clues": [(1, 1)],
        "first_clue_seen": True,
        "start_signal": True,
        "current_task_cell": None,
        "pending_collision_reallocation": False,
        "temporary_invalid_task_until": {},
        "allocation_probability_normalizer": max(probabilities),
        "_sent": sent,
    }
    namespace["normalized_target_probability"] = lambda cell: (
        float(namespace["target_p"][namespace["idx"](*cell)])
        / namespace["allocation_probability_normalizer"]
    )

    if algorithm == "acbba":
        namespace.update({
            "ACBBA_BUNDLE_SIZE": 3,
            "ACBBA_REWARD_FACTOR": 5,
            "ACBBA_NO_WINNER_CODE": "99",
            "ACBBA_EMPTY_FIELD": "X",
            "ACBBA_NO_BID": -1.0e18,
            "ACBBA_NO_TIME": -1.0e18,
            "ACBBA_EPS_BID": 1.0e-9,
            "ACBBA_EPS_TIME": 1.0e-9,
            "acbba_winner_by_cell": _cell_map(grid_size, False, optimized),
            "acbba_winning_bid_by_cell": _cell_map(
                grid_size, True, optimized),
            "acbba_bid_time_by_cell": _cell_map(
                grid_size, True, optimized),
            "acbba_path": [],
            "acbba_bundle": [],
            "acbba_bid_counter": 0,
            "acbba_pending_deltas": _cell_map(
                grid_size, False, optimized),
            "acbba_last_sent_signatures": _cell_map(
                grid_size, False, optimized),
            "acbba_pending_snapshot": False,
            "publish_acbba_payload": sent.append,
        })
    elif algorithm == "cbaa":
        namespace.update({
            "CBAA_NO_WINNER_CODE": "99",
            "CBAA_EMPTY_FIELD": "X",
            "CBAA_NO_BID": -1.0e18,
            "CBAA_EPS_BID": 1.0e-9,
            "cbaa_winner_by_cell": _cell_map(grid_size, False, optimized),
            "cbaa_winning_bid_by_cell": _cell_map(
                grid_size, True, optimized),
            "cbaa_current_task": None,
            "cbaa_pending_deltas": _cell_map(
                grid_size, False, optimized),
            "cbaa_last_sent_signatures": _cell_map(
                grid_size, False, optimized),
            "publish_cbaa_payload": sent.append,
        })
    elif algorithm == "hipc":
        namespace.update({
            "HIPC_BUNDLE_SIZE": 3,
            "HIPC_NO_WINNER_CODE": "99",
            "HIPC_EMPTY_FIELD": "X",
            "HIPC_NO_BID": -1.0e18,
            "HIPC_NO_TIME": -1.0e18,
            "HIPC_EPS_BID": 1.0e-9,
            "HIPC_BAD_PRED_LIMIT": 3,
            "HIPC_PREDICTION_TOLERANCE": 0,
            "hipc_winner_by_cell": _cell_map(grid_size, False, optimized),
            "hipc_winning_bid_by_cell": _cell_map(
                grid_size, True, optimized),
            "hipc_bid_time_by_cell": _cell_map(
                grid_size, True, optimized),
            "hipc_path": [],
            "hipc_bundle": [],
            "hipc_bid_counter": 0,
            "hipc_pending_snapshot": False,
            "hipc_last_sent_signature": None,
            "hipc_bad_prediction_count": {},
            "hipc_last_predicted_peer_first_task": {},
            "hipc_dropped_peers": set(),
            "publish_hipc_payload": sent.append,
        })
    else:
        namespace.update({
            "PI_BUNDLE_SIZE": 3,
            "PI_NO_OWNER_CODE": "99",
            "PI_EMPTY_FIELD": "X",
            "PI_INF_SIGNIFICANCE": 1.0e18,
            "PI_NO_TIME": -1.0e18,
            "PI_EPS_SIGNIFICANCE": 1.0e-9,
            "pi_owner_by_cell": _cell_map(grid_size, False, optimized),
            "pi_significance_by_cell": _cell_map(
                grid_size, True, optimized),
            "pi_time_by_cell": _cell_map(grid_size, True, optimized),
            "pi_path": [],
            "pi_bundle": [],
            "pi_time_counter": 0,
            "pi_pending_snapshot": False,
            "pi_last_sent_signature": None,
            "publish_pi_payload": sent.append,
        })

    namespace[algorithm + "_candidate_workspace"] = (
        PackedCandidateWorkspace(grid_size, topk)
    )
    exec(COMPILED[(algorithm, optimized)], namespace)
    if not optimized:
        valid_task = namespace["_" + algorithm + "_valid_task"]

        def canonical_candidate_cells():
            cells = [
                (x, y)
                for y in range(grid_size)
                for x in range(grid_size)
                if valid_task((x, y))
            ]
            if algorithm == "hipc" or len(cells) > topk:
                cells.sort(
                    key=lambda cell: (
                        -float(
                            namespace["target_p"][
                                namespace["idx"](*cell)
                            ]
                        ),
                        abs(namespace["pos"][0] - cell[0])
                        + abs(namespace["pos"][1] - cell[1]),
                        cell,
                    )
                )
                if len(cells) > topk:
                    del cells[topk:]
            return cells

        candidate_name = (
            "_hipc_candidates"
            if algorithm == "hipc"
            else "_pi_candidates"
            if algorithm == "pi"
            else "_" + algorithm + "_candidate_cells"
        )
        namespace[candidate_name] = canonical_candidate_cells
    return namespace


def _plain(mapping):
    return dict(mapping.items())


def _candidate_list(namespace, algorithm):
    function = (
        "_hipc_candidates"
        if algorithm == "hipc"
        else "_" + algorithm + "_candidate_cells"
        if algorithm in {"acbba", "cbaa"}
        else "_pi_candidates"
    )
    return list(namespace[function]())


def _state(namespace, algorithm):
    if algorithm == "acbba":
        return (
            namespace["acbba_path"],
            namespace["acbba_bundle"],
            _plain(namespace["acbba_winner_by_cell"]),
            _plain(namespace["acbba_winning_bid_by_cell"]),
            _plain(namespace["acbba_bid_time_by_cell"]),
        )
    if algorithm == "cbaa":
        return (
            namespace["cbaa_current_task"],
            _plain(namespace["cbaa_winner_by_cell"]),
            _plain(namespace["cbaa_winning_bid_by_cell"]),
        )
    if algorithm == "hipc":
        return (
            namespace["hipc_path"],
            namespace["hipc_bundle"],
            _plain(namespace["hipc_winner_by_cell"]),
            _plain(namespace["hipc_winning_bid_by_cell"]),
            _plain(namespace["hipc_bid_time_by_cell"]),
            namespace["hipc_last_predicted_peer_first_task"],
        )
    return (
        namespace["pi_path"],
        namespace["pi_bundle"],
        _plain(namespace["pi_owner_by_cell"]),
        _plain(namespace["pi_significance_by_cell"]),
        _plain(namespace["pi_time_by_cell"]),
    )


def _seed_consensus(namespace, algorithm, seed):
    grid_size = namespace["GRID_SIZE"]
    available = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if namespace["grid"][namespace["idx"](x, y)]
        == namespace["CELL_UNSEARCHED"]
        and (x, y) != tuple(namespace["pos"])
    ]
    random.Random(seed).shuffle(available)
    for timestamp, cell in enumerate(available[:2], start=1):
        if algorithm == "acbba":
            namespace["acbba_winner_by_cell"][cell] = "01"
            namespace["acbba_winning_bid_by_cell"][cell] = -500000 - timestamp
            namespace["acbba_bid_time_by_cell"][cell] = timestamp
        elif algorithm == "cbaa":
            namespace["cbaa_winner_by_cell"][cell] = "01"
            namespace["cbaa_winning_bid_by_cell"][cell] = -500000 - timestamp
        elif algorithm == "hipc":
            namespace["hipc_winner_by_cell"][cell] = "01"
            namespace["hipc_winning_bid_by_cell"][cell] = -500000 - timestamp
            namespace["hipc_bid_time_by_cell"][cell] = timestamp
        else:
            namespace["pi_owner_by_cell"][cell] = "01"
            namespace["pi_significance_by_cell"][cell] = 500000 + timestamp
            namespace["pi_time_by_cell"][cell] = timestamp


class HardwareAllocatorMemoryOptimizedEquivalenceTests(unittest.TestCase):
    def test_dga_packed_population_matches_canonical_25_generations(self):
        from hardware.test_dga_simulator_equivalence import (
            CANDIDATES,
            TEAM,
            _hardware_namespace,
            _simulator_robot,
        )
        from simulator.benchmark_sim.algorithms.DGA import DGAAllocator

        allocator = DGAAllocator()
        for seed in (7, 19, 37):
            with self.subTest(algorithm="dga", seed=seed):
                hardware = _hardware_namespace(seed)
                robot = _simulator_robot(seed)
                canonical = allocator._prepare_packed_population(
                    robot, TEAM, CANDIDATES)
                packed = hardware["_dga_prepare_population"](
                    TEAM, CANDIDATES)

                for _ in range(25):
                    canonical = allocator._next_packed_generation(
                        robot, canonical, TEAM, CANDIDATES)
                    packed = hardware["_dga_next_generation"](
                        packed, TEAM, CANDIDATES)

                canonical_ranked = allocator._rank_packed_population(
                    robot, canonical, TEAM)
                packed_ranked = hardware[
                    "_dga_rank_packed_population"
                ](packed, TEAM)
                self.assertEqual(
                    [
                        allocator._unpack_plan(score.plan)
                        for score in canonical_ranked
                    ],
                    [
                        hardware["_dga_unpack_plan"](score[0])
                        for score in packed_ranked
                    ],
                )
                self.assertEqual(
                    [score.fitness for score in canonical_ranked],
                    [score[1] for score in packed_ranked],
                )
                packed_bytes = sum(
                    len(plan[0]) * plan[0].itemsize
                    + len(plan[1]) * plan[1].itemsize
                    for plan in packed
                )
                self.assertEqual(
                    packed_bytes,
                    len(packed)
                    * (len(CANDIDATES) + len(TEAM))
                    * 2,
                )

    def test_randomized_candidates_paths_tables_and_messages_match(self):
        fractions = (0.20, 0.35, 0.50, 0.75, 1.0)
        for algorithm, (_old, _new, solve_name, flush_name) in CASES.items():
            for seed in range(16):
                grid_size = 5 + seed % 5
                fraction = fractions[seed % len(fractions)]
                team_size = 1 + seed % 4
                original = _namespace(
                    algorithm,
                    False,
                    grid_size,
                    fraction,
                    seed + 30,
                    team_size,
                )
                optimized = _namespace(
                    algorithm,
                    True,
                    grid_size,
                    fraction,
                    seed + 30,
                    team_size,
                )
                _seed_consensus(original, algorithm, seed + 300)
                _seed_consensus(optimized, algorithm, seed + 300)
                with self.subTest(
                    algorithm=algorithm,
                    seed=seed,
                    grid_size=grid_size,
                    fraction=fraction,
                ):
                    self.assertEqual(
                        _candidate_list(original, algorithm),
                        _candidate_list(optimized, algorithm),
                    )
                    original[solve_name]()
                    optimized[solve_name]()
                    self.assertEqual(
                        _state(original, algorithm),
                        _state(optimized, algorithm),
                    )
                    original[flush_name]()
                    optimized[flush_name]()
                    self.assertEqual(original["_sent"], optimized["_sent"])

    def test_production_grid_topk_271_matches(self):
        for algorithm, (_old, _new, solve_name, _flush) in CASES.items():
            original = _namespace(
                algorithm, False, 19, 0.75, 111, 4)
            optimized = _namespace(
                algorithm, True, 19, 0.75, 111, 4)
            _seed_consensus(original, algorithm, 411)
            _seed_consensus(optimized, algorithm, 411)
            with self.subTest(algorithm=algorithm):
                original_candidates = _candidate_list(original, algorithm)
                optimized_candidates = _candidate_list(optimized, algorithm)
                self.assertEqual(len(original_candidates), 271)
                self.assertEqual(original_candidates, optimized_candidates)
                original[solve_name]()
                optimized[solve_name]()
                self.assertEqual(
                    _state(original, algorithm),
                    _state(optimized, algorithm),
                )


if __name__ == "__main__":
    unittest.main()
