"""Desktop parity tests for simulator, reference, and optimized hardware DMCHBA.

The Pololu scripts cannot be imported on CPython because importing them starts
hardware initialization and the robot mission.  This harness extracts only the
allocator function definitions from each script and supplies deterministic
hardware-state snapshots.
"""

import ast
import gc
import math
import random
import sys
import unittest
from array import array
from pathlib import Path


HERE = Path(__file__).resolve().parent
ORIGINAL = HERE.parent / "archive" / "Pololu_DMCHBA_unoptimized.py"
OPTIMIZED = HERE / "Pololu_DMCHBA.py"
ROOT = HERE.parent
if str(ROOT / "simulator") not in sys.path:
    sys.path.insert(0, str(ROOT / "simulator"))

from benchmark_sim.algorithms.DMCHBA import DMCHBAAllocator  # noqa: E402
from benchmark_sim.comms.models import make_comm_model  # noqa: E402
from benchmark_sim.config import EAST, SimConfig  # noqa: E402
from benchmark_sim.core.scheduler import AsyncTrialRunner  # noqa: E402
from benchmark_sim.core.types import TrialScenario  # noqa: E402

STUDY_TOP_K_LIMITS = (361, 271, 181, 90, 36, 18)

COMMON_FUNCTIONS = {
    "idx",
    "manhattan",
    "_rid_sort_key",
    "_cell_sort_key",
    "_dmchba_clue_signature",
    "_dmchba_searched_count",
    "_dmchba_assignment_signature",
    "_dmchba_team_agents",
    "_dmchba_valid_task",
    "_dmchba_cost",
    "_dmchba_run_assignment_impl",
}

ORIGINAL_FUNCTIONS = COMMON_FUNCTIONS | {
    "_dmchba_candidate_cells",
    "_hungarian_minimize",
    "_dmchba_order_assigned_cells",
}

OPTIMIZED_FUNCTIONS = COMMON_FUNCTIONS | {
    "_dmchba_pack_cell",
    "_dmchba_unpack_cell",
    "_dmchba_candidate_precedes",
    "_dmchba_candidate_indices",
    "_dmchba_prepare_agent_task_costs",
    "_dmchba_virtual_cost",
    "_hungarian_minimize_virtual",
    "_dmchba_order_assigned_ids",
    "_dmchba_drop_invalid_path_cells",
    "_dmchba_should_reassign",
    "_dmchba_run_assignment",
    "_pick_task_cell_impl",
}


class _TimeStub:
    now = 0

    @staticmethod
    def ticks_us():
        return 0

    @classmethod
    def ticks_ms(cls):
        return cls.now

    @staticmethod
    def ticks_add(value, delta):
        return value + delta

    @staticmethod
    def ticks_diff(left, right):
        return left - right


def _extract_functions(path, function_names):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = [
        node for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name in function_names
    ]
    missing = function_names - {node.name for node in selected}
    if missing:
        raise AssertionError("missing allocator functions in {}: {}".format(path, sorted(missing)))
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    return compile(module, str(path), "exec")


ORIGINAL_CODE = _extract_functions(ORIGINAL, ORIGINAL_FUNCTIONS)
OPTIMIZED_CODE = _extract_functions(OPTIMIZED, OPTIMIZED_FUNCTIONS)


def _binary64_probabilities(grid_size, seed, uniform=False):
    count = grid_size * grid_size
    if uniform:
        values = [1.0] * count
    else:
        rng = random.Random(seed)
        values = [0.05 + rng.random() for _ in range(count)]
    total = sum(values)
    return array("d", [value / total for value in values])


def _snapshot(grid_size, seed, team_size, searched_count, uniform=False):
    rng = random.Random(seed)
    grid = bytearray(grid_size * grid_size)
    pos = [rng.randrange(grid_size), rng.randrange(grid_size)]
    all_cells = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if (x, y) != tuple(pos)
    ]
    rng.shuffle(all_cells)
    for x, y in all_cells[:searched_count]:
        grid[(grid_size - 1 - y) * grid_size + x] = 2
    for x, y in all_cells[searched_count:searched_count + max(1, searched_count // 5)]:
        grid[(grid_size - 1 - y) * grid_size + x] = 1

    peer_pos = {}
    occupied = {tuple(pos)}
    for peer_index in range(1, team_size):
        while True:
            cell = (rng.randrange(grid_size), rng.randrange(grid_size))
            if cell not in occupied:
                occupied.add(cell)
                peer_pos["{:02d}".format(peer_index)] = cell
                break

    probabilities = _binary64_probabilities(grid_size, seed + 1000, uniform)
    return {
        "grid": grid,
        "pos": pos,
        "peer_pos": peer_pos,
        "probabilities": probabilities,
        "clues": [(1, 1)] if grid_size > 2 else [(0, 0)],
    }


def _reference_candidates(namespace):
    grid_size = namespace["GRID_SIZE"]
    cells = [
        (x, y)
        for y in range(grid_size)
        for x in range(grid_size)
        if namespace["grid"][namespace["idx"](x, y)]
        == namespace["CELL_UNSEARCHED"]
        and (x, y) not in namespace["temporary_invalid_task_until"]
    ]
    limit = namespace["TOP_K_MAX_CELLS"]
    if len(cells) > limit:
        cells.sort(
            key=lambda cell: (
                -namespace["target_p"][namespace["idx"](*cell)],
                abs(namespace["pos"][0] - cell[0])
                + abs(namespace["pos"][1] - cell[1]),
                cell,
            )
        )
    return cells[:limit]


def _reference_base_cost(namespace, reference, cell):
    maximum = max(namespace["target_p"])
    normalizer = maximum if maximum > 1.0e-9 else 1.0
    probability = namespace["target_p"][namespace["idx"](*cell)] / normalizer
    probability = max(0.0, min(1.0, probability))
    distance = (
        abs(reference[0] - cell[0])
        + abs(reference[1] - cell[1])
    )
    return distance + 8.0 * (1.0 - probability)


def _reference_hungarian(matrix):
    n = len(matrix)
    if n == 0:
        return []
    u = [0.0] * (n + 1)
    v = [0.0] * (n + 1)
    match = [0] * (n + 1)
    way = [0] * (n + 1)
    for row in range(1, n + 1):
        match[0] = row
        column0 = 0
        minimum = [float("inf")] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[column0] = True
            row0 = match[column0]
            delta = float("inf")
            column1 = 0
            for column in range(1, n + 1):
                if used[column]:
                    continue
                reduced = (
                    matrix[row0 - 1][column - 1]
                    - u[row0]
                    - v[column]
                )
                if reduced < minimum[column]:
                    minimum[column] = reduced
                    way[column] = column0
                if minimum[column] < delta:
                    delta = minimum[column]
                    column1 = column
            for column in range(n + 1):
                if used[column]:
                    u[match[column]] += delta
                    v[column] -= delta
                else:
                    minimum[column] -= delta
            column0 = column1
            if match[column0] == 0:
                break
        while True:
            column1 = way[column0]
            match[column0] = match[column1]
            column0 = column1
            if column0 == 0:
                break
    assignment = [-1] * n
    for column in range(1, n + 1):
        if match[column] > 0:
            assignment[match[column] - 1] = column - 1
    return assignment


def _reference_order(namespace, cells):
    remaining = list(dict.fromkeys(cells))
    ordered = []
    reference = tuple(namespace["pos"])
    eps = 1.0e-9
    while remaining and len(ordered) < 3:
        best = None
        best_score = -float("inf")
        best_distance = 0
        for cell in remaining:
            distance = (
                abs(reference[0] - cell[0])
                + abs(reference[1] - cell[1])
            )
            score = -_reference_base_cost(namespace, reference, cell)
            if best is None or score > best_score + eps:
                best = cell
                best_score = score
                best_distance = distance
            elif abs(score - best_score) <= eps and (
                distance < best_distance
                or (distance == best_distance and cell < best)
            ):
                best = cell
                best_score = score
                best_distance = distance
        ordered.append(best)
        remaining.remove(best)
        reference = best
    return ordered


def _reference_run_assignment(namespace, _reason):
    tasks = _reference_candidates(namespace)
    team = namespace["_dmchba_team_agents"]()
    namespace["dmchba_last_assignment_signature"] = (
        tuple(sorted(namespace["clues"])),
        tuple(tasks),
        tuple(team),
    )
    if not tasks or not team:
        namespace["dmchba_path"] = []
        return
    clones_per_agent = (len(tasks) + len(team) - 1) // len(team)
    clone_rows = [
        (rid, position, clone_index)
        for rid, position in team
        for clone_index in range(clones_per_agent)
    ]
    matrix_n = len(clone_rows)
    columns = tasks + [None] * (matrix_n - len(tasks))
    matrix = []
    for row_index, (_rid, reference, clone_index) in enumerate(clone_rows):
        row = []
        for column_index, cell in enumerate(columns):
            if cell is None:
                cost = 1.0e9 + column_index * 1.0e-9
            else:
                cell_order = cell[1] * namespace["GRID_SIZE"] + cell[0]
                cost = _reference_base_cost(namespace, reference, cell)
                cost += 1.0e-9 * (
                    cell_order
                    + clone_index * 0.001
                    + row_index * 0.000001
                )
            row.append(cost)
        matrix.append(row)
    assignment = _reference_hungarian(matrix)
    assigned = [
        tasks[column]
        for row, column in enumerate(assignment)
        if 0 <= column < len(tasks)
        and clone_rows[row][0] == namespace["ROBOT_ID"]
    ]
    namespace["dmchba_path"] = _reference_order(namespace, assigned)


def _namespace(path, optimized, snapshot, robot_id="00", topk_fraction=0.75):
    grid_size = int(math.isqrt(len(snapshot["grid"])))
    topk = max(1, int(grid_size * grid_size * topk_fraction + 0.5))
    num_robots = 4
    max_matrix_n = topk + num_robots - 1

    def safe_assert(condition, message):
        if not condition:
            raise AssertionError(message)

    ns = {
        "array": array,
        "time": _TimeStub,
        "safe_assert": safe_assert,
        "record_candidate_filter_time": lambda _started: None,
        "record_allocator_solve_time": lambda _started, _filter_before: None,
        "candidate_filter_time_us_total": 0,
        "GRID_SIZE": grid_size,
        "TOP_K_MAX_CELLS": topk,
        "NUM_ROBOTS": num_robots,
        "ROBOT_ID": robot_id,
        "CELL_UNSEARCHED": 0,
        "CELL_OBSTACLE": 1,
        "CELL_SEARCHED": 2,
        "REWARD_FACTOR": 5,
        "allocation_probability_normalizer": max(snapshot["probabilities"]),
        "temporary_invalid_task_until": {},
        "DMCHBA_COMMITMENT_HORIZON": 3,
        "DMCHBA_PSEUDOTASK_COST": 1.0e9,
        "DMCHBA_COST_SCALE": 100000,
        "DMCHBA_TIE_EPS": 1.0e-9,
        "grid": bytearray(snapshot["grid"]),
        "pos": list(snapshot["pos"]),
        "peer_pos": dict(snapshot["peer_pos"]),
        "clues": list(snapshot["clues"]),
        "target_p": array("d", snapshot["probabilities"]),
        "prob_map": array("d", snapshot["probabilities"]),
        "dmchba_path": [],
        "dmchba_last_assignment_signature": None,
        "dmchba_clue_signature": None,
        "current_task_cell": None,
        "pending_collision_reallocation": False,
    }

    if optimized:
        ns.update({
            "DMCHBA_MAX_MATRIX_N": max_matrix_n,
            "DMCHBA_HUNGARIAN_INF": float("inf"),
            "dmchba_candidate_ids": array("H", [0] * topk),
            "dmchba_agent_task_costs": [
                array("d", [0.0] * topk) for _ in range(num_robots)
            ],
            "dmchba_h_u": array("d", [0.0] * (max_matrix_n + 1)),
            "dmchba_h_v": array("d", [0.0] * (max_matrix_n + 1)),
            "dmchba_h_minv": array("d", [0.0] * (max_matrix_n + 1)),
            "dmchba_h_p": array("H", [0] * (max_matrix_n + 1)),
            "dmchba_h_way": array("H", [0] * (max_matrix_n + 1)),
            "dmchba_h_used": bytearray(max_matrix_n + 1),
            "dmchba_h_assignment": array("h", [-1] * max_matrix_n),
            "dmchba_assigned_ids": array("H", [0] * topk),
        })
        exec(OPTIMIZED_CODE, ns)
    else:
        exec(ORIGINAL_CODE, ns)
        ns["_dmchba_candidate_cells"] = lambda: _reference_candidates(ns)
        ns["_dmchba_cost"] = (
            lambda reference, cell, _clone_index=0:
            _reference_base_cost(ns, reference, cell)
        )
        ns["_dmchba_run_assignment_impl"] = (
            lambda reason: _reference_run_assignment(ns, reason)
        )
    return ns


def _optimized_candidates(ns):
    count = ns["_dmchba_candidate_indices"]()
    unpack = ns["_dmchba_unpack_cell"]
    return [unpack(ns["dmchba_candidate_ids"][i]) for i in range(count)]


def _run_pair(snapshot, robot_id="00", topk_fraction=0.75):
    original = _namespace(ORIGINAL, False, snapshot, robot_id, topk_fraction)
    optimized = _namespace(OPTIMIZED, True, snapshot, robot_id, topk_fraction)

    original_candidates = original["_dmchba_candidate_cells"]()
    optimized_candidates = _optimized_candidates(optimized)

    original["_dmchba_run_assignment_impl"]("parity_test")
    optimized["_dmchba_run_assignment_impl"]("parity_test")
    return original_candidates, optimized_candidates, original["dmchba_path"], optimized["dmchba_path"]


def _make_dmchba_robot(top_k_limit):
    robot_ids = ["00", "01", "02", "03"]
    starts = {
        "00": (0, 0),
        "01": (0, 6),
        "02": (0, 12),
        "03": (0, 18),
    }
    cfg = SimConfig(
        grid_size=19,
        robot_ids=robot_ids,
        start_positions=starts,
        start_headings={rid: EAST for rid in robot_ids},
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
        DMCHBAAllocator,
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
        rid: cell for rid, cell in starts.items() if rid != robot.rid
    }
    for cell in ((2, 2), (3, 7), (12, 4), (17, 16)):
        robot.belief.mark_searched(cell)
    return robot


def _snapshot_from_simulator_robot(robot):
    grid_size = robot.cfg.grid_size
    index = lambda x, y: (grid_size - 1 - y) * grid_size + x
    grid = bytearray(grid_size * grid_size)
    for cell in robot.belief.searched:
        grid[index(*cell)] = 2
    probabilities = array(
        "d",
        [
            robot.belief.target_p[
                (offset % grid_size, grid_size - 1 - offset // grid_size)
            ]
            for offset in range(grid_size * grid_size)
        ],
    )
    return {
        "grid": grid,
        "pos": list(robot.pos),
        "peer_pos": dict(robot._peer_positions),
        "probabilities": probabilities,
        "clues": list(robot.belief.known_clues),
    }


def _sync_hardware_from_simulator(robot, hardware):
    grid_size = robot.cfg.grid_size
    for y in range(grid_size):
        for x in range(grid_size):
            cell = (x, y)
            cell_index = hardware["idx"](x, y)
            hardware["grid"][cell_index] = (
                hardware["CELL_SEARCHED"]
                if cell in robot.belief.searched
                else hardware["CELL_UNSEARCHED"]
            )
            probability = robot.belief.target_p[cell]
            hardware["target_p"][cell_index] = probability
            hardware["prob_map"][cell_index] = probability
    hardware["allocation_probability_normalizer"] = max(
        robot.belief.target_p.values()
    )
    hardware["clues"][:] = list(robot.belief.known_clues)
    hardware["pos"][:] = list(robot.pos)
    hardware["peer_pos"].clear()
    hardware["peer_pos"].update(robot._peer_positions)


class DMCHBAMemoryOptimizedParityTests(unittest.TestCase):
    def test_all_study_topk_match_simulator_across_signature_transitions(self):
        """Replay miss, clue, peer-position, and path-exhaustion transitions."""

        for top_k_limit in STUDY_TOP_K_LIMITS:
            with self.subTest(top_k_limit=top_k_limit):
                robot = _make_dmchba_robot(top_k_limit)
                snapshot = _snapshot_from_simulator_robot(robot)
                hardware = _namespace(
                    OPTIMIZED,
                    True,
                    snapshot,
                    "03",
                    top_k_limit / float(19 * 19),
                )

                simulator_candidates = (
                    robot.allocator._candidate_cells(robot)
                )
                hardware_candidates = _optimized_candidates(hardware)
                self.assertEqual(
                    simulator_candidates, hardware_candidates
                )
                simulator_goal = robot.allocator.choose_goal(robot).goal
                hardware_goal = hardware["_pick_task_cell_impl"]()
                self.assertEqual(simulator_goal, hardware_goal)
                self.assertEqual(
                    robot.dmchba_path, hardware["dmchba_path"]
                )

                # A non-path miss changes the exact candidate signature but
                # must not replace a nonempty committed path.
                miss = next(
                    cell
                    for cell in simulator_candidates
                    if cell not in robot.dmchba_path
                )
                robot.belief.mark_searched(miss)
                _sync_hardware_from_simulator(robot, hardware)
                retained = list(robot.dmchba_path)
                self.assertEqual(
                    robot.allocator.choose_goal(robot).goal,
                    hardware["_pick_task_cell_impl"](),
                )
                self.assertEqual(robot.dmchba_path, retained)
                self.assertEqual(hardware["dmchba_path"], retained)

                # A later clue rebuilds belief but preserves the commitment.
                clue = next(
                    cell
                    for cell in robot.belief.all_cells()
                    if cell not in robot.belief.searched
                    and cell not in robot.dmchba_path
                )
                robot.belief.add_clue(clue)
                _sync_hardware_from_simulator(robot, hardware)
                self.assertEqual(
                    robot.allocator.choose_goal(robot).goal,
                    hardware["_pick_task_cell_impl"](),
                )
                self.assertEqual(robot.dmchba_path, retained)
                self.assertEqual(hardware["dmchba_path"], retained)

                # A peer-position change also belongs to the exact signature,
                # but takes effect only after the retained path is exhausted.
                robot._peer_positions["01"] = (1, 6)
                _sync_hardware_from_simulator(robot, hardware)
                self.assertEqual(
                    robot.allocator.choose_goal(robot).goal,
                    hardware["_pick_task_cell_impl"](),
                )
                self.assertEqual(robot.dmchba_path, retained)
                self.assertEqual(hardware["dmchba_path"], retained)

                for cell in retained:
                    robot.belief.mark_searched(cell)
                _sync_hardware_from_simulator(robot, hardware)
                simulator_goal = robot.allocator.choose_goal(robot).goal
                hardware_goal = hardware["_pick_task_cell_impl"]()
                self.assertEqual(simulator_goal, hardware_goal)
                self.assertEqual(
                    robot.dmchba_path, hardware["dmchba_path"]
                )
                self.assertEqual(
                    robot.allocator._candidate_cells(robot),
                    _optimized_candidates(hardware),
                )

    def test_randomized_small_grid_parity(self):
        fractions = (0.20, 0.35, 0.50, 0.75, 1.0)
        for seed in range(20):
            grid_size = 5 + seed % 5
            team_size = 1 + seed % 4
            searched_count = seed % max(1, grid_size)
            fraction = fractions[seed % len(fractions)]
            robot_id = "{:02d}".format((team_size - 1) if seed % 2 else 0)
            snapshot = _snapshot(
                grid_size,
                100 + seed,
                team_size,
                searched_count,
                uniform=(seed % 4 == 0),
            )
            with self.subTest(
                seed=seed,
                grid_size=grid_size,
                team_size=team_size,
                fraction=fraction,
                robot_id=robot_id,
            ):
                original_candidates, optimized_candidates, original_path, optimized_path = (
                    _run_pair(snapshot, robot_id, fraction)
                )
                self.assertEqual(original_candidates, optimized_candidates)
                self.assertEqual(original_path, optimized_path)

    def test_candidates_and_committed_paths_match_across_snapshots(self):
        cases = [
            # Random objective, single locally known robot.
            (_snapshot(7, 11, 1, 6), "00", 0.75),
            # Four-robot assignment with searched cells and obstacles.
            (_snapshot(8, 22, 4, 9), "00", 0.75),
            (_snapshot(8, 23, 4, 11), "03", 0.50),
            # Uniform probabilities deliberately exercise deterministic ties.
            (_snapshot(7, 33, 4, 5, uniform=True), "02", 0.75),
        ]
        for snapshot, robot_id, fraction in cases:
            with self.subTest(
                grid_size=int(math.isqrt(len(snapshot["grid"]))),
                robot_id=robot_id,
                fraction=fraction,
            ):
                original_candidates, optimized_candidates, original_path, optimized_path = (
                    _run_pair(snapshot, robot_id, fraction)
                )
                self.assertEqual(original_candidates, optimized_candidates)
                self.assertEqual(original_path, optimized_path)

    def test_production_size_19x19_topk_75_percent_matches(self):
        snapshot = _snapshot(19, 44, 4, 37)
        original_candidates, optimized_candidates, original_path, optimized_path = (
            _run_pair(snapshot, "03", 0.75)
        )
        self.assertEqual(len(original_candidates), 271)
        self.assertEqual(original_candidates, optimized_candidates)
        self.assertEqual(original_path, optimized_path)

    def test_production_size_19x19_full_topk_completes_in_linear_workspace(self):
        snapshot = _snapshot(19, 91, 4, 0)
        optimized = _namespace(OPTIMIZED, True, snapshot, "03", 1.0)

        optimized["_dmchba_run_assignment_impl"]("k361_memory_probe")

        self.assertEqual(optimized["TOP_K_MAX_CELLS"], 361)
        self.assertEqual(len(optimized["dmchba_path"]), 3)
        self.assertEqual(
            optimized["dmchba_path"],
            [(1, 18), (0, 17), (1, 17)],
        )

        typed_arrays = [
            optimized["dmchba_candidate_ids"],
            *optimized["dmchba_agent_task_costs"],
            optimized["dmchba_h_u"],
            optimized["dmchba_h_v"],
            optimized["dmchba_h_minv"],
            optimized["dmchba_h_p"],
            optimized["dmchba_h_way"],
            optimized["dmchba_h_assignment"],
            optimized["dmchba_assigned_ids"],
        ]
        workspace_payload_bytes = sum(
            len(values) * values.itemsize for values in typed_arrays
        )
        workspace_payload_bytes += len(optimized["dmchba_h_used"])
        self.assertEqual(workspace_payload_bytes, 24309)
        self.assertLess(workspace_payload_bytes, 24 * 1024)

    def test_virtual_matrix_matches_every_dense_cost_on_small_case(self):
        snapshot = _snapshot(6, 55, 4, 4, uniform=True)
        original = _namespace(ORIGINAL, False, snapshot)
        optimized = _namespace(OPTIMIZED, True, snapshot)

        tasks = original["_dmchba_candidate_cells"]()
        task_count = optimized["_dmchba_candidate_indices"]()
        self.assertEqual(tasks, _optimized_candidates(optimized))
        team = original["_dmchba_team_agents"]()
        clones_per_agent = (len(tasks) + len(team) - 1) // len(team)
        matrix_n = clones_per_agent * len(team)
        columns = tasks + [None] * (matrix_n - len(tasks))
        clone_rows = [
            (rid, ref, clone_index)
            for rid, ref in team
            for clone_index in range(clones_per_agent)
        ]

        optimized["_dmchba_prepare_agent_task_costs"](team, task_count)
        virtual_cost = optimized["_dmchba_virtual_cost"]
        for row_index, (_rid, ref, clone_index) in enumerate(clone_rows):
            for col_index, cell in enumerate(columns):
                if cell is None:
                    expected = (
                        original["DMCHBA_PSEUDOTASK_COST"]
                        + col_index * original["DMCHBA_TIE_EPS"]
                    )
                else:
                    cell_order = cell[1] * original["GRID_SIZE"] + cell[0]
                    expected = original["_dmchba_cost"](ref, cell)
                    expected += original["DMCHBA_TIE_EPS"] * (
                        cell_order
                        + clone_index * 0.001
                        + row_index * 0.000001
                    )
                self.assertEqual(
                    expected,
                    virtual_cost(row_index, col_index, clones_per_agent, task_count),
                )

    def test_workspace_growth_is_linear_not_quadratic(self):
        topk = 271
        agents = 4
        matrix_n = math.ceil(topk / agents) * agents
        dense_entries = matrix_n * matrix_n
        optimized_numeric_slots = (
            agents * topk
            + topk
            + topk
            + 3 * (matrix_n + 1)
            + 2 * (matrix_n + 1)
            + (matrix_n + 1)
            + matrix_n
        )
        self.assertEqual(matrix_n, 272)
        self.assertEqual(dense_entries, 73984)
        self.assertLess(optimized_numeric_slots, dense_entries // 10)

        snapshot = _snapshot(19, 77, 4, 0)
        optimized = _namespace(OPTIMIZED, True, snapshot, "03", 0.75)
        typed_arrays = [
            optimized["dmchba_candidate_ids"],
            *optimized["dmchba_agent_task_costs"],
            optimized["dmchba_h_u"],
            optimized["dmchba_h_v"],
            optimized["dmchba_h_minv"],
            optimized["dmchba_h_p"],
            optimized["dmchba_h_way"],
            optimized["dmchba_h_assignment"],
            optimized["dmchba_assigned_ids"],
        ]
        workspace_payload_bytes = sum(len(values) * values.itemsize for values in typed_arrays)
        workspace_payload_bytes += len(optimized["dmchba_h_used"])
        dense_binary64_payload_bytes = dense_entries * 8
        self.assertLess(workspace_payload_bytes, 24 * 1024)
        self.assertGreater(
            dense_binary64_payload_bytes, workspace_payload_bytes * 20
        )


if __name__ == "__main__":
    gc.collect()
    unittest.main(verbosity=2)
