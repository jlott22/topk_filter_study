"""Compare reference and optimized DGA allocator time and peak allocation.

Run from ``simulator``:

    python benchmark_sim/tests/dga_optimization/benchmark_dga_optimized.py

CPython allocation measurements are useful for relative validation; they are
not RP2040 heap measurements.
"""

from __future__ import annotations

import argparse
import gc
import statistics
import sys
import time
import tracemalloc
from pathlib import Path


SIMULATOR_ROOT = Path(__file__).resolve().parents[3]
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from benchmark_sim.algorithms.DGA import (
    DGAAllocator,
    DGAReferenceAllocator,
)
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario


ROBOT_IDS = ["00", "01", "02", "03"]
POSITIONS = {
    "00": (0, 0),
    "01": (1, 0),
    "02": (2, 0),
    "03": (3, 0),
}


def _robot(allocator_cls, topk):
    cfg = SimConfig(
        grid_size=19,
        robot_ids=ROBOT_IDS,
        start_positions=POSITIONS,
        start_headings={rid: EAST for rid in ROBOT_IDS},
        max_candidate_cells=topk,
        async_initial_spread_s=0.0,
        async_step_jitter_s=0.0,
        comm_delay_s=0.0,
        comm_delay_jitter_s=0.0,
        collision_intent_settle_s=0.0,
        write_parquet=False,
    )
    scenario = TrialScenario(
        trial_id=0,
        target=(18, 18),
        clues=[(9, 9)],
    )
    state = AsyncTrialRunner(
        cfg,
        allocator_cls,
        make_comm_model("ideal", None),
        seed=0,
    ).new_trial(scenario)
    for rid, robot in state.robots.items():
        robot._peer_positions = {
            peer_id: position
            for peer_id, position in POSITIONS.items()
            if peer_id != rid
        }
        robot.belief.add_clue((9, 9))
    return state.robots["03"]


def _result_signature(robot, decision):
    return (
        decision.goal,
        robot.dga_best_plan,
        robot.dga_best_fitness,
        robot.dga_last_candidate_count,
    )


def _median_runtime(allocator_cls, topk, repeats):
    samples = []
    signature = None
    for _ in range(repeats):
        robot = _robot(allocator_cls, topk)
        gc.collect()
        started = time.perf_counter()
        decision = robot.allocator.choose_goal(robot)
        samples.append(time.perf_counter() - started)
        signature = _result_signature(robot, decision)
    return statistics.median(samples), signature


def _peak_allocation(allocator_cls, topk):
    robot = _robot(allocator_cls, topk)
    gc.collect()
    tracemalloc.start()
    tracemalloc.reset_peak()
    decision = robot.allocator.choose_goal(robot)
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return current, peak, _result_signature(robot, decision)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topk",
        default="18,36,48,72,90,181,271,361",
        help="comma-separated absolute candidate limits",
    )
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    topk_values = [int(value) for value in args.topk.split(",")]

    print(
        "topk,candidates,reference_median_s,optimized_median_s,"
        "speedup,reference_peak_kib,optimized_peak_kib,"
        "peak_reduction,outputs_equal"
    )
    for topk in topk_values:
        reference_time, reference_signature = _median_runtime(
            DGAReferenceAllocator, topk, args.repeats
        )
        optimized_time, optimized_signature = _median_runtime(
            DGAAllocator, topk, args.repeats
        )
        _, reference_peak, reference_memory_signature = _peak_allocation(
            DGAReferenceAllocator, topk
        )
        _, optimized_peak, optimized_memory_signature = _peak_allocation(
            DGAAllocator, topk
        )
        outputs_equal = (
            reference_signature
            == optimized_signature
            == reference_memory_signature
            == optimized_memory_signature
        )
        print(
            "{},{},{:.6f},{:.6f},{:.2f},{:.1f},{:.1f},{:.2f},{}".format(
                topk,
                reference_signature[3],
                reference_time,
                optimized_time,
                reference_time / optimized_time,
                reference_peak / 1024.0,
                optimized_peak / 1024.0,
                reference_peak / optimized_peak,
                outputs_equal,
            )
        )


if __name__ == "__main__":
    main()
