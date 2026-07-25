from __future__ import annotations

import gc
import statistics
import sys
import time
import tracemalloc
from collections.abc import Mapping
from typing import Any, Callable

from benchmark_sim.algorithms.memory_optimized import CellIndexedMap
from benchmark_sim.tests.test_allocator_memory_optimized_equivalence import (
    PAIRS,
    STATE_ATTRIBUTES,
    _prepare_pair,
)


def _measure(call: Callable[[], Any], repeats: int) -> tuple[float, int]:
    times_ns = []
    peaks = []
    for _ in range(repeats):
        gc.collect()
        tracemalloc.start()
        tracemalloc.reset_peak()
        started_ns = time.perf_counter_ns()
        call()
        times_ns.append(time.perf_counter_ns() - started_ns)
        peaks.append(tracemalloc.get_traced_memory()[1])
        tracemalloc.stop()
    return statistics.median(times_ns) / 1_000_000.0, int(
        statistics.median(peaks)
    )


def _deep_size(value: Any, seen: set[int] | None = None) -> int:
    if seen is None:
        seen = set()
    identity = id(value)
    if identity in seen:
        return 0
    seen.add(identity)
    size = sys.getsizeof(value)

    if isinstance(value, CellIndexedMap):
        return (
            size
            + _deep_size(value._active, seen)
            + _deep_size(value._values, seen)
        )
    if isinstance(value, Mapping):
        return size + sum(
            _deep_size(key, seen) + _deep_size(item, seen)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple, set, frozenset)):
        return size + sum(_deep_size(item, seen) for item in value)
    return size


def run(repeats: int = 7) -> list[dict[str, Any]]:
    rows = []
    for algorithm, reference_cls, optimized_cls in PAIRS:
        reference, optimized = _prepare_pair(
            reference_cls,
            optimized_cls,
            grid_size=19,
            team_size=4,
            max_candidates=271,
            seed=111,
            robot_id="03",
            uniform=False,
        )

        # Allocate optimized reusable workspaces before transient measurement.
        optimized.allocator._candidate_cells(optimized)
        reference_ms, reference_peak = _measure(
            lambda: reference.allocator._candidate_cells(reference),
            repeats,
        )
        optimized_ms, optimized_peak = _measure(
            lambda: optimized.allocator._candidate_cells(optimized),
            repeats,
        )

        reference_times = []
        optimized_times = []
        reference_peaks = []
        optimized_peaks = []
        last_reference = None
        last_optimized = None
        for sample in range(max(3, repeats // 2)):
            last_reference, last_optimized = _prepare_pair(
                reference_cls,
                optimized_cls,
                grid_size=19,
                team_size=4,
                max_candidates=271,
                seed=200 + sample,
                robot_id="03",
                uniform=False,
            )
            measured_ms, measured_peak = _measure(
                lambda: last_reference.allocator.choose_goal(last_reference),
                1,
            )
            reference_times.append(measured_ms)
            reference_peaks.append(measured_peak)
            measured_ms, measured_peak = _measure(
                lambda: last_optimized.allocator.choose_goal(last_optimized),
                1,
            )
            optimized_times.append(measured_ms)
            optimized_peaks.append(measured_peak)

        reference_state_size = _deep_size([
            getattr(last_reference, attribute)
            for attribute in STATE_ATTRIBUTES[algorithm]
        ])
        optimized_state_size = _deep_size([
            getattr(last_optimized, attribute)
            for attribute in STATE_ATTRIBUTES[algorithm]
        ])
        rows.append({
            "algorithm": algorithm,
            "candidate_reference_ms": round(reference_ms, 3),
            "candidate_optimized_ms": round(optimized_ms, 3),
            "candidate_reference_peak_bytes": reference_peak,
            "candidate_optimized_peak_bytes": optimized_peak,
            "allocation_reference_ms": round(
                statistics.median(reference_times),
                3,
            ),
            "allocation_optimized_ms": round(
                statistics.median(optimized_times),
                3,
            ),
            "allocation_reference_peak_bytes": int(
                statistics.median(reference_peaks)
            ),
            "allocation_optimized_peak_bytes": int(
                statistics.median(optimized_peaks)
            ),
            "reference_state_bytes_after_one_allocation": reference_state_size,
            "optimized_state_bytes_after_one_allocation": optimized_state_size,
            "optimized_candidate_workspace_payload_bytes": (
                last_optimized.allocator.candidate_workspace_payload_bytes()
            ),
            "optimized_state_payload_bytes": (
                last_optimized.allocator.optimized_state_payload_bytes(
                    last_optimized
                )
            ),
        })
    return rows


def _print_table(rows: list[dict[str, Any]]) -> None:
    print(
        "algorithm | candidate ms old/new | candidate peak old/new | "
        "allocation ms old/new | allocation peak old/new | retained state old/new"
    )
    for row in rows:
        print(
            "{algorithm} | {candidate_reference_ms}/{candidate_optimized_ms} | "
            "{candidate_reference_peak_bytes}/{candidate_optimized_peak_bytes} | "
            "{allocation_reference_ms}/{allocation_optimized_ms} | "
            "{allocation_reference_peak_bytes}/{allocation_optimized_peak_bytes} | "
            "{reference_state_bytes_after_one_allocation}/"
            "{optimized_state_bytes_after_one_allocation}".format(**row)
        )


if __name__ == "__main__":
    _print_table(run())
