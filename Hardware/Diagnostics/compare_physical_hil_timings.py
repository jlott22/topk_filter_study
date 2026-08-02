"""Compare validated physical timing metrics with matched-condition HIL data.

Physical and HIL scenarios do not overlap, so this is an unpaired
distributional comparison at the system-trial level. Each observation is the
call-weighted mean across four robot rows. Exact permutation tests avoid
parametric assumptions with only five physical trials per algorithm.
"""

from __future__ import annotations

import csv
import hashlib
import itertools
import json
import math
import platform
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, stdev

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
PHYSICAL_ROOT = ROOT / "Results" / "Hardware" / "PhysicalTrials"
PHYSICAL_COMBINED = PHYSICAL_ROOT / "CombinedTrials" / "combined_metrics_per_trial.csv"
PHYSICAL_ROBOTS = PHYSICAL_ROOT / "CombinedTrials" / "matched_robot_rows.csv"
HIL_ROOT = ROOT / "Results" / "HIL" / "Bayesian" / "CompletedTopKCampaignV8"
HIL_COMBINED = HIL_ROOT / "bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv"
HIL_ROBOTS = HIL_ROOT / "bayesian_hil_topk_non_k1_robot_trial_timing_and_allocator_metrics.csv"
SCENARIO_FILE = ROOT / "Simulation" / "Architecture" / "simulator" / "scenarios" / "final_trial_500.csv"
OUTPUT_ROOT = PHYSICAL_ROOT / "HILComparison"
ALGORITHMS = ("ACBBA", "CBAA", "DMCHBA", "HIPC", "PI")
METRICS = (
    (
        "allocation_exclusive_mean_us",
        "Allocation exclusive",
        "allocator_solve_time_us_total",
        "allocator_exclusive_time_total_us",
        "allocator_exclusive_time_mean_us",
        "allocator_calls",
        "timed_allocator_call_count",
    ),
    (
        "candidate_filter_mean_us",
        "Candidate filter",
        "candidate_filter_time_us_total",
        "candidate_filter_time_total_us",
        "candidate_filter_time_mean_us",
        "candidate_filter_calls",
        "candidate_filter_invocation_count",
    ),
    (
        "combined_allocator_mean_us",
        "Combined allocator",
        "allocator_time_us_total",
        "allocator_time_total_us",
        "allocator_time_mean_us",
        "allocator_calls",
        "timed_allocator_call_count",
    ),
)
SEED = 20260802
BOOTSTRAP_RESAMPLES = 50_000
STRATIFIED_PERMUTATION_RESAMPLES = 200_000
SCENARIO_FIELDS = (
    "object_x",
    "object_y",
    "clue1_x",
    "clue1_y",
    "clue2_x",
    "clue2_y",
    "clue3_x",
    "clue3_y",
    "clue4_x",
    "clue4_y",
)


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError(f"missing header: {path}")
        return list(reader.fieldnames), list(reader)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def read_scenarios(path: Path) -> dict[str, dict[str, str]]:
    lines = [
        line
        for line in path.read_text(encoding="utf-8-sig").splitlines()
        if line and not line.startswith("#")
    ]
    reader = csv.DictReader(lines)
    return {row["episode"]: row for row in reader}


def scenario_signature(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(row[field] for field in SCENARIO_FIELDS)


def clue_signature(row: dict[str, str]) -> str:
    return ";".join(
        f"{row[f'clue{index}_x']}/{row[f'clue{index}_y']}"
        for index in range(1, 5)
    )


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def as_float(row: dict[str, str], field: str) -> float:
    value = row[field].strip()
    return float(value) if value else 0.0


def as_int(row: dict[str, str], field: str) -> int:
    return int(round(as_float(row, field)))


def truthy(value: str) -> bool:
    return value.strip().lower() in {"true", "1", "yes"}


def average_ranks(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values), dtype=float)
    start = 0
    while start < len(values):
        end = start + 1
        while end < len(values) and values[order[end]] == values[order[start]]:
            end += 1
        ranks[order[start:end]] = (start + 1 + end) / 2.0
        start = end
    return ranks


def exact_permutation_p(
    x: np.ndarray,
    y: np.ndarray,
    combinations: np.ndarray,
    transform_log: bool = False,
) -> float:
    pooled = np.concatenate((x, y))
    if transform_log:
        if np.any(pooled <= 0):
            raise RuntimeError("log permutation test received a nonpositive value")
        pooled = np.log(pooled)
    n_x = len(x)
    n_y = len(y)
    observed = abs(float(np.mean(pooled[:n_x]) - np.mean(pooled[n_x:])))
    selected_sums = pooled[combinations].sum(axis=1)
    differences = selected_sums / n_x - (pooled.sum() - selected_sums) / n_y
    return float(np.mean(np.abs(differences) >= observed - 1e-12))


def exact_rank_permutation_p(
    x: np.ndarray,
    y: np.ndarray,
    combinations: np.ndarray,
) -> tuple[float, float]:
    pooled = np.concatenate((x, y))
    ranks = average_ranks(pooled)
    n_x = len(x)
    n_y = len(y)
    baseline = n_x * (n_x + 1) / 2.0
    expected_u = n_x * n_y / 2.0
    observed_u = ranks[:n_x].sum() - baseline
    all_u = ranks[combinations].sum(axis=1) - baseline
    p_value = float(np.mean(np.abs(all_u - expected_u) >= abs(observed_u - expected_u) - 1e-12))
    return float(observed_u), p_value


def bootstrap_intervals(
    x: np.ndarray,
    y: np.ndarray,
    seed: int,
) -> dict[str, float]:
    rng = np.random.default_rng(seed)
    x_samples = x[rng.integers(0, len(x), size=(BOOTSTRAP_RESAMPLES, len(x)))]
    y_samples = y[rng.integers(0, len(y), size=(BOOTSTRAP_RESAMPLES, len(y)))]
    x_means = x_samples.mean(axis=1)
    y_means = y_samples.mean(axis=1)
    differences = x_means - y_means
    ratios = x_means / y_means
    return {
        "mean_difference_ci95_low_us": float(np.percentile(differences, 2.5)),
        "mean_difference_ci95_high_us": float(np.percentile(differences, 97.5)),
        "mean_ratio_ci95_low": float(np.percentile(ratios, 2.5)),
        "mean_ratio_ci95_high": float(np.percentile(ratios, 97.5)),
    }


def cliffs_delta(x: np.ndarray, y: np.ndarray) -> float:
    comparisons = np.sign(x[:, None] - y[None, :])
    return float(comparisons.mean())


def benjamini_hochberg(p_values: list[float]) -> list[float]:
    count = len(p_values)
    order = sorted(range(count), key=lambda index: p_values[index])
    adjusted = [1.0] * count
    running = 1.0
    for rank_index in range(count - 1, -1, -1):
        original_index = order[rank_index]
        rank = rank_index + 1
        candidate = p_values[original_index] * count / rank
        running = min(running, candidate)
        adjusted[original_index] = min(1.0, running)
    return adjusted


def stratified_overall_log_test(
    values_by_algorithm: dict[str, tuple[np.ndarray, np.ndarray]],
    combinations: np.ndarray,
    seed: int,
) -> dict[str, float]:
    """Test a common environment shift while preserving algorithm strata."""
    rng = np.random.default_rng(seed)
    observed_parts: list[float] = []
    permuted_parts = np.zeros(STRATIFIED_PERMUTATION_RESAMPLES, dtype=float)
    bootstrap_parts = np.zeros(BOOTSTRAP_RESAMPLES, dtype=float)
    for algorithm in ALGORITHMS:
        x, y = values_by_algorithm[algorithm]
        log_x = np.log(x)
        log_y = np.log(y)
        pooled = np.concatenate((log_x, log_y))
        observed_parts.append(float(log_x.mean() - log_y.mean()))
        selected = combinations[
            rng.integers(0, len(combinations), size=STRATIFIED_PERMUTATION_RESAMPLES)
        ]
        selected_sums = pooled[selected].sum(axis=1)
        permuted_parts += selected_sums / len(x) - (pooled.sum() - selected_sums) / len(y)
        x_boot = log_x[rng.integers(0, len(x), size=(BOOTSTRAP_RESAMPLES, len(x)))]
        y_boot = log_y[rng.integers(0, len(y), size=(BOOTSTRAP_RESAMPLES, len(y)))]
        bootstrap_parts += x_boot.mean(axis=1) - y_boot.mean(axis=1)
    observed = mean(observed_parts)
    permuted_parts /= len(ALGORITHMS)
    bootstrap_parts /= len(ALGORITHMS)
    exceedances = int(np.sum(np.abs(permuted_parts) >= abs(observed) - 1e-12))
    p_value = (exceedances + 1) / (STRATIFIED_PERMUTATION_RESAMPLES + 1)
    return {
        "geometric_mean_ratio_physical_over_hil": math.exp(observed),
        "geometric_mean_ratio_ci95_low": math.exp(float(np.percentile(bootstrap_parts, 2.5))),
        "geometric_mean_ratio_ci95_high": math.exp(float(np.percentile(bootstrap_parts, 97.5))),
        "stratified_log_mean_permutation_p": p_value,
        "permutation_monte_carlo_se": math.sqrt(p_value * (1.0 - p_value) / (STRATIFIED_PERMUTATION_RESAMPLES + 1)),
    }


def validate_hil_reconstruction(
    combined_rows: list[dict[str, str]],
    robot_rows: list[dict[str, str]],
) -> None:
    by_key: dict[tuple[str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in robot_rows:
        by_key[(row["condition_id"], row["trial_id"], row["run_generation"])].append(row)
    for combined in combined_rows:
        key = (combined["condition_id"], combined["trial_id"], combined["run_generation"])
        rows = by_key[key]
        if len(rows) != 4 or sorted(row["robot_id"] for row in rows) != ["00", "01", "02", "03"]:
            raise RuntimeError(f"HIL trial does not reconstruct to four robots: {key}")
        checks = (
            ("allocator_call_count", "allocator_call_count"),
            ("timed_allocator_call_count", "timed_allocator_call_count"),
            ("candidate_filter_invocation_count", "candidate_filter_invocation_count"),
            ("allocator_time_total_us", "allocator_time_total_us"),
            ("candidate_filter_time_total_us", "candidate_filter_time_total_us"),
            ("allocator_exclusive_time_total_us", "allocator_exclusive_time_total_us"),
        )
        for combined_field, robot_field in checks:
            expected = sum(as_float(row, robot_field) for row in rows)
            actual = as_float(combined, combined_field)
            if not math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-6):
                raise RuntimeError(f"HIL reconstruction mismatch {key} {combined_field}: {actual} != {expected}")


def validate_physical_reconstruction(
    combined_rows: list[dict[str, str]],
    robot_rows: list[dict[str, str]],
) -> None:
    by_label: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in robot_rows:
        by_label[row["trial_label"]].append(row)
    for combined in combined_rows:
        label = combined["trial_label"]
        rows = by_label[label]
        if len(rows) != 4 or sorted(row["robot_id"] for row in rows) != ["00", "01", "02", "03"]:
            raise RuntimeError(f"physical trial does not reconstruct to four robots: {label}")
        checks = (
            ("total_allocator_calls", "allocator_calls"),
            ("total_candidate_filter_calls", "candidate_filter_calls"),
            ("total_allocator_time_us", "allocator_time_us_total"),
            ("total_filter_time_us", "candidate_filter_time_us_total"),
            ("total_allocator_solve_time_us", "allocator_solve_time_us_total"),
        )
        for combined_field, robot_field in checks:
            expected = sum(as_float(row, robot_field) for row in rows)
            actual = as_float(combined, combined_field)
            if not math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-6):
                raise RuntimeError(f"physical reconstruction mismatch {label} {combined_field}: {actual} != {expected}")


def main() -> int:
    _, physical = read_csv(PHYSICAL_COMBINED)
    _, physical_robot_rows = read_csv(PHYSICAL_ROBOTS)
    _, hil_all = read_csv(HIL_COMBINED)
    _, hil_robot_all = read_csv(HIL_ROBOTS)
    scenarios = read_scenarios(SCENARIO_FILE)
    if len(physical) != 25:
        raise RuntimeError(f"expected 25 validated physical trials, found {len(physical)}")
    validate_physical_reconstruction(physical, physical_robot_rows)

    physical_condition: dict[str, tuple[str, str]] = {}
    for algorithm in ALGORITHMS:
        rows = [row for row in physical if row["algorithm"] == algorithm]
        conditions = {(row["top_k_rate"], row["top_k_max_cells"]) for row in rows}
        if len(rows) != 5 or len(conditions) != 1:
            raise RuntimeError(f"unexpected physical coverage for {algorithm}: {len(rows)} rows, {conditions}")
        physical_condition[algorithm] = next(iter(conditions))

    hil_selected = [
        row
        for row in hil_all
        if row["algorithm"] in ALGORITHMS
        and truthy(row["analysis_valid"])
        and row["trial_status"] == "completed"
        and (row["top_k_rate"], row["top_k_cells"]) == physical_condition[row["algorithm"]]
    ]
    selected_keys = {
        (row["condition_id"], row["trial_id"], row["run_generation"])
        for row in hil_selected
    }
    hil_robot_selected = [
        row
        for row in hil_robot_all
        if (row["condition_id"], row["trial_id"], row["run_generation"]) in selected_keys
    ]
    validate_hil_reconstruction(hil_selected, hil_robot_selected)

    comparison_rows: list[dict[str, object]] = []
    matching_rows: list[dict[str, object]] = []
    for algorithm in ALGORITHMS:
        physical_rows = sorted(
            (row for row in physical if row["algorithm"] == algorithm),
            key=lambda row: int(row["trial_number"]),
        )
        hil_rows = sorted(
            (row for row in hil_selected if row["algorithm"] == algorithm),
            key=lambda row: int(row["trial_id"]),
        )
        if len(hil_rows) != 25:
            raise RuntimeError(f"expected 25 HIL trials for {algorithm}, found {len(hil_rows)}")
        physical_source_ids = {row["source_episode"] for row in physical_rows}
        hil_trial_ids = {row["trial_id"] for row in hil_rows}
        overlap = sorted(physical_source_ids & hil_trial_ids, key=int)
        condition_ids = sorted({row["condition_id"] for row in hil_rows})
        if len(condition_ids) != 1:
            raise RuntimeError(f"multiple HIL conditions selected for {algorithm}: {condition_ids}")
        matching_rows.append(
            {
                "algorithm": algorithm,
                "top_k_rate": physical_condition[algorithm][0],
                "top_k_cells": physical_condition[algorithm][1],
                "physical_trial_count": len(physical_rows),
                "hil_trial_count": len(hil_rows),
                "physical_source_episode_ids": ";".join(sorted(physical_source_ids, key=int)),
                "hil_trial_ids": ";".join(sorted(hil_trial_ids, key=int)),
                "exact_scenario_overlap_count": len(overlap),
                "overlapping_scenario_ids": ";".join(overlap),
                "hil_condition_id": condition_ids[0],
                "comparison_design": "unpaired_same_algorithm_topk_different_scenarios",
            }
        )
        for row in physical_rows:
            allocator_calls = as_int(row, "total_allocator_calls")
            filter_calls = as_int(row, "total_candidate_filter_calls")
            if allocator_calls <= 0 or filter_calls <= 0:
                raise RuntimeError(f"nonpositive physical timing call count: {row['trial_label']}")
            comparison_rows.append(
                {
                    "environment": "physical",
                    "algorithm": algorithm,
                    "trial_label": row["trial_label"],
                    "study_trial_number": row["trial_number"],
                    "source_episode_or_hil_trial_id": row["source_episode"],
                    "target_location": row["target_location"],
                    "top_k_rate": row["top_k_rate"],
                    "top_k_cells": row["top_k_max_cells"],
                    "scenario_sha256": row["scenario_sha256"],
                    "allocator_calls": allocator_calls,
                    "candidate_filter_calls": filter_calls,
                    "allocation_exclusive_mean_us": (
                        as_float(row, "total_allocator_time_us") - as_float(row, "total_filter_time_us")
                    )
                    / allocator_calls,
                    "reported_allocator_solve_mean_us": as_float(
                        row, "total_allocator_solve_time_us"
                    )
                    / allocator_calls,
                    "unattributed_nonfilter_overhead_mean_us": (
                        as_float(row, "total_allocator_time_us")
                        - as_float(row, "total_filter_time_us")
                        - as_float(row, "total_allocator_solve_time_us")
                    )
                    / allocator_calls,
                    "candidate_filter_mean_us": as_float(row, "total_filter_time_us") / filter_calls,
                    "candidate_filter_contribution_per_allocator_call_us": as_float(row, "total_filter_time_us") / allocator_calls,
                    "combined_allocator_mean_us": as_float(row, "total_allocator_time_us") / allocator_calls,
                }
            )
        for row in hil_rows:
            comparison_rows.append(
                {
                    "environment": "HIL",
                    "algorithm": algorithm,
                    "trial_label": f"HIL {algorithm} {row['trial_id']}",
                    "study_trial_number": "",
                    "source_episode_or_hil_trial_id": row["trial_id"],
                    "target_location": "",
                    "top_k_rate": row["top_k_rate"],
                    "top_k_cells": row["top_k_cells"],
                    "scenario_sha256": row["scenario_sha256"],
                    "allocator_calls": row["timed_allocator_call_count"],
                    "candidate_filter_calls": row["candidate_filter_invocation_count"],
                    "allocation_exclusive_mean_us": (
                        as_float(row, "allocator_time_total_us")
                        - as_float(row, "candidate_filter_time_total_us")
                    )
                    / as_int(row, "timed_allocator_call_count"),
                    "reported_allocator_solve_mean_us": "",
                    "unattributed_nonfilter_overhead_mean_us": "",
                    "candidate_filter_mean_us": as_float(row, "candidate_filter_time_total_us")
                    / as_int(row, "candidate_filter_invocation_count"),
                    "candidate_filter_contribution_per_allocator_call_us": as_float(
                        row, "candidate_filter_time_total_us"
                    )
                    / as_int(row, "timed_allocator_call_count"),
                    "combined_allocator_mean_us": row["allocator_time_mean_us"],
                }
            )

    physical_episode_ids = sorted({row["source_episode"] for row in physical}, key=int)
    hil_episode_ids = sorted({row["trial_id"] for row in hil_selected}, key=int)
    scenario_overlap_rows: list[dict[str, object]] = []
    target_only_pairs: list[tuple[str, str]] = []
    for physical_id in physical_episode_ids:
        physical_scenario = scenarios[physical_id]
        exact_ids = [
            hil_id
            for hil_id in hil_episode_ids
            if scenario_signature(scenarios[hil_id]) == scenario_signature(physical_scenario)
        ]
        same_target_ids = [
            hil_id
            for hil_id in hil_episode_ids
            if scenarios[hil_id]["object_x"] == physical_scenario["object_x"]
            and scenarios[hil_id]["object_y"] == physical_scenario["object_y"]
        ]
        target_only_ids = [hil_id for hil_id in same_target_ids if hil_id not in exact_ids]
        target_only_pairs.extend((physical_id, hil_id) for hil_id in target_only_ids)
        scenario_overlap_rows.append(
            {
                "physical_source_episode": physical_id,
                "physical_target_location": f"{physical_scenario['object_x']}/{physical_scenario['object_y']}",
                "physical_clue_locations": clue_signature(physical_scenario),
                "exact_content_match_count": len(exact_ids),
                "exact_content_hil_trial_ids": ";".join(exact_ids),
                "target_only_match_count": len(target_only_ids),
                "target_only_hil_trial_ids": ";".join(target_only_ids),
                "identity_conclusion": "identical_scenario_found" if exact_ids else "no_identical_hil_scenario",
            }
        )

    comparison_index = {
        (
            str(row["environment"]),
            str(row["algorithm"]),
            str(row["source_episode_or_hil_trial_id"]),
        ): row
        for row in comparison_rows
    }
    physical_source_index = {
        (row["algorithm"], row["source_episode"]): row for row in physical
    }
    hil_source_index = {
        (row["algorithm"], row["trial_id"]): row for row in hil_selected
    }
    physical_robot_by_label: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in physical_robot_rows:
        physical_robot_by_label[row["trial_label"]].append(row)
    target_pair_rows: list[dict[str, object]] = []
    for physical_id, hil_id in target_only_pairs:
        physical_scenario = scenarios[physical_id]
        hil_scenario = scenarios[hil_id]
        for algorithm in ALGORITHMS:
            physical_source = physical_source_index[(algorithm, physical_id)]
            hil_source = hil_source_index[(algorithm, hil_id)]
            physical_timing = comparison_index[("physical", algorithm, physical_id)]
            hil_timing = comparison_index[("HIL", algorithm, hil_id)]
            physical_steps = as_int(physical_source, "total_steps")
            hil_steps = as_int(hil_source, "total_team_steps")
            physical_max_steps = max(
                as_int(row, "steps") for row in physical_robot_by_label[physical_source["trial_label"]]
            )
            hil_max_steps = as_int(hil_source, "max_steps_any_robot")
            target_pair_rows.append(
                {
                    "algorithm": algorithm,
                    "target_location": f"{physical_scenario['object_x']}/{physical_scenario['object_y']}",
                    "physical_trial_label": physical_source["trial_label"],
                    "physical_source_episode": physical_id,
                    "hil_trial_id": hil_id,
                    "physical_clue_locations": clue_signature(physical_scenario),
                    "hil_clue_locations": clue_signature(hil_scenario),
                    "exact_scenario_match": "no",
                    "comparison_status": "descriptive_only_same_target_different_clues",
                    "physical_total_team_steps": physical_steps,
                    "hil_total_team_steps": hil_steps,
                    "steps_difference_physical_minus_hil": physical_steps - hil_steps,
                    "steps_ratio_physical_over_hil": physical_steps / hil_steps,
                    "physical_max_steps_any_robot": physical_max_steps,
                    "hil_max_steps_any_robot": hil_max_steps,
                    "max_steps_ratio_physical_over_hil": physical_max_steps / hil_max_steps,
                    "physical_allocation_exclusive_mean_us": physical_timing["allocation_exclusive_mean_us"],
                    "hil_allocation_exclusive_mean_us": hil_timing["allocation_exclusive_mean_us"],
                    "allocation_ratio_physical_over_hil": float(physical_timing["allocation_exclusive_mean_us"])
                    / float(hil_timing["allocation_exclusive_mean_us"]),
                    "physical_filter_mean_us": physical_timing["candidate_filter_mean_us"],
                    "hil_filter_mean_us": hil_timing["candidate_filter_mean_us"],
                    "filter_ratio_physical_over_hil": float(physical_timing["candidate_filter_mean_us"])
                    / float(hil_timing["candidate_filter_mean_us"]),
                    "physical_combined_allocator_mean_us": physical_timing["combined_allocator_mean_us"],
                    "hil_combined_allocator_mean_us": hil_timing["combined_allocator_mean_us"],
                    "combined_ratio_physical_over_hil": float(physical_timing["combined_allocator_mean_us"])
                    / float(hil_timing["combined_allocator_mean_us"]),
                    "inferential_test": "not_run_single_nonidentical_scenario_pair",
                }
            )

    call_mix_rows: list[dict[str, object]] = []
    for algorithm in ALGORITHMS:
        for environment in ("physical", "HIL"):
            rows = [
                row
                for row in comparison_rows
                if row["algorithm"] == algorithm and row["environment"] == environment
            ]
            allocator_calls = sum(int(row["allocator_calls"]) for row in rows)
            filter_calls = sum(int(row["candidate_filter_calls"]) for row in rows)
            call_mix_rows.append(
                {
                    "algorithm": algorithm,
                    "environment": environment,
                    "trial_count": len(rows),
                    "allocator_calls": allocator_calls,
                    "candidate_filter_invocations": filter_calls,
                    "filter_invocations_per_allocator_call": filter_calls / allocator_calls,
                    "mean_reported_solve_gap_us": mean(
                        float(row["unattributed_nonfilter_overhead_mean_us"])
                        for row in rows
                        if row["unattributed_nonfilter_overhead_mean_us"] != ""
                    )
                    if environment == "physical"
                    else "",
                    "filter_mean_denominator": "candidate_filter_invocations",
                    "combined_mean_denominator": "timed_allocator_calls",
                }
            )

    combinations = np.asarray(
        list(itertools.combinations(range(30), 5)),
        dtype=np.int16,
    )
    stats_rows: list[dict[str, object]] = []
    metric_index = {metric[0]: index for index, metric in enumerate(METRICS)}
    for algorithm_index, algorithm in enumerate(ALGORITHMS):
        for metric_key, metric_label, *_ in METRICS:
            x = np.asarray(
                [
                    float(row[metric_key])
                    for row in comparison_rows
                    if row["environment"] == "physical" and row["algorithm"] == algorithm
                ],
                dtype=float,
            )
            y = np.asarray(
                [
                    float(row[metric_key])
                    for row in comparison_rows
                    if row["environment"] == "HIL" and row["algorithm"] == algorithm
                ],
                dtype=float,
            )
            if len(x) != 5 or len(y) != 25 or np.any(x <= 0) or np.any(y <= 0):
                raise RuntimeError(f"invalid comparison vectors for {algorithm} {metric_key}")
            physical_mean = float(np.mean(x))
            hil_mean = float(np.mean(y))
            difference = physical_mean - hil_mean
            ratio = physical_mean / hil_mean
            u_value, rank_p = exact_rank_permutation_p(x, y, combinations)
            bootstrap = bootstrap_intervals(
                x,
                y,
                SEED + algorithm_index * 100 + metric_index[metric_key],
            )
            stats_rows.append(
                {
                    "algorithm": algorithm,
                    "metric": metric_key,
                    "metric_label": metric_label,
                    "physical_n_trials": len(x),
                    "hil_n_trials": len(y),
                    "physical_mean_us": physical_mean,
                    "physical_sd_us": float(np.std(x, ddof=1)),
                    "physical_median_us": float(np.median(x)),
                    "hil_mean_us": hil_mean,
                    "hil_sd_us": float(np.std(y, ddof=1)),
                    "hil_median_us": float(np.median(y)),
                    "mean_difference_physical_minus_hil_us": difference,
                    "mean_difference_pct_of_hil": difference / hil_mean * 100.0,
                    "mean_ratio_physical_over_hil": ratio,
                    **bootstrap,
                    "cliffs_delta": cliffs_delta(x, y),
                    "exact_mean_permutation_p": exact_permutation_p(x, y, combinations),
                    "exact_log_mean_permutation_p": exact_permutation_p(x, y, combinations, transform_log=True),
                    "mann_whitney_u": u_value,
                    "exact_rank_permutation_p": rank_p,
                    "multiple_testing_family": "15 algorithm-by-metric log-mean tests",
                    "equivalence_test": "not_run_no_practical_margin_specified",
                    "scenario_pairing": "unpaired_no_exact_scenario_overlap",
                }
            )

    adjusted = benjamini_hochberg(
        [float(row["exact_log_mean_permutation_p"]) for row in stats_rows]
    )
    for row, adjusted_p in zip(stats_rows, adjusted):
        row["bh_adjusted_log_mean_p"] = adjusted_p
        if adjusted_p < 0.05:
            direction = "physical_slower" if float(row["mean_ratio_physical_over_hil"]) > 1 else "physical_faster"
            row["difference_conclusion_alpha_0_05"] = f"different:{direction}"
        else:
            row["difference_conclusion_alpha_0_05"] = "no_detectable_difference_not_equivalence"

    for metric_key, *_ in METRICS:
        metric_indexes = [index for index, row in enumerate(stats_rows) if row["metric"] == metric_key]
        metric_adjusted = benjamini_hochberg(
            [float(stats_rows[index]["exact_log_mean_permutation_p"]) for index in metric_indexes]
        )
        for index, adjusted_p in zip(metric_indexes, metric_adjusted):
            stats_rows[index]["bh_adjusted_within_metric_p"] = adjusted_p

    overall_rows: list[dict[str, object]] = []
    for metric_number, (metric_key, metric_label, *_) in enumerate(METRICS):
        values_by_algorithm: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for algorithm in ALGORITHMS:
            physical_values = np.asarray(
                [
                    float(row[metric_key])
                    for row in comparison_rows
                    if row["environment"] == "physical" and row["algorithm"] == algorithm
                ],
                dtype=float,
            )
            hil_values = np.asarray(
                [
                    float(row[metric_key])
                    for row in comparison_rows
                    if row["environment"] == "HIL" and row["algorithm"] == algorithm
                ],
                dtype=float,
            )
            values_by_algorithm[algorithm] = (physical_values, hil_values)
        overall_rows.append(
            {
                "metric": metric_key,
                "metric_label": metric_label,
                "physical_trials": 25,
                "hil_trials": 125,
                "algorithms_stratified": len(ALGORITHMS),
                **stratified_overall_log_test(
                    values_by_algorithm,
                    combinations,
                    SEED + 10_000 + metric_number,
                ),
                "permutation_resamples": STRATIFIED_PERMUTATION_RESAMPLES,
                "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
                "design": "algorithm_stratified_unpaired_log_mean",
                "equivalence_test": "not_run_no_practical_margin_specified",
            }
        )
    overall_adjusted = benjamini_hochberg(
        [float(row["stratified_log_mean_permutation_p"]) for row in overall_rows]
    )
    for row, adjusted_p in zip(overall_rows, overall_adjusted):
        row["bh_adjusted_p_across_three_metrics"] = adjusted_p
        if adjusted_p < 0.05:
            direction = "physical_slower" if float(row["geometric_mean_ratio_physical_over_hil"]) > 1 else "physical_faster"
            row["overall_conclusion_alpha_0_05"] = f"different:{direction}"
        else:
            row["overall_conclusion_alpha_0_05"] = "no_detectable_difference_not_equivalence"

    overview_rows: list[dict[str, object]] = []
    for metric_key, metric_label, *_ in METRICS:
        rows = [row for row in stats_rows if row["metric"] == metric_key]
        ratios = [float(row["mean_ratio_physical_over_hil"]) for row in rows]
        overview_rows.append(
            {
                "metric": metric_key,
                "metric_label": metric_label,
                "algorithms_tested": len(rows),
                "algorithms_with_bh_significant_difference": sum(
                    str(row["difference_conclusion_alpha_0_05"]).startswith("different:") for row in rows
                ),
                "algorithms_physical_slower": sum(
                    row["difference_conclusion_alpha_0_05"] == "different:physical_slower" for row in rows
                ),
                "algorithms_physical_faster": sum(
                    row["difference_conclusion_alpha_0_05"] == "different:physical_faster" for row in rows
                ),
                "median_algorithm_mean_ratio_physical_over_hil": median(ratios),
                "min_algorithm_mean_ratio_physical_over_hil": min(ratios),
                "max_algorithm_mean_ratio_physical_over_hil": max(ratios),
                "equivalence_claim": "not_assessed_without_practical_equivalence_margin",
            }
        )

    comparison_path = OUTPUT_ROOT / "trial_level_timing_comparison.csv"
    stats_path = OUTPUT_ROOT / "statistical_tests_by_algorithm.csv"
    overall_path = OUTPUT_ROOT / "statistical_tests_overall_stratified.csv"
    overview_path = OUTPUT_ROOT / "metric_overview.csv"
    matching_path = OUTPUT_ROOT / "matching_audit.csv"
    call_mix_path = OUTPUT_ROOT / "call_mix_audit.csv"
    scenario_overlap_path = OUTPUT_ROOT / "scenario_content_overlap_audit.csv"
    target_pairs_path = OUTPUT_ROOT / "target_only_performance_timing_comparison.csv"
    write_csv(comparison_path, list(comparison_rows[0]), comparison_rows)
    write_csv(stats_path, list(stats_rows[0]), stats_rows)
    write_csv(overall_path, list(overall_rows[0]), overall_rows)
    write_csv(overview_path, list(overview_rows[0]), overview_rows)
    write_csv(matching_path, list(matching_rows[0]), matching_rows)
    write_csv(call_mix_path, list(call_mix_rows[0]), call_mix_rows)
    write_csv(scenario_overlap_path, list(scenario_overlap_rows[0]), scenario_overlap_rows)
    write_csv(target_pairs_path, list(target_pair_rows[0]), target_pair_rows)

    significant_count = sum(
        str(row["difference_conclusion_alpha_0_05"]).startswith("different:")
        for row in stats_rows
    )
    readme = f"""# Physical versus HIL timing comparison

This analysis compares the 25 validated physical trials with the Bayesian HIL
campaign at the same algorithm-specific Top-K settings. The five physical
source episodes are 4, 53, 232, 394, and 473. None occurs in the 25-scenario
HIL subset, so scenario-by-scenario pairing would be invalid. Comparisons are
therefore unpaired: five physical system trials versus 25 HIL system trials per
algorithm. Target-and-clue content was also compared from the canonical scenario
file and found zero identical scenarios. Two HIL trials share only a target:
HIL 371 with physical episode 4 at `5/9`, and HIL 67 with physical episode 473
at `13/16`; their clue layouts differ, so those comparisons are descriptive only.

The independent unit is a four-robot system trial, not an individual robot.
Each timing value is the call-weighted mean reconstructed from all four robot
rows. Allocation-exclusive is derived in both environments as end-to-end
allocator total minus filter total, divided by timed allocator calls. This is
necessary because physical `allocator_solve_time_us_total` omits a repeatable
algorithm-specific non-filter overhead that HIL allocator-exclusive includes.
Filter is total candidate-filter time divided by actual filter invocations in
both environments; combined is end-to-end allocator time divided by timed
allocator calls. The published HIL filter mean is not used directly because its
denominator is all timed allocator calls, not actual filter invocations.

For each of five algorithms and three timing metrics, the analysis reports an
exact two-sided permutation test of arithmetic means, an exact permutation test
of log means, an exact rank/Mann-Whitney permutation test, Cliff's delta, and a
50,000-resample stratified bootstrap interval for the mean difference and mean
    ratio. Benjamini-Hochberg correction is applied across the 15 log-mean tests.
    At adjusted alpha 0.05, {significant_count} of 15 comparisons show a detectable
    difference. Failure to reject is labeled as no detectable difference, not proof
    that physical and HIL timings are equal. A second algorithm-stratified test
    assesses the overall environment shift for each metric using 200,000 label
    permutations and controls false discovery across the three metrics. A formal
    equivalence test is not run because no practical equivalence margin was specified.

Files:

    - `trial_level_timing_comparison.csv`: all 150 system-trial observations.
    - `statistical_tests_by_algorithm.csv`: detailed tests and effect sizes.
    - `statistical_tests_overall_stratified.csv`: overall tests controlling for algorithm.
- `metric_overview.csv`: metric-level direction summary.
- `matching_audit.csv`: exact condition selection and zero scenario overlap.
- `call_mix_audit.csv`: filter invocation frequency by algorithm and environment.
- `scenario_content_overlap_audit.csv`: target-and-clue identity check for all physical trials.
- `target_only_performance_timing_comparison.csv`: descriptive comparisons for
  same-target HIL trials whose clue layouts differ.
"""
    readme_path = OUTPUT_ROOT / "README.md"
    readme_path.write_text(readme, encoding="utf-8")

    outputs = [
        comparison_path,
        stats_path,
        overall_path,
        overview_path,
        matching_path,
        call_mix_path,
        scenario_overlap_path,
        target_pairs_path,
        readme_path,
    ]
    manifest = {
        "schema_version": 1,
        "analysis_design": "unpaired four-robot system-trial comparison at matched algorithm and Top-K condition",
        "seed": SEED,
        "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
        "stratified_permutation_resamples": STRATIFIED_PERMUTATION_RESAMPLES,
        "exact_permutations_per_test": int(len(combinations)),
        "multiple_testing": "Benjamini-Hochberg across 15 algorithm-by-metric exact log-mean permutation tests",
        "inputs": [
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in (PHYSICAL_COMBINED, PHYSICAL_ROBOTS, HIL_COMBINED, HIL_ROBOTS, SCENARIO_FILE)
        ],
        "coverage": {
            "physical_trials": sum(row["environment"] == "physical" for row in comparison_rows),
            "hil_trials": sum(row["environment"] == "HIL" for row in comparison_rows),
            "algorithms": list(ALGORITHMS),
            "tests": len(stats_rows),
            "bh_significant_tests": significant_count,
            "overall_stratified_tests": len(overall_rows),
            "overall_bh_significant_tests": sum(
                str(row["overall_conclusion_alpha_0_05"]).startswith("different:")
                for row in overall_rows
            ),
            "exact_scenario_overlaps": sum(int(row["exact_scenario_overlap_count"]) for row in matching_rows),
            "exact_content_scenario_overlaps": sum(
                int(row["exact_content_match_count"]) for row in scenario_overlap_rows
            ),
            "target_only_scenario_pairs": len(target_only_pairs),
        },
        "runtime": {
            "python": platform.python_version(),
            "numpy": np.__version__,
        },
        "outputs": [
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in outputs
        ],
    }
    manifest_path = OUTPUT_ROOT / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
