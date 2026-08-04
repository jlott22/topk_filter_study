"""Match and aggregate physical robot metrics into one row per trial.

The lossless consolidated input is never modified. This script keeps only
groups containing one row from each of robots 00--03 at one of the five
declared hardware-study targets. Complete repeated trials are retained as
lettered variants and flagged for review. Every included and excluded source
row is written to a separate audit CSV.
"""

from __future__ import annotations

import csv
import hashlib
import itertools
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean


ROOT = Path(__file__).resolve().parents[2]
INPUT = (
    ROOT
    / "Results"
    / "Hardware"
    / "PhysicalTrials"
    / "Completed"
    / "all_robots_all_algorithms.csv"
)
OUTPUT_ROOT = (
    ROOT / "Results" / "Hardware" / "PhysicalTrials" / "CombinedTrials"
)
ROBOT_IDS = ("00", "01", "02", "03")
TARGETS = (
    (1, "5/9", "4"),
    (2, "7/2", "53"),
    (3, "3/11", "232"),
    (4, "5/4", "394"),
    (5, "13/16", "473"),
)
TARGET_NUMBER = {target: number for number, target, _ in TARGETS}
SOURCE_EPISODE = {target: episode for _, target, episode in TARGETS}
ALGORITHM_ORDER = {name: index for index, name in enumerate(("ACBBA", "CBAA", "DMCHBA", "HIPC", "PI", "DGA"))}
ALGORITHMS = ("ACBBA", "CBAA", "DMCHBA", "HIPC", "PI", "DGA")
CONSISTENT_FIELDS = (
    "alg",
    "target_location",
    "top_k_rate",
    "top_k_max_cells",
    "drop_rate",
    "trial_mode",
    "commitment_horizon",
    "logic_revision",
    "scenario_sha256",
)
PROVENANCE_FIELDS = (
    "capture_batch",
    "source_path",
    "source_file_sha256",
    "source_line",
    "source_column_count",
    "structural_repair",
    "source_row_sha256",
)


def read_rows() -> tuple[list[str], list[dict[str, str]]]:
    with INPUT.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError(f"missing CSV header: {INPUT}")
        rows = list(reader)
        return list(reader.fieldnames), rows


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def source_key(row: dict[str, object]) -> tuple[str, str, str]:
    return (
        str(row["source_path"]),
        str(row["source_line"]),
        str(row["source_row_sha256"]),
    )


def number(row: dict[str, str], field: str) -> float:
    value = row[field].strip()
    return float(value) if value else 0.0


def integer(row: dict[str, str], field: str) -> int:
    return int(round(number(row, field)))


def distinct(rows: list[dict[str, str]], field: str) -> list[str]:
    return sorted({row[field] for row in rows})


def complete_group(rows: list[dict[str, str]]) -> bool:
    return len(rows) == 4 and sorted(row["robot_id"] for row in rows) == list(ROBOT_IDS)


def validate_configuration(rows: list[dict[str, str]], label: str) -> None:
    inconsistent = {
        field: distinct(rows, field)
        for field in CONSISTENT_FIELDS
        if len(distinct(rows, field)) != 1
    }
    if inconsistent:
        raise RuntimeError(f"configuration mismatch in {label}: {inconsistent}")


def aggregate(
    label: str,
    trial_number: int,
    variant: str,
    rows: list[dict[str, str]],
    match_method: str,
    review_reasons: list[str],
) -> dict[str, object]:
    validate_configuration(rows, label)
    algorithm = rows[0]["alg"]
    target = rows[0]["target_location"]
    trial_times = [number(row, "trial_time_ms") for row in rows]
    motor_times = [number(row, "motor_time_ms") for row in rows]
    compute_times = [max(0.0, trial - motor) for trial, motor in zip(trial_times, motor_times)]
    steps = [integer(row, "steps") for row in rows]
    total_steps = sum(steps)
    filter_calls = sum(integer(row, "candidate_filter_calls") for row in rows)
    filter_total_us = sum(number(row, "candidate_filter_time_us_total") for row in rows)
    allocator_calls = sum(integer(row, "allocator_calls") for row in rows)
    allocator_total_us = sum(number(row, "allocator_time_us_total") for row in rows)
    allocator_solve_total_us = sum(number(row, "allocator_solve_time_us_total") for row in rows)
    total_trial_ms = sum(trial_times)
    total_motor_ms = sum(motor_times)
    total_compute_ms = sum(compute_times)
    trial_range_ms = max(trial_times) - min(trial_times)
    avg_trial_ms = mean(trial_times)
    relative_range_pct = (trial_range_ms / avg_trial_ms * 100.0) if avg_trial_ms else 0.0

    if relative_range_pct > 10.0:
        review_reasons.append(f"robot_trial_time_range_gt_10pct:{relative_range_pct:.2f}%")
    if any(value <= 0 for value in steps):
        review_reasons.append("one_or_more_robots_reported_zero_steps")
    if any(value <= 0 for value in trial_times):
        review_reasons.append("one_or_more_robots_reported_nonpositive_trial_time")

    total_msgs_sent = sum(integer(row, "msgs_sent") for row in rows)
    total_msgs_received = sum(integer(row, "msgs_received") for row in rows)
    unique_review_reasons = list(dict.fromkeys(review_reasons))
    result: dict[str, object] = {
        "trial_label": label,
        "algorithm": algorithm,
        "trial_number": trial_number,
        "variant": variant,
        "target_location": target,
        "source_episode": SOURCE_EPISODE[target],
        "match_status": "matched_review" if unique_review_reasons else "matched",
        "review_flag": "yes" if unique_review_reasons else "no",
        "review_reasons": " | ".join(unique_review_reasons),
        "match_method": match_method,
        "robot_count": len(rows),
        "robot_ids": ";".join(sorted(row["robot_id"] for row in rows)),
        "source_batches": ";".join(distinct(rows, "capture_batch")),
        "source_config_sequences": ";".join(distinct(rows, "config_sequence")),
        "top_k_rate": rows[0]["top_k_rate"],
        "top_k_max_cells": rows[0]["top_k_max_cells"],
        "drop_rate": rows[0]["drop_rate"],
        "trial_mode": rows[0]["trial_mode"],
        "commitment_horizon": rows[0]["commitment_horizon"],
        "logic_revision": rows[0]["logic_revision"],
        "scenario_sha256": rows[0]["scenario_sha256"],
        "source_structural_repair_rows": sum(row["structural_repair"] != "none" for row in rows),
        "total_steps": total_steps,
        "avg_steps_per_robot": mean(steps),
        "max_steps_any_robot": max(steps),
        "total_msgs_sent": total_msgs_sent,
        "total_msgs_received": total_msgs_received,
        "message_receive_to_send_pct": (total_msgs_received / total_msgs_sent * 100.0) if total_msgs_sent else 0.0,
        "total_bytes_sent": sum(integer(row, "bytes_sent") for row in rows),
        "total_bytes_received": sum(integer(row, "bytes_received") for row in rows),
        "avg_trial_time_ms": avg_trial_ms,
        "total_trial_time_ms": total_trial_ms,
        "min_trial_time_ms": min(trial_times),
        "max_trial_time_ms": max(trial_times),
        "trial_time_range_ms": trial_range_ms,
        "trial_time_relative_range_pct": relative_range_pct,
        "avg_motor_time_ms": mean(motor_times),
        "total_motor_time_ms": total_motor_ms,
        "avg_compute_time_ms": mean(compute_times),
        "total_compute_time_ms": total_compute_ms,
        "total_candidate_filter_calls": filter_calls,
        "total_filter_time_us": filter_total_us,
        "avg_filter_time_per_call_us": filter_total_us / filter_calls if filter_calls else 0.0,
        "max_filter_time_us": max(number(row, "candidate_filter_time_us_max") for row in rows),
        "total_allocator_calls": allocator_calls,
        "total_allocator_solve_time_us": allocator_solve_total_us,
        "total_allocator_time_us": allocator_total_us,
        "avg_allocator_time_per_call_us": allocator_total_us / allocator_calls if allocator_calls else 0.0,
        "max_allocator_time_us": max(number(row, "allocator_time_us_max") for row in rows),
        "avg_robot_allocator_time_pct": mean(number(row, "allocator_time_pct") for row in rows),
        "combined_allocator_time_pct_of_trial": (allocator_total_us / (total_trial_ms * 1000.0) * 100.0) if total_trial_ms else 0.0,
        "weighted_mean_step_time_ms": total_trial_ms / total_steps if total_steps else 0.0,
        "avg_cpu_util_pct": mean(number(row, "cpu_util_pct") for row in rows),
        "max_cpu_util_pct": max(number(row, "cpu_util_pct") for row in rows),
        "max_mem_used_peak": max(integer(row, "mem_used_peak") for row in rows),
        "min_mem_free_min": min(integer(row, "mem_free_min") for row in rows),
        "total_task_cell_replans": sum(integer(row, "task_cell_replans") for row in rows),
        "total_path_replans": sum(integer(row, "path_replans") for row in rows),
        "total_collision_prevention_events": sum(integer(row, "collision_prevention_events") for row in rows),
    }
    for topic in range(1, 6):
        result[f"total_topic_{topic}_sent"] = sum(integer(row, f"{topic}_sent") for row in rows)
        result[f"total_topic_{topic}_received"] = sum(integer(row, f"{topic}_rec") for row in rows)
    return result


def add_group(
    groups: list[dict[str, object]],
    algorithm: str,
    target: str,
    rows: list[dict[str, str]],
    variant: str = "",
    match_method: str = "algorithm_target_exact",
    review_reasons: list[str] | None = None,
) -> None:
    trial_number = TARGET_NUMBER[target]
    label = f"{algorithm} {trial_number}{variant}"
    if not complete_group(rows):
        raise RuntimeError(f"attempted to include incomplete group {label}: {len(rows)} rows")
    groups.append(
        {
            "label": label,
            "algorithm": algorithm,
            "trial_number": trial_number,
            "variant": variant,
            "target": target,
            "rows": sorted(rows, key=lambda row: row["robot_id"]),
            "match_method": match_method,
            "review_reasons": list(review_reasons or []),
        }
    )


def match_rows(rows: list[dict[str, str]]) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    groups: list[dict[str, object]] = []
    exclusions: list[dict[str, object]] = []
    usable: list[dict[str, str]] = []

    for row in rows:
        if row["target_location"] not in TARGET_NUMBER:
            exclusions.append(
                {
                    "row": row,
                    "exclusion_category": "nonstudy_target",
                    "exclusion_reason": "target is not one of the five declared hardware-study targets",
                    "related_trial_label": "",
                }
            )
        else:
            usable.append(row)

    by_algorithm_target: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in usable:
        by_algorithm_target[(row["alg"], row["target_location"])].append(row)

    for (algorithm, target), target_rows in sorted(
        by_algorithm_target.items(),
        key=lambda item: (ALGORITHM_ORDER.get(item[0][0], 999), TARGET_NUMBER[item[0][1]]),
    ):
        if algorithm == "ACBBA" and target == "7/2":
            by_sequence: dict[str, list[dict[str, str]]] = defaultdict(list)
            for row in target_rows:
                by_sequence[row["config_sequence"]].append(row)
            for sequence, sequence_rows in sorted(by_sequence.items()):
                if sequence == "2":
                    for row in sequence_rows:
                        exclusions.append(
                            {
                                "row": row,
                                "exclusion_category": "user_confirmed_failed_trial",
                                "exclusion_reason": "ACBBA 2A was confirmed failed by the operator on 2026-08-02",
                                "related_trial_label": "ACBBA 2A",
                            }
                        )
                elif sequence == "1" and complete_group(sequence_rows):
                    add_group(
                        groups,
                        algorithm,
                        target,
                        sequence_rows,
                        match_method="operator_validated_config_sequence_exact",
                    )
                else:
                    for row in sequence_rows:
                        exclusions.append(
                            {
                                "row": row,
                                "exclusion_category": "unvalidated_duplicate_variant",
                                "exclusion_reason": f"ACBBA trial 2 config_sequence={sequence} is not the operator-validated four-robot run",
                                "related_trial_label": "ACBBA 2",
                            }
                        )
            continue

        if algorithm == "HIPC" and target == "7/2":
            by_batch: dict[str, list[dict[str, str]]] = defaultdict(list)
            for row in target_rows:
                by_batch[row["capture_batch"]].append(row)
            batch_rank = {"initial_recovery": 0, "completed_trials": 1}
            ordered = sorted(by_batch.items(), key=lambda item: (batch_rank.get(item[0], 99), item[0]))
            for batch, batch_rows in ordered:
                if batch == "initial_recovery":
                    for row in batch_rows:
                        exclusions.append(
                            {
                                "row": row,
                                "exclusion_category": "user_confirmed_failed_trial",
                                "exclusion_reason": "HIPC 2A was confirmed failed by the operator on 2026-08-02",
                                "related_trial_label": "HIPC 2A",
                            }
                        )
                elif batch == "completed_trials" and complete_group(batch_rows):
                    add_group(
                        groups,
                        algorithm,
                        target,
                        batch_rows,
                        match_method="operator_validated_capture_batch_exact",
                    )
                else:
                    for row in batch_rows:
                        exclusions.append(
                            {
                                "row": row,
                                "exclusion_category": "unvalidated_duplicate_variant",
                                "exclusion_reason": f"HIPC trial 2 capture_batch={batch} is not the operator-validated four-robot run",
                                "related_trial_label": "HIPC 2",
                            }
                        )
            continue

        if algorithm == "DMCHBA" and target == "3/11" and len(target_rows) > 4:
            by_robot: dict[str, list[dict[str, str]]] = defaultdict(list)
            for row in target_rows:
                by_robot[row["robot_id"]].append(row)
            if set(by_robot) != set(ROBOT_IDS):
                for row in target_rows:
                    exclusions.append(
                        {
                            "row": row,
                            "exclusion_category": "incomplete_trial",
                            "exclusion_reason": "DMCHBA trial 3 lacks at least one robot and cannot be matched",
                            "related_trial_label": "DMCHBA 3",
                        }
                    )
                continue
            combinations = list(itertools.product(*(by_robot[robot] for robot in ROBOT_IDS)))
            combinations.sort(
                key=lambda combo: (
                    max(number(row, "trial_time_ms") for row in combo)
                    - min(number(row, "trial_time_ms") for row in combo),
                    sum(number(row, "trial_time_ms") for row in combo),
                )
            )
            selected = list(combinations[0])
            selected_hashes = {row["source_row_sha256"] for row in selected}
            duration_range = max(number(row, "trial_time_ms") for row in selected) - min(
                number(row, "trial_time_ms") for row in selected
            )
            add_group(
                groups,
                algorithm,
                target,
                selected,
                match_method="minimum_four_robot_trial_time_range",
                review_reasons=[f"duplicate_rows_duration_cluster_selected:range_ms={duration_range:.0f}"],
            )
            for row in target_rows:
                if row["source_row_sha256"] not in selected_hashes:
                    exclusions.append(
                        {
                            "row": row,
                            "exclusion_category": "incomplete_duplicate_remnant",
                            "exclusion_reason": "excluded from minimum-range four-robot duration cluster; no matching robot 00 row",
                            "related_trial_label": "DMCHBA 3",
                        }
                    )
            continue

        if complete_group(target_rows):
            add_group(groups, algorithm, target, target_rows)
        else:
            category = "ambiguous_duplicate_rows" if len(target_rows) > 4 else "incomplete_trial"
            for row in target_rows:
                exclusions.append(
                    {
                        "row": row,
                        "exclusion_category": category,
                        "exclusion_reason": "expected exactly one row from each of robots 00, 01, 02, and 03",
                        "related_trial_label": f"{algorithm} {TARGET_NUMBER[target]}",
                    }
                )

    groups.sort(
        key=lambda group: (
            ALGORITHM_ORDER.get(str(group["algorithm"]), 999),
            int(group["trial_number"]),
            str(group["variant"]),
        )
    )
    return groups, exclusions


def main() -> int:
    source_header, source_rows = read_rows()
    groups, exclusions = match_rows(source_rows)

    combined: list[dict[str, object]] = []
    matched_rows: list[dict[str, object]] = []
    for group in groups:
        aggregate_row = aggregate(
            str(group["label"]),
            int(group["trial_number"]),
            str(group["variant"]),
            list(group["rows"]),
            str(group["match_method"]),
            list(group["review_reasons"]),
        )
        combined.append(aggregate_row)
        for row in group["rows"]:
            matched_rows.append(
                {
                    "trial_label": group["label"],
                    "match_method": group["match_method"],
                    "group_review_flag": aggregate_row["review_flag"],
                    "group_review_reasons": aggregate_row["review_reasons"],
                    **row,
                }
            )

    excluded_rows = [
        {
            "exclusion_category": item["exclusion_category"],
            "exclusion_reason": item["exclusion_reason"],
            "related_trial_label": item["related_trial_label"],
            **item["row"],
        }
        for item in exclusions
    ]
    review_rows = [row for row in combined if row["review_flag"] == "yes"]

    source_keys = {source_key(row) for row in source_rows}
    matched_keys = {source_key(row) for row in matched_rows}
    excluded_keys = {source_key(row) for row in excluded_rows}
    if len(source_keys) != len(source_rows):
        raise RuntimeError("source row identity keys are not unique")
    if len(matched_keys) != len(matched_rows):
        raise RuntimeError("a source row was matched more than once")
    if len(excluded_keys) != len(excluded_rows):
        raise RuntimeError("a source row was excluded more than once")
    if matched_keys & excluded_keys:
        raise RuntimeError("at least one source row is both matched and excluded")
    if matched_keys | excluded_keys != source_keys:
        raise RuntimeError("matched and excluded audits do not account for every source row")
    if len({row["trial_label"] for row in combined}) != len(combined):
        raise RuntimeError("combined trial labels are not unique")

    combined_path = OUTPUT_ROOT / "combined_metrics_per_trial.csv"
    matched_path = OUTPUT_ROOT / "matched_robot_rows.csv"
    excluded_path = OUTPUT_ROOT / "excluded_trial_remnants.csv"
    review_path = OUTPUT_ROOT / "review_flags.csv"
    write_csv(combined_path, list(combined[0]), combined)
    write_csv(
        matched_path,
        ["trial_label", "match_method", "group_review_flag", "group_review_reasons", *source_header],
        matched_rows,
    )
    write_csv(
        excluded_path,
        ["exclusion_category", "exclusion_reason", "related_trial_label", *source_header],
        excluded_rows,
    )
    write_csv(review_path, list(combined[0]), review_rows)

    readme = """# Combined physical-trial metrics

`combined_metrics_per_trial.csv` contains one aggregate row for every matched
four-robot trial. Trial numbers follow the declared handpicked study mapping:
1=`5/9`, 2=`7/2`, 3=`3/11`, 4=`5/4`, and 5=`13/16`.

Matching is conservative: every included group contains exactly one source row
from each of robots 00, 01, 02, and 03. The operator-confirmed failed ACBBA 2A
and HIPC 2A runs are excluded. Their validated former 2B runs are the canonical
`ACBBA 2` and `HIPC 2`. DMCHBA trial 3 uses the unique four-robot duration
cluster with the smallest time range and remains flagged for review. No failed,
incomplete, or non-study-target rows are aggregated.

`matched_robot_rows.csv` is the row-level provenance for included trials.
`excluded_trial_remnants.csv` records every rejected source row and reason.
`review_flags.csv` is a filtered view of aggregate rows needing review.

Average filter and allocator times are weighted per call: total microseconds
divided by total calls across all four robots. Compute time is derived per robot
as `max(0, trial_time_ms - motor_time_ms)` before summing or averaging.
`max_steps_any_robot` is the highest per-robot `steps` value within the matched
four-robot trial.

Coverage is five validated trials for each of ACBBA, CBAA, DMCHBA, HIPC, and
PI. No DGA physical-trial log was captured, so DGA is absent from this dataset.
"""
    (OUTPUT_ROOT / "README.md").write_text(readme, encoding="utf-8")

    outputs = [
        combined_path,
        matched_path,
        excluded_path,
        review_path,
        OUTPUT_ROOT / "README.md",
    ]
    manifest = {
        "schema_version": 1,
        "input": INPUT.relative_to(ROOT).as_posix(),
        "input_sha256": sha256(INPUT),
        "declared_trial_mapping": [
            {"trial_number": number, "target_location": target, "source_episode": episode}
            for number, target, episode in TARGETS
        ],
        "matching_policy": {
            "required_robot_ids": list(ROBOT_IDS),
            "required_rows_per_group": 4,
            "nonstudy_targets": "excluded",
            "incomplete_groups": "excluded",
            "operator_rejected_trials": ["ACBBA 2A", "HIPC 2A"],
            "validated_trial_2_labels": ["ACBBA 2", "HIPC 2"],
            "dmchba_trial_3": "minimum_four_robot_trial_time_range_and_flagged",
        },
        "combined_trial_rows": len(combined),
        "matched_source_rows": len(matched_rows),
        "excluded_source_rows": len(excluded_rows),
        "review_flagged_trials": len(review_rows),
        "unaccounted_source_rows": len(source_rows) - len(matched_rows) - len(excluded_rows),
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
