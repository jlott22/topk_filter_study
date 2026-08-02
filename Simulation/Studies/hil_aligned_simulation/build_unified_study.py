from __future__ import annotations

import csv
import json
from collections import defaultdict
from decimal import Decimal
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[3]
BAYESIAN_OUTPUT = (
    REPO_ROOT / "Results" / "Simulation" / "Bayesian" / "Published"
    / "bayesian_topk_all_k_trial_level_system_and_timing_results.csv"
)
COLLABORATIVE_OUTPUT = (
    REPO_ROOT / "Results" / "Simulation" / "Collaborative" / "Published"
    / "collaborative_topk_all_k_trial_level_system_and_timing_results.csv"
)
VERIFICATION_OUTPUT = (
    REPO_ROOT / "Results" / "Simulation" / "TopKStudyVerification"
    / "bayesian_and_collaborative_topk_all_k_unified_dataset_verification_summary.json"
)
LOWK_DIR = (
    REPO_ROOT
    / "Results"
    / "Simulation"
    / "TopKLowKSupplement"
    / "AllMissionsFullCounts"
    / "combined"
)
BAYESIAN_PRIMARY_DIR = (
    REPO_ROOT
    / "Results"
    / "Simulation"
    / "Bayesian"
    / "primary_topk_campaign"
    / "combined"
)
COLLABORATIVE_PRIMARY_DIR = (
    REPO_ROOT
    / "Results"
    / "Simulation"
    / "Sensitivity"
    / "raw"
    / "collaborative_known_target_visit"
    / "topk_sensitivity"
    / "multitarget_g19_r4_t50"
)

ALGORITHM_ORDER = {name: index for index, name in enumerate(
    ("CBAA", "ACBBA", "PI", "HIPC", "DMCHBA", "DGA")
)}
TIMING_FIELDS = (
    "host_trial_runtime_ms",
    "allocator_calls_total",
    "allocator_time_ms_team_total",
    "allocator_time_ms_team_max",
    "allocator_solve_time_ms_team_total",
    "allocator_solve_time_ms_team_max",
    "candidate_filter_calls_total",
    "candidate_filter_time_ms_team_total",
    "candidate_filter_time_ms_team_max",
)
PREFERRED_COLUMNS = (
    "mission",
    "campaign_source",
    "top_k_level",
    "top_k_rate",
    "top_k_max_cells",
    "max_candidate_cells",
    "trial_id",
    "algorithm",
    "comm_model",
    "comm_level",
    "grid_size",
    "grid_cells",
    "robot_count",
    "target_count",
    "condition_id",
    "campaign_condition_id",
    "scenario_file",
    "scenario_file_sha256",
    "scenario_selection_sha256",
    "logic_revision",
    "study_profile",
    "trial_mode",
    "trial_status",
    "failure_type",
    "failure_message",
)


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def decimal(value: str | None) -> Decimal:
    return Decimal(value or "0")


def decimal_text(value: Decimal) -> str:
    text = format(value, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text or "0"


def integer(value: str | None) -> int:
    return int(Decimal(value or "0"))


def top_k_label(rate: str) -> str:
    percent = decimal(rate) * 100
    return f"{decimal_text(percent)}%"


def aggregate_bayesian_timing() -> dict[tuple[str, str], dict[str, str]]:
    _, rows = read_rows(BAYESIAN_PRIMARY_DIR / "all_computational_performance.csv")
    grouped: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[(row["condition_id"], row["trial_id"])].append(row)

    result: dict[tuple[str, str], dict[str, str]] = {}
    for key, trial_rows in grouped.items():
        allocator_totals = [decimal(row.get("allocator_time_ms_total")) for row in trial_rows]
        filter_totals = [decimal(row.get("candidate_filter_time_ms_total")) for row in trial_rows]
        solve_totals = [
            decimal(row.get("allocator_solve_time_ms_total"))
            if row.get("allocator_solve_time_ms_total", "") != ""
            else allocator_total - filter_total
            for row, allocator_total, filter_total in zip(
                trial_rows, allocator_totals, filter_totals, strict=True
            )
        ]
        result[key] = {
            "host_trial_runtime_ms": decimal_text(max(
                decimal(row.get("host_trial_runtime_ms")) for row in trial_rows
            )),
            "allocator_calls_total": str(sum(integer(row.get("allocator_calls")) for row in trial_rows)),
            "allocator_time_ms_team_total": decimal_text(sum(allocator_totals, Decimal())),
            "allocator_time_ms_team_max": decimal_text(max(allocator_totals)),
            "allocator_solve_time_ms_team_total": decimal_text(sum(solve_totals, Decimal())),
            "allocator_solve_time_ms_team_max": decimal_text(max(solve_totals)),
            "candidate_filter_calls_total": str(sum(
                integer(row.get("candidate_filter_calls")) for row in trial_rows
            )),
            "candidate_filter_time_ms_team_total": decimal_text(sum(filter_totals, Decimal())),
            "candidate_filter_time_ms_team_max": decimal_text(max(filter_totals)),
            "max_candidate_cells": trial_rows[0].get("max_candidate_cells", ""),
        }
    return result


def load_bayesian_rows() -> tuple[list[dict[str, str]], dict[str, int]]:
    _, primary = read_rows(BAYESIAN_PRIMARY_DIR / "all_system_performance.csv")
    _, lowk_all = read_rows(LOWK_DIR / "all_system_performance.csv")
    lowk = [row for row in lowk_all if row.get("mission") == "bayesian"]
    timing = aggregate_bayesian_timing()

    for row in primary:
        row.update(timing[(row["condition_id"], row["trial_id"])])
        row.update({
            "mission": "bayesian",
            "campaign_source": "primary_topk_5_to_100",
            "top_k_level": top_k_label(row["top_k_rate"]),
            "campaign_condition_id": row["condition_id"],
        })
    for row in lowk:
        row["campaign_source"] = "lowk_hil_aligned_supplement"

    return primary + lowk, {
        "primary_rows": len(primary),
        "lowk_rows": len(lowk),
        "expected_rows": 27000,
        "expected_conditions": 54,
        "expected_trials_per_condition": 500,
    }


def load_collaborative_rows() -> tuple[list[dict[str, str]], dict[str, int]]:
    primary: list[dict[str, str]] = []
    for config_path in sorted(COLLABORATIVE_PRIMARY_DIR.glob("*/*/config_used.json")):
        config = json.loads(config_path.read_text(encoding="utf-8"))
        _, rows = read_rows(config_path.parent / "system_performance.csv")
        suffix = config_path.parent.name.removeprefix("topk_")
        rate = Decimal(int(suffix)) / Decimal(100)
        max_cells = str(config["sim_config"]["max_candidate_cells"])
        for row in rows:
            row.update({
                "mission": "collaborative",
                "campaign_source": "primary_topk_5_to_100",
                "top_k_level": f"{int(suffix)}%",
                "top_k_rate": decimal_text(rate),
                "top_k_max_cells": max_cells,
                "max_candidate_cells": max_cells,
                "campaign_condition_id": row["condition_id"],
                "scenario_file_sha256": config["scenario_sha256"],
            })
        primary.extend(rows)

    _, lowk_all = read_rows(LOWK_DIR / "all_system_performance.csv")
    lowk = [row for row in lowk_all if row.get("mission") == "collaborative"]
    for row in lowk:
        row["campaign_source"] = "lowk_hil_aligned_supplement"

    return primary + lowk, {
        "primary_rows": len(primary),
        "lowk_rows": len(lowk),
        "expected_rows": 4800,
        "expected_conditions": 48,
        "expected_trials_per_condition": 100,
    }


def sort_rows(rows: list[dict[str, str]]) -> None:
    rows.sort(key=lambda row: (
        ALGORITHM_ORDER.get(row.get("algorithm", ""), len(ALGORITHM_ORDER)),
        integer(row.get("top_k_max_cells")),
        integer(row.get("trial_id")),
    ))


def output_columns(rows: list[dict[str, str]]) -> list[str]:
    encountered: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for field in row:
            if field not in seen:
                seen.add(field)
                encountered.append(field)
    populated = {field for field in encountered if any(row.get(field, "") != "" for row in rows)}
    ordered = [field for field in PREFERRED_COLUMNS if field in populated]
    ordered.extend(field for field in encountered if field in populated and field not in ordered)
    return ordered


def write_csv(path: Path, rows: list[dict[str, str]], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def source_projection(row: dict[str, str], source_fields: Iterable[str]) -> tuple[str, ...]:
    return tuple(row.get(field, "") for field in source_fields)


def verify_output(
    mission: str,
    path: Path,
    source_rows: list[dict[str, str]],
    expectations: dict[str, int],
) -> dict[str, object]:
    columns, output_rows = read_rows(path)
    key_fields = ("campaign_source", "campaign_condition_id", "trial_id")
    keys = [source_projection(row, key_fields) for row in output_rows]
    conditions: dict[str, int] = defaultdict(int)
    for row in output_rows:
        conditions[row["campaign_condition_id"]] += 1

    source_by_key = {
        source_projection(row, key_fields): row
        for row in source_rows
    }
    output_by_key = {
        source_projection(row, key_fields): row
        for row in output_rows
    }
    source_fields = {
        field
        for row in source_rows
        for field, value in row.items()
        if value != ""
    }
    mismatches = 0
    for key, source in source_by_key.items():
        output = output_by_key.get(key)
        if output is None or any(
            source.get(field, "") != output.get(field, "")
            for field in source_fields
            if source.get(field, "") != ""
        ):
            mismatches += 1

    timing_missing = sum(
        1 for row in output_rows
        if row.get("trial_status", "completed").lower() == "completed"
        and any(row.get(field, "") == "" for field in TIMING_FIELDS[1:])
    )
    max_step_field = "max_steps_any_robot" if mission == "bayesian" else "max_robot_steps"
    max_steps_missing = sum(
        1 for row in output_rows
        if row.get("trial_status", "completed").lower() == "completed"
        and row.get(max_step_field, "") == ""
    )
    expected_per_condition = expectations["expected_trials_per_condition"]
    wrong_condition_counts = {
        condition: count
        for condition, count in conditions.items()
        if count != expected_per_condition
    }
    checks = {
        "row_count": len(output_rows) == expectations["expected_rows"],
        "unique_keys": len(keys) == len(set(keys)),
        "condition_count": len(conditions) == expectations["expected_conditions"],
        "condition_trial_counts": not wrong_condition_counts,
        "source_rows_preserved": not mismatches and len(source_by_key) == len(output_by_key),
        "timing_complete_for_completed_trials": timing_missing == 0,
        "max_steps_complete": max_steps_missing == 0,
        "header_unique": len(columns) == len(set(columns)),
        "mission_consistent": {row.get("mission") for row in output_rows} == {mission},
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "output": str(path.relative_to(REPO_ROOT)),
        "rows": len(output_rows),
        "columns": len(columns),
        "conditions": len(conditions),
        "primary_rows": expectations["primary_rows"],
        "lowk_rows": expectations["lowk_rows"],
        "top_k_levels": sorted({row["top_k_level"] for row in output_rows}, key=lambda value: integer(
            next(row["top_k_max_cells"] for row in output_rows if row["top_k_level"] == value)
        )),
        "algorithms": sorted({row["algorithm"] for row in output_rows}, key=ALGORITHM_ORDER.get),
        "checks": checks,
        "source_mismatches": mismatches,
        "timing_missing": timing_missing,
        "max_steps_missing": max_steps_missing,
        "wrong_condition_counts": wrong_condition_counts,
    }


def build() -> None:
    missions = {
        "bayesian": (*load_bayesian_rows(), BAYESIAN_OUTPUT),
        "collaborative": (*load_collaborative_rows(), COLLABORATIVE_OUTPUT),
    }
    summaries: dict[str, object] = {}
    for mission, (rows, expectations, path) in missions.items():
        sort_rows(rows)
        write_csv(path, rows, output_columns(rows))
        summaries[mission] = verify_output(mission, path, rows, expectations)

    report = {
        "status": "passed"
        if all(summary["status"] == "passed" for summary in summaries.values())
        else "failed",
        "scope": "trial-level system performance with timing aggregates",
        "missions": summaries,
    }
    VERIFICATION_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    VERIFICATION_OUTPUT.write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    if report["status"] != "passed":
        raise SystemExit(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    build()
