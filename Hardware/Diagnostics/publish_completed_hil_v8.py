from __future__ import annotations

import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
CAMPAIGN = (
    ROOT
    / "Results"
    / "HIL"
    / "AllocatorReplay"
    / "ActiveCampaigns"
    / "pololu_native_persistent_v8"
)
REPORTS = CAMPAIGN / "reports"
COLLABORATIVE_OUTPUT = (
    ROOT / "Results" / "HIL" / "Collaborative" / "CompletedTopKCampaignV8"
)
BAYESIAN_OUTPUT = ROOT / "Results" / "HIL" / "Bayesian" / "CompletedTopKCampaignV8"
COMBINED_OUTPUT = ROOT / "Results" / "HIL" / "CombinedTopKCampaignV8"

ALGORITHMS = {"ACBBA", "CBAA", "DGA", "DMCHBA", "HIPC", "PI"}
EXPECTED_LEVELS = {
    "bayesian": {"1%", "3%", "5%", "10%", "25%", "50%", "75%", "100%"},
    "collaborative": {"K=1", "K=2", "5%", "10%", "25%", "50%", "75%", "100%"},
}
REPORT_FILES = {
    "condition": "condition_metrics.csv",
    "system": "system_trial_metrics.csv",
    "robot": "robot_trial_metrics.csv",
    "failure": "trial_failures.csv",
    "watchdog": "watchdog_threshold_adjustments.csv",
}
BAYESIAN_FILES = {
    "condition": "bayesian_hil_topk_non_k1_condition_aggregate_metrics.csv",
    "system": "bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv",
    "robot": "bayesian_hil_topk_non_k1_robot_trial_timing_and_allocator_metrics.csv",
    "failure": "bayesian_hil_topk_non_k1_failed_trial_log.csv",
    "watchdog": "bayesian_hil_topk_non_k1_watchdog_threshold_adjustment_log.csv",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise RuntimeError(f"missing CSV header: {path}")
        return list(reader.fieldnames), list(reader)


def write_csv(path: Path, fields: list[str], rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise RuntimeError(message)


def validate(
    tables: dict[str, tuple[list[str], list[dict[str, str]]]],
    schedule: dict[str, Any],
) -> dict[str, Any]:
    conditions = tables["condition"][1]
    systems = tables["system"][1]
    robots = tables["robot"][1]
    failures = tables["failure"][1]

    assert_true(schedule.get("status") == "complete", "campaign schedule is not complete")
    assert_true(not schedule.get("active_trials"), "campaign still has active trials")
    assert_true(len(conditions) == 96, "expected 96 core HIL condition rows")
    assert_true(len({row["condition_id"] for row in conditions}) == 96, "condition IDs are not unique")
    assert_true(
        all(row.get("analysis_valid", "").lower() == "true" for row in conditions + systems + robots),
        "at least one published metrics row is analysis-invalid",
    )

    for mission, levels in EXPECTED_LEVELS.items():
        observed = {
            (row["algorithm"], row["top_k_level"])
            for row in conditions
            if row["mission"] == mission
        }
        expected = {(algorithm, level) for algorithm in ALGORITHMS for level in levels}
        assert_true(observed == expected, f"{mission} HIL condition coverage differs from the expected matrix")

    system_counts = Counter(row["condition_id"] for row in systems)
    failure_counts = Counter(row["condition_id"] for row in failures)
    for row in conditions:
        condition_id = row["condition_id"]
        assert_true(
            system_counts[condition_id] == int(row["completed_trial_count"]),
            f"completed-trial mismatch for {condition_id}",
        )
        assert_true(
            failure_counts[condition_id] == int(row["failed_trial_count"]),
            f"failed-trial mismatch for {condition_id}",
        )

    system_keys = {(row["condition_id"], row["trial_id"]) for row in systems}
    assert_true(len(system_keys) == len(systems), "successful system-trial keys are not unique")
    robot_counts: Counter[tuple[str, str]] = Counter(
        (row["condition_id"], row["trial_id"]) for row in robots
    )
    assert_true(set(robot_counts) == system_keys, "robot and system trial keys differ")
    assert_true(set(robot_counts.values()) == {4}, "every successful system trial must have four robot rows")

    arithmetic_rows = conditions + systems + robots
    for row in arithmetic_rows:
        values = [
            row["allocator_time_total_us"],
            row["candidate_filter_time_total_us"],
            row["allocator_exclusive_time_total_us"],
        ]
        assert_true(
            all(value != "" for value in values) or all(value == "" for value in values),
            "partially missing allocator timing fields",
        )
        if not values[0]:
            continue
        allocator = float(row["allocator_time_total_us"])
        candidate_filter = float(row["candidate_filter_time_total_us"])
        exclusive = float(row["allocator_exclusive_time_total_us"])
        assert_true(
            abs(allocator - candidate_filter - exclusive) <= 1e-6,
            "allocator timing arithmetic mismatch",
        )

    checks = {
        "schedule_status_complete": True,
        "schedule_active_trial_count": 0,
        "core_condition_count": len(conditions),
        "unique_condition_ids": len({row["condition_id"] for row in conditions}),
        "expected_condition_matrix_present": True,
        "successful_system_trial_keys_unique": True,
        "four_robot_rows_per_successful_system_trial": True,
        "condition_trial_counts_match_detail_tables": True,
        "all_metrics_rows_analysis_valid": True,
        "allocator_equals_filter_plus_exclusive_for_all_metrics_rows": True,
    }
    return checks


def coverage(rows: list[dict[str, str]]) -> dict[str, Any]:
    statuses = Counter(row["status"] for row in rows)
    return {
        "condition_count": len(rows),
        "complete_condition_count": statuses["complete"],
        "completed_with_trial_failures_condition_count": statuses[
            "completed_with_trial_failures"
        ],
        "timing_cap_stopped_condition_count": statuses["stopped"],
        "successful_system_trial_count": sum(int(row["completed_trial_count"]) for row in rows),
        "failed_trial_count": sum(int(row["failed_trial_count"]) for row in rows),
    }


def file_record(path: Path, description: str) -> dict[str, Any]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        row_count = sum(1 for _ in handle) - 1 if path.suffix == ".csv" else None
    record: dict[str, Any] = {
        "name": path.name,
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
        "description": description,
    }
    if row_count is not None:
        record["rows"] = row_count
    return record


def publish_dataset(
    output: Path,
    prefix: str,
    tables: dict[str, tuple[list[str], list[dict[str, str]]]],
    missions: set[str],
    readme: str,
    checks: dict[str, Any],
    schedule: dict[str, Any],
    source_tables: list[str],
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    names = {
        "condition": f"{prefix}_condition_aggregate_metrics.csv",
        "system": f"{prefix}_combined_trial_level_system_and_timing_results.csv",
        "robot": f"{prefix}_robot_trial_timing_and_allocator_metrics.csv",
        "failure": f"{prefix}_failed_trial_log.csv",
        "watchdog": f"{prefix}_watchdog_threshold_adjustment_log.csv",
    }
    descriptions = {
        "condition": "One row per terminal algorithm-by-Top-K condition, including status, trial counts, timings, and provenance.",
        "system": "One row per successful HIL system trial with performance, timing, call-count, and provenance fields.",
        "robot": "Four device-level timing rows for each successful HIL system trial.",
        "failure": "All failed HIL trial attempts in this publication scope.",
        "watchdog": "All repeated-state watchdog threshold adjustments in this publication scope.",
    }
    published_paths: dict[str, Path] = {}
    for key, name in names.items():
        fields, source_rows = tables[key]
        selected = [row for row in source_rows if row.get("mission") in missions]
        destination = output / name
        write_csv(destination, fields, selected)
        published_paths[key] = destination

    (output / "README.md").write_text(readme, encoding="utf-8")
    condition_rows = [
        row for row in tables["condition"][1] if row.get("mission") in missions
    ]
    manifest = {
        "schema": 1,
        "campaign_id": schedule["campaign_id"],
        "publication_status": "terminal_with_capped_conditions",
        "missions": sorted(missions),
        "coverage": {
            mission: coverage([row for row in condition_rows if row["mission"] == mission])
            for mission in sorted(missions)
        },
        "verification": checks,
        "source": {
            "campaign_path": "Results/HIL/AllocatorReplay/ActiveCampaigns/pololu_native_persistent_v8",
            "published_tables": source_tables,
            "campaign_manifest_sha256": sha256(CAMPAIGN / "manifest.json"),
            "schedule_sha256": sha256(CAMPAIGN / "schedule.json"),
            "schedule_status": schedule["status"],
            "raw_call_attempts_bytes": (REPORTS / "raw_call_attempts.csv").stat().st_size,
            "raw_call_phases_bytes": (REPORTS / "raw_call_phases.csv").stat().st_size,
        },
        "files": [
            file_record(published_paths[key], descriptions[key])
            for key in ("condition", "system", "robot", "failure", "watchdog")
        ],
        "notes": [
            "Terminal does not mean every condition produced successful trials.",
            "Stopped conditions are explicit 30-second-per-call timing-cap results, not missing files.",
            "Raw call-attempt and call-phase reports remain in the ignored local campaign because each exceeds GitHub's per-file size limit.",
        ],
    }
    manifest_name = f"{prefix}_dataset_verification_manifest.json"
    (output / manifest_name).write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )


def main() -> None:
    tables = {key: read_csv(REPORTS / name) for key, name in REPORT_FILES.items()}
    schedule = json.loads((CAMPAIGN / "schedule.json").read_text(encoding="utf-8"))
    checks = validate(tables, schedule)

    collaborative_readme = """# Collaborative HIL Top-K Campaign V8

This is the completed collaborative portion of the mixed V8 HIL campaign.
All 48 algorithm-by-Top-K conditions are terminal: 33 completed, seven
completed with logged trial failures, and eight stopped by the 30-second
allocator-call timing guard. The successful-trial table contains 390 system
trials and the robot table contains the corresponding 1,560 rows.

The condition table is authoritative for censored coverage. Six DGA conditions
(5%-100%) and two DMCHBA conditions (75%-100%) have no successful trial rows
because the guard classified their allocator timing as unusable. Ten failed
attempts are retained in the failure log, and 35 watchdog-threshold adjustments
are retained in the adjustment log.

Use the combined trial-level CSV for successful-trial analysis and join the
condition and failure tables when reporting coverage. The verification manifest
records counts, hashes, condition-matrix checks, four-robot row checks, and
timing-arithmetic checks.
"""
    publish_dataset(
        COLLABORATIVE_OUTPUT,
        "collaborative_hil_topk_all_k",
        tables,
        {"collaborative"},
        collaborative_readme,
        checks,
        schedule,
        [
            "Results/HIL/AllocatorReplay/ActiveCampaigns/pololu_native_persistent_v8/reports/*.csv (collaborative rows)"
        ],
    )

    combined_readme = """# Combined HIL Top-K Campaign V8

This directory is the analysis-ready combined publication for the terminal V8
HIL campaign. It contains all 96 core conditions: 48 Bayesian non-K=1
conditions and 48 collaborative conditions. There are 1,183 successful system
trials, 4,732 robot rows, 13 failed attempts, and 36 watchdog-threshold
adjustments.

The condition table must accompany trial-level analyses because 25 conditions
were deliberately stopped by the 30-second allocator-call timing guard and nine
additional conditions completed with logged trial failures. Bayesian K=1 was
explicitly excluded from the core HIL design; it is present only in archived
diagnostic records and is not silently merged here.

The two raw call-level reports total more than 1.2 GB and remain in the ignored
local campaign tree. The compact metrics, failures, adjustments, provenance,
and validation hashes needed for analysis are published here.
"""
    bayesian_tables = {
        key: read_csv(BAYESIAN_OUTPUT / name) for key, name in BAYESIAN_FILES.items()
    }
    combined_tables: dict[str, tuple[list[str], list[dict[str, str]]]] = {}
    for key, (fields, source_rows) in tables.items():
        bayesian_fields, bayesian_rows = bayesian_tables[key]
        assert_true(
            set(bayesian_fields).issubset(fields),
            f"{key} Bayesian fields are not a subset of the final report fields",
        )
        final_bayesian_by_trial = {
            (row.get("condition_id"), row.get("trial_id")): row
            for row in source_rows
            if row.get("mission") == "bayesian"
        }
        normalized_bayesian_rows = []
        for row in bayesian_rows:
            final_row = final_bayesian_by_trial.get(
                (row.get("condition_id"), row.get("trial_id")), {}
            )
            normalized_bayesian_rows.append(
                {field: row.get(field, final_row.get(field, "")) for field in fields}
            )
        collaborative_rows = [
            row for row in source_rows if row.get("mission") == "collaborative"
        ]
        combined_tables[key] = (fields, normalized_bayesian_rows + collaborative_rows)
    combined_checks = validate(combined_tables, schedule)

    publish_dataset(
        COMBINED_OUTPUT,
        "hil_topk_combined",
        combined_tables,
        {"bayesian", "collaborative"},
        combined_readme,
        combined_checks,
        schedule,
        [
            "Results/HIL/Bayesian/CompletedTopKCampaignV8/*.csv (immutable Bayesian publication)",
            "Results/HIL/AllocatorReplay/ActiveCampaigns/pololu_native_persistent_v8/reports/*.csv (collaborative rows)",
        ],
    )

    print("Published and verified completed V8 HIL results")


if __name__ == "__main__":
    main()
