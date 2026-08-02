"""Build lossless completed physical-trial CSVs from immutable robot captures.

The source files are never modified. Repeated trials and error rows are kept.
Some robot logs contain an unquoted ``(-1, -1)`` target location, which makes
the raw line appear to have 46 fields. The consolidated CSVs merge only those
two target-location fragments and record that repair in provenance columns.
"""

from __future__ import annotations

import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RESULT_ROOT = ROOT / "Results" / "Hardware" / "PhysicalTrials"
ROBOT_IDS = ("00", "01", "02", "03")
EXPECTED_ALGORITHMS = ("ACBBA", "CBAA", "DGA", "DMCHBA", "HIPC", "PI")
BATCHES = (
    ("initial_recovery", Path("metrics") / "onboard"),
    (
        "completed_trials",
        Path("captures") / "2026-08-02_completed_trials" / "raw_onboard",
    ),
)
PROVENANCE_HEADER = (
    "capture_batch",
    "source_path",
    "source_file_sha256",
    "source_line",
    "source_column_count",
    "structural_repair",
    "source_row_sha256",
)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def write_csv(path: Path, header: list[str], rows: list[list[str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)


def parse_source(
    path: Path,
    robot_id: str,
    algorithm: str,
    batch: str,
) -> tuple[list[str], list[dict[str, object]]]:
    data = path.read_bytes()
    digest = sha256_bytes(data)
    lines = data.splitlines(keepends=True)
    if not lines:
        raise RuntimeError(f"empty metrics file: {path}")
    header = next(csv.reader([lines[0].decode("utf-8").rstrip("\r\n")]))
    parsed: list[dict[str, object]] = []
    for line_number, raw_line in enumerate(lines[1:], 2):
        text = raw_line.decode("utf-8").rstrip("\r\n")
        if not text:
            continue
        raw_fields = next(csv.reader([text]))
        source_column_count = len(raw_fields)
        repair = "none"
        fields = raw_fields
        if (
            len(raw_fields) == len(header) + 1
            and raw_fields[1].startswith("(")
            and raw_fields[2].strip().endswith(")")
            and raw_fields[3] == algorithm
        ):
            fields = [raw_fields[0], raw_fields[1] + "," + raw_fields[2]] + raw_fields[3:]
            repair = "merge_unquoted_target_location"
        if len(fields) != len(header):
            raise RuntimeError(
                f"unhandled schema at {path}:{line_number}: "
                f"{source_column_count} fields, expected {len(header)}"
            )
        if fields[0] != robot_id or fields[2] != algorithm:
            raise RuntimeError(
                f"identity mismatch at {path}:{line_number}: "
                f"robot={fields[0]!r}, algorithm={fields[2]!r}"
            )
        parsed.append(
            {
                "robot_id": robot_id,
                "algorithm": algorithm,
                "batch": batch,
                "source_path": path.relative_to(RESULT_ROOT).as_posix(),
                "source_file_sha256": digest,
                "source_line": line_number,
                "source_column_count": source_column_count,
                "structural_repair": repair,
                "source_row_sha256": sha256_bytes(raw_line),
                "fields": fields,
            }
        )
    return header, parsed


def output_row(item: dict[str, object]) -> list[str]:
    return [
        str(item["batch"]),
        str(item["source_path"]),
        str(item["source_file_sha256"]),
        str(item["source_line"]),
        str(item["source_column_count"]),
        str(item["structural_repair"]),
        str(item["source_row_sha256"]),
        *[str(value) for value in item["fields"]],
    ]


def main() -> int:
    common_header: list[str] | None = None
    records: list[dict[str, object]] = []
    source_files: list[dict[str, object]] = []
    for robot_id in ROBOT_IDS:
        robot_root = RESULT_ROOT / "Robots" / f"robot_{robot_id}"
        for batch, relative_root in BATCHES:
            source_root = robot_root / relative_root
            for path in sorted(source_root.glob("metrics-log-*.txt")):
                algorithm = path.stem.removeprefix("metrics-log-")
                header, parsed = parse_source(path, robot_id, algorithm, batch)
                if common_header is None:
                    common_header = header
                elif header != common_header:
                    raise RuntimeError(f"header differs: {path}")
                records.extend(parsed)
                source_files.append(
                    {
                        "robot_id": robot_id,
                        "algorithm": algorithm,
                        "capture_batch": batch,
                        "path": path.relative_to(RESULT_ROOT).as_posix(),
                        "bytes": path.stat().st_size,
                        "data_rows": len(parsed),
                        "sha256": sha256_bytes(path.read_bytes()),
                    }
                )
    if common_header is None:
        raise RuntimeError("no metrics files found")

    final_header = [*PROVENANCE_HEADER, *common_header]
    output_root = RESULT_ROOT / "Completed"
    all_rows = [output_row(item) for item in records]
    write_csv(output_root / "all_robots_all_algorithms.csv", final_header, all_rows)

    for algorithm in EXPECTED_ALGORITHMS:
        rows = [output_row(item) for item in records if item["algorithm"] == algorithm]
        write_csv(output_root / "by_algorithm" / f"{algorithm}.csv", final_header, rows)

    coverage_rows: list[list[str]] = []
    by_pair = Counter((str(item["robot_id"]), str(item["algorithm"])) for item in records)
    files_by_pair = Counter(
        (str(item["robot_id"]), str(item["algorithm"])) for item in source_files
    )
    for robot_id in ROBOT_IDS:
        for algorithm in EXPECTED_ALGORITHMS:
            count = by_pair[(robot_id, algorithm)]
            coverage_rows.append(
                [
                    robot_id,
                    algorithm,
                    str(files_by_pair[(robot_id, algorithm)]),
                    str(count),
                    "captured" if count else "no_source_log_found",
                ]
            )
    write_csv(
        output_root / "coverage.csv",
        ["robot_id", "algorithm", "source_files", "data_rows", "status"],
        coverage_rows,
    )

    by_robot = Counter(str(item["robot_id"]) for item in records)
    by_algorithm = Counter(str(item["algorithm"]) for item in records)
    by_batch = Counter(str(item["batch"]) for item in records)
    repairs = Counter(str(item["structural_repair"]) for item in records)
    generated_files = sorted(
        path for path in output_root.rglob("*.csv") if path.is_file()
    )
    manifest = {
        "schema_version": 1,
        "description": "Lossless completed physical-trial metrics; repeats and error rows retained",
        "robot_ids": list(ROBOT_IDS),
        "expected_algorithms": list(EXPECTED_ALGORITHMS),
        "observed_algorithms": sorted(by_algorithm),
        "missing_source_logs": [
            {"robot_id": robot_id, "algorithm": algorithm}
            for robot_id in ROBOT_IDS
            for algorithm in EXPECTED_ALGORITHMS
            if by_pair[(robot_id, algorithm)] == 0
        ],
        "total_rows": len(records),
        "rows_by_robot": dict(sorted(by_robot.items())),
        "rows_by_algorithm": dict(sorted(by_algorithm.items())),
        "rows_by_capture_batch": dict(sorted(by_batch.items())),
        "structural_repairs": dict(sorted(repairs.items())),
        "duplicates_removed": 0,
        "error_rows_removed": 0,
        "source_files": source_files,
        "generated_csvs": [
            {
                "path": path.relative_to(RESULT_ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_bytes(path.read_bytes()),
            }
            for path in generated_files
        ],
    }
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
