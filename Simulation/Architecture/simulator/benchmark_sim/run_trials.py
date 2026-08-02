from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, LOGIC_REVISION, SimConfig, edge_even_start_positions, generate_robot_ids
from benchmark_sim.core.scenario_loader import load_scenarios, validate_scenarios
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario
from benchmark_sim.metrics.export import write_outputs
from benchmark_sim.metrics.summary import build_computational_performance_rows, build_rows


DEFAULT_TOPK_SCENARIO_MANIFEST_LOCK = (
    Path(__file__).resolve().parents[4]
    / "Results"
    / "Simulation"
    / "Bayesian"
    / "primary_topk_campaign"
    / "scenario_manifest.json"
)


def parse_positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def parse_nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a nonnegative integer") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be a nonnegative integer")
    return parsed


def parse_max_candidate_cells(value: str) -> int | None:
    if str(value).lower() == "all":
        return None
    return parse_positive_int(value)


def parse_top_k_rate(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number greater than 0 and at most 1") from exc
    if not (0.0 < parsed <= 1.0):
        raise argparse.ArgumentTypeError("must be greater than 0 and at most 1")
    return parsed


def resolve_top_k_settings(
    grid_size: int,
    max_candidate_cells: int | None,
    top_k_rate: float | None,
) -> tuple[int | None, float]:
    grid_cells = grid_size * grid_size
    if top_k_rate is not None:
        return max(1, int(grid_cells * top_k_rate + 0.5)), top_k_rate
    if max_candidate_cells is None:
        return None, 1.0
    return max_candidate_cells, max_candidate_cells / grid_cells


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scenario_selection_sha256(scenarios: list[TrialScenario]) -> str:
    selection = [
        {
            "trial_id": str(scenario.trial_id),
            "target": list(scenario.target) if scenario.target is not None else None,
            "clues": [list(clue) for clue in scenario.clues],
        }
        for scenario in scenarios
    ]
    payload = json.dumps(selection, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def parse_sha256(value: str) -> str:
    normalized = str(value).strip().lower()
    if re.fullmatch(r"[0-9a-f]{64}", normalized) is None:
        raise argparse.ArgumentTypeError("must be exactly 64 hexadecimal characters")
    return normalized


def enforce_expected_scenario_sha256(actual: str, expected: str | None) -> None:
    if expected is None:
        return
    try:
        normalized = parse_sha256(expected)
    except argparse.ArgumentTypeError as exc:
        raise ValueError(f"invalid expected scenario SHA-256: {expected!r}") from exc
    if str(actual).strip().lower() != normalized:
        raise ValueError(
            "scenario selection SHA-256 mismatch: "
            f"expected {normalized}, got {actual}"
        )


def resolve_scenario_manifest_lock(
    study_profile: str,
    override: str | Path | None,
    disabled: bool,
) -> Path | None:
    if disabled:
        return None
    if override is not None:
        return Path(override).expanduser().resolve()
    if study_profile == "topk_filter":
        return DEFAULT_TOPK_SCENARIO_MANIFEST_LOCK
    return None


def enforce_scenario_manifest_lock(
    path: str | Path,
    scenarios: list[TrialScenario],
    *,
    grid_size: int,
    logic_revision: str,
) -> str:
    """Create or verify the ordered scenario selection shared by conditions."""

    manifest_hash = scenario_selection_sha256(scenarios)
    record = {
        "schema": 1,
        "grid_size": int(grid_size),
        "logic_revision": str(logic_revision),
        "scenario_sha256": manifest_hash,
        "trial_ids": [str(scenario.trial_id) for scenario in scenarios],
    }
    lock_path = Path(path).expanduser().resolve()
    lock_path.parent.mkdir(parents=True, exist_ok=True)

    def verify_existing() -> str:
        try:
            existing = json.loads(lock_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ValueError(
                f"cannot read scenario manifest lock {lock_path}"
            ) from exc
        for field in (
            "schema",
            "grid_size",
            "logic_revision",
            "scenario_sha256",
            "trial_ids",
        ):
            if not isinstance(existing, dict) or existing.get(field) != record[field]:
                raise ValueError(
                    "selected scenarios do not match study manifest lock "
                    f"{lock_path} (field {field})"
                )
        return manifest_hash

    if lock_path.exists():
        return verify_existing()

    try:
        with lock_path.open("x", encoding="utf-8") as stream:
            json.dump(record, stream, sort_keys=True, indent=2)
            stream.write("\n")
    except FileExistsError:
        # Another condition may have established the shared lock concurrently.
        return verify_existing()
    return manifest_hash


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run DCTA benchmark trials.")
    p.add_argument("--scenario-file", default=None, help="CSV/JSON scenario file from clue-object generator.")
    p.add_argument(
        "--expected-scenario-sha256",
        type=parse_sha256,
        default=None,
        help=(
            "fail before execution unless the canonical ordered selected-scenario "
            "manifest has this SHA-256"
        ),
    )
    p.add_argument("--trial-mode", default="clue_search", choices=["clue_search", "coverage"])
    p.add_argument("--num-trials", type=int, default=1, help="Number of generated trials for coverage mode.")
    p.add_argument("--algorithm", required=True, help="Allocator class as module.path:ClassName.")
    p.add_argument("--algorithm-name", default=None, help="Optional display name for outputs.")
    p.add_argument("--comm-model", default="ideal", choices=["ideal", "bernoulli", "gilbert_elliot", "rayleigh_style"])
    p.add_argument("--comm-level", type=float, default=None,
                   help="Model-specific level: Bernoulli drop probability, GE long-run delivery probability, or Rayleigh sensitivity dBm.")
    p.add_argument("--max-trials", type=parse_positive_int, default=None)
    p.add_argument(
        "--trial-shard-count",
        type=parse_positive_int,
        default=1,
        help=(
            "Partition the validated scenario selection into this many "
            "round-robin shards (default: 1)."
        ),
    )
    p.add_argument(
        "--trial-shard-index",
        type=parse_nonnegative_int,
        default=0,
        help=(
            "Zero-based round-robin shard to execute. The full selected-scenario "
            "hash and manifest are preserved in every shard."
        ),
    )
    p.add_argument(
        "--debug-max-events",
        type=parse_positive_int,
        default=5_000,
        help="Abort a non-progressing trial after this many scheduled events (default: 5000).",
    )
    p.add_argument(
        "--retry-failed",
        action="store_true",
        help="Remove previously recorded failed rows and rerun only those trial IDs.",
    )
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out-dir", default="runs/default")
    p.add_argument("--grid-size", type=parse_positive_int, default=19)
    p.add_argument("--num-robots", type=parse_positive_int, default=4)
    p.add_argument("--robot-start-layout", default="edge_even", choices=["edge_even"])
    p.add_argument("--condition-id", default="")
    p.add_argument("--target-cells-per-robot", type=float, default=None)
    p.add_argument("--actual-cells-per-robot", type=float, default=None)
    p.add_argument("--target-decay-exp", type=float, default=1.0)
    p.add_argument(
        "--commitment-horizon",
        type=parse_positive_int,
        default=None,
        help="Override bundle/commitment horizon for ACBBA, PI, HIPC, DMCHBA, and DGA. CBAA ignores this.",
    )
    top_k_group = p.add_mutually_exclusive_group()
    top_k_group.add_argument(
        "--max-candidate-cells",
        type=parse_max_candidate_cells,
        default=None,
        help="Top-K candidate prefilter for CBAA, ACBBA, PI, HIPC, DMCHBA, and DGA; use 'all' or omit for full candidates.",
    )
    top_k_group.add_argument(
        "--top-k-rate",
        type=parse_top_k_rate,
        default=None,
        help="Top-K rate in (0, 1]; converted to a grid-cell limit with round-half-up.",
    )
    p.add_argument(
        "--study-profile",
        choices=["topk_filter", "custom"],
        default="topk_filter",
        help=(
            "topk_filter locks the canonical clue-search controls; "
            "custom explicitly opts into legacy sensitivity or coverage settings."
        ),
    )
    manifest_lock_group = p.add_mutually_exclusive_group()
    manifest_lock_group.add_argument(
        "--scenario-manifest-lock",
        default=None,
        help=(
            "override the shared ordered-scenario lock path; topk_filter defaults "
            "to Results/Simulation/Bayesian/primary_topk_campaign/"
            "scenario_manifest.json"
        ),
    )
    manifest_lock_group.add_argument(
        "--no-scenario-manifest-lock",
        action="store_true",
        help=(
            "explicitly disable the automatic cross-condition scenario lock "
            "for this run"
        ),
    )
    p.add_argument("--no-parquet", action="store_true", help="Deprecated; metric outputs are always CSV-only.")
    return p.parse_args()


def scenarios_for_args(args: argparse.Namespace) -> list[TrialScenario]:
    if args.trial_mode == "coverage":
        if args.num_trials < 1:
            raise ValueError("--num-trials must be at least 1 for coverage mode")
        return [
            TrialScenario(trial_id=i, target=None, clues=[], metadata={"generated": "coverage"})
            for i in range(args.num_trials)
        ]
    if not args.scenario_file:
        raise ValueError("--scenario-file is required for clue_search mode")
    return load_scenarios(args.scenario_file, max_trials=args.max_trials)


def select_scenario_shard(
    scenarios: list[TrialScenario],
    shard_count: int,
    shard_index: int,
) -> list[TrialScenario]:
    if shard_count <= 0:
        raise ValueError("--trial-shard-count must be positive")
    if not 0 <= shard_index < shard_count:
        raise ValueError(
            "--trial-shard-index must be in "
            f"[0, {shard_count}), got {shard_index}"
        )
    return scenarios[shard_index::shard_count]


def clue_locations_text(scenario: TrialScenario) -> str:
    return ";".join(f"({x},{y})" for x, y in scenario.clues)


def apply_study_profile(args: argparse.Namespace) -> None:
    if args.study_profile == "custom":
        return

    violations: list[str] = []
    if args.trial_mode != "clue_search":
        violations.append("--trial-mode must be clue_search")
    if args.grid_size != 19:
        violations.append("--grid-size must be 19")
    if args.num_robots != 4:
        violations.append("--num-robots must be 4")
    if args.robot_start_layout != "edge_even":
        violations.append("--robot-start-layout must be edge_even")
    if args.commitment_horizon not in (None, 3):
        violations.append("--commitment-horizon must be 3")
    if violations:
        raise ValueError(
            "topk_filter study profile requires canonical controls: "
            + "; ".join(violations)
            + ". Use --study-profile custom for a non-study run."
        )

    # Make the effective default explicit in config and output provenance.
    args.commitment_horizon = 3


def read_existing_csv(path: str | Path) -> list[dict]:
    csv_path = Path(path)
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        return []
    with csv_path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def parse_trial_id(row: dict) -> int | None:
    value = row.get("trial_id")
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def recorded_trial_ids(*row_groups: list[dict]) -> set[int]:
    recorded: set[int] = set()
    for rows in row_groups:
        for row in rows:
            trial_id = parse_trial_id(row)
            if trial_id is None:
                continue
            status = str(row.get("trial_status", "")).strip().lower()
            if status in {"", "completed", "failed"}:
                recorded.add(trial_id)
    return recorded


def load_existing_outputs(out_dir: str | Path) -> tuple[list[dict], list[dict], list[dict], list[dict]]:
    out = Path(out_dir)
    return (
        read_existing_csv(out / "trial_summary.csv"),
        read_existing_csv(out / "system_performance.csv"),
        read_existing_csv(out / "robot_performance.csv"),
        read_existing_csv(out / "computational_performance.csv"),
    )


def failure_rows(
    scenario: TrialScenario,
    args: argparse.Namespace,
    cfg: SimConfig,
    algorithm_name: str,
    comm_level: str,
    scenario_file: str,
    exc: BaseException,
) -> tuple[dict, dict, list[dict], list[dict]]:
    error_type = type(exc).__name__
    error_message = str(exc)
    common = {
        "trial_id": scenario.trial_id,
        "logic_revision": cfg.logic_revision,
        "study_profile": cfg.study_profile,
        "scenario_file_sha256": cfg.scenario_file_sha256,
        "scenario_selection_sha256": cfg.scenario_selection_sha256,
        "algorithm": algorithm_name,
        "comm_model": args.comm_model,
        "comm_level": comm_level,
        "grid_size": cfg.grid_size,
        "grid_cells": cfg.grid_size * cfg.grid_size,
        "top_k_rate": cfg.top_k_rate,
        "top_k_max_cells": (
            cfg.max_candidate_cells
            if cfg.max_candidate_cells is not None
            else cfg.grid_size * cfg.grid_size
        ),
        "robot_count": len(cfg.robot_ids),
        "target_cells_per_robot": cfg.target_cells_per_robot,
        "actual_cells_per_robot": cfg.actual_cells_per_robot,
        "condition_id": cfg.condition_id,
        "scenario_file": scenario_file,
        "trial_mode": cfg.trial_mode,
        "trial_status": "failed",
        "failure_type": error_type,
        "failure_message": error_message,
    }
    trial_row = {
        **common,
        "target_x": scenario.target[0] if scenario.target else "",
        "target_y": scenario.target[1] if scenario.target else "",
        "clue_locations": clue_locations_text(scenario),
        "first_clue_robot": "",
        "first_clue_x": "",
        "first_clue_y": "",
        "robot_start_locations": ";".join(
            f"{rid}:({cfg.start_positions[rid][0]},{cfg.start_positions[rid][1]})"
            for rid in cfg.robot_ids
        ),
        "robot_end_locations": "",
        "clues_detected_by_robot": "",
        "target_found_by_robot": "",
    }
    system_row = {
        **common,
        "total_team_steps": "",
        "steps_before_first_clue": "",
        "post_clue_steps_to_find": "",
        "unique_cells_searched": "",
        "system_revisits": "",
        "task_cell_replans_total": "",
        "path_replans_total": "",
        "collision_prevention_events": "",
        "messages_sent_total": "",
        "messages_delivered_total": "",
        "messages_dropped_total": "",
        "debug_max_events": cfg.debug_max_events,
    }
    robot_rows = [
        {
            **common,
            "robot_id": rid,
            "steps_total": "",
            "steps_after_first_clue": "",
            "unique_cells_contributed": "",
            "system_revisits_by_robot": "",
            "task_cell_replans": "",
            "path_replans": "",
            "collision_prevention_events": "",
            "messages_sent": "",
            "messages_delivered_to_robot": "",
            "messages_dropped_to_robot": "",
        }
        for rid in cfg.robot_ids
    ]
    computational_rows = [
        {
            **common,
            "robot_id": rid,
            "host_trial_runtime_ms": "",
            "allocator_calls": "",
            "allocator_calls_pre_clue": "",
            "allocator_calls_post_clue": "",
            "allocator_time_ms_total": "",
            "allocator_time_ms_mean": "",
            "allocator_time_ms_median": "",
            "allocator_time_ms_p95": "",
            "allocator_time_ms_max": "",
            "allocator_solve_time_ms_total": "",
            "allocator_solve_time_ms_mean": "",
            "allocator_solve_time_ms_median": "",
            "allocator_solve_time_ms_p95": "",
            "allocator_solve_time_ms_max": "",
            "allocator_time_pct": "",
            "candidate_filter_calls": "",
            "candidate_filter_time_ms_total": "",
            "candidate_filter_time_ms_mean": "",
            "candidate_filter_time_ms_max": "",
            "allocator_time_ms_pre_clue_total": "",
            "allocator_time_ms_post_clue_total": "",
            "allocator_host_runtime_pct": "",
        }
        for rid in cfg.robot_ids
    ]
    return trial_row, system_row, robot_rows, computational_rows


def main() -> None:
    args = parse_args()
    apply_study_profile(args)
    scenario_file_hash = file_sha256(args.scenario_file) if args.scenario_file else ""
    top_k_max_cells, top_k_rate = resolve_top_k_settings(
        args.grid_size,
        args.max_candidate_cells,
        args.top_k_rate,
    )
    robot_ids = generate_robot_ids(args.num_robots)
    if args.robot_start_layout == "edge_even":
        start_positions = edge_even_start_positions(args.grid_size, robot_ids)
    else:  # argparse choices make this defensive branch unreachable.
        raise ValueError(f"unsupported robot start layout: {args.robot_start_layout}")
    cfg = SimConfig(
        logic_revision=LOGIC_REVISION,
        study_profile=args.study_profile,
        scenario_file_sha256=scenario_file_hash,
        trial_mode=args.trial_mode,
        grid_size=args.grid_size,
        robot_ids=robot_ids,
        start_positions=start_positions,
        start_headings={rid: EAST for rid in robot_ids},
        robot_start_layout=args.robot_start_layout,
        condition_id=args.condition_id,
        target_cells_per_robot=args.target_cells_per_robot,
        actual_cells_per_robot=args.actual_cells_per_robot,
        target_decay_exp=args.target_decay_exp,
        write_parquet=False,
        commitment_horizon=args.commitment_horizon,
        max_candidate_cells=top_k_max_cells,
        top_k_rate=top_k_rate,
        debug_max_events=args.debug_max_events,
    )
    allocator_cls = load_allocator_class(args.algorithm)
    algorithm_name = args.algorithm_name or getattr(allocator_cls, "name", allocator_cls.__name__)
    comm_model = make_comm_model(args.comm_model, args.comm_level)
    comm_level = comm_model.level_label()
    scenarios = scenarios_for_args(args)
    validate_scenarios(
        scenarios,
        grid_size=cfg.grid_size,
        start_positions=cfg.start_positions,
        trial_mode=cfg.trial_mode,
        expected_count=args.num_trials if args.trial_mode == "coverage" else args.max_trials,
    )
    cfg.scenario_selection_sha256 = scenario_selection_sha256(scenarios)
    enforce_expected_scenario_sha256(
        cfg.scenario_selection_sha256,
        args.expected_scenario_sha256,
    )
    scenario_manifest_lock = resolve_scenario_manifest_lock(
        args.study_profile,
        args.scenario_manifest_lock,
        args.no_scenario_manifest_lock,
    )
    if scenario_manifest_lock is not None:
        locked_hash = enforce_scenario_manifest_lock(
            scenario_manifest_lock,
            scenarios,
            grid_size=cfg.grid_size,
            logic_revision=cfg.logic_revision,
        )
        if locked_hash != cfg.scenario_selection_sha256:
            raise ValueError(
                "scenario manifest lock returned a different selection SHA-256"
            )
    output_config = {
        "sim_config": cfg.to_dict(),
        "logic_revision": cfg.logic_revision,
        "scenario_file_sha256": cfg.scenario_file_sha256,
        "scenario_selection_sha256": cfg.scenario_selection_sha256,
        "expected_scenario_sha256": args.expected_scenario_sha256 or "",
        "scenario_manifest_lock": (
            str(scenario_manifest_lock) if scenario_manifest_lock is not None else ""
        ),
        "algorithm": args.algorithm,
        "algorithm_name": algorithm_name,
        "comm_model": args.comm_model,
        "comm_level": comm_level,
        "scenario_file": str(Path(args.scenario_file)) if args.scenario_file else "",
        "trial_mode": args.trial_mode,
        "study_profile": args.study_profile,
        "seed": args.seed,
        "retry_failed": bool(args.retry_failed),
        "trial_shard_count": args.trial_shard_count,
        "trial_shard_index": args.trial_shard_index,
    }
    run_scenarios = select_scenario_shard(
        scenarios,
        args.trial_shard_count,
        args.trial_shard_index,
    )

    (
        trial_summary_rows,
        system_performance_rows,
        robot_performance_rows,
        computational_performance_rows,
    ) = load_existing_outputs(args.out_dir)
    if args.retry_failed:
        failed_trial_ids = {
            trial_id
            for rows in (trial_summary_rows, system_performance_rows, robot_performance_rows)
            for row in rows
            if str(row.get("trial_status", "")).strip().lower() == "failed"
            if (trial_id := parse_trial_id(row)) is not None
        }
        if failed_trial_ids:
            trial_summary_rows = [
                row for row in trial_summary_rows if parse_trial_id(row) not in failed_trial_ids
            ]
            system_performance_rows = [
                row for row in system_performance_rows if parse_trial_id(row) not in failed_trial_ids
            ]
            robot_performance_rows = [
                row for row in robot_performance_rows if parse_trial_id(row) not in failed_trial_ids
            ]
            computational_performance_rows = [
                row for row in computational_performance_rows if parse_trial_id(row) not in failed_trial_ids
            ]
            print(
                f"retrying {len(failed_trial_ids)} failed trial(s) from {args.out_dir} "
                f"with debug_max_events={cfg.debug_max_events}",
                flush=True,
            )
    done_trial_ids = recorded_trial_ids(trial_summary_rows, system_performance_rows)
    total_scenarios = len(run_scenarios)
    scenario_ids = {scenario.trial_id for scenario in run_scenarios}
    resumed_count = len(done_trial_ids & scenario_ids)
    if resumed_count:
        print(
            f"resuming {args.out_dir}: {resumed_count}/{total_scenarios} trials already recorded",
            flush=True,
        )

    scenario_file = str(Path(args.scenario_file)) if args.scenario_file else ""
    for scenario in run_scenarios:
        if scenario.trial_id in done_trial_ids:
            print(f"skipping recorded trial {scenario.trial_id}", flush=True)
            continue
        try:
            runner = AsyncTrialRunner(cfg=cfg, allocator_cls=allocator_cls, comm_model=comm_model, seed=args.seed + scenario.trial_id * 1009)
            state = runner.run_trial(scenario)
            trial_row, system_row, robot_rows = build_rows(
                state=state,
                algorithm_name=algorithm_name,
                comm_model=args.comm_model,
                comm_level=comm_level,
                scenario_file=scenario_file,
            )
            computational_rows = build_computational_performance_rows(
                state=state,
                algorithm_name=algorithm_name,
                comm_model=args.comm_model,
                comm_level=comm_level,
                scenario_file=scenario_file,
            )
            trial_row["trial_status"] = "completed"
            system_row["trial_status"] = "completed"
            for row in robot_rows:
                row["trial_status"] = "completed"
            for row in computational_rows:
                row["trial_status"] = "completed"
        except Exception as exc:
            trial_row, system_row, robot_rows, computational_rows = failure_rows(
                scenario=scenario,
                args=args,
                cfg=cfg,
                algorithm_name=algorithm_name,
                comm_level=comm_level,
                scenario_file=scenario_file,
                exc=exc,
            )
        trial_summary_rows.append(trial_row)
        system_performance_rows.append(system_row)
        robot_performance_rows.extend(robot_rows)
        computational_performance_rows.extend(computational_rows)
        done_trial_ids.add(scenario.trial_id)
        write_outputs(
            out_dir=args.out_dir,
            trial_summary_rows=trial_summary_rows,
            system_performance_rows=system_performance_rows,
            robot_performance_rows=robot_performance_rows,
            config=output_config,
            write_parquet=False,
            computational_performance_rows=computational_performance_rows,
        )
        if trial_row.get("trial_status") == "failed":
            print(
                f"failed trial {scenario.trial_id}: "
                f"{trial_row['failure_type']}: {trial_row['failure_message']}",
                flush=True,
            )
        elif args.trial_mode == "coverage":
            print(
                f"completed trial {scenario.trial_id}: "
                f"steps={system_row['total_team_steps']} unique={system_row['unique_cells_searched']}",
                flush=True,
            )
        else:
            print(
                f"completed trial {scenario.trial_id}: steps={system_row['total_team_steps']} "
                f"post_clue={system_row['post_clue_steps_to_find']}",
                flush=True,
            )

    write_outputs(
        out_dir=args.out_dir,
        trial_summary_rows=trial_summary_rows,
        system_performance_rows=system_performance_rows,
        robot_performance_rows=robot_performance_rows,
        config=output_config,
        write_parquet=False,
        computational_performance_rows=computational_performance_rows,
    )
    print(f"outputs written to {args.out_dir}")


if __name__ == "__main__":
    main()
