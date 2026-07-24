from __future__ import annotations

import argparse
import csv
from pathlib import Path

from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig, edge_even_start_positions, generate_robot_ids
from benchmark_sim.core.scenario_loader import load_scenarios
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import TrialScenario
from benchmark_sim.metrics.export import write_outputs
from benchmark_sim.metrics.summary import build_rows


def parse_positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def parse_max_candidate_cells(value: str) -> int | None:
    if str(value).lower() == "all":
        return None
    return parse_positive_int(value)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run DCTA benchmark trials.")
    p.add_argument("--scenario-file", default=None, help="CSV/JSON scenario file from clue-object generator.")
    p.add_argument("--trial-mode", default="clue_search", choices=["clue_search", "coverage"])
    p.add_argument("--num-trials", type=int, default=1, help="Number of generated trials for coverage mode.")
    p.add_argument("--algorithm", required=True, help="Allocator class as module.path:ClassName.")
    p.add_argument("--algorithm-name", default=None, help="Optional display name for outputs.")
    p.add_argument("--comm-model", default="ideal", choices=["ideal", "bernoulli", "gilbert_elliot", "rayleigh_style"])
    p.add_argument("--comm-level", type=float, default=None,
                   help="Model-specific level: Bernoulli drop probability, GE long-run delivery probability, or Rayleigh sensitivity dBm.")
    p.add_argument("--max-trials", type=int, default=None)
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
    p.add_argument(
        "--max-candidate-cells",
        type=parse_max_candidate_cells,
        default=None,
        help="Top-K candidate prefilter for CBAA, ACBBA, PI, HIPC, DMCHBA, and DGA; use 'all' or omit for full candidates.",
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


def clue_locations_text(scenario: TrialScenario) -> str:
    return ";".join(f"({x},{y})" for x, y in scenario.clues)


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


def load_existing_outputs(out_dir: str | Path) -> tuple[list[dict], list[dict], list[dict]]:
    out = Path(out_dir)
    return (
        read_existing_csv(out / "trial_summary.csv"),
        read_existing_csv(out / "system_performance.csv"),
        read_existing_csv(out / "robot_performance.csv"),
    )


def failure_rows(
    scenario: TrialScenario,
    args: argparse.Namespace,
    cfg: SimConfig,
    algorithm_name: str,
    comm_level: str,
    scenario_file: str,
    exc: BaseException,
) -> tuple[dict, dict, list[dict]]:
    error_type = type(exc).__name__
    error_message = str(exc)
    common = {
        "trial_id": scenario.trial_id,
        "algorithm": algorithm_name,
        "comm_model": args.comm_model,
        "comm_level": comm_level,
        "grid_size": cfg.grid_size,
        "grid_cells": cfg.grid_size * cfg.grid_size,
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
    return trial_row, system_row, robot_rows


def main() -> None:
    args = parse_args()
    robot_ids = generate_robot_ids(args.num_robots)
    if args.robot_start_layout == "edge_even":
        start_positions = edge_even_start_positions(args.grid_size, robot_ids)
    else:  # argparse choices make this defensive branch unreachable.
        raise ValueError(f"unsupported robot start layout: {args.robot_start_layout}")
    cfg = SimConfig(
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
        max_candidate_cells=args.max_candidate_cells,
        debug_max_events=args.debug_max_events,
    )
    allocator_cls = load_allocator_class(args.algorithm)
    algorithm_name = args.algorithm_name or getattr(allocator_cls, "name", allocator_cls.__name__)
    comm_model = make_comm_model(args.comm_model, args.comm_level)
    comm_level = comm_model.level_label()
    scenarios = scenarios_for_args(args)
    output_config = {
        "sim_config": cfg.to_dict(),
        "algorithm": args.algorithm,
        "algorithm_name": algorithm_name,
        "comm_model": args.comm_model,
        "comm_level": comm_level,
        "scenario_file": str(Path(args.scenario_file)) if args.scenario_file else "",
        "trial_mode": args.trial_mode,
        "seed": args.seed,
        "retry_failed": bool(args.retry_failed),
    }

    trial_summary_rows, system_performance_rows, robot_performance_rows = load_existing_outputs(args.out_dir)
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
            print(
                f"retrying {len(failed_trial_ids)} failed trial(s) from {args.out_dir} "
                f"with debug_max_events={cfg.debug_max_events}",
                flush=True,
            )
    done_trial_ids = recorded_trial_ids(trial_summary_rows, system_performance_rows)
    total_scenarios = len(scenarios)
    scenario_ids = {scenario.trial_id for scenario in scenarios}
    resumed_count = len(done_trial_ids & scenario_ids)
    if resumed_count:
        print(
            f"resuming {args.out_dir}: {resumed_count}/{total_scenarios} trials already recorded",
            flush=True,
        )

    scenario_file = str(Path(args.scenario_file)) if args.scenario_file else ""
    for scenario in scenarios:
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
            trial_row["trial_status"] = "completed"
            system_row["trial_status"] = "completed"
            for row in robot_rows:
                row["trial_status"] = "completed"
        except Exception as exc:
            trial_row, system_row, robot_rows = failure_rows(
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
        done_trial_ids.add(scenario.trial_id)
        write_outputs(
            out_dir=args.out_dir,
            trial_summary_rows=trial_summary_rows,
            system_performance_rows=system_performance_rows,
            robot_performance_rows=robot_performance_rows,
            config=output_config,
            write_parquet=False,
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
    )
    print(f"outputs written to {args.out_dir}")


if __name__ == "__main__":
    main()
