from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter, deque
from pathlib import Path
from typing import Any

from .suite import DEFAULT_RUN_ROOT, load_manifest, read_csv_rows


def replace_arg(command: list[str], option: str, value: str) -> None:
    index = command.index(option)
    command[index + 1] = value


def safe(value: Any, limit: int = 4_000) -> str:
    text = repr(value)
    return text if len(text) <= limit else text[:limit] + "...<truncated>"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--condition-id", required=True)
    parser.add_argument("--trial-id", type=int, required=True)
    parser.add_argument("--event-cap", type=int, default=20_000)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    condition = next(
        row for row in load_manifest(args.run_root)
        if row["condition_id"] == args.condition_id
    )
    args.out_dir.mkdir(parents=True, exist_ok=False)
    scenario_rows = read_csv_rows(Path(condition["scenario_file"]))
    selected = [row for row in scenario_rows if int(row["trial_id"]) == args.trial_id]
    if len(selected) != 1:
        raise RuntimeError("expected exactly one selected scenario")
    fields = list(selected[0])
    scenario_path = args.out_dir / "scenario.csv"
    with scenario_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow(selected[0])

    simulator_root = Path(condition["working_directory"])
    sys.path.insert(0, str(simulator_root))
    from benchmark_sim.core import scheduler

    reason_counts: dict[str, Counter[str]] = {}
    recent: deque[dict[str, Any]] = deque(maxlen=100)
    original = scheduler.AsyncTrialRunner.process_next_event

    def snapshot(state: Any, error: BaseException) -> None:
        robots: dict[str, Any] = {}
        for rid, robot in state.robots.items():
            belief = getattr(robot, "belief", None)
            robots[rid] = {
                "position": list(robot.pos),
                "heading": robot.heading,
                "current_goal": robot.current_goal,
                "last_goal": robot.last_goal,
                "last_event": robot.last_event,
                "searched_count": len(getattr(belief, "searched", ())),
                "searched": sorted(getattr(belief, "searched", ())),
                "known_clues": safe(getattr(belief, "clues", None)),
                "path": safe(getattr(robot, "_path", getattr(robot, "path", None))),
                "temporary_invalid_tasks": safe(
                    getattr(robot, "_temporary_invalid_task_until", None)
                ),
                "allocator_state": safe(getattr(robot, "allocator", None).__dict__),
                "reason_counts": dict(reason_counts.get(rid, Counter())),
            }
        data = {
            "condition_id": condition["condition_id"],
            "trial_id": args.trial_id,
            "event_cap": args.event_cap,
            "error": f"{type(error).__name__}: {error}",
            "events_processed": state.events_processed,
            "clock_s": state.clock_s,
            "world_unique_cells_searched": state.world.unique_cells_searched(),
            "target": state.scenario.target,
            "robots": robots,
            "recent_events": list(recent),
        }
        (args.out_dir / "diagnostic_snapshot.json").write_text(
            json.dumps(data, indent=2, default=str) + "\n", encoding="utf-8"
        )

    def instrumented(self: Any, state: Any, queue: Any, order: int):
        try:
            processed, next_order = original(self, state, queue, order)
        except RuntimeError as exc:
            if "Debug safety cap reached" in str(exc):
                snapshot(state, exc)
            raise
        if processed is not None:
            reason_counts.setdefault(processed.rid, Counter())[
                processed.result.reason
            ] += 1
            robot = state.robots[processed.rid]
            recent.append({
                "event": state.events_processed,
                "clock_s": state.clock_s,
                "robot_id": processed.rid,
                "reason": processed.result.reason,
                "position": list(robot.pos),
                "goal": robot.current_goal,
                "searched_count": len(robot.belief.searched),
            })
        return processed, next_order

    scheduler.AsyncTrialRunner.process_next_event = instrumented
    command = list(json.loads(condition["command"]))
    replace_arg(command, "--scenario-file", str(scenario_path.resolve()))
    replace_arg(command, "--max-trials", "1")
    replace_arg(command, "--out-dir", str((args.out_dir / "sim_output").resolve()))
    replace_arg(command, "--debug-max-events", str(args.event_cap))
    sys.argv = ["benchmark_sim.run_trials", *command[3:]]
    from benchmark_sim import run_trials

    run_trials.main()


if __name__ == "__main__":
    main()
