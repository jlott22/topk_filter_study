from __future__ import annotations

from typing import Dict, List

from benchmark_sim.core.scheduler import TrialState
from benchmark_sim.core.types import Cell


def _cell_list_str(cells: List[Cell]) -> str:
    return ";".join(f"({x},{y})" for x, y in cells)


def _loc_dict_str(locs: Dict[str, Cell]) -> str:
    return ";".join(f"{rid}:({cell[0]},{cell[1]})" for rid, cell in sorted(locs.items()))


def _robot_counts_dict_str(counts: Dict[str, int]) -> str:
    return ";".join(f"{rid}:{count}" for rid, count in sorted(counts.items()))


def _counts_dict_str(counts: Dict[str, int]) -> str:
    return ";".join(f"{key}:{count}" for key, count in sorted(counts.items()))


def gini(values: List[float]) -> float:
    vals = [float(v) for v in values if v >= 0]
    n = len(vals)
    if n == 0:
        return 0.0
    total = sum(vals)
    if total == 0:
        return 0.0
    vals.sort()
    weighted = sum((i + 1) * v for i, v in enumerate(vals))
    return (2.0 * weighted) / (n * total) - (n + 1.0) / n


def build_rows(state: TrialState, algorithm_name: str, comm_model: str, comm_level: str, scenario_file: str) -> tuple[dict, dict, list[dict]]:
    world = state.world
    robots = state.robots
    bus_counts = state.bus.counters

    robot_start_locations = {rid: state.cfg.start_positions[rid] for rid in state.cfg.robot_ids}
    robot_end_locations = {rid: rb.pos for rid, rb in robots.items()}

    clues_detected_by_robot: Dict[str, int] = {rid: 0 for rid in robots}
    for clue in world.clue_set:
        rec = world.visits.get(clue)
        if rec:
            for rid in rec.by_robot:
                clues_detected_by_robot[rid] += 1

    target = world.target
    target_found_by_robot = world.target_found_by or ""
    first_clue = world.first_clue_cell

    common_metadata = {
        "trial_id": world.scenario.trial_id,
        "algorithm": algorithm_name,
        "comm_model": comm_model,
        "comm_level": comm_level,
        "grid_size": state.cfg.grid_size,
        "grid_cells": state.cfg.grid_size * state.cfg.grid_size,
        "robot_count": len(state.cfg.robot_ids),
        "target_cells_per_robot": state.cfg.target_cells_per_robot,
        "actual_cells_per_robot": state.cfg.actual_cells_per_robot,
        "condition_id": state.cfg.condition_id,
        "scenario_file": scenario_file,
    }

    trial_summary = {
        **common_metadata,
        "trial_mode": getattr(state.cfg, "trial_mode", "clue_search"),
        "target_x": target[0] if target is not None else "",
        "target_y": target[1] if target is not None else "",
        "clue_locations": _cell_list_str(world.clues),
        "first_clue_robot": world.first_clue_robot or "",
        "first_clue_x": first_clue[0] if first_clue else "",
        "first_clue_y": first_clue[1] if first_clue else "",
        "robot_start_locations": _loc_dict_str(robot_start_locations),
        "robot_end_locations": _loc_dict_str(robot_end_locations),
        "clues_detected_by_robot": _robot_counts_dict_str(clues_detected_by_robot),
        "target_found_by_robot": target_found_by_robot,
    }

    total_team_steps = sum(rb.counters.steps_total for rb in robots.values())
    steps_after_first = sum(rb.counters.steps_after_first_clue for rb in robots.values())
    steps_before_first = max(0, total_team_steps - steps_after_first)
    unique_cells = world.unique_cells_searched()
    sent_total = bus_counts.sent_total
    delivered_total = bus_counts.delivered_total
    dropped_total = bus_counts.dropped_total
    unprotected_attempted_deliveries = bus_counts.unprotected_delivered_total + dropped_total
    message_drop_fraction = (
        dropped_total / unprotected_attempted_deliveries
        if unprotected_attempted_deliveries
        else 0.0
    )
    messages_per_unique = sent_total / unique_cells if unique_cells else 0.0
    messages_per_post_clue_step = (
        bus_counts.post_clue_sent_total / steps_after_first
        if steps_after_first
        else 0.0
    )
    allocation_messages_per_step = (
        bus_counts.allocation_sent_total / total_team_steps
        if total_team_steps
        else 0.0
    )
    allocation_messages_per_post_clue_step = (
        bus_counts.post_clue_allocation_sent_total / steps_after_first
        if steps_after_first
        else 0.0
    )
    allocation_messages_per_unique_cell = (
        bus_counts.allocation_sent_total / unique_cells
        if unique_cells
        else 0.0
    )

    robot_steps = [rb.counters.steps_total for rb in robots.values()]
    robot_unique_cells = [rb.counters.unique_cells_contributed for rb in robots.values()]
    robot_messages = [bus_counts.sent_by_robot.get(rid, 0) for rid in robots]
    # Historical Gini fields were step-based, not unique-cell balance.
    # Recompute old runs separately if you need this metric for archived outputs.
    workload_gini_unique_cells_contributed = gini(robot_unique_cells)

    system_performance = {
        **common_metadata,
        "trial_mode": getattr(state.cfg, "trial_mode", "clue_search"),
        "total_team_steps": total_team_steps,
        "steps_before_first_clue": steps_before_first,
        "post_clue_steps_to_find": steps_after_first,
        "unique_cells_searched": unique_cells,
        "system_revisits": world.system_revisits(),
        "task_cell_replans_total": sum(rb.counters.task_cell_replans for rb in robots.values()),
        "path_replans_total": sum(rb.counters.path_replans for rb in robots.values()),
        "collision_prevention_events": sum(rb.counters.collision_prevention_events for rb in robots.values()),
        "messages_sent_total": sent_total,
        "messages_delivered_total": delivered_total,
        "messages_dropped_total": dropped_total,
        "protected_messages_sent_total": bus_counts.protected_sent_total,
        "unprotected_messages_sent_total": bus_counts.unprotected_sent_total,
        "core_messages_sent_total": bus_counts.core_sent_total,
        "allocation_messages_sent_total": bus_counts.allocation_sent_total,
        "post_clue_messages_sent_total": bus_counts.post_clue_sent_total,
        "post_clue_allocation_messages_sent_total": bus_counts.post_clue_allocation_sent_total,
        "message_drop_fraction": message_drop_fraction,
        "messages_per_unique_cell": messages_per_unique,
        "messages_per_post_clue_step": messages_per_post_clue_step,
        "allocation_messages_per_step": allocation_messages_per_step,
        "allocation_messages_per_post_clue_step": allocation_messages_per_post_clue_step,
        "allocation_messages_per_unique_cell": allocation_messages_per_unique_cell,
        "messages_sent_by_topic": _counts_dict_str(bus_counts.sent_by_topic),
        "max_steps_any_robot": max(robot_steps) if robot_steps else 0,
        "max_messages_any_robot": max(robot_messages) if robot_messages else 0,
        "workload_gini_unique_cells_contributed": workload_gini_unique_cells_contributed,
    }

    robot_rows: List[dict] = []
    for rid, rb in sorted(robots.items()):
        c = rb.counters
        robot_rows.append({
            **common_metadata,
            "trial_mode": getattr(state.cfg, "trial_mode", "clue_search"),
            "robot_id": rid,
            "steps_total": c.steps_total,
            "steps_after_first_clue": c.steps_after_first_clue,
            "unique_cells_contributed": c.unique_cells_contributed,
            "system_revisits_by_robot": c.system_revisits_by_robot,
            "task_cell_replans": c.task_cell_replans,
            "path_replans": c.path_replans,
            "collision_prevention_events": c.collision_prevention_events,
            "messages_sent": bus_counts.sent_by_robot.get(rid, 0),
            "protected_messages_sent": bus_counts.protected_sent_by_robot.get(rid, 0),
            "unprotected_messages_sent": bus_counts.unprotected_sent_by_robot.get(rid, 0),
            "core_messages_sent": bus_counts.core_sent_by_robot.get(rid, 0),
            "allocation_messages_sent": bus_counts.allocation_sent_by_robot.get(rid, 0),
            "post_clue_messages_sent": bus_counts.post_clue_sent_by_robot.get(rid, 0),
            "messages_sent_by_topic": _counts_dict_str(bus_counts.sent_by_robot_topic.get(rid, {})),
            "messages_delivered_to_robot": bus_counts.delivered_to_robot.get(rid, 0),
            "messages_dropped_to_robot": bus_counts.dropped_to_robot.get(rid, 0),
        })
    return trial_summary, system_performance, robot_rows
