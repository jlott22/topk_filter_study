from __future__ import annotations

from pathlib import Path
from typing import Any

from allocator_replay.hil.manifest import load_manifest


def hil_status(root: Path) -> dict[str, Any]:
    schedule = load_manifest(root)
    counts: dict[str, int] = {}
    completed_trials = 0
    planned_trials = 0
    active: list[dict[str, Any]] = []
    for job in schedule["jobs"]:
        status = str(job["status"])
        counts[status] = counts.get(status, 0) + 1
        completed_trials += len(job["completed_trials"])
        planned_trials += len(job["trial_ids"])
        if status == "running":
            active.append(
                {
                    "device_id": job.get("device_id", ""),
                    "condition_id": job["condition_id"],
                    "completed_trials": len(job["completed_trials"]),
                    "planned_trials": len(job["trial_ids"]),
                }
            )
    return {
        "campaign_id": schedule["campaign_id"],
        "status": schedule.get("status", "unknown"),
        "condition_counts": counts,
        "completed_trials": completed_trials,
        "planned_trials": planned_trials,
        "progress": completed_trials / planned_trials if planned_trials else 0,
        "active": active,
        "devices": schedule.get("devices", {}),
        "updated_at": schedule.get("updated_at", ""),
    }
