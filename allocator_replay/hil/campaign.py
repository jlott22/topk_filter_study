from __future__ import annotations

import importlib
import json
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from allocator_replay.capture.tracer import (
    ALGORITHM_CLASSES,
    ALGORITHM_MODULES,
    _import_environment,
)
from allocator_replay.config.study import (
    BAYESIAN_SIM_ROOT,
    COLLABORATIVE_SIM_ROOT,
    Condition,
)
from allocator_replay.hil.bridge import (
    AuthoritativeBridge,
    HilConditionStop,
    JsonlJournal,
    make_proxy_allocator,
)
from allocator_replay.hil.manifest import (
    collaborative_scenario_for,
    load_manifest,
    save_schedule,
)
from allocator_replay.hil.memory import require_safe_commit
from allocator_replay.host.transport import ReplayTransportError


COLLABORATIVE_RUN_SEED = 839_171


def _condition(job: dict[str, Any]) -> Condition:
    return Condition(
        mission=str(job["mission"]),
        algorithm=str(job["algorithm"]),
        top_k_level=str(job["top_k_level"]),
        top_k_rate=float(job["top_k_rate"]),
        top_k_cells=int(job["top_k_cells"]),
    )


def _scenario(
    manifest: dict[str, Any],
    condition: Condition,
    trial_id: int,
):
    if condition.mission == "bayesian":
        if str(BAYESIAN_SIM_ROOT) not in sys.path:
            sys.path.insert(0, str(BAYESIAN_SIM_ROOT))
        from benchmark_sim.core.scenario_loader import load_scenarios

        path = Path(manifest["scenario_sources"]["bayesian"]["path"])
        scenarios = load_scenarios(path, max_trials=None)
    else:
        if str(COLLABORATIVE_SIM_ROOT) not in sys.path:
            sys.path.insert(0, str(COLLABORATIVE_SIM_ROOT))
        from known_visit_sim.core.scenario_loader import load_scenarios

        path = collaborative_scenario_for(manifest, trial_id)
        starts = {(0, 0), (0, 6), (0, 12), (0, 18)}
        scenarios = load_scenarios(path, 19, starts, None)
    try:
        return path, next(
            scenario for scenario in scenarios if int(scenario.trial_id) == trial_id
        )
    except StopIteration as exc:
        raise ValueError(
            f"scenario source does not contain {condition.mission} trial {trial_id}"
        ) from exc


def _decision_type(mission: str) -> type:
    if mission == "bayesian":
        from benchmark_sim.core.types import AllocationDecision
    else:
        from known_visit_sim.core.types import AllocationDecision
    return AllocationDecision


def _run_trial(
    root: Path,
    manifest: dict[str, Any],
    job: dict[str, Any],
    trial_id: int,
    generation: int,
    device: Any,
) -> dict[str, Any]:
    require_safe_commit()
    condition = _condition(job)
    cfg, _, comm, runner_class, base_class, _ = _import_environment(condition)
    scenario_path, scenario = _scenario(manifest, condition, trial_id)
    identity = device.identity or device.hello()
    journal = JsonlJournal(root / "journals" / f"{identity.device_id}.jsonl")
    bridge = AuthoritativeBridge(
        device=device,
        condition=condition,
        trial_id=trial_id,
        run_generation=generation,
        journal=journal,
    )
    proxy = make_proxy_allocator(base_class, bridge, _decision_type(condition.mission))
    seed = (
        trial_id * 1009
        if condition.mission == "bayesian"
        else COLLABORATIVE_RUN_SEED + trial_id * 1009
    )
    started = time.time()
    try:
        state = runner_class(
            cfg=cfg,
            allocator_cls=proxy,
            comm_model=comm,
            seed=seed,
        ).run_trial(scenario)
    finally:
        try:
            bridge.close()
        except ReplayTransportError:
            # The incomplete generation remains in the append-only journal.
            # A resumed campaign starts the trial again with a new generation.
            pass
    robot_steps = {
        str(rid): int(robot.counters.steps_total)
        for rid, robot in state.robots.items()
    }
    row = {
        "schema": 1,
        "record_type": "trial_complete",
        "campaign_mode": "pololu_authoritative_hil",
        "condition_id": condition.condition_id,
        "mission": condition.mission,
        "algorithm": condition.algorithm,
        "top_k_level": condition.top_k_level,
        "top_k_rate": condition.top_k_rate,
        "top_k_cells": condition.top_k_cells,
        "trial_id": trial_id,
        "run_generation": generation,
        "device_id": identity.device_id,
        "build_id": identity.build_id,
        "frequency_hz": identity.frequency_hz,
        "scenario_file": str(scenario_path.resolve()),
        "scenario_sha256": (
            manifest["scenario_sources"]["bayesian"]["sha256"]
            if condition.mission == "bayesian"
            else manifest["scenario_sources"]["collaborative"][str(trial_id)]["sha256"]
        ),
        "trial_status": "completed",
        "total_team_steps": sum(robot_steps.values()),
        "max_steps_any_robot": max(robot_steps.values()) if robot_steps else 0,
        "robot_steps": robot_steps,
        "events_processed": int(state.events_processed),
        "allocator_call_count": bridge.accepted_call_count,
        "wall_seconds": time.time() - started,
        "journaled_at": time.time(),
    }
    journal.append(row)
    return row


def _weight(job: dict[str, Any]) -> float:
    factors = {
        "CBAA": 1.0,
        "ACBBA": 2.0,
        "PI": 1.8,
        "HIPC": 2.2,
        "DMCHBA": 4.0,
        "DGA": 6.0,
    }
    mission = 2.0 if job["mission"] == "collaborative" else 1.0
    return mission * factors[job["algorithm"]] * max(1, int(job["top_k_cells"])) ** 2


class HilCampaignRunner:
    def __init__(self, root: Path, devices: list[Any]) -> None:
        self.root = root.resolve()
        self.devices = devices
        self.lock = threading.Lock()
        self.schedule = load_manifest(self.root)
        self.manifest = load_manifest(self.root, "manifest.json")
        self.schedule["status"] = "running"
        self.schedule.setdefault("devices", {})
        for job in self.schedule["jobs"]:
            if job["status"] == "running":
                job["status"] = "pending"
        save_schedule(self.root, self.schedule)

    def _claim(self, device_id: str) -> dict[str, Any] | None:
        with self.lock:
            eligible = [
                job
                for job in self.schedule["jobs"]
                if job["status"] == "pending"
                and (not job.get("device_id") or job["device_id"] == device_id)
            ]
            if not eligible:
                return None
            job = max(eligible, key=_weight)
            job["status"] = "running"
            job["device_id"] = device_id
            self.schedule["devices"][device_id] = {
                "status": "running",
                "condition_id": job["condition_id"],
                "last_seen_at": datetime.now(timezone.utc).isoformat(),
            }
            save_schedule(self.root, self.schedule)
            return job

    def _save_trial(self, job: dict[str, Any], trial_id: int) -> None:
        with self.lock:
            completed = set(int(item) for item in job["completed_trials"])
            completed.add(trial_id)
            job["completed_trials"] = sorted(completed)
            save_schedule(self.root, self.schedule)

    def _stop(self, job: dict[str, Any], reason: str) -> None:
        with self.lock:
            job["status"] = "stopped"
            job["stopped_reason"] = reason
            save_schedule(self.root, self.schedule)

    def _complete(self, job: dict[str, Any]) -> None:
        with self.lock:
            job["status"] = "complete"
            save_schedule(self.root, self.schedule)

    def _worker(self, device: Any) -> None:
        identity = device.identity or device.hello()
        restart_before_claim = False
        while True:
            if restart_before_claim:
                try:
                    restart = getattr(
                        device,
                        "restart_clean_worker",
                        None,
                    )
                    restarted = (
                        restart()
                        if callable(restart)
                        else identity
                    )
                    if (
                        restarted.device_id != identity.device_id
                        or (
                            getattr(identity, "build_id", None)
                            is not None
                            and getattr(restarted, "build_id", None)
                            != identity.build_id
                        )
                    ):
                        raise ReplayTransportError(
                            "device identity/build changed after clean restart"
                        )
                except ReplayTransportError:
                    with self.lock:
                        self.schedule["devices"].setdefault(
                            identity.device_id,
                            {},
                        )["status"] = "disconnected"
                        save_schedule(self.root, self.schedule)
                    return
            job = self._claim(identity.device_id)
            if job is None:
                break
            restart_before_claim = True
            generations = job.setdefault("trial_generations", {})
            try:
                for trial_id in job["trial_ids"]:
                    trial_id = int(trial_id)
                    if trial_id in {int(item) for item in job["completed_trials"]}:
                        continue
                    require_safe_commit()
                    generation = int(generations.get(str(trial_id), 0)) + 1
                    generations[str(trial_id)] = generation
                    with self.lock:
                        save_schedule(self.root, self.schedule)
                    _run_trial(
                        self.root,
                        self.manifest,
                        job,
                        trial_id,
                        generation,
                        device,
                    )
                    self._save_trial(job, trial_id)
                self._complete(job)
            except HilConditionStop as exc:
                self._stop(job, exc.reason)
            except ReplayTransportError:
                with self.lock:
                    job["status"] = "pending"
                    self.schedule["devices"][identity.device_id]["status"] = "disconnected"
                    save_schedule(self.root, self.schedule)
                return
            except Exception as exc:
                journal = JsonlJournal(
                    self.root / "journals" / f"{identity.device_id}.jsonl"
                )
                journal.append(
                    {
                        "schema": 1,
                        "record_type": "condition_error",
                        "condition_id": job["condition_id"],
                        "device_id": identity.device_id,
                        "error": f"{type(exc).__name__}: {exc}",
                        "journaled_at": time.time(),
                    }
                )
                self._stop(job, "simulator_or_host_failure")

    def run(self) -> dict[str, Any]:
        with ThreadPoolExecutor(max_workers=len(self.devices)) as executor:
            futures = [executor.submit(self._worker, device) for device in self.devices]
            for future in futures:
                future.result()
        with self.lock:
            statuses = {job["status"] for job in self.schedule["jobs"]}
            if statuses <= {"complete", "stopped"}:
                self.schedule["status"] = "complete"
            elif "running" not in statuses:
                self.schedule["status"] = "paused"
            save_schedule(self.root, self.schedule)
            return self.schedule
