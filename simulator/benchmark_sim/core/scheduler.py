from __future__ import annotations

import heapq
import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Type

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.comms.bus import MessageBus
from benchmark_sim.comms.models import CommunicationModel
from benchmark_sim.config import SimConfig
from .planner import AStarPlanner
from .robot import RobotShell, StepResult
from .types import TrialScenario
from .world import World


@dataclass(order=True)
class WakeEvent:
    time_s: float
    order: int
    rid: str = field(compare=False)


@dataclass
class TrialState:
    cfg: SimConfig
    scenario: TrialScenario
    world: World
    robots: Dict[str, RobotShell]
    bus: MessageBus
    planner: AStarPlanner
    clock_s: float = 0.0
    events_processed: int = 0
    done: bool = False


@dataclass
class ProcessedEvent:
    time_s: float
    rid: str
    result: StepResult


class AsyncTrialRunner:
    def __init__(
        self,
        cfg: SimConfig,
        allocator_cls: Type[AllocatorBase],
        comm_model: CommunicationModel,
        seed: int = 0,
    ) -> None:
        self.cfg = cfg
        self.allocator_cls = allocator_cls
        self.comm_model = comm_model
        self.rng = random.Random(seed)
        self.seed = seed

    def new_trial(self, scenario: TrialScenario) -> TrialState:
        bus = MessageBus(
            model=self.comm_model,
            delay_s=self.cfg.comm_delay_s,
            delay_jitter_s=self.cfg.comm_delay_jitter_s,
            rng=self.rng,
        )
        world = World(grid_size=self.cfg.grid_size, scenario=scenario)
        planner = AStarPlanner(
            grid_size=self.cfg.grid_size,
            move_cost=self.cfg.move_cost,
            turn_cost=self.cfg.turn_cost,
            visited_step_penalty=self.cfg.visited_step_penalty,
            reward_factor=self.cfg.reward_factor,
            min_step_cost=self.cfg.min_step_cost,
        )
        robots: Dict[str, RobotShell] = {}
        for rid in self.cfg.robot_ids:
            allocator = self.allocator_cls()
            robots[rid] = RobotShell(
                rid=rid,
                pos=self.cfg.start_positions[rid],
                heading=self.cfg.start_headings[rid],
                cfg=self.cfg,
                world=world,
                bus=bus,
                allocator=allocator,
            )
        return TrialState(cfg=self.cfg, scenario=scenario, world=world, robots=robots, bus=bus, planner=planner)

    def initial_queue(self, state: TrialState) -> List[WakeEvent]:
        q: List[WakeEvent] = []
        base = self.cfg.async_step_mean_s
        spread = max(0.0, self.cfg.async_initial_spread_s)
        span = base * spread
        for i, rid in enumerate(state.robots.keys()):
            t = self.rng.uniform(0.0, span) if span > 0 else 0.0
            heapq.heappush(q, WakeEvent(t, i, rid))
        return q

    def run_trial(self, scenario: TrialScenario, on_step: Optional[Callable[[TrialState, RobotShell, StepResult], None]] = None) -> TrialState:
        state = self.new_trial(scenario)
        if self._coverage_complete(state):
            state.done = True
            return state
        q = self.initial_queue(state)
        order = len(q)
        while q:
            processed, order = self.process_next_event(state, q, order)
            if processed is None:
                break
            robot = state.robots[processed.rid]
            result = processed.result
            if on_step:
                on_step(state, robot, result)
            if self._coverage_complete(state):
                state.done = True
            if state.done:
                break
        return state

    def process_next_event(
        self,
        state: TrialState,
        q: List[WakeEvent],
        order: int,
    ) -> tuple[Optional[ProcessedEvent], int]:
        if state.done or not q:
            return None, order

        event = heapq.heappop(q)
        state.clock_s = event.time_s
        state.bus.pump(state.clock_s)
        robot = state.robots[event.rid]
        result = robot.step(state.clock_s, state.planner)
        state.events_processed += 1

        processed = ProcessedEvent(time_s=state.clock_s, rid=event.rid, result=result)
        if self._coverage_complete(state):
            state.done = True
            return processed, order

        if result.found_target and getattr(state.cfg, "trial_mode", "clue_search") != "coverage":
            state.done = True
            # Guaranteed target message delivery path gets pumped for consistency.
            state.bus.pump(state.clock_s + self.cfg.comm_delay_s + 1e-9)
            return processed, order

        if state.events_processed >= self.cfg.debug_max_events:
            raise RuntimeError(
                f"Debug safety cap reached in trial {state.scenario.trial_id}. "
                "This usually means the allocator is stuck or returning no reachable goals."
            )

        interval = self._interval_for(result)
        order += 1
        heapq.heappush(q, WakeEvent(state.clock_s + interval, order, event.rid))
        return processed, order

    def _coverage_complete(self, state: TrialState) -> bool:
        if getattr(state.cfg, "trial_mode", "clue_search") != "coverage":
            return False
        return state.world.unique_cells_searched() >= state.cfg.grid_size * state.cfg.grid_size

    def step_until(
        self,
        state: TrialState,
        q: List[WakeEvent],
        order: int,
        time_limit_s: float,
        allow_overshoot: bool = True,
    ) -> tuple[List[ProcessedEvent], int]:
        processed_events: List[ProcessedEvent] = []
        progressed = False
        while q and not state.done:
            next_time = q[0].time_s
            if next_time > time_limit_s:
                if progressed or not allow_overshoot:
                    break
                time_limit_s = next_time
            processed, order = self.process_next_event(state, q, order)
            if processed is None:
                break
            processed_events.append(processed)
            progressed = True
        return processed_events, order

    def _interval_for(self, result: StepResult) -> float:
        if result.reason == "turn":
            return max(self.cfg.turn_quarter_s, 1e-3)
        if result.reason == "intent_sync":
            return max(self.cfg.collision_intent_settle_s, 1e-3)
        if result.reason == "path_failed":
            return max(self.cfg.replan_delay_s, 1e-3)
        if result.reason in {"no_goal", "idle"}:
            return max(self.cfg.no_goal_delay_s, 1e-3)
        if result.moved:
            return self._move_interval()
        return max(result.time_cost_s, 1e-3)

    def _move_interval(self) -> float:
        jitter = self.rng.uniform(-self.cfg.async_step_jitter_s, self.cfg.async_step_jitter_s)
        interval = self.cfg.async_step_mean_s + jitter
        interval = max(self.cfg.async_min_delay_s, interval)
        if self.cfg.async_max_delay_s > 0:
            interval = min(self.cfg.async_max_delay_s, interval)
        return max(interval, 1e-3)
