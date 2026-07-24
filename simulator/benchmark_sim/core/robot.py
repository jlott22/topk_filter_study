from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Set, Tuple

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.comms.bus import MessageBus
from benchmark_sim.comms.message import Message, topic_for
from benchmark_sim.config import SimConfig
from benchmark_sim.metrics.counters import RobotCounters
from .belief import BeliefMap
from .planner import AStarPlanner
from .types import Cell, DIRS4, Heading, Observation, in_bounds
from .world import World


@dataclass
class StepResult:
    reason: str
    moved: bool = False
    found_target: bool = False
    found_clue: bool = False
    time_cost_s: float = 0.0


@dataclass
class PendingAction:
    kind: str
    target: Optional[Cell] = None
    heading: Optional[Heading] = None


class RobotShell:
    """Robot wrapper owned by the simulator.

    The task allocator chooses task cells and handles algorithm messages. This shell
    handles movement, sensing, belief updates, protected collision safety, and
    metrics.
    """

    def __init__(
        self,
        rid: str,
        pos: Cell,
        heading: Heading,
        cfg: SimConfig,
        world: World,
        bus: MessageBus,
        allocator: AllocatorBase,
    ) -> None:
        self.rid = rid
        self.pos = pos
        self.heading = heading
        self.cfg = cfg
        self.grid_size = cfg.grid_size
        self.world = world
        self.bus = bus
        self.allocator = allocator
        self.belief = BeliefMap(cfg.grid_size, cfg.target_decay_exp)
        self.counters = RobotCounters(rid=rid)
        self.current_goal: Optional[Cell] = None
        self.last_goal: Optional[Cell] = None
        self.last_path: List[Cell] = []
        self.last_next_cell: Optional[Cell] = None
        self.last_event: str = "init"
        self.last_decision_debug: Dict[str, Any] = {}
        self.collision_avoidance_active = False
        self._collision_event_counted_since_move = False
        self._blocked_goal_failures: Dict[Cell, int] = {}
        self._temporary_invalid_task_until: Dict[Cell, float] = {}
        self._communicated_collision_intent: Optional[Cell] = None
        self._last_published_state_pos: Optional[Cell] = None
        self._now: float = 0.0
        self.pending_actions: Deque[PendingAction] = deque()
        self.forwarded_clues: Set[Cell] = set()

        # Droppable coordination knowledge.
        self._peer_positions: Dict[str, Cell] = {}
        # Protected collision-avoidance cache. Not used for task allocation.
        self._collision_peer_positions: Dict[str, Cell] = {}
        self._collision_peer_intents: Dict[str, Optional[Cell]] = {}
        self.temp_blocked_next: Set[Cell] = set()
        self._active_peer_positions: Optional[Dict[str, Cell]] = None
        self._perception_pending_positions: Dict[str, Cell] = {}
        self._perception_pending_collision_positions: Dict[str, Cell] = {}
        self._perception_pending_collision_intents: Dict[str, Optional[Cell]] = {}
        self._perception_pending_valid = False
        self._perception_initialized = False

        # Local truth/knowledge.
        self.belief.mark_searched(pos)
        if not self.world.record_visit(rid, pos):
            self.counters.unique_cells_contributed += 1

        self.bus.register(self)
        self.allocator.initialize(self)

    @property
    def known_clues(self) -> List[Cell]:
        return self.belief.known_clues

    @property
    def searched(self) -> Set[Cell]:
        return self.belief.searched

    @property
    def local_searched(self) -> Set[Cell]:
        return self.belief.searched

    @property
    def target_p(self) -> Dict[Cell, float]:
        return self.belief.target_p

    @property
    def known_obstacles(self) -> Set[Cell]:
        return set()

    @property
    def obstacles(self) -> Set[Cell]:
        return self.known_obstacles

    @property
    def blocked(self) -> Set[Cell]:
        return self.known_obstacles | set(self._temporary_invalid_task_until.keys())

    @property
    def blocked_cells(self) -> Set[Cell]:
        return self.blocked

    @property
    def peer_positions(self) -> Dict[str, Cell]:
        return self._active_peer_positions if self._active_peer_positions is not None else self._peer_positions

    def _publish(self, category: str, payload: Dict[str, Any]) -> None:
        self.bus.publish(
            self.rid,
            topic_for(self.rid, category),
            payload,
            self._now,
            post_clue=self._post_clue_started(),
        )

    def publish_algorithm_message(self, category: str, payload: Dict[str, Any]) -> None:
        self._publish(category, payload)

    def publish_state(self) -> None:
        if self._last_published_state_pos == self.pos:
            return
        self._last_published_state_pos = self.pos
        self._publish("state", {"loc": list(self.pos)})

    def publish_clue(self, cell: Cell) -> None:
        self.forwarded_clues.add(cell)
        self._publish("clue", {"loc": list(cell)})

    def publish_target(self, cell: Cell) -> None:
        # Protected: target messages are never dropped by the bus.
        self._publish("target", {"loc": list(cell)})

    def publish_collision_intent(self, intent: Optional[Cell]) -> None:
        # Protected: collision avoidance is not part of degraded comm evaluation.
        payload = {"loc": list(self.pos), "intent": list(intent) if intent is not None else None}
        self._publish("collision_intent", payload)

    def _set_collision_intent(self, intent: Optional[Cell]) -> None:
        if intent is None:
            if self._communicated_collision_intent is None:
                return
            self._communicated_collision_intent = None
            self.publish_collision_intent(None)
            return
        normalized = (int(intent[0]), int(intent[1]))
        if self._communicated_collision_intent == normalized:
            return
        self._communicated_collision_intent = normalized
        self.publish_collision_intent(normalized)

    def receive_message(self, message: Message) -> None:
        category = message.category
        payload = message.payload
        sender = message.sender
        if sender == self.rid:
            return
        if category == "state":
            loc = _payload_cell(payload.get("loc"))
            if loc is not None and in_bounds(loc, self.grid_size):
                self._peer_positions[sender] = loc
                # Delivered peer state is treated as a searched-cell update.
                self.belief.mark_searched(loc)
            return
        if category == "clue":
            loc = _payload_cell(payload.get("loc"))
            if loc is not None and in_bounds(loc, self.grid_size):
                new_clue = self.belief.add_clue(loc)
                if new_clue and loc not in self.forwarded_clues:
                    self.publish_clue(loc)
                if new_clue:
                    self._notify_allocator_clue_change()
            return
        if category == "target":
            # Trial runner also stops from world truth, but this mirrors hardware behavior.
            loc = _payload_cell(payload.get("loc"))
            if loc is not None:
                self.last_event = "peer_target_found"
            return
        if category == "collision_intent":
            loc = _payload_cell(payload.get("loc"))
            intent = _payload_cell(payload.get("intent")) if payload.get("intent") is not None else None
            if intent is not None and in_bounds(intent, self.grid_size):
                self._collision_peer_positions[sender] = intent
                self._collision_peer_intents[sender] = intent
            elif loc is not None and in_bounds(loc, self.grid_size):
                self._collision_peer_positions[sender] = loc
                self._collision_peer_intents.pop(sender, None)
            return
        if category in {
            "cbaa_entry",
            "acbba_entry",
            "pi_entry",
            "pi_clear_path",
            "hipc_entry",
            "hipc_clear_bundle",
            "dga_entry",
        }:
            self._deliver_allocator_payload(payload)
            return
        self.allocator.handle_message(self, message)

    def step(self, now_s: float, planner: AStarPlanner) -> StepResult:
        self._now = now_s
        self._expire_temporary_invalid_tasks()
        self.bus.pump(now_s)
        if self.pending_actions:
            return self._execute_pending_action(planner)

        return self._plan_next_action(planner)

    def _plan_next_action(self, planner: AStarPlanner) -> StepResult:
        (
            plan_peer_positions,
            plan_collision_positions,
            plan_collision_intents,
        ) = self._promote_perception()

        previous_task = self.current_goal
        previous_task_completed = previous_task is not None and previous_task in self.belief.searched
        previous_task_invalidated = (
            (previous_task is not None and not previous_task_completed)
            or (previous_task is None and self.last_goal is not None and self.last_goal not in self.belief.searched)
        )

        if self.current_goal is None or self.current_goal in self.belief.searched:
            self.current_goal = None
            self._active_peer_positions = plan_peer_positions
            try:
                decision = self.allocator.choose_goal(self)
            finally:
                self._active_peer_positions = None
                self.collision_avoidance_active = False
            self.current_goal = decision.goal
            self.last_decision_debug = decision.debug
            if self.current_goal is not None and self.current_goal != self.last_goal:
                if previous_task_invalidated and self._post_clue_started():
                    self.counters.task_cell_replans += 1
                self.last_goal = self.current_goal
            self._publish_allocator_messages()

        if self.current_goal is None:
            self._set_collision_intent(None)
            self.last_event = "no_goal"
            return StepResult(reason="no_goal", time_cost_s=self.cfg.no_goal_delay_s)

        prior_temp_blocked_next = set(self.temp_blocked_next)
        blocked = set(plan_peer_positions.values())
        blocked.update(prior_temp_blocked_next)
        self.temp_blocked_next.clear()
        blocked.discard(self.pos)

        max_collision_replans = max(1, self.grid_size * self.grid_size)
        for _ in range(max_collision_replans):
            path = planner.plan(
                start=self.pos,
                heading=self.heading,
                goal=self.current_goal,
                target_p=self.target_p,
                searched=self.belief.searched,
                blocked=blocked,
            )
            self.last_path = path
            if len(path) < 2:
                if self._post_clue_started():
                    self.counters.path_replans += 1
                if self.current_goal is not None and self.current_goal in prior_temp_blocked_next:
                    backoff = self._maybe_temporarily_invalidate_blocked_goal(self.current_goal)
                    if backoff is not None:
                        return backoff
                self.current_goal = None
                self._set_collision_intent(None)
                self.last_event = "path_failed"
                return StepResult(reason="path_failed", time_cost_s=self.cfg.replan_delay_s)

            next_cell = path[1]
            if not self._collision_blocked_by(next_cell, plan_collision_positions, plan_collision_intents):
                self.temp_blocked_next.clear()
                self.last_next_cell = next_cell
                self._queue_actions_for_next_cell(next_cell)
                self._set_collision_intent(next_cell)
                return self._execute_pending_action(planner)

            self._record_collision_prevention(next_cell)
            backoff = self._maybe_temporarily_invalidate_blocked_goal(next_cell)
            if backoff is not None:
                return backoff
            blocked.add(next_cell)

        self.current_goal = None
        self._set_collision_intent(None)
        self.last_event = "path_failed"
        return StepResult(reason="path_failed", time_cost_s=self.cfg.replan_delay_s)

    def _queue_actions_for_next_cell(self, next_cell: Cell) -> None:
        self.pending_actions.clear()
        move_vec = (next_cell[0] - self.pos[0], next_cell[1] - self.pos[1])
        if move_vec not in DIRS4:
            raise ValueError(f"Next cell {next_cell} is not adjacent to {self.pos}")

        desired_heading = move_vec
        if self.heading not in DIRS4:
            self.pending_actions.append(PendingAction(kind="turn", target=next_cell, heading=desired_heading))
        elif self.heading != desired_heading:
            cur_idx = DIRS4.index(self.heading)
            desired_idx = DIRS4.index(desired_heading)
            cw_steps = (desired_idx - cur_idx) % len(DIRS4)
            ccw_steps = (cur_idx - desired_idx) % len(DIRS4)
            step = 1 if cw_steps <= ccw_steps else -1
            turns = min(cw_steps, ccw_steps)
            for _ in range(turns):
                cur_idx = (cur_idx + step) % len(DIRS4)
                self.pending_actions.append(
                    PendingAction(kind="turn", target=next_cell, heading=DIRS4[cur_idx])
                )

        if self.cfg.collision_intent_settle_s > 0:
            self.pending_actions.append(PendingAction(kind="intent_sync", target=next_cell, heading=desired_heading))
        self.pending_actions.append(PendingAction(kind="move", target=next_cell, heading=desired_heading))

    def _execute_pending_action(self, planner: Optional[AStarPlanner] = None) -> StepResult:
        if not self.pending_actions:
            self.last_next_cell = None
            return StepResult(reason="idle", time_cost_s=self.cfg.no_goal_delay_s)

        action = self.pending_actions.popleft()
        if action.target is not None:
            self.last_next_cell = action.target
        else:
            self.last_next_cell = None

        if action.kind == "turn":
            if action.heading is not None:
                self.heading = action.heading
            self.last_event = "turn"
            return StepResult(reason="turn", time_cost_s=self.cfg.turn_quarter_s)

        if action.kind == "intent_sync":
            self.last_event = "intent_sync"
            return StepResult(reason="intent_sync", time_cost_s=self.cfg.collision_intent_settle_s)

        if action.kind != "move" or action.target is None:
            self._clear_pending_actions()
            self.last_event = "path_failed"
            return StepResult(reason="path_failed", time_cost_s=self.cfg.replan_delay_s)

        return self._complete_move(action, planner)

    def _complete_move(self, action: PendingAction, planner: Optional[AStarPlanner] = None) -> StepResult:
        next_cell = action.target
        if next_cell is None:
            self._clear_pending_actions()
            self.last_event = "path_failed"
            return StepResult(reason="path_failed", time_cost_s=self.cfg.replan_delay_s)

        if self._collision_blocked(next_cell):
            self._clear_pending_actions()
            self._record_collision_prevention(next_cell)
            backoff = self._maybe_temporarily_invalidate_blocked_goal(next_cell)
            if backoff is not None:
                return backoff
            if planner is not None:
                return self._plan_next_action(planner)
            self.last_event = "path_failed"
            return StepResult(reason="path_failed", time_cost_s=self.cfg.replan_delay_s)

        move_vec = (next_cell[0] - self.pos[0], next_cell[1] - self.pos[1])
        self.heading = action.heading or move_vec
        self.pos = next_cell
        self._collision_event_counted_since_move = False
        self._blocked_goal_failures.clear()
        self.counters.steps_total += 1
        if self._post_clue_started():
            self.counters.steps_after_first_clue += 1
        self.publish_state()

        revisit = self.world.record_visit(self.rid, self.pos)
        if revisit:
            self.counters.system_revisits_by_robot += 1
        else:
            self.counters.unique_cells_contributed += 1
        self.belief.mark_searched(self.pos)

        found_clue = self.world.detect_clue(self.rid, self.pos, self._now)
        if found_clue:
            new_clue = self.belief.add_clue(self.pos)
            if new_clue:
                self.publish_clue(self.pos)
            self.last_event = "clue_found"

        found_target = self.world.detect_target(self.rid, self.pos, self._now)
        obs = Observation(
            time_s=self._now,
            cell=self.pos,
            searched=True,
            clue_detected=found_clue,
            target_detected=found_target,
        )
        self.allocator.on_observation(self, obs)

        if found_target:
            self.publish_target(self.pos)
            self.last_event = "target_found"
            self._capture_perception()
            self.pending_actions.clear()
            self.last_next_cell = None
            return StepResult(reason="target_found", moved=True, found_target=True, found_clue=found_clue, time_cost_s=self.cfg.async_step_mean_s)

        if not found_clue:
            self.last_event = "moved"
        self._capture_perception()
        self._publish_allocator_messages()
        self.pending_actions.clear()
        self.last_next_cell = None
        return StepResult(reason=self.last_event, moved=True, found_clue=found_clue, time_cost_s=self.cfg.async_step_mean_s)

    def _collision_blocked(self, cell: Cell) -> bool:
        return self._collision_blocked_by(cell, self._collision_peer_positions, self._collision_peer_intents)

    def _collision_blocked_by(
        self,
        cell: Cell,
        positions: Dict[str, Cell],
        intents: Dict[str, Optional[Cell]],
    ) -> bool:
        if cell in positions.values():
            return True
        if cell in [c for c in intents.values() if c is not None]:
            return True
        return False

    def _record_collision_prevention(self, cell: Cell) -> None:
        if self._post_clue_started():
            self.counters.path_replans += 1
            if not self._collision_event_counted_since_move:
                self.counters.collision_prevention_events += 1
                self._collision_event_counted_since_move = True
        self.temp_blocked_next.add(cell)
        self.collision_avoidance_active = True
        self.last_event = "collision_replan"

    def _maybe_temporarily_invalidate_blocked_goal(self, blocked_cell: Cell) -> Optional[StepResult]:
        goal = self.current_goal
        if goal is None or not self._allocation_active():
            return None

        self._blocked_goal_failures[goal] = self._blocked_goal_failures.get(goal, 0) + 1
        if self._blocked_goal_failures[goal] < 2:
            return None

        wait_s = self.bus.rng.uniform(0.0, self.cfg.collision_goal_backoff_max_s)
        self._temporary_invalid_task_until[goal] = self._now + max(wait_s, 1.0e-3)
        self._blocked_goal_failures.pop(goal, None)
        self.current_goal = None
        self._set_collision_intent(None)
        self.last_event = "blocked_goal_backoff"
        return StepResult(reason="blocked_goal_backoff", time_cost_s=wait_s)

    def _expire_temporary_invalid_tasks(self) -> None:
        for cell, expires_at in list(self._temporary_invalid_task_until.items()):
            if self._now >= expires_at:
                self._temporary_invalid_task_until.pop(cell, None)

    def _post_clue_started(self) -> bool:
        return self.world.first_clue_time_s is not None

    def _allocation_active(self) -> bool:
        return self._post_clue_started() or getattr(self.cfg, "trial_mode", "clue_search") == "coverage"

    def _clear_pending_actions(self) -> None:
        self.pending_actions.clear()
        self.last_next_cell = None

    def _publish_allocator_messages(self) -> None:
        for payload in self._allocator_outbound_payloads():
            if not isinstance(payload, dict):
                continue
            category = payload.get("type")
            if not isinstance(category, str) or not category:
                continue
            self._publish(category, payload)

    def _notify_allocator_clue_change(self) -> None:
        handler = getattr(self.allocator, "on_clue_set_changed", None)
        if callable(handler) and handler(self) is not False:
            self.current_goal = None

    def _allocator_outbound_payloads(self) -> List[Dict[str, Any]]:
        for method_name in (
            "make_messages",
            "get_outbound_messages",
            "build_dga_messages",
            "build_acbba_messages",
            "build_cbaa_messages",
            "make_message",
            "get_outbound_message",
            "build_dga_message",
            "build_acbba_message",
            "build_cbaa_message",
        ):
            method = getattr(self.allocator, method_name, None)
            if not callable(method):
                continue
            payloads = method(self)
            if payloads is None:
                return []
            if isinstance(payloads, dict):
                return [payloads]
            return [payload for payload in payloads if isinstance(payload, dict)]
        return []

    def _deliver_allocator_payload(self, payload: Dict[str, Any]) -> None:
        category = payload.get("type")
        for receiver_name in ("receive_message", "on_message", "process_message"):
            receiver = getattr(self.allocator, receiver_name, None)
            if callable(receiver):
                receiver(self, payload)
                return
        handler_names = (
            ("handle_acbba_message", "handle_cbaa_message")
            if category == "acbba_entry"
            else ("handle_cbaa_message", "handle_acbba_message")
        )
        for handler_name in handler_names:
            handler = getattr(self.allocator, handler_name, None)
            if callable(handler):
                handler(self, payload)
                return

    def _capture_perception(self) -> None:
        self._perception_pending_positions = dict(self._peer_positions)
        self._perception_pending_collision_positions = dict(self._collision_peer_positions)
        self._perception_pending_collision_intents = dict(self._collision_peer_intents)
        self._perception_pending_valid = True

    def _promote_perception(
        self,
    ) -> Tuple[
        Dict[str, Cell],
        Dict[str, Cell],
        Dict[str, Optional[Cell]],
    ]:
        if not self._perception_initialized and not self._perception_pending_valid:
            self._capture_perception()
        if self._perception_pending_valid:
            positions = dict(self._perception_pending_positions)
            collision_positions = dict(self._perception_pending_collision_positions)
            collision_intents = dict(self._perception_pending_collision_intents)
            self._perception_pending_valid = False
            self._perception_initialized = True
            return positions, collision_positions, collision_intents
        return (
            dict(self._peer_positions),
            dict(self._collision_peer_positions),
            dict(self._collision_peer_intents),
        )


def _payload_cell(raw: Any) -> Optional[Cell]:
    if raw is None:
        return None
    try:
        return (int(raw[0]), int(raw[1]))
    except (TypeError, ValueError, IndexError):
        return None
