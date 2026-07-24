from __future__ import annotations

import unittest

from benchmark_sim.algorithms.base import AllocatorBase
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, NORTH, WEST, SimConfig
from benchmark_sim.core.scheduler import AsyncTrialRunner
from benchmark_sim.core.types import AllocationDecision, Cell, TrialScenario


class _FixedGoalAllocator(AllocatorBase):
    goal: Cell = (0, 0)

    def choose_goal(self, robot):
        return AllocationDecision(goal=self.goal)


class _NorthGoalAllocator(_FixedGoalAllocator):
    goal = (0, 1)


class _WestGoalAllocator(_FixedGoalAllocator):
    goal = (0, 1)


class _TwoNorthGoalAllocator(_FixedGoalAllocator):
    goal = (0, 2)


class _SequentialGoalAllocator(AllocatorBase):
    goals: list[Cell] = [(0, 1), (0, 2)]

    def initialize(self, robot) -> None:
        self.index = 0

    def choose_goal(self, robot):
        goal = self.goals[min(self.index, len(self.goals) - 1)]
        self.index += 1
        return AllocationDecision(goal=goal)


class _FirstUnblockedGoalAllocator(AllocatorBase):
    goals: list[Cell] = [(0, 1), (1, 0), (1, 1), (0, 1)]

    def choose_goal(self, robot):
        blocked = set(getattr(robot, "blocked_cells", set()))
        for goal in self.goals:
            if goal not in blocked and goal not in robot.searched:
                return AllocationDecision(goal=goal)
        return AllocationDecision(goal=None)


def _cfg(start=(0, 0), heading=EAST, collision_intent_settle_s=0.05) -> SimConfig:
    return SimConfig(
        grid_size=3,
        robot_ids=["00"],
        start_positions={"00": start},
        start_headings={"00": heading},
        async_step_mean_s=1.6,
        async_step_jitter_s=0.0,
        async_min_delay_s=1.6,
        async_max_delay_s=1.6,
        async_initial_spread_s=0.0,
        collision_intent_settle_s=collision_intent_settle_s,
        turn_quarter_s=0.30,
        debug_max_events=100,
        write_parquet=False,
    )


def _cfg_two_robots() -> SimConfig:
    return SimConfig(
        grid_size=3,
        robot_ids=["00", "01"],
        start_positions={"00": (0, 0), "01": (1, 0)},
        start_headings={"00": EAST, "01": WEST},
        async_step_mean_s=1.6,
        async_step_jitter_s=0.0,
        async_min_delay_s=1.6,
        async_max_delay_s=1.6,
        async_initial_spread_s=0.0,
        collision_intent_settle_s=0.0,
        turn_quarter_s=0.30,
        debug_max_events=100,
        write_parquet=False,
    )


def _runner(cfg: SimConfig, allocator_cls: type[AllocatorBase]) -> tuple[AsyncTrialRunner, object, list, int]:
    scenario = TrialScenario(trial_id=0, target=(2, 2), clues=[])
    runner = AsyncTrialRunner(cfg, allocator_cls, make_comm_model("ideal", None), seed=0)
    state = runner.new_trial(scenario)
    queue = runner.initial_queue(state)
    return runner, state, queue, len(queue)


class AsyncMovementTests(unittest.TestCase):
    def test_turn_intent_sync_then_move_counts_one_step(self) -> None:
        runner, state, queue, order = _runner(_cfg(), _NorthGoalAllocator)
        robot = state.robots["00"]

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "turn")
        self.assertEqual(robot.pos, (0, 0))
        self.assertEqual(robot.heading, (0, 1))
        self.assertEqual(robot.counters.steps_total, 0)

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "intent_sync")
        self.assertEqual(robot.pos, (0, 0))
        self.assertEqual(robot.counters.steps_total, 0)

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (0, 1))
        self.assertEqual(robot.counters.steps_total, 1)

    def test_collision_intent_publishes_once_per_selected_transition(self) -> None:
        runner, state, queue, order = _runner(_cfg(), _NorthGoalAllocator)
        robot = state.robots["00"]

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "turn")
        self.assertEqual(state.bus.counters.protected_sent_total, 1)
        self.assertEqual(robot._communicated_collision_intent, (0, 1))

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "intent_sync")
        self.assertEqual(state.bus.counters.protected_sent_total, 1)

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(state.bus.counters.protected_sent_total, 1)
        self.assertEqual(robot._communicated_collision_intent, (0, 1))

    def test_state_publishes_once_per_reached_location(self) -> None:
        runner, state, queue, order = _runner(_cfg(), _NorthGoalAllocator)
        robot = state.robots["00"]

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "turn")
        self.assertEqual(state.bus.counters.unprotected_sent_total, 0)

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "intent_sync")
        self.assertEqual(state.bus.counters.unprotected_sent_total, 0)

        event, order = runner.process_next_event(state, queue, order)
        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (0, 1))
        self.assertEqual(state.bus.counters.unprotected_sent_total, 1)
        self.assertEqual(state.bus.counters.sent_by_topic.get("state"), 1)

        robot.publish_state()
        self.assertEqual(state.bus.counters.unprotected_sent_total, 1)
        self.assertEqual(state.bus.counters.sent_by_topic.get("state"), 1)

    def test_peer_collision_intent_persists_until_replaced(self) -> None:
        runner, state, queue, order = _runner(_cfg_two_robots(), _NorthGoalAllocator)
        robot = state.robots["00"]
        peer = state.robots["01"]

        robot._set_collision_intent((0, 1))
        state.bus.pump(1.0)

        self.assertEqual(peer._collision_peer_intents["00"], (0, 1))
        self.assertEqual(peer._collision_peer_positions["00"], (0, 1))

        robot.pos = (0, 1)
        robot.publish_state()
        state.bus.pump(2.0)

        self.assertEqual(peer._collision_peer_intents["00"], (0, 1))
        self.assertEqual(peer._collision_peer_positions["00"], (0, 1))

        robot._set_collision_intent((0, 2))
        state.bus.pump(3.0)

        self.assertEqual(peer._collision_peer_intents["00"], (0, 2))
        self.assertEqual(peer._collision_peer_positions["00"], (0, 2))

    def test_step_until_processes_a_simulated_time_window(self) -> None:
        runner, state, queue, order = _runner(_cfg(), _NorthGoalAllocator)

        events, order = runner.step_until(state, queue, order, time_limit_s=0.5, allow_overshoot=True)

        self.assertEqual([event.result.reason for event in events], ["turn", "intent_sync", "moved"])
        self.assertAlmostEqual(state.clock_s, 0.35)
        self.assertEqual(state.robots["00"].pos, (0, 1))
        self.assertEqual(state.robots["00"].counters.steps_total, 1)

    def test_one_eighty_turn_uses_two_turn_events_before_move(self) -> None:
        runner, state, queue, order = _runner(_cfg(start=(1, 1), heading=EAST), _WestGoalAllocator)

        events, order = runner.step_until(state, queue, order, time_limit_s=1.0, allow_overshoot=True)

        self.assertEqual([event.result.reason for event in events], ["turn", "turn", "intent_sync", "moved"])
        self.assertAlmostEqual(state.clock_s, 0.65)
        self.assertEqual(state.robots["00"].pos, (0, 1))
        self.assertEqual(state.robots["00"].heading, (-1, 0))
        self.assertEqual(state.robots["00"].counters.steps_total, 1)

    def test_astar_initially_blocks_droppable_peer_positions(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(collision_intent_settle_s=0.0),
            _TwoNorthGoalAllocator,
        )
        robot = state.robots["00"]
        robot._peer_positions["01"] = (0, 1)

        event, order = runner.process_next_event(state, queue, order)

        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (1, 0))
        self.assertNotIn((0, 1), robot.last_path)
        self.assertEqual(robot.counters.collision_prevention_events, 0)

    def test_pre_clue_collision_replan_does_not_increment_churn_metrics(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(collision_intent_settle_s=0.0),
            _TwoNorthGoalAllocator,
        )
        robot = state.robots["00"]
        robot._collision_peer_intents["01"] = (0, 1)

        event, order = runner.process_next_event(state, queue, order)

        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (1, 0))
        self.assertNotIn((0, 1), robot.last_path)
        self.assertEqual(robot.counters.collision_prevention_events, 0)
        self.assertEqual(robot.counters.path_replans, 0)

    def test_post_clue_collision_replan_increments_churn_metrics(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(collision_intent_settle_s=0.0),
            _TwoNorthGoalAllocator,
        )
        state.world.first_clue_time_s = 0.0
        robot = state.robots["00"]
        robot._collision_peer_intents["01"] = (0, 1)

        event, order = runner.process_next_event(state, queue, order)

        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (1, 0))
        self.assertNotIn((0, 1), robot.last_path)
        self.assertEqual(robot.counters.collision_prevention_events, 1)
        self.assertEqual(robot.counters.path_replans, 1)

    def test_post_clue_collision_event_counts_once_until_robot_moves(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(collision_intent_settle_s=0.0),
            _NorthGoalAllocator,
        )
        state.world.first_clue_time_s = 0.0
        robot = state.robots["00"]
        robot._collision_peer_intents["01"] = (0, 1)

        first, order = runner.process_next_event(state, queue, order)
        second, order = runner.process_next_event(state, queue, order)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertEqual(first.result.reason, "path_failed")
        self.assertEqual(second.result.reason, "blocked_goal_backoff")
        self.assertEqual(robot.pos, (0, 0))
        self.assertGreaterEqual(second.result.time_cost_s, 0.0)
        self.assertLessEqual(second.result.time_cost_s, 5.0)
        self.assertEqual(robot.counters.collision_prevention_events, 1)
        self.assertGreater(robot.counters.path_replans, robot.counters.collision_prevention_events)

    def test_repeated_blocked_goal_is_temporarily_invalid_until_backoff_time(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(collision_intent_settle_s=0.0),
            _FirstUnblockedGoalAllocator,
        )
        state.world.first_clue_time_s = 0.0
        robot = state.robots["00"]
        robot._collision_peer_intents["01"] = (0, 1)

        first, order = runner.process_next_event(state, queue, order)
        self.assertEqual(first.result.reason, "path_failed")

        second, order = runner.process_next_event(state, queue, order)
        self.assertEqual(second.result.reason, "blocked_goal_backoff")
        self.assertIn((0, 1), robot.blocked_cells)
        expires_at = robot._temporary_invalid_task_until[(0, 1)]

        robot._now = expires_at - 0.0001
        robot._expire_temporary_invalid_tasks()
        self.assertIn((0, 1), robot.blocked_cells)

        robot._now = expires_at
        robot._expire_temporary_invalid_tasks()
        self.assertNotIn((0, 1), robot.blocked_cells)

    def test_completed_task_cell_replacement_is_not_task_churn(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(heading=NORTH, collision_intent_settle_s=0.0),
            _SequentialGoalAllocator,
        )
        robot = state.robots["00"]

        events, order = runner.step_until(state, queue, order, time_limit_s=2.0, allow_overshoot=True)

        self.assertEqual([event.result.reason for event in events], ["moved", "moved"])
        self.assertEqual(robot.pos, (0, 2))
        self.assertEqual(robot.counters.task_cell_replans, 0)

    def test_pre_clue_replacing_unsearched_task_cell_does_not_count_as_task_churn(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(heading=NORTH, collision_intent_settle_s=0.0),
            _NorthGoalAllocator,
        )
        robot = state.robots["00"]
        robot.last_goal = (0, 2)
        robot.current_goal = None

        event, order = runner.process_next_event(state, queue, order)

        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (0, 1))
        self.assertEqual(robot.counters.task_cell_replans, 0)

    def test_post_clue_replacing_unsearched_task_cell_counts_as_task_churn(self) -> None:
        runner, state, queue, order = _runner(
            _cfg(heading=NORTH, collision_intent_settle_s=0.0),
            _NorthGoalAllocator,
        )
        state.world.first_clue_time_s = 0.0
        robot = state.robots["00"]
        robot.last_goal = (0, 2)
        robot.current_goal = None

        event, order = runner.process_next_event(state, queue, order)

        self.assertIsNotNone(event)
        self.assertEqual(event.result.reason, "moved")
        self.assertEqual(robot.pos, (0, 1))
        self.assertEqual(robot.counters.task_cell_replans, 1)

    def test_unique_cells_contributed_uses_team_truth_not_robot_belief(self) -> None:
        runner, state, queue, order = _runner(_cfg_two_robots(), _FixedGoalAllocator)
        robot = state.robots["01"]
        robot.belief.searched.discard((0, 0))

        event = robot.step(0.0, state.planner)

        self.assertEqual(event.reason, "moved")
        self.assertEqual(robot.pos, (0, 0))
        self.assertEqual(robot.counters.unique_cells_contributed, 1)
        self.assertEqual(robot.counters.system_revisits_by_robot, 1)


if __name__ == "__main__":
    unittest.main()
