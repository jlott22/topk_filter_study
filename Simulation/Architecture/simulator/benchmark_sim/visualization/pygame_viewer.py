from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Optional

from benchmark_sim.algorithms.registry import load_allocator_class
from benchmark_sim.comms.models import make_comm_model
from benchmark_sim.config import EAST, SimConfig, edge_even_start_positions, generate_robot_ids
from benchmark_sim.core.scenario_loader import load_scenarios
from benchmark_sim.core.scheduler import AsyncTrialRunner, TrialState
from benchmark_sim.core.types import Cell
from benchmark_sim.metrics.export import write_outputs
from benchmark_sim.metrics.summary import build_rows


class Viewer:
    def __init__(self, args: argparse.Namespace) -> None:
        import pygame
        self.pygame = pygame
        pygame.init()
        self.args = args
        robot_ids = generate_robot_ids(args.num_robots)
        self.cfg = SimConfig(
            grid_size=args.grid_size,
            robot_ids=robot_ids,
            start_positions=edge_even_start_positions(args.grid_size, robot_ids),
            start_headings={rid: EAST for rid in robot_ids},
            robot_start_layout=args.robot_start_layout,
            condition_id=args.condition_id,
            target_cells_per_robot=args.target_cells_per_robot,
            actual_cells_per_robot=args.actual_cells_per_robot,
            target_decay_exp=args.target_decay_exp,
            async_tick_span_s=args.async_tick_span,
            write_parquet=False,
        )
        self.allocator_cls = load_allocator_class(args.algorithm)
        self.algorithm_name = args.algorithm_name or getattr(self.allocator_cls, "name", self.allocator_cls.__name__)
        self.scenarios = load_scenarios(args.scenario_file, max_trials=args.max_trials)
        if not self.scenarios:
            raise RuntimeError("No scenarios loaded")
        self.scenario_index = 0
        self.comm_model_name = args.comm_model
        self.comm_level_input = args.comm_level
        self.out_dir = Path(args.out_dir)
        self.trial_rows: List[dict] = []
        self.system_rows: List[dict] = []
        self.robot_rows: List[dict] = []

        self.cell_px = args.cell_px
        self.margin = 40
        self.panel_w = 360
        self.grid_px = self.cfg.grid_size * self.cell_px
        self.width = self.grid_px + self.margin * 2 + self.panel_w
        self.height = self.grid_px + self.margin * 2
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("DCTA benchmark simulator viewer")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 20)
        self.font_small = pygame.font.SysFont(None, 16)
        self.running = True
        self.paused = True
        self.selected_robot = self.cfg.robot_ids[0]
        self.view_mode = "robot"
        self.speed = args.viewer_speed
        self.step_accum = 0.0
        self.async_tick_span = self.cfg.async_tick_span_s
        self.completed_captured = False
        self.state: Optional[TrialState] = None
        self.queue_runner: Optional[AsyncTrialRunner] = None
        self._new_trial()

    def _new_trial(self) -> None:
        scenario = self.scenarios[self.scenario_index % len(self.scenarios)]
        comm_model = make_comm_model(self.comm_model_name, self.comm_level_input)
        self.comm_level = comm_model.level_label()
        self.runner = AsyncTrialRunner(self.cfg, self.allocator_cls, comm_model, seed=self.args.seed + scenario.trial_id * 1009)
        self.state = self.runner.new_trial(scenario)
        self.queue = self.runner.initial_queue(self.state)
        self.order = len(self.queue)
        self.paused = True
        self.step_accum = 0.0
        self.completed_captured = False

    def run(self) -> None:
        pygame = self.pygame
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self._handle_key(event)
            if not self.paused:
                self.step_accum += dt * max(1, int(self.speed))
                while self.step_accum >= 1.0:
                    self.step_accum -= 1.0
                    if not self._advance_tick():
                        self.step_accum = 0.0
                        break
            self._draw()
        self._write_completed_outputs()
        pygame.quit()

    def _handle_key(self, event) -> None:
        pygame = self.pygame
        if event.key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif event.key == pygame.K_p:
            self.paused = not self.paused
            if self.paused:
                self.step_accum = 0.0
        elif event.key == pygame.K_PERIOD:
            if self.paused:
                self._advance_tick()
                self.step_accum = 0.0
        elif event.key == pygame.K_t:
            self.view_mode = "truth"
        elif event.key == pygame.K_n:
            if self.state and self.state.done:
                self.scenario_index = (self.scenario_index + 1) % len(self.scenarios)
                self._new_trial()
        elif event.key == pygame.K_TAB:
            idx = (self.cfg.robot_ids.index(self.selected_robot) + 1) % len(self.cfg.robot_ids)
            self.selected_robot = self.cfg.robot_ids[idx]
            self.view_mode = "robot"
        elif event.key == pygame.K_LEFTBRACKET:
            self.speed = max(1, self.speed - 1)
            self.step_accum = min(self.step_accum, 0.99)
        elif event.key == pygame.K_RIGHTBRACKET:
            self.speed = min(100, self.speed + 1)
            self.step_accum = min(self.step_accum, 0.99)

    def _advance_tick(self) -> bool:
        if self.state is None or self.state.done or not self.queue:
            return False
        time_limit = self.state.clock_s + max(self.async_tick_span, 1e-3)
        events, self.order = self.runner.step_until(
            self.state,
            self.queue,
            self.order,
            time_limit_s=time_limit,
            allow_overshoot=True,
        )
        if self.state.done and not self.completed_captured:
            self._capture_completed_trial()
            self.completed_captured = True
        return bool(events)

    def _capture_completed_trial(self) -> None:
        if self.state is None:
            return
        trial_row, system_row, robot_rows = build_rows(
            self.state,
            algorithm_name=self.algorithm_name,
            comm_model=self.comm_model_name,
            comm_level=self.comm_level,
            scenario_file=str(Path(self.args.scenario_file)),
        )
        self.trial_rows.append(trial_row)
        self.system_rows.append(system_row)
        self.robot_rows.extend(robot_rows)
        self._write_completed_outputs()

    def _write_completed_outputs(self) -> None:
        if not self.system_rows:
            return
        write_outputs(
            out_dir=self.out_dir,
            trial_summary_rows=self.trial_rows,
            system_performance_rows=self.system_rows,
            robot_performance_rows=self.robot_rows,
            config={
                "sim_config": self.cfg.to_dict(),
                "algorithm": self.args.algorithm,
                "algorithm_name": self.algorithm_name,
                "comm_model": self.comm_model_name,
                "comm_level": self.comm_level,
                "scenario_file": str(Path(self.args.scenario_file)),
                "seed": self.args.seed,
                "viewer": True,
            },
            write_parquet=False,
        )

    def world_to_screen(self, cell: Cell) -> tuple[int, int]:
        x, y = cell
        sx = self.margin + x * self.cell_px
        sy = self.margin + (self.cfg.grid_size - 1 - y) * self.cell_px
        return sx, sy

    def world_to_center(self, cell: Cell) -> tuple[int, int]:
        sx, sy = self.world_to_screen(cell)
        return sx + self.cell_px // 2, sy + self.cell_px // 2

    def _draw(self) -> None:
        pygame = self.pygame
        if self.state is None:
            return
        screen = self.screen
        screen.fill((28, 28, 34))
        selected = self.state.robots.get(self.selected_robot)
        robot_view = self.view_mode == "robot" and selected is not None
        colors = [(120, 200, 255), (255, 170, 95), (160, 255, 130), (255, 130, 205)]
        robot_colors = {rid: colors[i % len(colors)] for i, rid in enumerate(self.cfg.robot_ids)}
        prob = selected.target_p if robot_view else self._global_target_p()
        max_p = max(prob.values()) if prob else 0.0
        visited = set(selected.searched) if robot_view else set(self.state.world.visits.keys())
        for y in range(self.cfg.grid_size):
            for x in range(self.cfg.grid_size):
                cell = (x, y)
                sx, sy = self.world_to_screen(cell)
                rect = pygame.Rect(sx, sy, self.cell_px - 1, self.cell_px - 1)
                base = (54, 54, 64)
                if cell in visited:
                    if self.view_mode == "truth":
                        rec = self.state.world.visits.get(cell)
                        rid = sorted(rec.by_robot.keys())[0] if rec and rec.by_robot else None
                        base = self._muted_color(robot_colors.get(rid, (88, 88, 115)))
                    else:
                        base = (88, 88, 115)
                pygame.draw.rect(screen, base, rect)
                if max_p > 0:
                    intensity = prob.get(cell, 0.0) / max_p
                    if intensity > 0:
                        overlay = pygame.Surface((self.cell_px - 1, self.cell_px - 1), pygame.SRCALPHA)
                        overlay.fill((255, 145, 0, int(190 * intensity)))
                        screen.blit(overlay, rect.topleft)
                pygame.draw.rect(screen, (70, 70, 78), rect, 1)
        self._draw_truth_markers()

        for i, rid in enumerate(self.cfg.robot_ids):
            rb = self.state.robots[rid]
            color = colors[i % len(colors)]
            cx, cy = self.world_to_center(rb.pos)
            pygame.draw.circle(screen, color, (cx, cy), max(6, self.cell_px // 3))
            label = self.font_small.render(rid, True, (10, 10, 20))
            screen.blit(label, label.get_rect(center=(cx, cy)))
            if rb.current_goal is not None:
                gx, gy = self.world_to_screen(rb.current_goal)
                pygame.draw.rect(screen, color, pygame.Rect(gx + 3, gy + 3, self.cell_px - 7, self.cell_px - 7), 2)
            if rb.last_next_cell is not None:
                nx, ny = self.world_to_center(rb.last_next_cell)
                pygame.draw.circle(screen, (255, 255, 255), (nx, ny), max(3, self.cell_px // 6), 1)

        self._draw_panel()
        pygame.display.flip()

    @staticmethod
    def _muted_color(color: tuple[int, int, int]) -> tuple[int, int, int]:
        return (
            min(255, int(color[0] * 0.55 + 42)),
            min(255, int(color[1] * 0.55 + 42)),
            min(255, int(color[2] * 0.55 + 48)),
        )

    def _global_target_p(self) -> Dict[Cell, float]:
        if self.state is None:
            return {}
        combined: Dict[Cell, float] = {}
        robots = list(self.state.robots.values())
        if not robots:
            return combined
        for rb in robots:
            for cell, value in rb.target_p.items():
                combined[cell] = combined.get(cell, 0.0) + float(value)
        scale = float(len(robots))
        return {cell: value / scale for cell, value in combined.items()}

    def _draw_truth_markers(self) -> None:
        pygame = self.pygame
        if self.state is None:
            return
        clue_radius = max(5, self.cell_px // 4)
        clue_outline = clue_radius + 2
        for clue in self.state.world.clues:
            center = self.world_to_center(clue)
            pygame.draw.circle(self.screen, (8, 20, 26), center, clue_outline)
            pygame.draw.circle(self.screen, (40, 220, 220), center, clue_radius)

        if self.state.world.target is not None:
            tx, ty = self.world_to_screen(self.state.world.target)
            inset = max(3, self.cell_px // 8)
            target_rect = pygame.Rect(
                tx + inset,
                ty + inset,
                self.cell_px - (2 * inset) - 1,
                self.cell_px - (2 * inset) - 1,
            )
            pygame.draw.rect(self.screen, (24, 10, 24), target_rect.inflate(4, 4), border_radius=4)
            pygame.draw.rect(self.screen, (255, 40, 150), target_rect, border_radius=4)

    def _draw_panel(self) -> None:
        pygame = self.pygame
        if self.state is None:
            return
        x = self.margin + self.grid_px + 18
        y = self.margin
        panel_rect = pygame.Rect(x - 10, y - 10, self.panel_w - 20, self.grid_px)
        pygame.draw.rect(self.screen, (42, 42, 50), panel_rect, border_radius=8)
        pygame.draw.rect(self.screen, (80, 80, 95), panel_rect, 1, border_radius=8)
        lines = []
        w = self.state.world
        bus_counts = self.state.bus.counters
        total_steps = sum(rb.counters.steps_total for rb in self.state.robots.values())
        post_steps = sum(rb.counters.steps_after_first_clue for rb in self.state.robots.values())
        lines.extend([
            f"trial={w.scenario.trial_id} alg={self.algorithm_name}",
            f"comm={self.comm_model_name} {self.comm_level}",
            f"time={self.state.clock_s:.2f}s events={self.state.events_processed}",
            f"paused={self.paused} speed={self.speed} ticks/s tick={self.async_tick_span:.2f}s",
            f"view={self.selected_robot if self.view_mode == 'robot' else 'global truth'}",
            f"steps_total={total_steps}",
            f"post_clue_steps={post_steps}",
            f"unique_cells={w.unique_cells_searched()}",
            f"system_revisits={w.system_revisits()}",
            f"messages published={bus_counts.sent_total}",
            f"protected={bus_counts.protected_sent_total} unprotected={bus_counts.unprotected_sent_total}",
            f"core={bus_counts.core_sent_total} algorithm={bus_counts.allocation_sent_total}",
            f"receiver outcomes: delivered={bus_counts.delivered_total} dropped={bus_counts.dropped_total}",
            f"first_clue={w.first_clue_robot}@{w.first_clue_cell}",
            f"target_found={w.target_found_by}@{w.target_found_time_s}",
        ])
        y0 = y
        for line in lines:
            label = self.font.render(line, True, (232, 232, 240))
            self.screen.blit(label, (x, y0))
            y0 += label.get_height() + 4
        y0 += 10
        for rid, rb in self.state.robots.items():
            line = f"{rid}: pos={rb.pos} goal={rb.current_goal} steps={rb.counters.steps_total} last={rb.last_event}"
            label = self.font_small.render(line, True, (220, 220, 228))
            self.screen.blit(label, (x, y0))
            y0 += label.get_height() + 3
        y0 += 12
        for line in ["p pause | . step | t truth view", "tab next robot | n next trial", "[ ] speed | q quit"]:
            label = self.font_small.render(line, True, (190, 190, 200))
            self.screen.blit(label, (x, y0))
            y0 += label.get_height() + 3


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live viewer for DCTA benchmark simulator.")
    p.add_argument("--scenario-file", required=True)
    p.add_argument("--algorithm", required=True, help="Allocator class as module.path:ClassName.")
    p.add_argument("--algorithm-name", default=None)
    p.add_argument("--comm-model", default="ideal", choices=["ideal", "bernoulli", "gilbert_elliot", "rayleigh_style"])
    p.add_argument("--comm-level", type=float, default=None)
    p.add_argument("--max-trials", type=int, default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out-dir", default="runs/viewer")
    p.add_argument("--grid-size", type=int, default=19)
    p.add_argument("--num-robots", type=int, default=4)
    p.add_argument("--robot-start-layout", default="edge_even", choices=["edge_even"])
    p.add_argument("--condition-id", default="")
    p.add_argument("--target-cells-per-robot", type=float, default=None)
    p.add_argument("--actual-cells-per-robot", type=float, default=None)
    p.add_argument("--target-decay-exp", type=float, default=1.0)
    p.add_argument("--async-tick-span", type=float, default=0.25)
    p.add_argument("--viewer-speed", "--viewer-fps", dest="viewer_speed", type=int, default=1)
    p.add_argument("--cell-px", type=int, default=34)
    p.add_argument("--no-parquet", action="store_true", help="Deprecated; metric outputs are always CSV-only.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    Viewer(args).run()
