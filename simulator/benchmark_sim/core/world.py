from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from .types import Cell, TrialScenario


@dataclass
class VisitRecord:
    total_visits: int = 0
    by_robot: Dict[str, int] = field(default_factory=dict)


@dataclass
class World:
    grid_size: int
    scenario: TrialScenario
    visits: Dict[Cell, VisitRecord] = field(default_factory=dict)
    first_clue_time_s: Optional[float] = None
    first_clue_robot: Optional[str] = None
    first_clue_cell: Optional[Cell] = None
    target_found_time_s: Optional[float] = None
    target_found_by: Optional[str] = None

    @property
    def target(self) -> Optional[Cell]:
        return self.scenario.target

    @property
    def clues(self) -> List[Cell]:
        return self.scenario.clues

    @property
    def clue_set(self) -> Set[Cell]:
        return set(self.scenario.clues)

    def record_visit(self, rid: str, cell: Cell) -> bool:
        """Record a team-truth visit. Returns True if this is a global revisit."""
        rec = self.visits.setdefault(cell, VisitRecord())
        was_visited = rec.total_visits > 0
        rec.total_visits += 1
        rec.by_robot[rid] = rec.by_robot.get(rid, 0) + 1
        return was_visited

    def detect_clue(self, rid: str, cell: Cell, time_s: float) -> bool:
        if cell not in self.clue_set:
            return False
        if self.first_clue_time_s is None:
            self.first_clue_time_s = time_s
            self.first_clue_robot = rid
            self.first_clue_cell = cell
        return True

    def detect_target(self, rid: str, cell: Cell, time_s: float) -> bool:
        if self.target is None or cell != self.target:
            return False
        self.target_found_time_s = time_s
        self.target_found_by = rid
        return True

    def unique_cells_searched(self) -> int:
        return len(self.visits)

    def system_revisits(self) -> int:
        return sum(max(0, rec.total_visits - 1) for rec in self.visits.values())
