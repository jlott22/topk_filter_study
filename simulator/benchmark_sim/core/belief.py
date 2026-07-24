from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Set, Tuple

from .types import Cell, manhattan


@dataclass
class BeliefMap:
    """Target-only posterior over grid cells.

    There is intentionally no clue_p field. Clues reshape target_p; searched
    cells are eliminated because target POD is assumed to be 1.
    """

    grid_size: int
    target_decay_exp: float = 1.0
    searched: Set[Cell] = field(default_factory=set)
    known_clues: List[Cell] = field(default_factory=list)
    target_p: Dict[Cell, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.recompute()

    def all_cells(self) -> Iterable[Cell]:
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                yield (x, y)

    def mark_searched(self, cell: Cell) -> None:
        self.searched.add(cell)
        self.recompute()

    def add_clue(self, cell: Cell) -> bool:
        if cell not in self.known_clues:
            self.known_clues.append(cell)
            # Clue cell has been searched; it is not the target in generated scenarios.
            self.searched.add(cell)
            self.recompute()
            return True
        return False

    def recompute(self) -> None:
        values: Dict[Cell, float] = {}
        if not self.known_clues:
            for cell in self.all_cells():
                values[cell] = 0.0 if cell in self.searched else 1.0
        else:
            for cell in self.all_cells():
                if cell in self.searched:
                    values[cell] = 0.0
                    continue
                s = 0.0
                for clue in self.known_clues:
                    d = manhattan(cell, clue)
                    s += 1.0 / ((1.0 + d) ** self.target_decay_exp)
                values[cell] = s
        self.target_p = self._normalized(values)

    @staticmethod
    def _normalized(values: Dict[Cell, float]) -> Dict[Cell, float]:
        total = sum(values.values())
        if total <= 0.0:
            n = len(values)
            return {cell: 1.0 / n for cell in values} if n else {}
        return {cell: val / total for cell, val in values.items()}

    def probability(self, cell: Cell) -> float:
        return self.target_p.get(cell, 0.0)

    def as_dense_rows(self) -> List[List[float]]:
        return [
            [self.probability((x, y)) for x in range(self.grid_size)]
            for y in range(self.grid_size)
        ]
