"""Shared bounded-memory allocator primitives for RP2040 MicroPython."""

from array import array


def require_binary64(array_factory=array):
    """Fail at startup unless arithmetic and packed storage are binary64."""
    one = 1.0
    next_binary64 = one + 2.220446049250313e-16
    if next_binary64 == one:
        raise RuntimeError("binary64 floating-point arithmetic is required")
    try:
        probe = array_factory("d", [next_binary64])
    except (TypeError, ValueError, OverflowError):
        raise RuntimeError("array('d') binary64 storage is required")
    if len(probe) != 1 or probe[0] != next_binary64 or probe[0] == one:
        raise RuntimeError("array('d') failed binary64 round-trip probe")
    return True


class CellIndexedMap:
    """Fixed-grid mapping compatible with the cell-keyed dict operations in allocators."""

    def __init__(self, grid_size, numeric=False):
        self.grid_size = int(grid_size)
        cell_count = self.grid_size * self.grid_size
        self._active = bytearray(cell_count)
        self._count = 0
        self._numeric = bool(numeric)
        if self._numeric:
            # Binary64 is the canonical simulator score representation and has
            # the same eight-byte footprint as the previous int64 storage.
            # Deliberately fail at import/allocation on a port without ``d``
            # support instead of silently changing allocator decisions.
            self._values = array("d", [0.0] * cell_count)
        else:
            self._values = [None] * cell_count

    def _cell_id(self, cell):
        x = int(cell[0])
        y = int(cell[1])
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            raise KeyError(cell)
        return y * self.grid_size + x

    def _cell(self, cell_id):
        return (cell_id % self.grid_size, cell_id // self.grid_size)

    def __getitem__(self, cell):
        cell_id = self._cell_id(cell)
        if not self._active[cell_id]:
            raise KeyError(cell)
        return self._values[cell_id]

    def __setitem__(self, cell, value):
        cell_id = self._cell_id(cell)
        if not self._active[cell_id]:
            self._active[cell_id] = 1
            self._count += 1
        self._values[cell_id] = float(value) if self._numeric else value

    def __contains__(self, cell):
        try:
            return bool(self._active[self._cell_id(cell)])
        except (KeyError, TypeError, ValueError, IndexError):
            return False

    def __iter__(self):
        for cell_id in range(len(self._active)):
            if self._active[cell_id]:
                yield self._cell(cell_id)

    def __len__(self):
        return self._count

    def get(self, cell, default=None):
        try:
            cell_id = self._cell_id(cell)
        except (KeyError, TypeError, ValueError, IndexError):
            return default
        if not self._active[cell_id]:
            return default
        return self._values[cell_id]

    def pop(self, cell, default=None):
        try:
            cell_id = self._cell_id(cell)
        except (KeyError, TypeError, ValueError, IndexError):
            return default
        if not self._active[cell_id]:
            return default
        value = self._values[cell_id]
        self._active[cell_id] = 0
        self._count -= 1
        self._values[cell_id] = 0.0 if self._numeric else None
        return value

    def keys(self):
        return iter(self)

    def items(self):
        for cell_id in range(len(self._active)):
            if self._active[cell_id]:
                yield self._cell(cell_id), self._values[cell_id]

    def clear(self):
        if not self._count:
            return
        for cell_id in range(len(self._active)):
            if not self._active[cell_id]:
                continue
            self._active[cell_id] = 0
            self._values[cell_id] = 0.0 if self._numeric else None
        self._count = 0


class PackedCandidateWorkspace:
    """Reusable bounded candidate workspace.

    Canonical simulator filtering preserves the caller's y-major source order
    when every valid cell fits. Probability/distance/cell ranking is applied
    only after a real Top-K overflow, unless ``rank_always`` is requested by an
    algorithm (DGA/HIPC) that ranks its source before filtering.
    """

    def __init__(self, grid_size, capacity):
        self.grid_size = int(grid_size)
        self.capacity = int(capacity)
        self.ids = array("H", [0] * self.capacity)
        self.count = 0

    def _precedes(self, left_id, right_id, target_p, map_index, origin):
        grid_size = self.grid_size
        left_x = left_id % grid_size
        left_y = left_id // grid_size
        right_x = right_id % grid_size
        right_y = right_id // grid_size

        left_probability = target_p[map_index(left_x, left_y)]
        right_probability = target_p[map_index(right_x, right_y)]
        if left_probability != right_probability:
            return left_probability > right_probability

        left_distance = abs(origin[0] - left_x) + abs(origin[1] - left_y)
        right_distance = abs(origin[0] - right_x) + abs(origin[1] - right_y)
        if left_distance != right_distance:
            return left_distance < right_distance
        if left_x != right_x:
            return left_x < right_x
        return left_y < right_y

    def _sort_prefix(self, count, target_p, map_index, origin):
        ids = self.ids
        for index in range(1, count):
            cell_id = ids[index]
            insertion = index
            while insertion > 0 and self._precedes(
                    cell_id, ids[insertion - 1], target_p, map_index, origin):
                ids[insertion] = ids[insertion - 1]
                insertion -= 1
            ids[insertion] = cell_id

    def fill(
        self,
        grid,
        target_p,
        map_index,
        origin,
        unsearched_value,
        rank_always=False,
    ):
        count = 0
        capacity = self.capacity
        ids = self.ids
        grid_size = self.grid_size
        ranked = bool(rank_always)
        for y in range(grid_size):
            for x in range(grid_size):
                if grid[map_index(x, y)] != unsearched_value:
                    continue
                cell_id = y * grid_size + x
                if count < capacity:
                    if not ranked:
                        ids[count] = cell_id
                        count += 1
                        continue
                    insertion = count
                    while insertion > 0 and self._precedes(
                            cell_id, ids[insertion - 1], target_p, map_index, origin):
                        ids[insertion] = ids[insertion - 1]
                        insertion -= 1
                    ids[insertion] = cell_id
                    count += 1
                    continue

                if not ranked:
                    self._sort_prefix(
                        count, target_p, map_index, origin
                    )
                    ranked = True
                if not self._precedes(
                        cell_id, ids[capacity - 1], target_p, map_index, origin):
                    continue
                insertion = capacity - 1
                while insertion > 0 and self._precedes(
                        cell_id, ids[insertion - 1], target_p, map_index, origin):
                    ids[insertion] = ids[insertion - 1]
                    insertion -= 1
                ids[insertion] = cell_id
        self.count = count
        return self

    def __len__(self):
        return self.count

    def __iter__(self):
        grid_size = self.grid_size
        for index in range(self.count):
            cell_id = self.ids[index]
            yield (cell_id % grid_size, cell_id // grid_size)

    def __getitem__(self, index):
        if index < 0:
            index += self.count
        if not (0 <= index < self.count):
            raise IndexError(index)
        cell_id = self.ids[index]
        return (cell_id % self.grid_size, cell_id // self.grid_size)
