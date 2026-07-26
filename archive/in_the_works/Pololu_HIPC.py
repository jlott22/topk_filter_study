# Copyright 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ===========================================================
# Pololu 3pi+ 2040 OLED — HIPC Coordinated Search (UART → ESP32 → MQTT)
# ===========================================================
# Runs on the Pololu 3pi+ 2040 OLED using MicroPython.
# Communication uses simple text frames over UART; an attached ESP32 relays
# those frames to MQTT topics.
#
# Behavior overview:
#   * Before any clue is found the robot sweeps its row band in a fixed
#     lawn-mower/serpentine pattern.
#   * After a clue appears, HIPC runs a local greedy team-level TAA over this
#     robot and predictable peers, then commits its own three-cell path.
#   * Topic 3 carries lightweight owned-bundle snapshots for plan consensus.
#   * Repeated peer-prediction mismatches prune peers from local team planning.
#   * Topic 2 carries only next-step intent for low-level collision avoidance.
#   * Bump sensors detect the target; on a bump all robots halt and report.
#   * A clue is any intersection where the centered line sensor reads white.
#
# Threads:
#   * A background movement thread handles forward motion while the main thread processes UART and coordinates movement.
#   * The main thread plans paths and moves the robot, always stopping the
#     motors if the program exits unexpectedly.
#
# Tuning hints:
#   * Set UART pins and baud rate to match the hardware.
#   * Calibrate line sensors and adjust cfg.MIDDLE_WHITE_THRESH accordingly.
#   * Tune yaw timings (cfg.YAW_90_MS / cfg.YAW_180_MS) for your platform.

# ===========================================================
import random
import time
import _thread
import heapq
import sys
import gc
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
    """Fixed-grid mapping for cell-keyed allocator tables."""

    def __init__(self, grid_size, numeric=False):
        self.grid_size = int(grid_size)
        cell_count = self.grid_size * self.grid_size
        self._active = bytearray(cell_count)
        self._count = 0
        self._numeric = bool(numeric)
        if self._numeric:
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
    """Reusable bounded Top-K candidate workspace."""

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
                            cell_id, ids[insertion - 1],
                            target_p, map_index, origin):
                        ids[insertion] = ids[insertion - 1]
                        insertion -= 1
                    ids[insertion] = cell_id
                    count += 1
                    continue
                if not ranked:
                    self._sort_prefix(count, target_p, map_index, origin)
                    ranked = True
                if not self._precedes(
                        cell_id, ids[capacity - 1],
                        target_p, map_index, origin):
                    continue
                insertion = capacity - 1
                while insertion > 0 and self._precedes(
                        cell_id, ids[insertion - 1],
                        target_p, map_index, origin):
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


from machine import UART, Pin
from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
from pololu_3pi_2040_robot.buzzer import Buzzer

try:
    require_binary64()
except RuntimeError:
    print("WARNING: using native Pololu float precision")
del require_binary64
gc.collect()

# -----------------------------
# Robot identity & start pose
# -----------------------------
ROBOT_ID = "01"  # set to "00", "01", "02", or "03" at deployment
ALGORITHM_NAME = "HIPC"
GRID_SIZE = 19
EPS = 1.0e-9
TRIAL_MODE = "clue_search"
LOGIC_REVISION = "dcta_parity_v1"
COMMITMENT_HORIZON = 3
# Fraction of total grid cells retained by the post-clue candidate prefilter.
TOP_K_PERCENT = 1.0
if not (0.0 < TOP_K_PERCENT <= 1.0):
    raise ValueError("TOP_K_PERCENT must be greater than 0 and at most 1")
TOP_K_MAX_CELLS = max(1, int(GRID_SIZE * GRID_SIZE * TOP_K_PERCENT + 0.5))

DEBUG_LOG_FILE = "debug-log.txt"

METRICS_LOG_FILE = "metrics-log-HIPC.txt"
BOOT_TIME_MS = time.ticks_ms()
METRIC_START_TIME_MS = None  # set after first post-calibration intersection
start_signal = False  # set when hub command received
pre_start_signal = False  # set when hub pre-start command received
trial_active = False       # True only while trial metrics/search are active
abort_signal = False       # wake a stationary controller after an armed abort
returning_home = False     # suppress target completion while navigating home
return_home_blocked = False # bump detected during an unmetered return-home move
intersection_count = 0          # steps taken by this robot
task_cell_replan_count = 0      # post-clue replacement of an unsearched task cell
path_replan_count = 0           # post-clue path failures/collision replans
collision_prevention_count = 0  # post-clue collision-prevention events
target_location = None          # set when target is found

# Team membership (robot IDs as strings matching ROBOT_ID)
TEAM_IDS = ["00", "01", "02", "03"]
NUM_ROBOTS = len(TEAM_IDS)

#expiremental variables
msg_drop_rate = 0  # simulated message drop rate (0.0 to 1.0)
CONFIG_RATE_SCALE = 1000000
applied_config_sequence = 0
applied_top_k_ppm = CONFIG_RATE_SCALE
applied_drop_ppm = 0
applied_trial_mode = TRIAL_MODE
applied_commitment_horizon = COMMITMENT_HORIZON
applied_logic_revision = LOGIC_REVISION
applied_scenario_sha256 = ""
last_config_request = None
last_config_status = "OK"
control_state = "BOOT"

_metrics_logged = False
_metrics_cache = None
metrics_frozen = False
metric_freeze_time_ms = None
terminal_target_step_counted = False

buzzer = None  # replaced after hardware initialization

# Energy/Time metrics
motor_time_ms = 0              # cumulative ms motors were commanded non-zero
_motor_start_ms = None         # internal tracker for motor activity
candidate_filter_calls = 0
candidate_filter_time_us_total = 0
candidate_filter_time_us_max = 0
allocator_solve_time_us_total = 0
allocator_time_us_total = 0
allocator_calls = 0
allocator_time_us_max = 0

def finalize_motor_time(now_ticks=None):
    """Ensure motor_time_ms captures any active span before sampling metrics."""
    global _motor_start_ms, motor_time_ms
    if _motor_start_ms is not None:
        if now_ticks is None:
            now_ticks = time.ticks_ms()
        motor_time_ms += time.ticks_diff(now_ticks, _motor_start_ms)
        _motor_start_ms = None


def busy_timer_reset():
    """Start a fresh busy-time measurement for the current control loop."""
    global _busy_start_us, _busy_accum_us
    _busy_accum_us = 0
    _busy_start_us = time.ticks_us()


def busy_timer_pause():
    """Accumulate elapsed busy time and pause the timer."""
    global _busy_start_us, _busy_accum_us
    if _busy_start_us is not None:
        now_us = time.ticks_us()
        _busy_accum_us += time.ticks_diff(now_us, _busy_start_us)
        _busy_start_us = None


def busy_timer_resume():
    """Resume the busy-time timer after a pause."""
    global _busy_start_us
    _busy_start_us = time.ticks_us()


def busy_timer_value_ms():
    """Return the current busy time in milliseconds, pausing measurement."""
    global _busy_start_us, _busy_accum_us
    if _busy_start_us is not None:
        now_us = time.ticks_us()
        _busy_accum_us += time.ticks_diff(now_us, _busy_start_us)
        _busy_start_us = None
    return _busy_accum_us // 1000


def record_candidate_filter_time(start_us):
    global candidate_filter_calls, candidate_filter_time_us_total, candidate_filter_time_us_max
    if metrics_frozen:
        return
    elapsed_us = max(0, time.ticks_diff(time.ticks_us(), start_us))
    candidate_filter_calls += 1
    candidate_filter_time_us_total += elapsed_us
    if elapsed_us > candidate_filter_time_us_max:
        candidate_filter_time_us_max = elapsed_us


def record_allocator_solve_time(start_us, filter_time_before_us):
    global allocator_solve_time_us_total
    if metrics_frozen:
        return
    elapsed_us = max(0, time.ticks_diff(time.ticks_us(), start_us))
    filter_us = max(0, candidate_filter_time_us_total - filter_time_before_us)
    allocator_solve_time_us_total += max(0, elapsed_us - filter_us)


def record_allocator_time(start_us):
    global allocator_calls, allocator_time_us_total, allocator_time_us_max
    if metrics_frozen:
        return
    elapsed_us = max(0, time.ticks_diff(time.ticks_us(), start_us))
    allocator_calls += 1
    allocator_time_us_total += elapsed_us
    if elapsed_us > allocator_time_us_max:
        allocator_time_us_max = elapsed_us


def update_mem_headroom():
    """Refresh current free heap measurement and track the lowest observed value."""
    global mem_free_min
    current = gc.mem_free()
    if metrics_frozen:
        return current
    if current < mem_free_min:
        mem_free_min = current
    return current


def reset_trial_metrics():
    """Reset counters at the trial start so metrics share one time window."""
    global intersection_count, task_cell_replan_count, path_replan_count, collision_prevention_count
    global last_task_cell, collision_event_counted_since_move
    global motor_time_ms, _motor_start_ms, busy_ms, mem_free_min
    global candidate_filter_calls, candidate_filter_time_us_total, candidate_filter_time_us_max
    global allocator_solve_time_us_total, allocator_time_us_total
    global allocator_calls, allocator_time_us_max
    global topic_1_rec, topic_2_rec, topic_3_rec, topic_4_rec, topic_5_rec
    global topic_1_sent, topic_2_sent, topic_3_sent, topic_4_sent, topic_5_sent
    global bytes_sent, bytes_received, _metrics_logged, _metrics_cache
    global metrics_frozen, metric_freeze_time_ms, terminal_target_step_counted

    intersection_count = 0
    task_cell_replan_count = 0
    path_replan_count = 0
    collision_prevention_count = 0
    last_task_cell = None
    collision_event_counted_since_move = False
    motor_time_ms = 0
    _motor_start_ms = None
    busy_ms = 0
    mem_free_min = gc.mem_free()
    candidate_filter_calls = 0
    candidate_filter_time_us_total = 0
    candidate_filter_time_us_max = 0
    allocator_solve_time_us_total = 0
    allocator_time_us_total = 0
    allocator_calls = 0
    allocator_time_us_max = 0

    topic_1_rec = 0
    topic_2_rec = 0
    topic_3_rec = 0
    topic_4_rec = 0
    topic_5_rec = 0
    topic_1_sent = 0
    topic_2_sent = 0
    topic_3_sent = 0
    topic_4_sent = 0
    topic_5_sent = 0
    bytes_sent = 0
    bytes_received = 0
    _metrics_logged = False
    _metrics_cache = None
    metrics_frozen = False
    metric_freeze_time_ms = None
    terminal_target_step_counted = False


#message counters
topic_1_rec = 0
topic_2_rec = 0
topic_3_rec = 0
topic_4_rec = 0
topic_5_rec = 0
topic_1_sent = 0
topic_2_sent = 0
topic_3_sent = 0
topic_4_sent = 0
topic_5_sent = 0
bytes_sent = 0                 # raw UART bytes sent
bytes_received = 0             # raw UART bytes received
# Time metrics
busy_ms = 0                 # cumulative compute time spent outside motion/sleeps (ms)
mem_free_min = gc.mem_free()  # lowest observed free heap bytes

_busy_start_us = None       # internal timer start (microseconds)
_busy_accum_us = 0          # accumulated busy time (microseconds)


def log_error(message):
    """Log errors with a timestamp and play a low buzzer tone."""
    elapsed_ms = time.ticks_diff(time.ticks_ms(), BOOT_TIME_MS)
    try:
        with open(DEBUG_LOG_FILE, "a") as _fp:
            _fp.write(f"{elapsed_ms} ERROR: {message}\n")
    except (OSError, MemoryError):
        pass
    try:
        if buzzer is not None:
            buzzer.play("O2c16")
    except Exception:
        pass


def safe_assert(condition, message):
    if not condition:
        log_error(message)
        raise AssertionError(message)


def record_intersection(x, y):
    """Track this robot's completed intersection steps."""
    if metrics_frozen:
        return False
    safe_assert(0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE, "intersection out of range")
    global intersection_count
    intersection_count += 1
    return True


def freeze_trial_metrics(now_ticks=None):
    """Freeze exported trial counters at the first target alert."""
    global metrics_frozen, metric_freeze_time_ms, busy_ms
    if metrics_frozen:
        return False
    if now_ticks is None:
        now_ticks = time.ticks_ms()
    finalize_motor_time(now_ticks)
    busy_ms += busy_timer_value_ms()
    metric_freeze_time_ms = now_ticks
    metrics_frozen = True
    return True


def messaging_metrics():
    """Calculate time metrics only - power can be computed offline if needed."""
    # Compute time = everything except motor time

    # Message totals
    total_msgs_sent = topic_1_sent + topic_2_sent + topic_3_sent + topic_4_sent + topic_5_sent
    total_msgs_received = topic_1_rec + topic_2_rec + topic_3_rec + topic_4_rec + topic_5_rec

    return {
        'topic_1_received': topic_1_rec,
        'topic_1_sent': topic_1_sent,
        'topic_2_received': topic_2_rec,
        'topic_2_sent': topic_2_sent,
        'topic_3_received': topic_3_rec,
        'topic_3_sent': topic_3_sent,
        'topic_4_received': topic_4_rec,
        'topic_4_sent': topic_4_sent,
        'topic_5_received': topic_5_rec,
        'topic_5_sent': topic_5_sent,
        'msgs_received': total_msgs_received,
        'msgs_sent': total_msgs_sent,
    }

def metrics_log():
    """Write summary metrics for the search run and return them."""
    global busy_ms, mem_free_min, _metrics_logged, _metrics_cache
    if _metrics_logged and _metrics_cache is not None:
        return _metrics_cache
    start = METRIC_START_TIME_MS if METRIC_START_TIME_MS is not None else BOOT_TIME_MS
    now = metric_freeze_time_ms
    if now is None:
        now = time.ticks_ms()
        finalize_motor_time(now)
    elapsed_ms = time.ticks_diff(now, start)
    mean_step_time_ms = elapsed_ms / intersection_count if intersection_count > 0 else 0.0
    candidate_filter_time_us_mean = (
        candidate_filter_time_us_total / candidate_filter_calls
        if candidate_filter_calls > 0 else 0.0
    )
    allocator_time_us_mean = allocator_time_us_total / allocator_calls if allocator_calls > 0 else 0.0
    allocator_time_pct = (
        allocator_time_us_total * 100.0 / (elapsed_ms * 1000)
        if elapsed_ms > 0 else 0.0
    )
    mem_total = gc.mem_alloc() + gc.mem_free()
    mem_used_peak = mem_total - mem_free_min
    cpu_util_pct = (busy_ms * 100) // elapsed_ms if elapsed_ms > 0 else 0
    metric_target_location = f"{target_location[0]}/{target_location[1]}" if target_location is not None else (-1, -1)
    # Calculate time metrics
    messaging = messaging_metrics()

    metrics = {
        "robot_id": ROBOT_ID,
        "target_location": metric_target_location,
        "alg": ALGORITHM_NAME,
        "top_k_rate": TOP_K_PERCENT,
        "top_k_max_cells": TOP_K_MAX_CELLS,
        "drop_rate": msg_drop_rate,
        "config_sequence": applied_config_sequence,
        "trial_mode": applied_trial_mode,
        "commitment_horizon": applied_commitment_horizon,
        "logic_revision": applied_logic_revision,
        "scenario_sha256": applied_scenario_sha256,
        "steps": intersection_count,
        "msgs_sent": messaging['msgs_sent'],
        "msgs_received": messaging['msgs_received'],
        "1_rec": messaging['topic_1_received'],
        "1_sent": messaging['topic_1_sent'],
        "2_rec": messaging['topic_2_received'],
        "2_sent": messaging['topic_2_sent'],
        "3_rec": messaging['topic_3_received'],
        "3_sent": messaging['topic_3_sent'],
        "4_rec": messaging['topic_4_received'],
        "4_sent": messaging['topic_4_sent'],
        "5_rec": messaging['topic_5_received'],
        "5_sent": messaging['topic_5_sent'],
        "bytes_sent": bytes_sent,
        "bytes_received": bytes_received,
        "motor_time_ms": motor_time_ms,
        "trial_time_ms": elapsed_ms,
        "cpu_util_pct": cpu_util_pct,
        "mem_used_peak": mem_used_peak,
        "mem_free_min": mem_free_min,
        "candidate_filter_calls": candidate_filter_calls,
        "candidate_filter_time_us_total": candidate_filter_time_us_total,
        "candidate_filter_time_us_mean": candidate_filter_time_us_mean,
        "candidate_filter_time_us_max": candidate_filter_time_us_max,
        "allocator_solve_time_us_total": allocator_solve_time_us_total,
        "allocator_calls": allocator_calls,
        "allocator_time_us_total": allocator_time_us_total,
        "allocator_time_us_mean": allocator_time_us_mean,
        "allocator_time_us_max": allocator_time_us_max,
        "allocator_time_pct": allocator_time_pct,
        "mean_step_time_ms": mean_step_time_ms,
        "task_cell_replans": task_cell_replan_count,
        "path_replans": path_replan_count,
        "collision_prevention_events": collision_prevention_count,
    }

    fieldnames = [
        "robot_id",
        "target_location",
        "alg",
        "top_k_rate",
        "top_k_max_cells",
        "drop_rate",
        "config_sequence",
        "trial_mode",
        "commitment_horizon",
        "logic_revision",
        "scenario_sha256",
        "steps",
        "msgs_sent",
        "msgs_received",
        "1_rec",
        "1_sent",
        "2_rec",
        "2_sent",
        "3_rec",
        "3_sent",
        "4_rec",
        "4_sent",
        "5_rec",
        "5_sent",
        "bytes_sent",
        "bytes_received",
        "motor_time_ms",
        "trial_time_ms",
        "cpu_util_pct",
        "mem_used_peak",
        "mem_free_min",
        "candidate_filter_calls",
        "candidate_filter_time_us_total",
        "candidate_filter_time_us_mean",
        "candidate_filter_time_us_max",
        "allocator_solve_time_us_total",
        "allocator_calls",
        "allocator_time_us_total",
        "allocator_time_us_mean",
        "allocator_time_us_max",
        "allocator_time_pct",
        "mean_step_time_ms",
        "task_cell_replans",
        "path_replans",
        "collision_prevention_events",
    ]

    try:
        try:
            with open(METRICS_LOG_FILE) as _fp:
                write_header = _fp.read(1) == ""
        except OSError:
            write_header = True
        with open(METRICS_LOG_FILE, "a") as _fp:
            if write_header:
                _fp.write(",".join(fieldnames) + "\n")
            _fp.write(",".join(str(metrics[f]) for f in fieldnames) + "\n")
    except OSError:
        pass
    _metrics_cache = metrics
    _metrics_logged = True
    return metrics


try:
    open(DEBUG_LOG_FILE, "a").close()
except OSError:
    pass

try:
    open(METRICS_LOG_FILE, "a").close()
except OSError:
    pass

# Starting position & heading (grid coordinates, cardinal heading)
# pos = (x, y)    heading = (dx, dy) where (0,1)=N, (1,0)=E, (0,-1)=S, (-1,0)=W
START_CONFIG = {
    "00": ((0, 0), (1, 0)),                       # west edge, evenly spaced facing east
    "01": ((0, 6), (1, 0)),
    "02": ((0, 12), (1, 0)),
    "03": ((0, 18), (1, 0)),
}
DIRS4 = ((0, 1), (1, 0), (0, -1), (-1, 0))

try:
    START_POS, START_HEADING = START_CONFIG[ROBOT_ID]
except KeyError as e:
    raise ValueError("ROBOT_ID must be one of '00', '01', '02', or '03'") from e
safe_assert(0 <= START_POS[0] < GRID_SIZE and 0 <= START_POS[1] < GRID_SIZE,
            "start position out of bounds")

#for starting partition
BANDS = {
    "00": (0, 4),
    "01": (5, 9),
    "02": (10, 14),
    "03": (15, 18),
}
try:
    BAND_Y_MIN, BAND_Y_MAX = BANDS[ROBOT_ID]
except KeyError as e:
    raise ValueError("No band defined for this ROBOT_ID") from e

safe_assert(0 <= BAND_Y_MIN <= BAND_Y_MAX < GRID_SIZE, "band rows out of bounds")
safe_assert(BAND_Y_MIN <= START_POS[1] <= BAND_Y_MAX,
            "start row must lie inside this robot's band")

# UART0 for ESP32 communication (TX=GP28, RX=GP29)
uart = UART(
    0, baudrate=115200, tx=28, rx=29,
    rxbuf=4096, txbuf=1024, timeout=1000, timeout_char=10,
)

# -----------------------------
# Grid / Maps / Shared State
# -----------------------------
# Grid cell states
CELL_UNSEARCHED = 0
CELL_OBSTACLE   = 1  # physical/static obstacle marker if used
CELL_SEARCHED   = 2

grid = bytearray(GRID_SIZE * GRID_SIZE)
# target_p is the single search-value map. prob_map is kept as an alias-style
# working array for A* compatibility and is always copied from target_p.
prob_map = array('d', [1 / (GRID_SIZE * GRID_SIZE)] * (GRID_SIZE * GRID_SIZE))
REWARD_FACTOR = 5
clues = []                            # list of (x, y) clue cells

# --- Target belief map ---
# P_target[i]: belief target is at cell i. There is no separate clue-value map.
target_p = array('d', [1 / (GRID_SIZE * GRID_SIZE)] * (GRID_SIZE * GRID_SIZE))
allocation_probability_normalizer = 1.0 / (GRID_SIZE * GRID_SIZE)

# --- Decay exponent (tunable) ---
# Higher exponent -> stronger / narrower target probability around clues.
TARGET_DECAY_EXP = 1.0


# Preallocated arrays for A* planning
# ----------------------------------
# Parent indices and path costs for each cell are stored here. Reusing these
# arrays each planning cycle avoids repeated allocations, which are expensive
# on MicroPython.
came_from = array('i', [-1] * (GRID_SIZE * GRID_SIZE))
cost_so_far = array('d', [0.0] * (GRID_SIZE * GRID_SIZE))
frontier = []


def idx(x, y):
    """Convert Cartesian (x, y) to linear index in map arrays."""
    safe_assert(0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE, "idx out of range")
    return (GRID_SIZE - 1 - y) * GRID_SIZE + x

def manhattan(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


def renorm(arr):
    """Normalize an array of floats in-place so it sums to 1 (if possible)."""
    total = 0.0
    for v in arr:
        total += v
    if total <= 0.0:
        # fallback: uniform over all cells
        n = GRID_SIZE * GRID_SIZE
        val = 1.0 / n
        for i in range(n):
            arr[i] = val
        return
    inv = 1.0 / total
    for i in range(len(arr)):
        arr[i] *= inv


def recompute_value_map():
    """Copy target_p into prob_map so target_p and prob_map stay identical."""
    global allocation_probability_normalizer
    n = GRID_SIZE * GRID_SIZE
    maximum = 0.0
    for i in range(n):
        prob_map[i] = target_p[i]
        if target_p[i] > maximum:
            maximum = target_p[i]
    allocation_probability_normalizer = (
        maximum if 0.0 < maximum < float("inf") else 1.0)


pos = [START_POS[0], START_POS[1]]    # current grid position
heading = (START_HEADING[0], START_HEADING[1])

# Flags used by threads for clean exits
running = True                         # global run flag
found_target = False                   # set True on bump or peer alert
target_bump_stop = False               # True only when this robot's bump interrupted a cell move
first_clue_seen = False                # once True, disable lawn‑mower bias
move_forward_flag = False

# Peer state used for safety and shared situational awareness.
peer_intent = {}      # peer_id -> (x, y) next-step safety intent only
peer_pos = {}         # peer_id -> (x, y) last reported position (post-drop)
peer_pos_yield = {}   # peer_id -> (x, y) last reported position for collision checks
published_clues = set()  # each locally detected or forwarded clue is sent once
communicated_intent = None
current_task_cell = None   # local internal task cell only; never published as a reservation
last_task_cell = None
collision_event_counted_since_move = False
blocked_goal_cell = None
blocked_goal_conflicts = 0
temporary_invalid_task_until = {}
pending_collision_reallocation = False

# -----------------------------
# HIPC allocator state
# -----------------------------
# HIPC performs a local team-level greedy TAA, commits only this robot's
# three-cell bundle, and exchanges lightweight bundle-consensus snapshots.
HIPC_BUNDLE_SIZE = COMMITMENT_HORIZON
HIPC_NO_WINNER_CODE = "99"
HIPC_EMPTY_FIELD = "X"
HIPC_NO_BID = -1.0e18
HIPC_NO_TIME = -1.0e18
HIPC_EPS_BID = EPS
HIPC_BAD_PRED_LIMIT = 3
HIPC_PREDICTION_TOLERANCE = 0

hipc_winner_by_cell = CellIndexedMap(GRID_SIZE)
hipc_winning_bid_by_cell = CellIndexedMap(GRID_SIZE, numeric=True)
hipc_bid_time_by_cell = CellIndexedMap(GRID_SIZE, numeric=True)
hipc_path = []
hipc_bundle = []
hipc_bid_counter = 0
hipc_clue_signature = None
hipc_pending_snapshot = False
hipc_last_sent_signature = None
hipc_bad_prediction_count = {}
hipc_last_predicted_peer_first_task = {}
hipc_seen_peer_bundle_signature = {}
hipc_dropped_peers = set()
hipc_candidate_workspace = PackedCandidateWorkspace(
    GRID_SIZE, TOP_K_MAX_CELLS)


def _apply_top_k_capacity(capacity):
    global hipc_candidate_workspace
    if (
        hipc_candidate_workspace is not None
        and hipc_candidate_workspace.capacity == capacity
    ):
        return
    hipc_candidate_workspace = None
    gc.collect()
    hipc_candidate_workspace = PackedCandidateWorkspace(
        GRID_SIZE, capacity)


TURN_COST = 0.3

# -----------------------------
# Motion configuration
# -----------------------------
class MotionConfig:
    def __init__(self):
        self.MIDDLE_WHITE_THRESH = 200  # center sensor threshold for "white" (tune by calibration)
        self.VISITED_STEP_PENALTY = 4
        self.KP = 0.7                # proportional gain around LINE_CENTER
        self.CALIBRATE_SPEED = 1140  # speed to rotate when calibrating
        self.BASE_SPEED = 650        # nominal wheel speed
        self.MIN_SPD = 350           # clamp low (avoid stall)
        self.MAX_SPD = 1100          # clamp high
        self.LINE_CENTER = 2000      # weighted position target (0..4000)
        self.BLACK_THRESH = 600      # calibrated "black" threshold (0..1000)
        self.STRAIGHT_CREEP = 650    # forward speed while "locked" straight
        self.START_LOCK_MS = 250     # hold straight this long after function starts
        self.TURN_SPEED = 1000
        self.YAW_90_MS = 0.31
        self.YAW_180_MS = 0.61

cfg = MotionConfig()

#UART handling globals
DELIM = ord('-')

# ---------- bounded streaming frame parser ----------
MSG_BUF_SIZE = 256
msg_buf = bytearray(MSG_BUF_SIZE)
msg_len = 0
rx_discarding_oversize = False

# ---------- serialized outbound framing ----------
TX_BUF_SIZE = 256
tx_buf = bytearray(TX_BUF_SIZE)
tx_view = memoryview(tx_buf)
uart_tx_lock = _thread.allocate_lock()
UART_WRITE_DEADLINE_MS = 1500
uart_tx_failed = False

def _msg_buf_ascii(length):
    """Convert buffered UART protocol bytes to ASCII without UTF-8 decoding."""
    chars = []
    for i in range(length):
        b = msg_buf[i]
        if 32 <= b <= 126:
            chars.append(chr(b))
    return "".join(chars).strip()

def _write_int(buf, idx, val):
    """Write an integer as ASCII into buf starting at idx.

    Returns the new index after writing."""
    if val < 0:
        buf[idx] = ord('-')
        idx += 1
        val = -val
    if val == 0:
        buf[idx] = ord('0')
        return idx + 1
    # Determine number of digits
    tmp = val
    digits = 0
    while tmp:
        tmp //= 10
        digits += 1
    end = idx + digits
    for _ in range(digits):
        buf[end - 1] = ord('0') + (val % 10)
        val //= 10
        end -= 1
    return idx + digits

# -----------------------------
# Hardware interfaces
# -----------------------------
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump = robot.BumpSensors()
rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(10)
buzzer = Buzzer()

# ===========================================================
# Utility: Motors & Stop Control
# ===========================================================

RED   = (230, 0, 0)
GREEN = (0, 230, 0)
BLUE = (0, 0, 230)
OFF   = (0, 0, 0)

def flash_LEDS(color, n):
    for _ in range(n):
        for led in range(6):
            rgb_leds.set(led, color)  # reuses same tuple, no new allocation
        rgb_leds.show()
        time.sleep_ms(100)
        for led in range(6):
            rgb_leds.set(led, OFF)
        rgb_leds.show()
        time.sleep_ms(100)

def flash_trial_count():
    """Flash green once per metrics log entry (excluding header)."""
    try:
        with open(METRICS_LOG_FILE) as fp:
            lines = fp.readlines()
    except OSError:
        return
    count = len(lines)
    if count and lines[0].strip().lower().startswith("robot_id"):
        count -= 1
    if count < 0:
        count = 0
    for _ in range(count):
        flash_LEDS(GREEN, 1)
        time.sleep_ms(100)

def buzz(event):
    """
    Play short chirps for turn, intersection, clue,
    and a longer sequence for target.
    """
    if event == "turn":
        buzzer.play("O5c16")            # short high chirp
    elif event == "intersection":
        buzzer.play("O4g16")            # short mid chirp
    elif event == "clue":
        buzzer.play("O6e16")            # short very high chirp
    elif event == "target":
        buzzer.play("O4c8e8g8c5")       # longer sequence, rising melody



def set_speeds(left, right):
    """Wrapper to track motor active time before delegating to hardware."""
    global _motor_start_ms
    if left != 0 or right != 0:
        if not metrics_frozen and _motor_start_ms is None:
            _motor_start_ms = time.ticks_ms()
    else:
        finalize_motor_time()
    motors.set_speeds(left, right)


def motors_off():
    """Hard stop both wheels (safety: call in finally/stop paths)."""
    set_speeds(0, 0)

def stop_all():
    """
    Idempotent global stop:
      - Set flags so all loops/threads exit
      - Ensure motors are off
      - Set a green LED to indicate finished
    """
    global running, move_forward_flag
    running = False
    move_forward_flag = False
    publish_intent()
    motors_off()

def stop_and_alert_target():
    """
    Called when THIS robot detects the target via bump.
    Publishes the alert and ends only the current trial. The controller and
    movement thread remain alive so the robot can return home.

    The robot may bump into the target before reaching the next
    intersection, leaving ``pos`` pointing to the last intersection it
    successfully crossed.  Report the target at the *next* intersection in
    the current heading direction so external consumers know where it is.
    """
    global target_location, found_target, move_forward_flag, target_bump_stop
    global terminal_target_step_counted
    detected_at_ms = time.ticks_ms()
    next_x = pos[0] + heading[0]
    next_y = pos[1] + heading[1]
    if target_bump_stop:
        return
    target_location = (next_x, next_y)
    target_bump_stop = True
    if (
        trial_active and not metrics_frozen
        and 0 <= next_x < GRID_SIZE and 0 <= next_y < GRID_SIZE
    ):
        # The bump is the simulator-equivalent terminal entry. Keep ``pos`` at
        # the last physical intersection so retreat remains correct.
        record_intersection(next_x, next_y)
        grid[idx(next_x, next_y)] = CELL_SEARCHED
        terminal_target_step_counted = True
    # Stop timing at the bump instant, but include the terminal protected
    # messages in the frozen communication counters.
    finalize_motor_time(detected_at_ms)
    motors_off()
    found_target = True
    move_forward_flag = False
    try:
        publish_target(next_x, next_y)
    finally:
        freeze_trial_metrics(detected_at_ms)
        motors_off()
    buzz('target')
    flash_LEDS(BLUE, 1)
# ===========================================================
# UART Messaging
# Format: "<topic#>.<payload>-" sent from Pololu to ESP32.
# position/state = 1, next-step intent = 2, HIPC allocation entry = 3,
# clue = 4, target/alert = 5, hub command = 7
#
# Topic 3 is the only HIPC allocator-specific exchange. Payload format:
#   x,y,winner,bid,t,b0x,b0y,b1x,b1y,b2x,b2y
# A clear snapshot uses X,X,99. Missing bundle cells use X,X, and negative bids
# are encoded as "N<abs>" because '-' is the UART frame delimiter.
# Topic-3 payloads must not contain '-' because '-' is the UART frame delimiter.
# Examples:
#   001.3,4-                         robot 00 position/state
#   002.7,8-                         robot 00 next-step intent
#   003.7,8,00,123456,4,7,8,7,9,X,X-  robot 00 HIPC entry
#   004.5,2-                         robot 00 clue
# ===========================================================
def _uart_write_all_locked(frame_len):
    """Write one frame completely while ``uart_tx_lock`` is held."""
    global uart_tx_failed
    offset = 0
    deadline = time.ticks_add(time.ticks_ms(), UART_WRITE_DEADLINE_MS)
    try:
        while offset < frame_len:
            if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
                raise OSError(
                    "UART write timeout ({}/{})".format(offset, frame_len)
                )
            written = uart.write(tx_view[offset:frame_len])
            if written is None or written == 0:
                time.sleep_ms(1)
                continue
            if written < 0 or written > frame_len - offset:
                raise OSError("UART write returned invalid length")
            offset += written
    except Exception:
        uart_tx_failed = True
        raise
    return frame_len


def uart_send(topic, payload_len):
    """Finish and write the prepared shared-buffer frame with the TX lock held."""
    global bytes_sent
    frame_len = payload_len + 3
    if len(topic) != 1 or frame_len > TX_BUF_SIZE:
        raise ValueError("invalid UART frame")
    for index in range(2, payload_len + 2):
        if tx_buf[index] == DELIM:
            raise ValueError("UART payload contains frame delimiter")
    tx_buf[0] = ord(topic)
    tx_buf[1] = ord('.')
    tx_buf[payload_len + 2] = DELIM
    _uart_write_all_locked(frame_len)
    if not metrics_frozen:
        bytes_sent += frame_len
    return frame_len


def _uart_send_text(topic, payload, count_bytes=True):
    """Build and write one text frame atomically using the shared TX buffer."""
    global bytes_sent
    payload = str(payload)
    frame_len = len(payload) + 3
    if len(topic) != 1 or frame_len > TX_BUF_SIZE or "-" in payload:
        raise ValueError("invalid UART frame")
    uart_tx_lock.acquire()
    try:
        tx_buf[0] = ord(topic)
        tx_buf[1] = ord('.')
        for index in range(len(payload)):
            code = ord(payload[index])
            if code < 32 or code > 126:
                raise ValueError("UART payload must be printable ASCII")
            tx_buf[index + 2] = code
        tx_buf[frame_len - 1] = DELIM
        _uart_write_all_locked(frame_len)
    finally:
        uart_tx_lock.release()
    if count_bytes and not metrics_frozen:
        bytes_sent += frame_len
    return frame_len

def publish_position():
    """Publish current pose (for UI/diagnostics)."""
    global topic_1_sent
    uart_tx_lock.acquire()
    try:
        i = 2
        i = _write_int(tx_buf, i, pos[0])
        tx_buf[i] = ord(','); i += 1
        i = _write_int(tx_buf, i, pos[1])
        uart_send('1', i - 2)
    finally:
        uart_tx_lock.release()
    if _trial_traffic_enabled() and not metrics_frozen:
        topic_1_sent += 1

def publish_clue(x, y):
    """Publish a clue at (x,y)."""
    global topic_4_sent
    clue = (int(x), int(y))
    if clue in published_clues:
        return False
    uart_tx_lock.acquire()
    try:
        i = 2
        i = _write_int(tx_buf, i, x)
        tx_buf[i] = ord(','); i += 1
        i = _write_int(tx_buf, i, y)
        uart_send('4', i - 2)
    finally:
        uart_tx_lock.release()
    published_clues.add(clue)
    if not metrics_frozen:
        topic_4_sent += 1
    return True

def publish_target(x, y):
    """Publish that we found the target at (x,y)."""
    global topic_5_sent
    uart_tx_lock.acquire()
    try:
        i = 2
        i = _write_int(tx_buf, i, x)
        tx_buf[i] = ord(','); i += 1
        i = _write_int(tx_buf, i, y)
        uart_send('5', i - 2)
    finally:
        uart_tx_lock.release()
    if not metrics_frozen:
        topic_5_sent += 1

def publish_intent(x=None, y=None):
    """
    Publish our intended next cell for low-level collision avoidance only.
    This is not an HIPC claim, task owner, or task-cell reservation.
    """
    global topic_2_sent, communicated_intent
    intent = None if x is None or y is None else (int(x), int(y))
    current_cell = (int(pos[0]), int(pos[1]))
    intent_signature = (current_cell, intent)
    if intent_signature == communicated_intent:
        return False
    uart_tx_lock.acquire()
    try:
        i = 2
        i = _write_int(tx_buf, i, current_cell[0])
        tx_buf[i] = ord(','); i += 1
        i = _write_int(tx_buf, i, current_cell[1])
        tx_buf[i] = ord(','); i += 1
        if intent is None:
            tx_buf[i] = ord('X'); i += 1
            tx_buf[i] = ord(','); i += 1
            tx_buf[i] = ord('X'); i += 1
        else:
            i = _write_int(tx_buf, i, intent[0])
            tx_buf[i] = ord(','); i += 1
            i = _write_int(tx_buf, i, intent[1])
        uart_send('2', i - 2)
    finally:
        uart_tx_lock.release()
    if not metrics_frozen:
        topic_2_sent += 1
    communicated_intent = intent_signature
    return True


def publish_hipc_payload(payload):
    """Publish one compact HIPC table-delta payload on topic 3."""
    global topic_3_sent
    if not _trial_traffic_enabled():
        return
    _uart_send_text("3", payload)
    if not metrics_frozen:
        topic_3_sent += 1

def _valid_scenario_sha256(value):
    return (
        len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _send_config_ack(
        sequence, top_k_ppm, top_k_cells, drop_ppm, trial_mode,
        horizon, logic_revision, scenario_sha256, status):
    payload = "CFGACK,{},{},{},{},{},{},{},{},{},{}".format(
        sequence, ALGORITHM_NAME, top_k_ppm, top_k_cells, drop_ppm,
        trial_mode, horizon, logic_revision, scenario_sha256, status)
    _uart_send_text("6", payload, False)


def _send_command_ack(sequence, state):
    payload = "CMDACK,{},{},{}".format(sequence, ROBOT_ID, state)
    _uart_send_text("6", payload, False)


def _trial_traffic_enabled():
    return start_signal or control_state == "STARTED"


def _clear_start_transport_caches():
    global communicated_intent
    peer_pos.clear()
    peer_pos_yield.clear()
    peer_intent.clear()
    communicated_intent = None


def _handle_control_command(payload):
    """Apply one sequence-tagged PRESTART/START/RUN/ABORT transition."""
    global control_state, pre_start_signal, start_signal
    global found_target, move_forward_flag, abort_signal
    global METRIC_START_TIME_MS, trial_active
    try:
        fields = payload.strip().split(",")
        if len(fields) != 3 or fields[0] != "CMD":
            return False
        command = fields[1]
        sequence = int(fields[2])
    except (ValueError, IndexError):
        return False
    if sequence <= 0 or sequence != applied_config_sequence:
        return False
    if command == "PRESTART":
        if control_state == "CONFIGURED":
            pre_start_signal = True
            control_state = "READY"
        elif control_state != "READY":
            return False
        _send_command_ack(sequence, "READY")
        return True
    if command == "START":
        if control_state == "READY":
            _clear_start_transport_caches()
            reset_trial_metrics()
            start_signal = False
            control_state = "STARTED"
        elif control_state not in ("STARTED", "RUNNING"):
            return False
        _send_command_ack(sequence, "STARTED")
        return True
    if command == "RUN":
        if control_state == "STARTED":
            if found_target:
                abort_signal = True
                control_state = "ABORTED"
                _send_command_ack(sequence, "ABORTED")
                return True
            METRIC_START_TIME_MS = time.ticks_ms()
            trial_active = True
            start_signal = True
            control_state = "RUNNING"
        elif control_state != "RUNNING":
            return False
        _send_command_ack(sequence, "RUNNING")
        return True
    if command == "ABORT":
        if control_state not in (
            "CONFIGURED", "READY", "STARTED", "RUNNING", "ABORTED"
        ):
            return False
        if control_state != "ABORTED":
            abort_signal = True
            pre_start_signal = False
            start_signal = False
            move_forward_flag = False
            if trial_active:
                found_target = True
                freeze_trial_metrics()
            control_state = "ABORTED"
        _send_command_ack(sequence, "ABORTED")
        return True
    return False


def _handle_config_command(payload):
    global TOP_K_PERCENT, TOP_K_MAX_CELLS, msg_drop_rate
    global applied_config_sequence, applied_top_k_ppm, applied_drop_ppm
    global applied_trial_mode, applied_commitment_horizon
    global applied_logic_revision, applied_scenario_sha256
    global last_config_request, last_config_status
    global control_state

    sequence = 0
    top_k_ppm = 0
    top_k_cells = 0
    drop_ppm = 0
    trial_mode = TRIAL_MODE
    horizon = COMMITMENT_HORIZON
    logic_revision = LOGIC_REVISION
    scenario_sha256 = "0" * 64
    try:
        fields = payload.strip().split(",")
        if len(fields) != 9 or fields[0] != "CFG":
            raise ValueError
        sequence = int(fields[1])
        top_k_ppm = int(fields[2])
        top_k_cells = int(fields[3])
        drop_ppm = int(fields[4])
        trial_mode = fields[5].strip()
        horizon = int(fields[6])
        logic_revision = fields[7].strip()
        scenario_sha256 = fields[8].strip().lower()
    except (ValueError, IndexError):
        _send_config_ack(
            sequence, top_k_ppm, top_k_cells, drop_ppm, trial_mode,
            horizon, logic_revision, scenario_sha256, "INVALID")
        return

    request = (
        sequence, top_k_ppm, top_k_cells, drop_ppm, trial_mode,
        horizon, logic_revision, scenario_sha256)
    if request == last_config_request:
        _send_config_ack(
            sequence, top_k_ppm, top_k_cells, drop_ppm,
            trial_mode, horizon, logic_revision, scenario_sha256,
            last_config_status)
        return
    applied_request = (
        applied_config_sequence, applied_top_k_ppm, TOP_K_MAX_CELLS,
        applied_drop_ppm, applied_trial_mode,
        applied_commitment_horizon, applied_logic_revision,
        applied_scenario_sha256)
    if request == applied_request:
        last_config_request = request
        last_config_status = "OK"
        _send_config_ack(
            sequence, top_k_ppm, top_k_cells, drop_ppm,
            trial_mode, horizon, logic_revision, scenario_sha256, "OK")
        return

    expected_cells = max(
        1,
        (GRID_SIZE * GRID_SIZE * top_k_ppm
         + CONFIG_RATE_SCALE // 2) // CONFIG_RATE_SCALE,
    )
    status = "OK"
    if (
        trial_active or start_signal or pre_start_signal or returning_home
        or sequence <= applied_config_sequence
        or not (0 < top_k_ppm <= CONFIG_RATE_SCALE)
        or not (0 <= drop_ppm <= CONFIG_RATE_SCALE)
        or top_k_cells != expected_cells
        or trial_mode != TRIAL_MODE
        or horizon != COMMITMENT_HORIZON
        or logic_revision != LOGIC_REVISION
        or not _valid_scenario_sha256(scenario_sha256)
    ):
        status = "INVALID"
    else:
        try:
            _apply_top_k_capacity(top_k_cells)
            TOP_K_PERCENT = top_k_ppm / CONFIG_RATE_SCALE
            TOP_K_MAX_CELLS = top_k_cells
            msg_drop_rate = drop_ppm / CONFIG_RATE_SCALE
            applied_config_sequence = sequence
            applied_top_k_ppm = top_k_ppm
            applied_drop_ppm = drop_ppm
            applied_trial_mode = trial_mode
            applied_commitment_horizon = horizon
            applied_logic_revision = logic_revision
            applied_scenario_sha256 = scenario_sha256
            control_state = "CONFIGURED"
        except MemoryError:
            status = "MEMORY_ERROR"

    last_config_request = request
    last_config_status = status
    _send_config_ack(
        sequence, top_k_ppm, top_k_cells, drop_ppm, trial_mode,
        horizon, logic_revision, scenario_sha256, status)


def handle_msg(line):
    """
    Parse and apply incoming messages from the other robot or hub.

    Accepts:
    011.3,4-       # topic 1: position (x,y only) - previous pos treated as visited
    002.7,8-       # topic 2: intent
    003.7,8,00,123456,4,7,8,X,X,X,X-  # topic 3: HIPC entry
    004.5,2-       # topic 4: clue
    005.6,1-       # topic 5: target/alert
    996.1-         # topic 7: hub command

    Ignores:
      - other status fields we don't currently need
    """
    global pre_start_signal, peer_intent, peer_pos, current_task_cell
    global first_clue_seen, target_location, start_signal, found_target
    global move_forward_flag, communicated_intent

    # Minimal parsing: "<sender>/<topic>:<payload>"
    try:
        left, payload = line.split(".", 1)
        if len(left) < 3:
            return
        sender = left[0:2]
        topic  = left[2]
    except ValueError:
        return
    if sender == ROBOT_ID:
        return

    if topic == "1": #position
        global topic_1_rec
        try:
            ox, oy = map(int, payload.split(","))
        except ValueError:
            return
        if not (0 <= ox < GRID_SIZE and 0 <= oy < GRID_SIZE):
            return
        # Topic-1 frames are also used as pre-trial home/readiness beacons.
        # Keep return-home routing current, but command 2 is the boundary at
        # which a delivered position becomes a canonical search observation.
        if not _trial_traffic_enabled():
            if returning_home:
                peer_pos[sender] = (ox, oy)
            return
        if random.random() < msg_drop_rate:
            return  # simulate message drop
        if not metrics_frozen:
            topic_1_rec += 1
        peer_pos[sender] = (ox, oy)
        current_index = idx(ox, oy)
        if grid[current_index] != CELL_SEARCHED:
            grid[current_index] = CELL_SEARCHED
            update_target_on_miss(current_index)
        if current_task_cell == (ox, oy) and (pos[0], pos[1]) != (ox, oy):
            current_task_cell = None

    elif topic == "2": #intent
        global topic_2_rec
        if not metrics_frozen:
            topic_2_rec += 1
        fields = payload.split(",")
        if len(fields) != 4:
            return
        try:
            px = int(fields[0])
            py = int(fields[1])
        except ValueError:
            return
        if not (0 <= px < GRID_SIZE and 0 <= py < GRID_SIZE):
            return
        peer_pos_yield[sender] = (px, py)
        if fields[2] == "X" and fields[3] == "X":
            peer_intent.pop(sender, None)
            return
        try:
            ix = int(fields[2])
            iy = int(fields[3])
        except ValueError:
            return
        if 0 <= ix < GRID_SIZE and 0 <= iy < GRID_SIZE:
            peer_intent[sender] = (ix, iy)

    elif topic == "3": # HIPC allocation entry, droppable
        if not _trial_traffic_enabled():
            return
        if random.random() < msg_drop_rate:
            return
        global topic_3_rec
        if not metrics_frozen:
            topic_3_rec += 1
        _hipc_receive_payload(sender, payload)

    elif topic == "4":   #clue
        if not _trial_traffic_enabled():
            return
        if random.random() < msg_drop_rate:
            return  # simulate message drop
        global topic_4_rec
        if not metrics_frozen:
            topic_4_rec += 1
        try:
            x, y = map(int, payload.split(","))
        except ValueError:
            return
        if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
            clue = (x, y)
            if clue not in clues:
                clues.append(clue)
                first_clue_seen = True
                i = idx(clue[0], clue[1])
                grid[i] = CELL_SEARCHED
                update_prob_map()
                publish_clue(x, y)
                gc.collect()

    elif topic == "5": #target
        # Peer found the target: finish this trial without killing the program.
        if not _trial_traffic_enabled():
            return
        global topic_5_rec
        if not metrics_frozen:
            topic_5_rec += 1
        try:
            x, y = map(int, payload.split(","))
            target_location = (x, y)
        except ValueError:
            target_location = None
        if not returning_home:
            # Stop physical motion before processing the peer's target alert.
            motors_off()
            move_forward_flag = False
            found_target = True
            if trial_active:
                freeze_trial_metrics()

    elif topic == "7":  # hub command
        if payload.strip().startswith("CFG,"):
            _handle_config_command(payload)
        elif payload.strip().startswith("CMD,"):
            _handle_control_command(payload)

def _rx_feed_bytes(data):
    """Parse an arbitrary UART chunk without a lossy downstream ring."""
    global msg_len, rx_discarding_oversize, bytes_received
    completed = 0
    for b in data:
        if b == DELIM:
            if rx_discarding_oversize:
                rx_discarding_oversize = False
                msg_len = 0
                continue
            if msg_len:
                frame = _msg_buf_ascii(msg_len)
                msg_len = 0
                if frame:
                    left = frame.split(".", 1)[0]
                    if (
                        _trial_traffic_enabled() and not metrics_frozen
                        and len(left) >= 3 and left[2] in "12345"
                    ):
                        bytes_received += len(frame) + 1
                    handle_msg(frame)
                    completed += 1
            continue
        if rx_discarding_oversize:
            continue
        if msg_len < MSG_BUF_SIZE:
            msg_buf[msg_len] = b
            msg_len += 1
        else:
            msg_len = 0
            rx_discarding_oversize = True
    return completed

# ---------- UART service ----------
def uart_service():
    """Read and parse any complete messages from UART."""
    while True:
        available = uart.any()
        if not available:
            return
        data = uart.read(min(available, 256))
        if not data:
            return
        _rx_feed_bytes(data)

# ===========================================================
# Sensing & Motion
# ===========================================================
def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def move_forward_one_cell():
    """
    Drive forward following the line until an intersection is detected:
      - T or + intersections: trigger if either outer sensor is black.
      - Require 3 consecutive qualifying reads (debounce).
      - On first candidate, lock steering straight (no P-correction)
        until intersection is confirmed → avoids grabbing side lines.
      - Also hold a 0.5 s straight "roll-through" at start to clear
        the cross you’re sitting on before re-engaging P-control.
    Returns:
      True  -> reached an intersection (no bump)
      False -> stopped due to bump or external stop condition
    """
    global move_forward_flag, return_home_blocked
    first_loop = False
    lock_release_time = time.ticks_ms() #flag to reset start lock time
    #outter infinite loop to keep thread check for activation
    while running:

        while move_forward_flag:
            # 1) Safety/target check
            if first_loop:
                # Initial lock to roll straight for half a second
                lock_release_time = time.ticks_add(time.ticks_ms(), cfg.START_LOCK_MS)
                first_loop = False

            # 3) During initial lock window, always drive straight
            if time.ticks_diff(time.ticks_ms(), lock_release_time) < 0:
                set_speeds(cfg.STRAIGHT_CREEP, cfg.STRAIGHT_CREEP)
                continue

            # 2) Read sensors
            readings = line_sensors.read_calibrated()

            if readings[0] >= cfg.BLACK_THRESH or readings[4] >= cfg.BLACK_THRESH:
                motors_off()
                flash_LEDS(GREEN,1)
                move_forward_flag = False
                first_loop = True
                break

            bump.read()
            if bump.left_is_pressed() or bump.right_is_pressed():
                # The hard stop is the first physical action after a bump.
                motors_off()
                move_forward_flag = False
                if returning_home:
                    return_home_blocked = True
                else:
                    stop_and_alert_target()
                break

            # 6) Normal P-control when not locked
            total = readings[0] + readings[1] + readings[2] + readings[3] + readings[4]
            if total == 0:
                set_speeds(cfg.STRAIGHT_CREEP, cfg.STRAIGHT_CREEP)
                continue
            # weights: 0, 1000, 2000, 3000, 4000
            pos = (0*readings[0] + 1000*readings[1] + 2000*readings[2] + 3000*readings[3] + 4000*readings[4]) // total
            error = pos - cfg.LINE_CENTER
            correction = int(cfg.KP * error)

            left  = _clamp(cfg.BASE_SPEED + correction, cfg.MIN_SPD, cfg.MAX_SPD)
            right = _clamp(cfg.BASE_SPEED - correction, cfg.MIN_SPD, cfg.MAX_SPD)
            set_speeds(left, right)

        # Shorter sleep to allow rapid response when move_forward_flag is set
        time.sleep_ms(20)

def calibrate():
    """Calibrate line sensors then advance to the first intersection.

    The robot spins in place while repeatedly sampling the line sensors to
    establish min/max values.  The robot should be placed one cell behind its
    intended starting position; after calibration it drives forward to the
    first intersection and updates the global ``pos`` to ``START_POS`` so the
    caller sees that intersection as the starting point of the search. The
    metric timer begins once this intersection is reached.
    """
    global pos, move_forward_flag, METRIC_START_TIME_MS

    # 1) Spin in place to expose sensors to both edges of the line.
    #    A single full rotation is enough, so spin in one direction while
    #    repeatedly sampling the sensors.  The Pololu library recommends
    #    speeds of 920/-920 with ~10 ms pauses for calibration.
    for _ in range(50):
        if not running:
            motors_off()
            return

        set_speeds(cfg.CALIBRATE_SPEED, -cfg.CALIBRATE_SPEED)
        line_sensors.calibrate()
        time.sleep_ms(5)

    motors_off()
    bump.calibrate()
    time.sleep_ms(5)


    # 2) Move forward until an intersection is detected.  After the forward
    #    move the robot is sitting on our true starting cell (defined by
    #    ``START_POS`` at the top of the file) so overwrite any temporary
    #    position with that constant and mark the cell visited.
    move_forward_flag = True
    while move_forward_flag:
        uart_service()
        time.sleep_ms(1)
    pos[0], pos[1] = START_POS
    if 0 <= pos[0] < GRID_SIZE and 0 <= pos[1] < GRID_SIZE:
        grid[idx(pos[0], pos[1])] = CELL_SEARCHED
        update_target_on_miss(idx(pos[0], pos[1]))  # start cell is a searched/target-miss cell
    update_prob_map()

    motors_off()
    gc.collect()


def at_intersection_and_white():
    """
    Detect a clue when the center line sensor reads white.
    Returns bool.
    """
    r = line_sensors.read_calibrated()      # [0]..[4], center is [2]
    if r[2] < cfg.MIDDLE_WHITE_THRESH:
        flash_LEDS(BLUE,1)
        buzz('clue')
        return True
    return False


def check_current_cell_for_clue(stage="start"):
    """Check the current cell for a clue without moving off of it. Only used on startup. at_intersection_and_white() is used during normal movement.""""""Check the current cell for a clue without moving off of it."""
    global first_clue_seen
    if not running or found_target:
        return
    if at_intersection_and_white():
        clue = (pos[0], pos[1])
        is_new = clue not in clues
        if is_new:
            clues.append(clue)
        first_clue_seen = True
        if is_new:
            update_prob_map()
            gc.collect()
        publish_clue(pos[0], pos[1])

# ===========================================================
# Heading / Turning (cardinal NSEW)
# ===========================================================
def rotate_degrees(deg):
    """
    Rotate in place by a signed multiple of 90°.
    deg ∈ {-180, -90, 0, 90, 180}
    Obeys 'running' flag and always cuts motors at the end.
    """

    if deg == 0 or not running:
        motors_off()
        return

    #inch forward to make clean turn
    set_speeds(cfg.BASE_SPEED, cfg.BASE_SPEED)
    time.sleep(.3)
    motors_off()

    if deg == 180 or deg == -180:
        buzz('turn')
        set_speeds(cfg.TURN_SPEED, -cfg.TURN_SPEED)
        if running: time.sleep(cfg.YAW_180_MS)

    elif deg == 90:
        buzz('turn')
        set_speeds(cfg.TURN_SPEED, -cfg.TURN_SPEED)
        if running: time.sleep(cfg.YAW_90_MS)

    elif deg == -90:
        buzz('turn')
        set_speeds(-cfg.TURN_SPEED, cfg.TURN_SPEED)
        if running: time.sleep(cfg.YAW_90_MS)

    motors_off()

def quarter_turns(from_dir, to_dir):
    if from_dir == to_dir:
        return 0
    if from_dir is None:
        return 1
    try:
        fi = DIRS4.index(from_dir)
        ti = DIRS4.index(to_dir)
    except ValueError:
        return 1
    delta = (ti - fi) % 4
    if delta == 2:
        return 2
    return 1

def turn_towards(cur, nxt):
    """
    Turn from current heading to face the neighbor cell `nxt`.
    - cur: (x,y) current cell
    - nxt: (x,y) next cell (must be a 4-neighbor of cur)
    Updates the global 'heading'.
    """
    global heading
    dx, dy = nxt[0] - cur[0], nxt[1] - cur[1]
    target = (dx, dy)

    i = DIRS4.index(heading)
    j = DIRS4.index(target)
    delta = (j - i) % 4

    # Map delta to minimal signed degrees
    if delta == 0:   deg = 0
    elif delta == 1: deg = 90
    elif delta == 2: deg = 180
    elif delta == 3: deg = -90

    rotate_degrees(deg)
    heading = target
# ===========================================================
# Reward Model (clues) & Pre-Clue Serpentine Bias
# ===========================================================
def update_prob_map():
    """
    Recompute target_p from all known clues, then copy it into prob_map.

    Pre-clue:
        unsearched cells are uniform, searched cells are zero.
    Post-clue:
        target_p[i] is proportional to the sum of Manhattan-distance clue
        influence terms for unsearched cells and zero for searched cells.
    """
    has_clues = len(clues) > 0

    if has_clues:
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                i = idx(x, y)
                if grid[i] == CELL_SEARCHED:
                    target_p[i] = 0.0
                    continue
                total = 0.0
                for (cx, cy) in clues:
                    d = manhattan(x, y, cx, cy)
                    total += 1.0 / ((1.0 + d) ** TARGET_DECAY_EXP)
                target_p[i] = total
        renorm(target_p)
    else:
        open_count = 0
        for i in range(GRID_SIZE * GRID_SIZE):
            if grid[i] == CELL_SEARCHED:
                target_p[i] = 0.0
            else:
                open_count += 1
        if open_count > 0:
            val = 1.0 / open_count
            for i in range(GRID_SIZE * GRID_SIZE):
                if grid[i] != CELL_SEARCHED:
                    target_p[i] = val
        else:
            renorm(target_p)

    recompute_value_map()


def update_target_on_miss(i):
    """
    We have searched cell i for the target with POD_target = 1 and did not find
    it. Set P_target(i) = 0 and renormalize target_p/prob_map.
    """
    # BeliefMap parity recomputes the complete posterior after every miss.
    # Repeated in-place renormalization is mathematically close but can drift
    # enough to change EPS-governed allocator ties.
    update_prob_map()


def i_should_yield(ix, iy):
    """Yield if a peer intends to enter or currently occupies (ix, iy)."""
    # Simple energy tracking - no function call counting needed
    for _pid, (px, py) in peer_intent.items():
        if (px, py) == (ix, iy):
            return True
    for _pid, (px, py) in peer_pos_yield.items():
        if (px, py) == (ix, iy):
            return True
    return False


def _expire_temporary_invalid_tasks():
    now = time.ticks_ms()
    for cell, expires_at in list(temporary_invalid_task_until.items()):
        if time.ticks_diff(now, expires_at) >= 0:
            temporary_invalid_task_until.pop(cell, None)


def _task_temporarily_invalid(cell):
    expires_at = temporary_invalid_task_until.get(cell)
    if expires_at is None:
        return False
    if time.ticks_diff(time.ticks_ms(), expires_at) >= 0:
        temporary_invalid_task_until.pop(cell, None)
        return False
    return True


def _register_goal_conflict(cell):
    """Return the consecutive protected-step conflict count for this goal."""
    global blocked_goal_cell, blocked_goal_conflicts
    if blocked_goal_cell == cell:
        blocked_goal_conflicts += 1
    else:
        blocked_goal_cell = cell
        blocked_goal_conflicts = 1
    return blocked_goal_conflicts


def _temporarily_invalidate_task(cell, backoff_ms, now_ms=None):
    """Keep a twice-blocked goal out of allocation through its backoff."""
    if now_ms is None:
        now_ms = time.ticks_ms()
    temporary_invalid_task_until[cell] = time.ticks_add(
        now_ms, max(int(backoff_ms), 1))
    return now_ms


def _rid_sort_key(rid):
    try:
        return (0, int(rid))
    except (TypeError, ValueError):
        return (1, str(rid))


def _same_robot_id(a, b):
    if a is None or b is None:
        return a is None and b is None
    return str(a) == str(b)


def _robot_id_less(a, b):
    return _rid_sort_key(a) < _rid_sort_key(b)


def _hipc_valid_task(cell):
    if cell is None:
        return False
    x, y = cell
    checker = globals().get("_task_temporarily_invalid")
    temporarily_invalid = (
        checker(cell)
        if checker is not None
        else cell in globals().get("temporary_invalid_task_until", {})
    )
    return (
        0 <= x < GRID_SIZE
        and 0 <= y < GRID_SIZE
        and grid[idx(x, y)] == CELL_UNSEARCHED
        and not temporarily_invalid
    )


def _hipc_probability_normalizer():
    return allocation_probability_normalizer


def _hipc_bid_from_reference(cell, reference, normalizer=None):
    if normalizer is None:
        normalizer = _hipc_probability_normalizer()
    probability = target_p[idx(cell[0], cell[1])] / normalizer
    probability = max(0.0, min(1.0, probability))
    distance = manhattan(reference[0], reference[1], cell[0], cell[1])
    return -(distance + 8.0 * (1.0 - probability))


def _hipc_candidates():
    started_us = time.ticks_us()
    expiry = globals().get("_expire_temporary_invalid_tasks")
    if expiry is not None:
        expiry()
    backups = []
    for cell in globals().get("temporary_invalid_task_until", {}):
        x, y = cell
        if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
            cell_index = idx(x, y)
            if grid[cell_index] == CELL_UNSEARCHED:
                backups.append(cell_index)
                grid[cell_index] = 255
    try:
        cells = hipc_candidate_workspace.fill(
            grid, target_p, idx, pos, CELL_UNSEARCHED,
            rank_always=True)
    finally:
        for cell_index in backups:
            grid[cell_index] = CELL_UNSEARCHED
    record_candidate_filter_time(started_us)
    return cells


def _hipc_team_agents():
    global hipc_dropped_peers
    team = {str(ROBOT_ID): (pos[0], pos[1])}
    dropped = set()
    for rid in sorted(peer_pos, key=_rid_sort_key):
        if str(rid) == str(ROBOT_ID):
            continue
        if hipc_bad_prediction_count.get(str(rid), 0) >= HIPC_BAD_PRED_LIMIT:
            dropped.add(str(rid))
            continue
        cell = peer_pos.get(rid)
        if cell is None:
            dropped.add(str(rid))
            continue
        team[str(rid)] = (int(cell[0]), int(cell[1]))
    hipc_dropped_peers = dropped
    return team


def _hipc_run_local_team_taa(team, candidates, normalizer=None):
    if normalizer is None:
        normalizer = _hipc_probability_normalizer()
    team_order = list(team)
    team_count = len(team_order)
    plan_ids = array(
        "H", [0] * max(1, team_count * HIPC_BUNDLE_SIZE))
    plan_counts = bytearray(team_count)
    endpoints = [team[rid] for rid in team_order]
    assigned = bytearray(GRID_SIZE * GRID_SIZE)
    for _ in range(max(1, len(team) * HIPC_BUNDLE_SIZE)):
        best = None
        for rid in sorted(team, key=_rid_sort_key):
            row = team_order.index(rid)
            if plan_counts[row] >= HIPC_BUNDLE_SIZE:
                continue
            for cell in candidates:
                cell_id = cell[1] * GRID_SIZE + cell[0]
                if assigned[cell_id]:
                    continue
                known_winner = hipc_winner_by_cell.get(cell)
                known_bid = hipc_winning_bid_by_cell.get(cell, HIPC_NO_BID)
                if known_winner is not None and str(known_winner) not in team:
                    continue
                score = _hipc_bid_from_reference(
                    cell, endpoints[row], normalizer)
                if (
                    known_winner is not None
                    and str(known_winner) != rid
                    and score < known_bid - HIPC_EPS_BID
                ):
                    continue
                if best is None or score > best[0] + HIPC_EPS_BID or (
                        abs(score - best[0]) <= HIPC_EPS_BID
                        and (str(rid), cell) < (str(best[1]), best[2])):
                    best = (score, rid, cell)
        if best is None:
            break
        _, rid, cell = best
        row = team_order.index(rid)
        offset = row * HIPC_BUNDLE_SIZE + plan_counts[row]
        plan_ids[offset] = cell[1] * GRID_SIZE + cell[0]
        plan_counts[row] += 1
        endpoints[row] = cell
        assigned[cell[1] * GRID_SIZE + cell[0]] = 1
    plan = {}
    for row, rid in enumerate(team_order):
        route = []
        offset = row * HIPC_BUNDLE_SIZE
        for route_index in range(plan_counts[row]):
            cell_id = plan_ids[offset + route_index]
            route.append((cell_id % GRID_SIZE, cell_id // GRID_SIZE))
        plan[rid] = route
    return plan


def _hipc_next_bid_time():
    global hipc_bid_counter
    hipc_bid_counter += 1
    return hipc_bid_counter


def _hipc_can_claim(cell, bid):
    winner = hipc_winner_by_cell.get(cell)
    known_bid = hipc_winning_bid_by_cell.get(cell, HIPC_NO_BID)
    if winner is None or _same_robot_id(winner, ROBOT_ID):
        return True
    if bid > known_bid + HIPC_EPS_BID:
        return True
    return abs(bid - known_bid) <= HIPC_EPS_BID and _robot_id_less(ROBOT_ID, winner)


def _hipc_release_local_path():
    global hipc_path, hipc_bundle, hipc_pending_snapshot
    if not hipc_path:
        return
    for cell in hipc_path:
        if _same_robot_id(hipc_winner_by_cell.get(cell), ROBOT_ID):
            hipc_winner_by_cell[cell] = None
            hipc_winning_bid_by_cell[cell] = HIPC_NO_BID
            hipc_bid_time_by_cell[cell] = HIPC_NO_TIME
    hipc_path = []
    hipc_bundle = []
    hipc_pending_snapshot = True


def _hipc_replace_own_bundle(new_path, normalizer=None):
    global hipc_path, hipc_bundle, hipc_pending_snapshot
    normalized = [cell for cell in new_path[:HIPC_BUNDLE_SIZE] if _hipc_valid_task(cell)]
    if tuple(normalized) == tuple(hipc_path):
        return
    _hipc_release_local_path()
    prefix = []
    if normalizer is None:
        normalizer = _hipc_probability_normalizer()
    for cell in normalized:
        reference = prefix[-1] if prefix else (pos[0], pos[1])
        bid = _hipc_bid_from_reference(cell, reference, normalizer)
        if not _hipc_can_claim(cell, bid):
            continue
        prefix.append(cell)
        hipc_winner_by_cell[cell] = str(ROBOT_ID)
        hipc_winning_bid_by_cell[cell] = bid
        hipc_bid_time_by_cell[cell] = _hipc_next_bid_time()
    hipc_path = list(prefix)
    hipc_bundle = list(prefix)
    hipc_pending_snapshot = True


def _hipc_bundle_signature():
    return tuple(
        (cell, hipc_winning_bid_by_cell.get(cell, HIPC_NO_BID),
         hipc_bid_time_by_cell.get(cell, HIPC_NO_TIME))
        for cell in hipc_path)


def _hipc_encode_winner(winner):
    return HIPC_NO_WINNER_CODE if winner is None else str(winner)


def _hipc_encode_signed(value, empty):
    if value == empty:
        return HIPC_EMPTY_FIELD
    # '-' terminates a UART frame, including when it appears in a scientific
    # exponent. Escape every occurrence, not only a leading sign.
    return "{:.17g}".format(float(value)).replace("-", "N")


def _hipc_decode_signed(value, empty):
    if value == HIPC_EMPTY_FIELD:
        return empty
    return float(value.replace("N", "-"))


def _hipc_bundle_fields(bundle):
    fields = []
    for i in range(HIPC_BUNDLE_SIZE):
        if i < len(bundle):
            fields.extend((str(bundle[i][0]), str(bundle[i][1])))
        else:
            fields.extend((HIPC_EMPTY_FIELD, HIPC_EMPTY_FIELD))
    return fields


def hipc_flush_messages():
    global hipc_pending_snapshot, hipc_last_sent_signature
    if (
        not first_clue_seen
        or not _trial_traffic_enabled()
        or not hipc_pending_snapshot
    ):
        return
    signature = _hipc_bundle_signature()
    if signature == hipc_last_sent_signature:
        hipc_pending_snapshot = False
        return
    bundle_fields = _hipc_bundle_fields(hipc_path)
    if not hipc_path:
        payload = ",".join(
            [HIPC_EMPTY_FIELD, HIPC_EMPTY_FIELD, HIPC_NO_WINNER_CODE,
             HIPC_EMPTY_FIELD, str(_hipc_next_bid_time())] + bundle_fields)
        publish_hipc_payload(payload)
    else:
        for cell in hipc_path:
            if not _same_robot_id(hipc_winner_by_cell.get(cell), ROBOT_ID):
                continue
            payload = ",".join(
                [str(cell[0]), str(cell[1]), str(ROBOT_ID),
                 _hipc_encode_signed(hipc_winning_bid_by_cell[cell], HIPC_NO_BID),
                 str(hipc_bid_time_by_cell[cell])] + bundle_fields)
            publish_hipc_payload(payload)
    hipc_last_sent_signature = signature
    hipc_pending_snapshot = False


def _hipc_parse_bundle(fields):
    bundle = []
    for i in range(HIPC_BUNDLE_SIZE):
        x, y = fields[i * 2], fields[i * 2 + 1]
        if x == HIPC_EMPTY_FIELD and y == HIPC_EMPTY_FIELD:
            continue
        try:
            cell = (int(x), int(y))
        except ValueError:
            return None
        if not (0 <= cell[0] < GRID_SIZE and 0 <= cell[1] < GRID_SIZE):
            return None
        bundle.append(cell)
    return bundle


def _hipc_clear_sender_claims_not_in_bundle(sender, bundle):
    allowed = set(bundle)
    for cell, winner in hipc_winner_by_cell.items():
        if _same_robot_id(winner, sender) and cell not in allowed:
            hipc_winner_by_cell[cell] = None
            hipc_winning_bid_by_cell[cell] = HIPC_NO_BID
            hipc_bid_time_by_cell[cell] = HIPC_NO_TIME


def _hipc_update_prediction(sender, bundle):
    if _same_robot_id(sender, ROBOT_ID):
        return
    # A clear snapshot removes stale claims, but it is not a prediction
    # observation. Preserve the last nonempty signature so A -> clear -> A is
    # assessed only once, matching the simulator.
    if not bundle:
        return
    signature = tuple(bundle)
    sender = str(sender)
    if hipc_seen_peer_bundle_signature.get(sender) == signature:
        return
    hipc_seen_peer_bundle_signature[sender] = signature
    predicted = hipc_last_predicted_peer_first_task.get(sender)
    if predicted is None:
        return
    actual = bundle[0]
    error = manhattan(predicted[0], predicted[1], actual[0], actual[1])
    count = hipc_bad_prediction_count.get(sender, 0)
    if error <= HIPC_PREDICTION_TOLERANCE:
        hipc_bad_prediction_count[sender] = max(0, count - 1)
    else:
        hipc_bad_prediction_count[sender] = count + 1


def _hipc_repair_after_consensus():
    global hipc_path, hipc_bundle, current_task_cell, hipc_pending_snapshot
    first_bad = None
    for i, cell in enumerate(hipc_path):
        if not _hipc_valid_task(cell) or not _same_robot_id(hipc_winner_by_cell.get(cell), ROBOT_ID):
            first_bad = i
            break
    if first_bad is not None:
        for cell in hipc_path[first_bad:]:
            if _same_robot_id(hipc_winner_by_cell.get(cell), ROBOT_ID):
                hipc_winner_by_cell[cell] = None
                hipc_winning_bid_by_cell[cell] = HIPC_NO_BID
                hipc_bid_time_by_cell[cell] = HIPC_NO_TIME
        hipc_path = hipc_path[:first_bad]
        hipc_bundle = hipc_bundle[:first_bad]
        current_task_cell = hipc_path[0] if hipc_path else None
        hipc_pending_snapshot = True


def _hipc_receive_payload(sender, payload):
    try:
        fields = payload.split(",")
        if len(fields) != 5 + HIPC_BUNDLE_SIZE * 2:
            return
        bundle = _hipc_parse_bundle(fields[5:])
        if bundle is None:
            return
        timestamp = float(fields[4])
    except (TypeError, ValueError):
        return
    _hipc_update_prediction(sender, bundle)
    _hipc_clear_sender_claims_not_in_bundle(sender, bundle)
    if fields[0] == HIPC_EMPTY_FIELD and fields[1] == HIPC_EMPTY_FIELD:
        _hipc_repair_after_consensus()
        return
    try:
        cell = (int(fields[0]), int(fields[1]))
        winner = None if fields[2] == HIPC_NO_WINNER_CODE else fields[2]
        bid = _hipc_decode_signed(fields[3], HIPC_NO_BID)
    except (TypeError, ValueError):
        return
    if not _hipc_valid_task(cell):
        hipc_winner_by_cell[cell] = None
        hipc_winning_bid_by_cell[cell] = HIPC_NO_BID
        hipc_bid_time_by_cell[cell] = HIPC_NO_TIME
        _hipc_repair_after_consensus()
        return

    local_winner = hipc_winner_by_cell.get(cell)
    local_bid = hipc_winning_bid_by_cell.get(cell, HIPC_NO_BID)
    local_time = hipc_bid_time_by_cell.get(cell, HIPC_NO_TIME)
    update = False
    if _same_robot_id(winner, sender) and _same_robot_id(local_winner, sender):
        update = timestamp >= local_time - HIPC_EPS_BID
    elif bid > local_bid + HIPC_EPS_BID:
        update = True
    elif abs(bid - local_bid) <= HIPC_EPS_BID:
        update = local_winner is None or _robot_id_less(winner, local_winner)
    if update:
        hipc_winner_by_cell[cell] = winner
        hipc_winning_bid_by_cell[cell] = bid
        hipc_bid_time_by_cell[cell] = timestamp
    _hipc_repair_after_consensus()


def _hipc_clear_invalid_or_completed_cells():
    for cell in hipc_winner_by_cell:
        if not _hipc_valid_task(cell):
            hipc_winner_by_cell[cell] = None
            hipc_winning_bid_by_cell[cell] = HIPC_NO_BID
            hipc_bid_time_by_cell[cell] = HIPC_NO_TIME
    _hipc_repair_after_consensus()


def _hipc_handle_allocator_goal_arrival(arrived_cell):
    """Clear only the wrapper goal; allocator repair runs on the next choose."""
    global current_task_cell
    if current_task_cell is None:
        return False
    if (arrived_cell[0] != current_task_cell[0]
            or arrived_cell[1] != current_task_cell[1]):
        return False
    current_task_cell = None
    return True


def _hipc_defer_collision_reallocation():
    """Leave allocator state intact until the next canonical choose boundary."""
    global current_task_cell, pending_collision_reallocation
    current_task_cell = None
    pending_collision_reallocation = True


def _retry_original_goal_after_failed_alternate(blocked_retry_cells):
    """Retry the protected route once when blocking its first step has no path."""
    if not first_clue_seen or not blocked_retry_cells:
        return False
    publish_intent()
    blocked_retry_cells.clear()
    return True


def _hipc_complete_cell_arrival(cell_i):
    """Publish/observe an arrival before emitting allocator completion state."""
    global first_clue_seen

    publish_position()
    grid[cell_i] = CELL_SEARCHED
    update_target_on_miss(cell_i)

    reached_allocator_goal = False
    if first_clue_seen:
        reached_allocator_goal = _hipc_handle_allocator_goal_arrival(pos)

    if not found_target:
        potential_clue = (pos[0], pos[1])
        if potential_clue not in clues and at_intersection_and_white():
            clues.append(potential_clue)
            first_clue_seen = True
            update_prob_map()
            publish_clue(pos[0], pos[1])
            update_mem_headroom()
            gc.collect()

    if not found_target:
        hipc_flush_messages()
    return reached_allocator_goal


def _hipc_reset_if_new_clue_information():
    global hipc_clue_signature, hipc_path, hipc_bundle, hipc_pending_snapshot
    global hipc_last_sent_signature, hipc_last_predicted_peer_first_task
    signature = tuple(sorted(set(clues)))
    if hipc_clue_signature is None:
        hipc_winner_by_cell.clear()
        hipc_winning_bid_by_cell.clear()
        hipc_bid_time_by_cell.clear()
        hipc_path = []
        hipc_bundle = []
        hipc_pending_snapshot = False
        hipc_last_sent_signature = None
        hipc_last_predicted_peer_first_task = {}
    hipc_clue_signature = signature


def _hipc_build_bundle_impl():
    global hipc_last_predicted_peer_first_task
    candidates = _hipc_candidates()
    team = _hipc_team_agents()
    normalizer = _hipc_probability_normalizer()
    plan = _hipc_run_local_team_taa(
        team, candidates, normalizer)
    hipc_last_predicted_peer_first_task = {
        rid: route[0] for rid, route in plan.items()
        if rid != str(ROBOT_ID) and route}
    _hipc_replace_own_bundle(
        plan.get(str(ROBOT_ID), [])[:HIPC_BUNDLE_SIZE],
        normalizer)


def _hipc_build_bundle():
    started_us = time.ticks_us()
    filter_time_before_us = candidate_filter_time_us_total
    try:
        _hipc_build_bundle_impl()
    finally:
        record_allocator_solve_time(started_us, filter_time_before_us)


def _hipc_release_own_bundle_for_replan():
    _hipc_release_local_path()


def _pick_task_cell_impl():
    global pending_collision_reallocation
    _hipc_reset_if_new_clue_information()
    _hipc_clear_invalid_or_completed_cells()
    if pending_collision_reallocation:
        pending_collision_reallocation = False
        _hipc_release_own_bundle_for_replan()
    _hipc_repair_after_consensus()
    _hipc_build_bundle()
    return hipc_path[0] if hipc_path else None


def pick_task_cell():
    started_us = time.ticks_us()
    try:
        return _pick_task_cell_impl()
    finally:
        record_allocator_time(started_us)


def next_serpentine_task_cell_in_band():
    """
    Pre-clue: return the next unsearched cell in this robot's row band
    following a serpentine (boustrophedon) pattern.
    """
    cur_x, cur_y = pos[0], pos[1]
    if cur_y < BAND_Y_MIN:
        cur_y = BAND_Y_MIN
    elif cur_y > BAND_Y_MAX:
        cur_y = BAND_Y_MAX

    passed_current = False

    for y in range(BAND_Y_MIN, BAND_Y_MAX + 1):
        row_offset = y - BAND_Y_MIN
        if row_offset % 2 == 0:
            x_iter = range(0, GRID_SIZE)
        else:
            x_iter = range(GRID_SIZE - 1, -1, -1)

        for x in x_iter:
            if not passed_current:
                if x == cur_x and y == cur_y:
                    passed_current = True
                continue
            if grid[idx(x, y)] == CELL_UNSEARCHED:
                return (x, y)

    for y in range(BAND_Y_MIN, BAND_Y_MAX + 1):
        row_offset = y - BAND_Y_MIN
        if row_offset % 2 == 0:
            x_iter = range(0, GRID_SIZE)
        else:
            x_iter = range(GRID_SIZE - 1, -1, -1)

        for x in x_iter:
            if x == cur_x and y == cur_y:
                return None
            if grid[idx(x, y)] == CELL_UNSEARCHED:
                return (x, y)

    return None

# ===========================================================
# A* Planner (4-neighbor grid, cardinal)
# ===========================================================
def a_star(start, task_cell):
    """
    A* over the 4-neighbor grid, with costs:
      +1 per step
      + TURN_COST per 90-degree heading change
      + cfg.VISITED_STEP_PENALTY if stepping onto a visited cell (grid==2)
      (all protected peer positions are route-blocked)
    The target_p/prob_map reward is applied as a bonus in the node priority.
    Returns a path as a list: [start, ..., task_cell], or [] if failure.
    """
    # Simple energy tracking - no function call counting needed
    frontier.clear()
    if start == task_cell:
        return [start]
    # Whole-route planning uses accepted (droppable) shared state. Protected
    # position/intent observations are reserved for the final one-step check.
    blocked_peers = set(peer_pos.values())
    blocked_peers.discard(start)
    if task_cell in blocked_peers:
        return []
    for i in range(GRID_SIZE * GRID_SIZE):
        came_from[i] = -1
        cost_so_far[i] = 1e30

    start_idx = idx(start[0], start[1])
    task_cell_idx = idx(task_cell[0], task_cell[1])
    tie = 0
    heapq.heappush(frontier, (0.0, tie, start_idx, heading))
    came_from[start_idx] = start_idx
    cost_so_far[start_idx] = 0.0
    while frontier and running and not found_target:
        _, _, current_idx, cur_dir = heapq.heappop(frontier)
        if current_idx == task_cell_idx:
            break

        cx = current_idx % GRID_SIZE
        cy = GRID_SIZE - 1 - (current_idx // GRID_SIZE)
        for dx, dy in DIRS4:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                continue
            i = idx(nx, ny)
            if grid[i] == CELL_OBSTACLE:  # obstacle/reserved
                continue
            if (nx, ny) in blocked_peers:
                continue

            move_cost = 1.0
            turns = quarter_turns(cur_dir, (dx, dy))
            turn_cost = TURN_COST * turns
            visited_pen = cfg.VISITED_STEP_PENALTY if grid[i] == CELL_SEARCHED else 0.0
            base_cost = move_cost + turn_cost + visited_pen

            reward_bonus = target_p[i] * REWARD_FACTOR
            max_bonus = base_cost - 0.01
            if max_bonus < 0.0:
                max_bonus = 0.0
            if reward_bonus > max_bonus:
                reward_bonus = max_bonus

            step_cost = base_cost - reward_bonus
            if step_cost < 0.01:
                step_cost = 0.01

            new_cost = cost_so_far[current_idx] + step_cost

            if new_cost < cost_so_far[i]:
                cost_so_far[i] = new_cost
                priority = (
                    new_cost
                    + abs(task_cell[0] - nx)
                    + abs(task_cell[1] - ny)
                )
                tie += 1
                heapq.heappush(frontier, (priority, tie, i, (dx, dy)))
                came_from[i] = current_idx

    if came_from[task_cell_idx] == -1:
        return []

    # Reconstruct path
    path = []
    cur_idx = task_cell_idx
    while cur_idx != start_idx:
        x = cur_idx % GRID_SIZE
        y = GRID_SIZE - 1 - (cur_idx // GRID_SIZE)
        path.append((x, y))
        cur_idx = came_from[cur_idx]
    path.reverse()
    return [start] + path


def _reset_allocator_for_next_trial():
    global hipc_path, hipc_bundle, hipc_bid_counter, hipc_clue_signature
    global hipc_pending_snapshot, hipc_last_sent_signature
    global hipc_bad_prediction_count, hipc_last_predicted_peer_first_task
    global hipc_seen_peer_bundle_signature, hipc_dropped_peers
    hipc_winner_by_cell.clear()
    hipc_winning_bid_by_cell.clear()
    hipc_bid_time_by_cell.clear()
    hipc_path = []
    hipc_bundle = []
    hipc_bid_counter = 0
    hipc_clue_signature = None
    hipc_pending_snapshot = False
    hipc_last_sent_signature = None
    hipc_bad_prediction_count = {}
    hipc_last_predicted_peer_first_task = {}
    hipc_seen_peer_bundle_signature = {}
    hipc_dropped_peers = set()

def reset_search_state_for_next_trial():
    """Clear trial/world knowledge before a manually repositioned trial."""
    global first_clue_seen, found_target, target_location, current_task_cell
    global target_bump_stop, abort_signal
    global last_task_cell, collision_event_counted_since_move, METRIC_START_TIME_MS
    global peer_intent, peer_pos, peer_pos_yield, heading, pos
    global communicated_intent, blocked_goal_cell, blocked_goal_conflicts
    global allocation_probability_normalizer, pending_collision_reallocation

    for i in range(GRID_SIZE * GRID_SIZE):
        grid[i] = CELL_UNSEARCHED
        target_p[i] = 1.0 / (GRID_SIZE * GRID_SIZE)
        prob_map[i] = target_p[i]
    clues[:] = []
    published_clues.clear()
    peer_intent.clear()
    peer_pos.clear()
    peer_pos_yield.clear()
    communicated_intent = None
    allocation_probability_normalizer = 1.0 / (GRID_SIZE * GRID_SIZE)
    first_clue_seen = False
    found_target = False
    abort_signal = False
    target_bump_stop = False
    target_location = None
    current_task_cell = None
    last_task_cell = None
    collision_event_counted_since_move = False
    blocked_goal_cell = None
    blocked_goal_conflicts = 0
    pending_collision_reallocation = False
    temporary_invalid_task_until.clear()
    METRIC_START_TIME_MS = None
    # The operator physically returns the robot to this east-facing pose.
    pos[0], pos[1] = START_POS
    heading = (START_HEADING[0], START_HEADING[1])
    _reset_allocator_for_next_trial()
    gc.collect()


def recover_target_finder_to_last_intersection():
    """Turn away from the target and regain the last confirmed grid cell."""
    global returning_home, return_home_blocked, move_forward_flag, heading
    if not target_bump_stop:
        return True
    returning_home = True
    return_home_blocked = False
    motors_off()
    try:
        away_heading = (
            pos[0] - target_location[0],
            pos[1] - target_location[1],
        )
        if heading != away_heading:
            buzz('turn')
            set_speeds(cfg.TURN_SPEED, -cfg.TURN_SPEED)
            if running:
                time.sleep(cfg.YAW_180_MS)
            motors_off()
            heading = away_heading
        return_home_blocked = False
        move_forward_flag = True
        while move_forward_flag and running:
            uart_service()
            time.sleep_ms(1)
        if return_home_blocked or not running:
            return False
        publish_position()
        publish_intent()
        return True
    finally:
        returning_home = False
        return_home_blocked = False
        motors_off()


def return_home():
    """Navigate to START_POS outside the completed trial's metric window."""
    global returning_home, return_home_blocked, move_forward_flag, heading
    returning_home = True
    return_home_blocked = False
    blocked_cells = set()
    motors_off()
    try:
        while running and (pos[0], pos[1]) != START_POS:
            temporary_blocks = set(blocked_cells)
            if target_location is not None:
                tx, ty = target_location
                if 0 <= tx < GRID_SIZE and 0 <= ty < GRID_SIZE:
                    temporary_blocks.add((tx, ty))
            backups = []
            for bx, by in temporary_blocks:
                cell_index = idx(bx, by)
                backups.append((cell_index, grid[cell_index]))
                grid[cell_index] = CELL_OBSTACLE
            try:
                path = a_star((pos[0], pos[1]), START_POS)
            finally:
                for cell_index, previous_state in backups:
                    grid[cell_index] = previous_state
            if len(path) < 2:
                log_error("return-home path unavailable")
                return False
            nxt = path[1]
            publish_intent(nxt[0], nxt[1])
            for _ in range(5):
                uart_service()
                time.sleep_ms(10)
            if i_should_yield(nxt[0], nxt[1]):
                blocked_cells.add(nxt)
                time.sleep_ms(300)
                continue
            turn_towards((pos[0], pos[1]), nxt)
            return_home_blocked = False
            move_forward_flag = True
            while move_forward_flag and running:
                uart_service()
                time.sleep_ms(1)
            if return_home_blocked:
                blocked_cells.add(nxt)
                continue
            pos[0], pos[1] = nxt
            blocked_cells.clear()
            publish_position()
            publish_intent()

        if running:
            desired_neighbor = (
                pos[0] + START_HEADING[0],
                pos[1] + START_HEADING[1],
            )
            turn_towards((pos[0], pos[1]), desired_neighbor)
            heading = (START_HEADING[0], START_HEADING[1])
            motors_off()
            # Back off the home intersection by about one inch after facing east.
            set_speeds(-cfg.BASE_SPEED, -cfg.BASE_SPEED)
            time.sleep(.3)
            motors_off()
            publish_position()
            publish_intent()
            return True
        return False
    finally:
        returning_home = False
        return_home_blocked = False
        motors_off()


def wait_for_trial_start():
    """Remain responsive at home until RUN releases this armed trial."""
    last_pose_publish = time.ticks_ms()
    while running and not start_signal and not abort_signal:
        uart_service()
        now = time.ticks_ms()
        if time.ticks_diff(now, last_pose_publish) >= 500 and not pre_start_signal:
            publish_position()
            last_pose_publish = now
        time.sleep_ms(10)
    if abort_signal:
        return False
    return running and start_signal



# ===========================================================
# Main Search Loop
# ===========================================================
def run_active_trial():
    """Run one metered search trial; return when its target is reported."""
    global first_clue_seen, move_forward_flag, pos, target_bump_stop
    global task_cell_replan_count, path_replan_count, collision_prevention_count
    global current_task_cell, last_task_cell, collision_event_counted_since_move
    global blocked_goal_cell, blocked_goal_conflicts
    global pending_collision_reallocation
    global busy_ms, mem_free_min
    try:
        while running and not found_target:
            busy_timer_reset()
            gc.collect()
            update_mem_headroom()

            blocked_retry_cells = set()
            try:
                prev_task_cell = current_task_cell
                previous_task_completed = (
                    prev_task_cell is not None
                    and grid[idx(prev_task_cell[0], prev_task_cell[1])]
                    == CELL_SEARCHED
                )
                previous_task_invalidated = (
                    (prev_task_cell is not None and not previous_task_completed)
                    or (
                        prev_task_cell is None
                        and last_task_cell is not None
                        and grid[idx(last_task_cell[0], last_task_cell[1])]
                        != CELL_SEARCHED
                    )
                )
                if (
                    current_task_cell is not None
                    and grid[idx(
                        current_task_cell[0], current_task_cell[1]
                    )] == CELL_UNSEARCHED
                    and not _task_temporarily_invalid(current_task_cell)
                ):
                    # Match RobotShell: retain a valid goal and do not invoke
                    # the allocator once per traversed route cell.
                    task_cell = current_task_cell
                else:
                    current_task_cell = None
                    if not first_clue_seen:
                        task_cell = next_serpentine_task_cell_in_band()
                    else:
                        task_cell = pick_task_cell()
                        hipc_flush_messages()

                if task_cell is None:
                    current_task_cell = None
                    publish_intent()
                    busy_timer_pause()
                    for _ in range(10):
                        uart_service()
                        time.sleep_ms(20)
                    busy_timer_resume()
                    continue

                if task_cell != prev_task_cell:
                    if task_cell is not None and task_cell != last_task_cell:
                        if (
                            previous_task_invalidated
                            and first_clue_seen
                            and not metrics_frozen
                        ):
                            task_cell_replan_count += 1
                        last_task_cell = task_cell
                    # Core task cells/current tasks are internal only. HIPC table
                    # entries are sent separately as topic-3 deltas.
                    current_task_cell = task_cell

                path = []
                while True:
                    _block_backup = []
                    for bx, by in blocked_retry_cells:
                        ci = idx(bx, by)
                        _block_backup.append((ci, grid[ci]))
                        grid[ci] = CELL_OBSTACLE
                    try:
                        path = a_star(tuple(pos), task_cell)
                    finally:
                        for ci, prev_state in _block_backup:
                            grid[ci] = prev_state

                    update_mem_headroom()
                    gc.collect()
                    if len(path) < 2:
                        if first_clue_seen and not metrics_frozen:
                            path_replan_count += 1
                        if _retry_original_goal_after_failed_alternate(
                            blocked_retry_cells
                        ):
                            continue
                        break

                    nxt = path[1]
                    publish_intent(nxt[0], nxt[1])
                    busy_timer_pause()
                    turn_towards(tuple(pos), nxt)
                    for _ in range(10):
                        uart_service()
                        time.sleep_ms(10)
                    if not running or found_target:
                        break
                    collision_blocked = i_should_yield(nxt[0], nxt[1])
                    busy_timer_resume()

                    if collision_blocked:
                        if first_clue_seen:
                            _register_goal_conflict(task_cell)
                            if not metrics_frozen:
                                path_replan_count += 1
                                if not collision_event_counted_since_move:
                                    collision_prevention_count += 1
                                    collision_event_counted_since_move = True
                        blocked_retry_cells.add(nxt)
                        if first_clue_seen and blocked_goal_conflicts >= 2:
                            publish_intent()
                            _hipc_defer_collision_reallocation()
                            backoff_ms = int(random.random() * 5000.0)
                            now_ms = _temporarily_invalidate_task(
                                task_cell, backoff_ms)
                            blocked_goal_cell = None
                            blocked_goal_conflicts = 0
                            busy_timer_pause()
                            deadline = time.ticks_add(
                                now_ms, backoff_ms)
                            while (
                                running and not found_target
                                and time.ticks_diff(
                                    deadline, time.ticks_ms()) > 0
                            ):
                                uart_service()
                                time.sleep_ms(10)
                            busy_timer_resume()
                            path = []
                            break
                        continue
                    break

                if not running or found_target:
                    break
                if len(path) < 2:
                    current_task_cell = None
                    publish_intent()
                    busy_timer_pause()
                    for _ in range(10):
                        uart_service()
                        time.sleep_ms(20)
                    busy_timer_resume()
                    continue

                busy_timer_pause()
                move_forward_flag = True
                while move_forward_flag:
                    uart_service()
                    time.sleep_ms(1)
                busy_timer_resume()

                # A target alert may stop this move before the intersection.
                # Keep pos at the last physically confirmed grid cell.
                if found_target and target_bump_stop:
                    break

                # Arrived + update state & publish
                pos[0], pos[1] = nxt[0], nxt[1]
                collision_event_counted_since_move = False
                blocked_goal_cell = None
                blocked_goal_conflicts = 0
                if not metrics_frozen:
                    record_intersection(pos[0], pos[1])
                cell_i = idx(pos[0], pos[1])
                _hipc_complete_cell_arrival(cell_i)

                if found_target:
                    break
            finally:
                if not metrics_frozen:
                    busy_ms += busy_timer_value_ms()
                update_mem_headroom()
    finally:
        motors_off()


def search_loop():
    """Calibrate once, then run repeated search/log/manual-reset trials."""
    global start_signal, pre_start_signal, trial_active, found_target
    try:
        calibrate()
        while running:
            reset_search_state_for_next_trial()
            if not wait_for_trial_start():
                if not running:
                    break
                continue
            pre_start_signal = False

            grid[idx(pos[0], pos[1])] = CELL_SEARCHED
            update_target_on_miss(idx(pos[0], pos[1]))
            update_prob_map()
            check_current_cell_for_clue("start_signal")

            run_active_trial()
            motors_off()
            if not running:
                break

            trial_active = False
            start_signal = False
            metrics_log()
            flash_LEDS(GREEN, 2)

            # Automatic post-trial movement is temporarily disabled. The robot
            # stays at its final pose while the operator returns it to START_POS
            # facing east before the next RUN command.
            # if target_bump_stop:
            #     while running and not recover_target_finder_to_last_intersection():
            #         log_error("target retreat failed; retrying")
            #         uart_service()
            #         time.sleep_ms(250)
            if not running:
                break

            # found_target = False
            # if not return_home():
            #     log_error("return-home failed; waiting at current position")
            #     while running and (pos[0], pos[1]) != START_POS:
            #         uart_service()
            #         time.sleep_ms(250)
            #         if return_home():
            #             break
            if running:
                flash_LEDS(BLUE, 2)
    finally:
        trial_active = False
        motors_off()

# Entry Point
# ===========================================================

flash_trial_count()
flash_LEDS(RED,1)
# Start the single UART RX thread (clean exit when 'running' goes False)
_thread.start_new_thread(move_forward_one_cell, ())

# Kick off the mission
try:
    search_loop()
finally:
    # Ensure absolutely everything is stopped
    running = False
    time.sleep_ms(200)
    flash_LEDS(RED,5)
    time.sleep_ms(200)  # give RX thread time to fall out cleanly
