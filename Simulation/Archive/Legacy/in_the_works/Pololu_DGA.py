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
# Pololu 3pi+ 2040 OLED — DGA Coordinated Search (UART → ESP32 → MQTT)
# ===========================================================
# Runs on the Pololu 3pi+ 2040 OLED using MicroPython.
# Communication uses simple text frames over UART; an attached ESP32 relays
# those frames to MQTT topics.
#
# Behavior overview:
#   * Before any clue is found the robot sweeps its row band in a fixed
#     lawn-mower/serpentine pattern.
#   * After a clue appears, DGA evolves complete local team plans using
#     elitism, tournament selection, crossover, mutation, and repair.
#   * The robot commits to the first three cells of its route in the best plan.
#   * Topic 3 carries independently droppable owner-path cells from new best
#     plans; it never carries populations or dense maps.
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
ALGORITHM_NAME = "DGA"
GRID_SIZE = 19
LOGIC_REVISION = "dcta_parity_v1"
TRIAL_MODE = "clue_search"
COMMITMENT_HORIZON = 3
# Fraction of total grid cells retained by the post-clue candidate prefilter.
TOP_K_PERCENT = 1.0
if not (0.0 < TOP_K_PERCENT <= 1.0):
    raise ValueError("TOP_K_PERCENT must be greater than 0 and at most 1")
TOP_K_MAX_CELLS = max(1, int(GRID_SIZE * GRID_SIZE * TOP_K_PERCENT + 0.5))

DEBUG_LOG_FILE = "debug-log.txt"

METRICS_LOG_FILE = "metrics-log-DGA.txt"
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
    """Freeze the metered trial at the first target alert."""
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
        "trial_mode": TRIAL_MODE,
        "commitment_horizon": COMMITMENT_HORIZON,
        "logic_revision": LOGIC_REVISION,
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
def _float_array(values):
    """Require binary64 storage; unsupported ports fail loudly at startup."""
    return array("d", values)


prob_map = _float_array(
    [1.0 / (GRID_SIZE * GRID_SIZE)] * (GRID_SIZE * GRID_SIZE))
REWARD_FACTOR = 5
clues = []                            # list of (x, y) clue cells
forwarded_clues = set()               # each clue is forwarded at most once

# --- Target belief map ---
# P_target[i]: belief target is at cell i. There is no separate clue-value map.
target_p = _float_array(
    [1.0 / (GRID_SIZE * GRID_SIZE)] * (GRID_SIZE * GRID_SIZE))
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
cost_so_far = _float_array([0.0] * (GRID_SIZE * GRID_SIZE))
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
    n = GRID_SIZE * GRID_SIZE
    for i in range(n):
        prob_map[i] = target_p[i]


def refresh_probability_normalizer():
    """Refresh M=max(target_p) over the complete belief map."""
    global allocation_probability_normalizer
    maximum = 0.0
    for probability in target_p:
        value = float(probability)
        if (
            value == value
            and value != float("inf")
            and value != -float("inf")
            and value > maximum
        ):
            maximum = value
    allocation_probability_normalizer = (
        maximum
        if (
            maximum == maximum
            and maximum != float("inf")
            and maximum != -float("inf")
            and maximum > 0.0
        )
        else 1.0
    )
    return allocation_probability_normalizer


def normalized_target_probability(cell):
    normalizer = allocation_probability_normalizer
    if (
        normalizer != normalizer
        or normalizer == float("inf")
        or normalizer == -float("inf")
        or normalizer <= 0.0
    ):
        normalizer = refresh_probability_normalizer()
    probability = float(target_p[idx(cell[0], cell[1])]) / normalizer
    if (
        probability != probability
        or probability == float("inf")
        or probability == -float("inf")
        or probability < 0.0
    ):
        return 0.0
    if probability > 1.0:
        return 1.0
    return probability


def mark_cell_searched_miss(x, y):
    """Apply a delivered/local target miss and rebuild the exact posterior."""
    if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
        return False
    cell_index = idx(x, y)
    if grid[cell_index] == CELL_SEARCHED:
        return False
    grid[cell_index] = CELL_SEARCHED
    update_prob_map()
    return True


def add_clue_if_new(x, y):
    """Add one clue observation, mark its cell searched, and rebuild belief."""
    clue = (x, y)
    if clue in clues:
        return False
    clues.append(clue)
    grid[idx(x, y)] = CELL_SEARCHED
    update_prob_map()
    return True


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
current_task_cell = None   # local internal task cell only; never published as a reservation
last_task_cell = None
collision_event_counted_since_move = False
published_intent = None
blocked_goal_failures = {}
temporary_invalid_task_until = {}
pending_collision_reallocation = False

# -----------------------------
# DGA allocator state
# -----------------------------
# Match the simulator DGA search parameters and operators. The shared Top-K
# prefilter remains the hardware working-set control for RP2040 deployments.
DGA_POPULATION_SIZE = 30
DGA_ITERATIONS_PER_TRIGGER = 25
DGA_COMMITMENT_HORIZON = COMMITMENT_HORIZON
DGA_MIN_SUM_TIE_WEIGHT = 0.05
DGA_CROSSOVER_RATE = 0.7
DGA_MUTATION_RATE = 0.3
DGA_ELITE_COUNT = 2
DGA_FITNESS_EPS = 1.0e-9
DGA_EMPTY_CELL = "X"
DGA_BAD_PRED_LIMIT = 3
DGA_PREDICTION_TOLERANCE_CELLS = 0
DGA_INF = float("inf")


def _dga_seed_for_robot():
    try:
        return 1009 + int(ROBOT_ID)
    except (TypeError, ValueError):
        return 1009 + sum(ord(ch) for ch in str(ROBOT_ID))


class _DgaCpythonRandom:
    """Compact CPython ``random.Random`` subset backed by MT19937.

    MicroPython does not consistently expose ``random.Random``.  DGA needs a
    private stream with CPython-identical seeding and draws, so the small
    subset used by the simulator operators lives here instead of falling back
    to the module-level packet-loss/backoff generator.
    """

    _N = 624
    _M = 397
    _MASK32 = 0xFFFFFFFF
    _UPPER_MASK = 0x80000000
    _LOWER_MASK = 0x7FFFFFFF

    def __init__(self, seed):
        # ``I`` is the compact unsigned-32 storage used by MT19937.  Let an
        # unsupported array representation or allocation fail loudly.
        self._state = array("I", [0] * self._N)
        self._index = self._N
        self.seed(seed)

    def _init_genrand(self, seed):
        state = self._state
        state[0] = int(seed) & self._MASK32
        for i in range(1, self._N):
            previous = int(state[i - 1])
            state[i] = (
                1812433253 * (previous ^ (previous >> 30)) + i
            ) & self._MASK32
        self._index = self._N

    def seed(self, seed):
        # CPython treats integer seeds by magnitude and feeds little-endian
        # 32-bit words through init_by_array.
        value = abs(int(seed))
        key = []
        while value:
            key.append(value & self._MASK32)
            value >>= 32
        if not key:
            key.append(0)

        self._init_genrand(19650218)
        state = self._state
        i = 1
        j = 0
        count = self._N if self._N > len(key) else len(key)
        for _ in range(count):
            previous = int(state[i - 1])
            state[i] = (
                (
                    int(state[i])
                    ^ ((previous ^ (previous >> 30)) * 1664525)
                )
                + int(key[j])
                + j
            ) & self._MASK32
            i += 1
            j += 1
            if i >= self._N:
                state[0] = state[self._N - 1]
                i = 1
            if j >= len(key):
                j = 0

        for _ in range(self._N - 1):
            previous = int(state[i - 1])
            state[i] = (
                (
                    int(state[i])
                    ^ ((previous ^ (previous >> 30)) * 1566083941)
                )
                - i
            ) & self._MASK32
            i += 1
            if i >= self._N:
                state[0] = state[self._N - 1]
                i = 1
        state[0] = self._UPPER_MASK
        self._index = self._N

    def _uint32(self):
        if self._index >= self._N:
            state = self._state
            for i in range(self._N):
                value = (
                    (int(state[i]) & self._UPPER_MASK)
                    | (int(state[(i + 1) % self._N]) & self._LOWER_MASK)
                )
                twisted = int(state[(i + self._M) % self._N]) ^ (value >> 1)
                if value & 1:
                    twisted ^= 0x9908B0DF
                state[i] = twisted & self._MASK32
            self._index = 0

        value = int(self._state[self._index])
        self._index += 1
        value ^= value >> 11
        value ^= (value << 7) & 0x9D2C5680
        value ^= (value << 15) & 0xEFC60000
        value ^= value >> 18
        return value & self._MASK32

    def random(self):
        first = self._uint32() >> 5
        second = self._uint32() >> 6
        return (
            first * 67108864.0 + second
        ) * (1.0 / 9007199254740992.0)

    def getrandbits(self, bits):
        bits = int(bits)
        if bits <= 0:
            if bits == 0:
                return 0
            raise ValueError("number of bits must be non-negative")
        if bits <= 32:
            return self._uint32() >> (32 - bits)
        value = 0
        shift = 0
        while bits > 0:
            take = min(32, bits)
            word = self._uint32()
            if take < 32:
                word >>= 32 - take
            value |= word << shift
            shift += take
            bits -= take
        return value

    def _randbelow(self, upper):
        upper = int(upper)
        if upper <= 0:
            raise ValueError("upper bound must be positive")
        bits = 0
        probe = upper
        while probe:
            bits += 1
            probe >>= 1
        value = self.getrandbits(bits)
        while value >= upper:
            value = self.getrandbits(bits)
        return value

    def randrange(self, start, stop=None):
        if stop is None:
            stop = start
            start = 0
        width = int(stop) - int(start)
        if width <= 0:
            raise ValueError("empty range for randrange")
        return int(start) + self._randbelow(width)

    def choice(self, sequence):
        if not sequence:
            raise IndexError("cannot choose from an empty sequence")
        return sequence[self._randbelow(len(sequence))]

    def shuffle(self, sequence):
        for i in range(len(sequence) - 1, 0, -1):
            j = self._randbelow(i + 1)
            sequence[i], sequence[j] = sequence[j], sequence[i]

    def sample(self, population, count):
        population_size = len(population)
        count = int(count)
        if count < 0 or count > population_size:
            raise ValueError("sample larger than population")
        result = [None] * count
        set_size = 21
        if count > 5:
            power = 1
            target = count * 3
            while power < target:
                power *= 4
            set_size += power

        if population_size <= set_size:
            pool = list(population)
            for i in range(count):
                j = self._randbelow(population_size - i)
                result[i] = pool[j]
                pool[j] = pool[population_size - i - 1]
        else:
            selected = set()
            for i in range(count):
                j = self._randbelow(population_size)
                while j in selected:
                    j = self._randbelow(population_size)
                selected.add(j)
                result[i] = population[j]
        return result


def _new_dga_rng():
    """Create the simulator-compatible private per-trial DGA stream."""
    return _DgaCpythonRandom(_dga_seed_for_robot())


def _new_dga_backoff_rng():
    """Create a private collision-backoff stream, isolated from GA and drops."""
    return _DgaCpythonRandom(_dga_seed_for_robot() + 0x9E3779B9)


dga_rng = _new_dga_rng()
dga_backoff_rng = _new_dga_backoff_rng()
dga_population = []
dga_best_plan = {}
dga_best_fitness = DGA_INF
dga_path = []
dga_generation = 0
dga_clue_signature = None
dga_pending_deltas = []
dga_last_sent_signatures = {}
dga_received_entries = {}
dga_received_solution_pool = []
dga_received_solutions = []
dga_received_latest_owner_prefix = {}
dga_received_better_solution = False
dga_delta_counter = 0
dga_probability_normalizer = 0.0
dga_bad_prediction_count = {}
dga_last_predicted_peer_first_task = {}
dga_last_assessed_peer_solution = set()
dga_dropped_peers = set()
# Reused repair workspaces avoid allocating two sets for every repair.
dga_candidate_mask = bytearray(GRID_SIZE * GRID_SIZE)
dga_seen_mask = bytearray(GRID_SIZE * GRID_SIZE)
dga_candidate_workspace = PackedCandidateWorkspace(
    GRID_SIZE, TOP_K_MAX_CELLS)


def _apply_top_k_capacity(capacity):
    global dga_candidate_workspace
    if not (1 <= capacity <= GRID_SIZE * GRID_SIZE):
        raise ValueError("invalid DGA Top-K capacity")
    if (
        dga_candidate_workspace is not None
        and dga_candidate_workspace.capacity == capacity
    ):
        return
    dga_candidate_workspace = None
    gc.collect()
    dga_candidate_workspace = PackedCandidateWorkspace(
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
        and 0 <= next_x < GRID_SIZE
        and 0 <= next_y < GRID_SIZE
    ):
        record_intersection(next_x, next_y)
        grid[idx(next_x, next_y)] = CELL_SEARCHED
        terminal_target_step_counted = True
    found_target = True
    move_forward_flag = False
    motors_off()
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
# position/state = 1, next-step intent = 2, DGA allocation entry = 3,
# clue = 4, target/alert = 5, hub command = 7
#
# Topic 3 is the only DGA allocator-specific exchange. Payload format:
#   solution,generation,fitness,owner,order,x,y,path_size,removed,timestamp
# Coordinates are "X,X" when removed=1. All numeric fields are nonnegative so
# the UART '-' frame delimiter never appears inside a payload.
# Topic-3 payloads must not contain '-' because '-' is the UART frame delimiter.
# Examples:
#   001.3,4-                         robot 00 position/state
#   002.7,8-                         robot 00 next-step intent
#   003.00g8,8,1234500,00,0,7,8,3,0,1-  robot 00 DGA entry
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
    """Publish/forward a clue at most once in this trial."""
    global topic_4_sent
    clue = (int(x), int(y))
    if clue in forwarded_clues:
        return False
    uart_tx_lock.acquire()
    try:
        i = 2
        i = _write_int(tx_buf, i, clue[0])
        tx_buf[i] = ord(','); i += 1
        i = _write_int(tx_buf, i, clue[1])
        uart_send('4', i - 2)
    finally:
        uart_tx_lock.release()
    forwarded_clues.add(clue)
    if not metrics_frozen:
        topic_4_sent += 1
    return True

def publish_target(x, y):
    """Publish that we found the target at (x,y)."""
    global topic_5_sent, found_target
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
    found_target = True

def publish_intent(x=None, y=None):
    """
    Publish our intended next cell for low-level collision avoidance only.
    This is not a DGA claim, task owner, or task-cell reservation.
    """
    global topic_2_sent, published_intent
    intent = None if x is None or y is None else (int(x), int(y))
    signature = ((int(pos[0]), int(pos[1])), intent)
    if published_intent == signature:
        return False
    uart_tx_lock.acquire()
    try:
        i = 2
        i = _write_int(tx_buf, i, signature[0][0])
        tx_buf[i] = ord(','); i += 1
        i = _write_int(tx_buf, i, signature[0][1])
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
    published_intent = signature
    if not metrics_frozen:
        topic_2_sent += 1
    return True


def publish_dga_payload(payload):
    """Publish one compact DGA owner-path delta on topic 3."""
    global topic_3_sent
    if not _trial_traffic_enabled():
        return
    _uart_send_text("3", payload)
    if not metrics_frozen:
        topic_3_sent += 1

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
    global published_intent
    peer_pos.clear()
    peer_pos_yield.clear()
    peer_intent.clear()
    published_intent = None


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


def _valid_scenario_sha256(value):
    return (
        len(value) == 64
        and value == value.lower()
        and all(ch in "0123456789abcdef" for ch in value)
    )


def _handle_config_command(payload):
    global TOP_K_PERCENT, TOP_K_MAX_CELLS, msg_drop_rate
    global applied_config_sequence, applied_top_k_ppm, applied_drop_ppm
    global applied_scenario_sha256
    global last_config_request, last_config_status
    global control_state

    sequence = 0
    top_k_ppm = 0
    top_k_cells = 0
    drop_ppm = 0
    trial_mode = ""
    horizon = 0
    logic_revision = ""
    scenario_sha256 = ""
    try:
        fields = payload.strip().split(",")
        if len(fields) != 9 or fields[0] != "CFG":
            raise ValueError
        sequence = int(fields[1])
        top_k_ppm = int(fields[2])
        top_k_cells = int(fields[3])
        drop_ppm = int(fields[4])
        trial_mode = fields[5]
        horizon = int(fields[6])
        logic_revision = fields[7]
        scenario_sha256 = fields[8]
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
            sequence, top_k_ppm, top_k_cells, drop_ppm, trial_mode,
            horizon, logic_revision, scenario_sha256, last_config_status)
        return
    applied_request = (
        applied_config_sequence, applied_top_k_ppm, TOP_K_MAX_CELLS,
        applied_drop_ppm, TRIAL_MODE, COMMITMENT_HORIZON,
        LOGIC_REVISION, applied_scenario_sha256)
    if request == applied_request:
        last_config_request = request
        last_config_status = "OK"
        _send_config_ack(
            sequence, top_k_ppm, top_k_cells, drop_ppm, trial_mode,
            horizon, logic_revision, scenario_sha256, "OK")
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
    003.00g8,8,1234500,00,0,7,8,3,0,1-  # topic 3: DGA entry
    004.5,2-       # topic 4: clue
    005.6,1-       # topic 5: target/alert
    996.1-         # topic 7: hub command

    Ignores:
      - other status fields we don't currently need
    """
    global pre_start_signal, peer_intent, peer_pos, peer_pos_yield
    global current_task_cell, first_clue_seen, target_location
    global start_signal, found_target, move_forward_flag, published_intent

    # Minimal parsing: "<sender>/<topic>:<payload>"
    try:
        left, payload = line.split(".", 1)
        if len(left) < 3:
            return
        sender = left[0:2]
        topic  = left[2]
    except ValueError:
        return

    # The simulator never delivers a robot's own broadcast back to it.
    # Match that contract even when the UART bridge echoes a frame.
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
        mark_cell_searched_miss(ox, oy)
        if current_task_cell == (ox, oy) and (pos[0], pos[1]) != (ox, oy):
            current_task_cell = None

    elif topic == "2": #intent
        global topic_2_rec
        if not metrics_frozen:
            topic_2_rec += 1
        fields = payload.split(",")
        if len(fields) not in (2, 4):
            return
        intent_fields = fields
        if len(fields) == 4:
            try:
                px, py = int(fields[0]), int(fields[1])
            except ValueError:
                return
            if not (0 <= px < GRID_SIZE and 0 <= py < GRID_SIZE):
                return
            peer_pos_yield[sender] = (px, py)
            intent_fields = fields[2:]
        if intent_fields[0] == "X" and intent_fields[1] == "X":
            peer_intent.pop(sender, None)
            return
        try:
            ix, iy = map(int, intent_fields)
        except ValueError:
            return
        if not (0 <= ix < GRID_SIZE and 0 <= iy < GRID_SIZE):
            return
        peer_intent[sender] = (ix, iy)

    elif topic == "3": # DGA allocation entry, droppable
        if not _trial_traffic_enabled():
            return
        if random.random() < msg_drop_rate:
            return
        global topic_3_rec
        if not metrics_frozen:
            topic_3_rec += 1
        _dga_receive_payload(sender, payload)

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
            if add_clue_if_new(x, y):
                first_clue_seen = True
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
    """Check the current cell for a clue without moving off of it."""
    global first_clue_seen
    if not running or found_target:
        return
    if at_intersection_and_white():
        if add_clue_if_new(pos[0], pos[1]):
            first_clue_seen = True
            publish_clue(pos[0], pos[1])
            gc.collect()

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

    refresh_probability_normalizer()
    recompute_value_map()


def update_target_on_miss(i):
    """
    We have searched cell i for the target with POD_target = 1 and did not find
    it. Set P_target(i) = 0 and renormalize target_p/prob_map.
    """
    if not (0 <= i < GRID_SIZE * GRID_SIZE):
        return
    grid[i] = CELL_SEARCHED
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
    blocked = globals().get("temporary_invalid_task_until", {})
    expires_at = blocked.get(cell)
    if expires_at is None:
        return False
    now = time.ticks_ms()
    if time.ticks_diff(now, expires_at) >= 0:
        blocked.pop(cell, None)
        return False
    return True


def _rid_sort_key(rid):
    try:
        return (0, int(rid))
    except (TypeError, ValueError):
        return (1, str(rid))


def _dga_valid_task(cell):
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


def _dga_candidates():
    """Build the exact probability/distance/cell ranking in packed storage."""
    started_us = time.ticks_us()
    workspace = dga_candidate_workspace
    workspace.count = 0
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cell = (x, y)
            if not _dga_valid_task(cell):
                continue
            cell_id = y * GRID_SIZE + x
            count = workspace.count
            if count < workspace.capacity:
                insertion = count
                while insertion > 0 and workspace._precedes(
                        cell_id, workspace.ids[insertion - 1],
                        target_p, idx, pos):
                    workspace.ids[insertion] = workspace.ids[insertion - 1]
                    insertion -= 1
                workspace.ids[insertion] = cell_id
                workspace.count += 1
            elif workspace._precedes(
                    cell_id, workspace.ids[workspace.capacity - 1],
                    target_p, idx, pos):
                insertion = workspace.capacity - 1
                while insertion > 0 and workspace._precedes(
                        cell_id, workspace.ids[insertion - 1],
                        target_p, idx, pos):
                    workspace.ids[insertion] = workspace.ids[insertion - 1]
                    insertion -= 1
                workspace.ids[insertion] = cell_id
    record_candidate_filter_time(started_us)
    return workspace


def _dga_team_agents():
    global dga_dropped_peers
    team = {str(ROBOT_ID): (pos[0], pos[1])}
    dropped = set()
    for rid, cell in peer_pos.items():
        peer_id = str(rid)
        if peer_id == str(ROBOT_ID):
            continue
        if int(dga_bad_prediction_count.get(peer_id, 0)) >= DGA_BAD_PRED_LIMIT:
            dropped.add(peer_id)
            continue
        if cell is None:
            dropped.add(peer_id)
            continue
        candidate = (int(cell[0]), int(cell[1]))
        if not (
            0 <= candidate[0] < GRID_SIZE
            and 0 <= candidate[1] < GRID_SIZE
        ):
            dropped.add(peer_id)
            continue
        team[peer_id] = candidate
    dga_dropped_peers = dropped
    return team


def _dga_refresh_probability_normalizer():
    """Cache max(target_p), matching the simulator's normalized objective."""
    global dga_probability_normalizer
    maximum = 0.0
    for probability in target_p:
        value = float(probability)
        # NaN fails the comparison and positive infinity fails ``< inf``.
        # Only finite positive values may define the shared normalizer.
        if value > maximum and value < float("inf"):
            maximum = value
    dga_probability_normalizer = maximum if maximum > 0.0 else 1.0
    return dga_probability_normalizer


def _dga_normalized_probability(cell):
    normalizer = dga_probability_normalizer
    if normalizer <= 0.0:
        normalizer = _dga_refresh_probability_normalizer()
    probability = float(target_p[idx(cell[0], cell[1])]) / normalizer
    if probability < 0.0:
        return 0.0
    if probability > 1.0:
        return 1.0
    return probability


def _dga_edge_cost(previous, cell):
    distance = manhattan(previous[0], previous[1], cell[0], cell[1])
    return distance + 8.0 * (1.0 - _dga_normalized_probability(cell))


def _dga_route_cost(start, route):
    total = 0.0
    previous = start
    for cell in route:
        total += _dga_edge_cost(previous, cell)
        previous = cell
    return total


def _dga_fitness(plan, team):
    costs = [_dga_route_cost(team[rid], plan.get(rid, [])) for rid in sorted(team, key=_rid_sort_key)]
    if not costs:
        return DGA_INF
    return max(costs) + DGA_MIN_SUM_TIE_WEIGHT * sum(costs)


def _dga_copy_plan(plan):
    return {str(rid): list(route) for rid, route in plan.items()}


def _dga_plan_signature(plan):
    return tuple((rid, tuple(plan.get(rid, []))) for rid in sorted(plan, key=_rid_sort_key))


def _dga_pack_cell(cell):
    # x-major encoding preserves Python's (x, y) tuple comparison.
    return int(cell[0]) * GRID_SIZE + int(cell[1])


def _dga_unpack_cell(cell_id):
    return (int(cell_id) // GRID_SIZE, int(cell_id) % GRID_SIZE)


def _dga_is_packed_plan(plan):
    return (
        isinstance(plan, tuple)
        and len(plan) == 3
        and isinstance(plan[0], array)
        and isinstance(plan[1], array)
    )


def _dga_pack_plan(plan, team_ids):
    cells = array("H")
    lengths = array("H")
    for rid in team_ids:
        route = plan.get(rid, [])
        lengths.append(len(route))
        for cell in route:
            cells.append(_dga_pack_cell(cell))
    return (cells, lengths, team_ids)


def _dga_clone_packed_plan(plan):
    return (array("H", plan[0]), array("H", plan[1]), plan[2])


def _dga_unpack_plan(plan):
    cells, lengths, team_ids = plan
    unpacked = {}
    offset = 0
    for route_index in range(len(team_ids)):
        rid = team_ids[route_index]
        length = int(lengths[route_index])
        route = []
        end = offset + length
        while offset < end:
            route.append(_dga_unpack_cell(cells[offset]))
            offset += 1
        unpacked[rid] = route
    return unpacked


def _dga_compare_packed_plans(left, right):
    left_cells, left_lengths, left_ids = left
    right_cells, right_lengths, right_ids = right
    if left_ids < right_ids:
        return -1
    if left_ids > right_ids:
        return 1

    left_offset = 0
    right_offset = 0
    for route_index in range(len(left_ids)):
        left_length = int(left_lengths[route_index])
        right_length = int(right_lengths[route_index])
        shared = min(left_length, right_length)
        for cell_index in range(shared):
            left_cell = left_cells[left_offset + cell_index]
            right_cell = right_cells[right_offset + cell_index]
            if left_cell < right_cell:
                return -1
            if left_cell > right_cell:
                return 1
        if left_length < right_length:
            return -1
        if left_length > right_length:
            return 1
        left_offset += left_length
        right_offset += right_length
    return 0


def _dga_compare_packed_scores(left, right):
    # Score layout: (packed_plan, fitness, stable_ordinal).
    if left[1] < right[1]:
        return -1
    if left[1] > right[1]:
        return 1
    plan_order = _dga_compare_packed_plans(left[0], right[0])
    if plan_order:
        return plan_order
    if left[2] < right[2]:
        return -1
    if left[2] > right[2]:
        return 1
    return 0


def _dga_insert_packed_score(ranked, score, limit=None):
    insert_at = len(ranked)
    while insert_at > 0 and _dga_compare_packed_scores(score, ranked[insert_at - 1]) < 0:
        insert_at -= 1
    if limit is not None and insert_at >= limit:
        return
    ranked.insert(insert_at, score)
    if limit is not None and len(ranked) > limit:
        ranked.pop()


def _dga_fitness_packed(plan, team):
    cells, lengths, team_ids = plan
    costs = []
    offset = 0
    for route_index in range(len(team_ids)):
        rid = team_ids[route_index]
        previous = team[rid]
        route_cost = 0.0
        end = offset + int(lengths[route_index])
        while offset < end:
            cell = _dga_unpack_cell(cells[offset])
            route_cost += _dga_edge_cost(previous, cell)
            previous = cell
            offset += 1
        costs.append(route_cost)
    if not costs:
        return DGA_INF
    return max(costs) + DGA_MIN_SUM_TIE_WEIGHT * sum(costs)


def _dga_rank_packed_population(population, team):
    ranked = []
    for ordinal, plan in enumerate(population):
        _dga_insert_packed_score(
            ranked, (plan, _dga_fitness_packed(plan, team), ordinal))
    return ranked


def _dga_append_cost(start, route, cell):
    return _dga_edge_cost(route[-1] if route else start, cell)


def _dga_prepare_repair_masks(candidates):
    for cell_index in range(len(dga_candidate_mask)):
        dga_candidate_mask[cell_index] = 0
        dga_seen_mask[cell_index] = 0
    for cell in candidates:
        dga_candidate_mask[idx(cell[0], cell[1])] = 1


def _dga_repair_plan(plan, team, candidates):
    _dga_prepare_repair_masks(candidates)
    repaired = {rid: [] for rid in team}
    if isinstance(plan, dict):
        for rid in sorted(team, key=_rid_sort_key):
            for raw in plan.get(rid, []):
                try:
                    cell = (int(raw[0]), int(raw[1]))
                except (TypeError, ValueError, IndexError):
                    continue
                cell_index = idx(cell[0], cell[1]) if (
                    0 <= cell[0] < GRID_SIZE and 0 <= cell[1] < GRID_SIZE) else -1
                if (
                    cell_index >= 0
                    and dga_candidate_mask[cell_index]
                    and not dga_seen_mask[cell_index]
                    and _dga_valid_task(cell)
                ):
                    repaired[rid].append(cell)
                    dga_seen_mask[cell_index] = 1
    for cell in candidates:
        cell_index = idx(cell[0], cell[1])
        if dga_seen_mask[cell_index]:
            continue
        owner = min(
            repaired,
            key=lambda rid: (
                _dga_append_cost(team[rid], repaired[rid], cell),
                len(repaired[rid]), _rid_sort_key(rid)))
        repaired[owner].append(cell)
        dga_seen_mask[cell_index] = 1
    return repaired


def _dga_nearest_neighbor_order(plan, team):
    ordered = {rid: [] for rid in team}
    for rid in sorted(team, key=_rid_sort_key):
        remaining = list(plan.get(rid, []))
        previous = team[rid]
        while remaining:
            cell = min(
                remaining,
                key=lambda c: (_dga_edge_cost(previous, c), -target_p[idx(c[0], c[1])], c))
            ordered[rid].append(cell)
            remaining.remove(cell)
            previous = cell
    return ordered


def _dga_greedy_seed(team, candidates):
    plan = {rid: [] for rid in team}
    for cell in candidates:
        owner = min(
            team,
            key=lambda rid: (
                _dga_append_cost(team[rid], plan[rid], cell),
                len(plan[rid]), _rid_sort_key(rid)))
        plan[owner].append(cell)
    return _dga_nearest_neighbor_order(plan, team)


def _dga_random_seed(team, candidates):
    ids = sorted(team, key=_rid_sort_key)
    cells = list(candidates)
    rng = globals().get("dga_rng", random)
    rng.shuffle(cells)
    plan = {rid: [] for rid in ids}
    for i, cell in enumerate(cells):
        plan[ids[i % len(ids)]].append(cell)
    return _dga_nearest_neighbor_order(plan, team)


def _dga_current_path_seed(team, candidates):
    own_id = str(ROBOT_ID)
    if own_id not in team:
        return None
    candidate_set = set(candidates)
    own = [cell for cell in dga_path if cell in candidate_set]
    if not own:
        return None

    plan = _dga_greedy_seed(team, candidates)
    used = set(own)
    plan[own_id] = own + [cell for cell in plan.get(own_id, []) if cell not in used]
    for rid in list(plan):
        if rid != own_id:
            plan[rid] = [cell for cell in plan[rid] if cell not in used]
    return _dga_repair_plan(plan, team, candidates)


def _dga_rank_population(population, team, candidates):
    scored = []
    for raw in population:
        plan = _dga_repair_plan(raw, team, candidates)
        scored.append((plan, _dga_fitness(plan, team)))
    if not scored:
        plan = _dga_greedy_seed(team, candidates)
        scored.append((plan, _dga_fitness(plan, team)))
    scored.sort(key=lambda item: (item[1], _dga_plan_signature(item[0])))
    return scored


def _dga_tournament(scored):
    count = min(3, len(scored))
    rng = globals().get("dga_rng", random)
    contenders = rng.sample(scored, count)
    contenders.sort(key=lambda item: (item[1], _dga_plan_signature(item[0])))
    return _dga_copy_plan(contenders[0][0])


def _dga_crossover(parent_a, parent_b, team, candidates):
    rng = globals().get("dga_rng", random)
    child = {rid: [] for rid in team}
    for rid in sorted(team, key=_rid_sort_key):
        route_a = list(parent_a.get(rid, []))
        route_b = list(parent_b.get(rid, []))
        if not route_a:
            child[rid].extend(route_b[:len(route_b) // 2])
            continue
        if not route_b:
            child[rid].extend(route_a[:len(route_a) // 2])
            continue

        a_start = rng.randrange(0, len(route_a))
        a_end = rng.randrange(a_start + 1, len(route_a) + 1)
        b_start = rng.randrange(0, len(route_b))
        b_end = rng.randrange(b_start + 1, len(route_b) + 1)
        child[rid].extend(route_a[a_start:a_end])
        child[rid].extend(route_b[b_start:b_end])
    return _dga_repair_plan(child, team, candidates)


def _dga_mutate(plan, team, candidates):
    rng = globals().get("dga_rng", random)
    result = _dga_copy_plan(plan)
    ids = sorted(team, key=_rid_sort_key)
    if not ids:
        return result

    operation = rng.choice(("move", "swap", "reinsert", "reverse", "clean"))
    if operation == "move":
        sources = [rid for rid in ids if result.get(rid)]
        if sources:
            source = rng.choice(sources)
            destination = rng.choice(ids)
            cell = result[source].pop(rng.randrange(len(result[source])))
            result.setdefault(destination, []).insert(
                rng.randrange(len(result.get(destination, [])) + 1), cell)
    elif operation == "swap":
        sources = [rid for rid in ids if result.get(rid)]
        if len(sources) >= 2:
            first, second = rng.sample(sources, 2)
            first_index = rng.randrange(len(result[first]))
            second_index = rng.randrange(len(result[second]))
            result[first][first_index], result[second][second_index] = (
                result[second][second_index], result[first][first_index])
    elif operation == "reinsert":
        rid = rng.choice(ids)
        route = result.get(rid, [])
        if len(route) >= 2:
            cell = route.pop(rng.randrange(len(route)))
            route.insert(rng.randrange(len(route) + 1), cell)
    elif operation == "reverse":
        rid = rng.choice(ids)
        route = result.get(rid, [])
        if len(route) >= 3:
            start = rng.randrange(0, len(route) - 1)
            end = rng.randrange(start + 2, len(route) + 1)
            route[start:end] = list(reversed(route[start:end]))
    else:
        for rid in ids:
            result[rid] = [cell for cell in result.get(rid, []) if _dga_valid_task(cell)]
    return _dga_repair_plan(result, team, candidates)


def _dga_prepare_population(team, candidates):
    team_ids = tuple(sorted(team, key=_rid_sort_key))
    ranked = []
    ordinal = 0
    population_count = 0

    def consider(plan, repair):
        nonlocal ordinal, population_count
        if _dga_is_packed_plan(plan):
            plan = _dga_unpack_plan(plan)
        if repair:
            plan = _dga_repair_plan(plan, team, candidates)
        packed = _dga_pack_plan(plan, team_ids)
        score = (packed, _dga_fitness_packed(packed, team), ordinal)
        _dga_insert_packed_score(ranked, score, max(1, DGA_POPULATION_SIZE))
        ordinal += 1
        population_count += 1

    # Iterate the three sources separately to avoid allocating concatenated
    # population lists. The order and duplicate weighting match the simulator.
    for plan in dga_population:
        consider(plan, True)
    for plan in dga_received_solutions:
        consider(plan, True)
    for plan in dga_received_solution_pool:
        consider(plan, True)

    consider(_dga_greedy_seed(team, candidates), False)
    preserved = _dga_current_path_seed(team, candidates)
    if preserved is not None:
        consider(preserved, False)

    while population_count < max(1, DGA_POPULATION_SIZE):
        consider(_dga_random_seed(team, candidates), False)

    return [item[0] for item in ranked]


def _dga_tournament_packed(ranked):
    count = min(3, len(ranked))
    rng = globals().get("dga_rng", random)
    contenders = rng.sample(ranked, count)
    winner_rank = len(ranked)
    winner = None
    for contender in contenders:
        for rank_index in range(len(ranked)):
            if contender is ranked[rank_index]:
                if rank_index < winner_rank:
                    winner_rank = rank_index
                    winner = contender
                break
    return _dga_unpack_plan(winner[0])


def _dga_next_generation(population, team, candidates):
    rng = globals().get("dga_rng", random)
    ranked = _dga_rank_packed_population(population, team)
    elite_count = max(0, min(DGA_ELITE_COUNT, len(ranked), DGA_POPULATION_SIZE))
    next_population = [_dga_clone_packed_plan(item[0]) for item in ranked[:elite_count]]
    team_ids = tuple(sorted(team, key=_rid_sort_key))

    while len(next_population) < max(1, DGA_POPULATION_SIZE):
        parent_a = _dga_tournament_packed(ranked)
        parent_b = _dga_tournament_packed(ranked)
        if rng.random() < DGA_CROSSOVER_RATE:
            child = _dga_crossover(parent_a, parent_b, team, candidates)
        else:
            child = parent_a
        if rng.random() < DGA_MUTATION_RATE:
            child = _dga_mutate(child, team, candidates)
        # Packed parents, crossover results, and mutation results are already
        # repaired. Avoid an equivalent but allocation-heavy second repair.
        next_population.append(_dga_pack_plan(child, team_ids))

    return next_population


def _dga_encode_float(value):
    return "{:.17g}".format(float(value)).replace("-", "N")


def _dga_decode_float(value):
    return float(str(value).replace("N", "-"))


def _dga_queue_plan(plan, fitness):
    global dga_pending_deltas, dga_delta_counter
    dga_delta_counter += 1
    solution_id = str(ROBOT_ID) + "g" + str(dga_generation)
    exact_fitness = float(fitness)
    pending = []
    owners = set(str(owner) for owner in plan)
    owners.update(str(owner) for owner in dga_last_sent_signatures)
    for owner in sorted(owners, key=_rid_sort_key):
        prefix = list(plan.get(owner, []))[:DGA_COMMITMENT_HORIZON]
        path_signature = tuple(prefix)
        if (
            owner in dga_last_sent_signatures
            and tuple(dga_last_sent_signatures.get(owner, ()))
            == path_signature
        ):
            continue
        if not prefix:
            pending.append((
                solution_id, dga_generation, exact_fitness, owner,
                0, None, 0, 1, float(dga_delta_counter)))
        else:
            for order, cell in enumerate(prefix):
                pending.append((
                    solution_id, dga_generation, exact_fitness, owner,
                    order, cell, len(prefix), 0,
                    float(dga_delta_counter)))
    dga_pending_deltas = pending


def _dga_commit(plan, fitness):
    global dga_best_plan, dga_best_fitness, dga_path
    global dga_last_predicted_peer_first_task
    dga_best_plan = _dga_copy_plan(plan)
    dga_best_fitness = fitness
    dga_path = list(plan.get(str(ROBOT_ID), []))[:DGA_COMMITMENT_HORIZON]
    dga_last_predicted_peer_first_task = {
        str(rid): route[0]
        for rid, route in plan.items()
        if str(rid) != str(ROBOT_ID) and route
    }
    # Queue against the last transmitted owner prefixes every time.  The
    # queue itself deduplicates unchanged prefixes and emits empty clears for
    # owners that were advertised by the previous plan.
    _dga_queue_plan(plan, fitness)


def _dga_run_impl():
    global dga_population, dga_generation, dga_best_plan, dga_best_fitness, dga_path
    _dga_refresh_probability_normalizer()
    candidates = _dga_candidates()
    team = _dga_team_agents()
    if not candidates or not team:
        _dga_commit({str(ROBOT_ID): []}, 0.0)
        dga_population = []
        return

    population = _dga_prepare_population(team, candidates)

    for _ in range(DGA_ITERATIONS_PER_TRIGGER):
        population = _dga_next_generation(population, team, candidates)
        dga_generation += 1

    scored = _dga_rank_packed_population(population, team)
    best_plan = _dga_unpack_plan(scored[0][0])
    _dga_commit(best_plan, scored[0][1])
    dga_population = [
        _dga_clone_packed_plan(item[0])
        for item in scored[:DGA_POPULATION_SIZE]
    ]


def _dga_run():
    started_us = time.ticks_us()
    filter_time_before_us = candidate_filter_time_us_total
    try:
        _dga_run_impl()
    finally:
        record_allocator_solve_time(started_us, filter_time_before_us)


def dga_flush_messages():
    if not first_clue_seen or not _trial_traffic_enabled():
        return
    while dga_pending_deltas:
        solution, generation, fitness, owner, order, cell, size, removed, timestamp = dga_pending_deltas.pop(0)
        x = DGA_EMPTY_CELL if cell is None else str(cell[0])
        y = DGA_EMPTY_CELL if cell is None else str(cell[1])
        payload = ",".join((
            solution, str(generation), _dga_encode_float(fitness),
            str(owner), str(order), x, y, str(size), str(removed),
            _dga_encode_float(timestamp)))
        publish_dga_payload(payload)
        dga_last_sent_signatures[str(owner)] = tuple(dga_best_plan.get(str(owner), [])[:DGA_COMMITMENT_HORIZON])


def _dga_owner_path_from_solution(solution, owner):
    entries = solution["owners"].get(owner, {})
    size = solution["sizes"].get(owner, DGA_COMMITMENT_HORIZON)
    route = []
    for order in range(max(0, min(size, DGA_COMMITMENT_HORIZON))):
        if order not in entries:
            break
        route.append(entries[order])
    return route


def _dga_reconstruct_received(sender, solution_id):
    solution = dga_received_entries.get((sender, solution_id))
    if not solution:
        return {}
    plan = {
        str(owner): list(route)
        for owner, route in dga_received_latest_owner_prefix.get(str(sender), {}).items()
    }
    for owner, entries in solution["owners"].items():
        plan[str(owner)] = _dga_owner_path_from_solution(solution, str(owner))
    return plan


def _dga_assess_peer_prediction(
        sender, solution_id, generation, owner, order, cell, removed):
    """Assess each fresh sender-owned first-prefix prediction at most once."""
    if (
        str(sender) == str(ROBOT_ID)
        or str(owner) != str(sender)
        or int(order) != 0
        or removed
        or cell is None
    ):
        return

    sender_key = str(sender)
    assessment_key = (
        sender_key, int(generation), str(solution_id))
    if assessment_key in dga_last_assessed_peer_solution:
        return
    dga_last_assessed_peer_solution.add(assessment_key)

    predicted = dga_last_predicted_peer_first_task.get(sender_key)
    if predicted is None:
        return

    current = int(dga_bad_prediction_count.get(sender_key, 0))
    if current >= DGA_BAD_PRED_LIMIT:
        # Exclusion is permanent until the next physical trial reset.
        return

    matched = (
        manhattan(
            predicted[0], predicted[1], cell[0], cell[1])
        <= DGA_PREDICTION_TOLERANCE_CELLS
    )
    if matched:
        dga_bad_prediction_count[sender_key] = max(0, current - 1)
    else:
        dga_bad_prediction_count[sender_key] = min(
            DGA_BAD_PRED_LIMIT, current + 1)


def _dga_receive_payload(sender, payload):
    global dga_received_better_solution, dga_received_solutions
    global dga_best_plan, dga_best_fitness, current_task_cell
    try:
        fields = payload.split(",")
        if len(fields) != 10:
            return
        solution_id = fields[0]
        generation = int(fields[1])
        received_fitness = float(str(fields[2]).replace("N", "-"))
        owner = str(fields[3])
        order = int(fields[4])
        cell = None if fields[5] == DGA_EMPTY_CELL else (int(fields[5]), int(fields[6]))
        size = int(fields[7])
        removed = int(fields[8])
        timestamp = float(str(fields[9]).replace("N", "-"))
    except (TypeError, ValueError):
        return
    if (
        not solution_id
        or order < 0
        or size < 0
        or size > DGA_COMMITMENT_HORIZON
        or removed not in (0, 1)
        or received_fitness != received_fitness
        or abs(received_fitness) == float("inf")
        or timestamp != timestamp
        or abs(timestamp) == float("inf")
    ):
        return
    if not removed and (cell is None or not (0 <= cell[0] < GRID_SIZE and 0 <= cell[1] < GRID_SIZE)):
        return

    key = (str(sender), solution_id)
    solution = dga_received_entries.setdefault(
        key, {
            "generation": generation,
            "fitness": received_fitness,
            "owners": {},
            "sizes": {},
            "latest": {},
        })
    latest_key = (owner, order)
    previous = solution["latest"].get(latest_key)
    if previous is not None:
        if generation < previous[0]:
            return
        if (
            generation == previous[0]
            and timestamp <= previous[1] + DGA_FITNESS_EPS
        ):
            return
    solution["generation"] = max(solution["generation"], generation)
    solution["fitness"] = min(
        float(solution["fitness"]), float(received_fitness))
    solution["sizes"][owner] = size
    entries = solution["owners"].setdefault(owner, {})
    if removed or order >= size:
        entries.pop(order, None)
    else:
        entries[order] = cell
    solution["latest"][latest_key] = (generation, timestamp)

    assessor = globals().get("_dga_assess_peer_prediction")
    if assessor is not None:
        assessor(
            sender, solution_id, generation, owner, order, cell,
            bool(removed))

    sender_key = str(sender)
    sender_prefix = dga_received_latest_owner_prefix.setdefault(sender_key, {})
    sender_prefix[owner] = _dga_owner_path_from_solution(solution, owner)

    plan = _dga_reconstruct_received(sender_key, solution_id)
    if plan:
        dga_received_solution_pool.append(plan)
        if len(dga_received_solution_pool) > DGA_POPULATION_SIZE:
            dga_received_solution_pool.pop(0)
        dga_received_solutions = list(dga_received_solution_pool)

        better = received_fitness < dga_best_fitness - DGA_FITNESS_EPS
        if abs(received_fitness - dga_best_fitness) <= DGA_FITNESS_EPS:
            better = generation > dga_generation
        if better:
            old_prefix = tuple(dga_path)
            new_prefix = tuple(
                plan.get(str(ROBOT_ID), [])[:DGA_COMMITMENT_HORIZON])
            dga_best_plan = _dga_copy_plan(plan)
            dga_best_fitness = received_fitness
            dga_received_better_solution = True
            if new_prefix != old_prefix:
                current_task_cell = None


def _dga_clear_invalid_or_completed_cells():
    global dga_path, dga_best_plan
    dga_path = [cell for cell in dga_path if _dga_valid_task(cell)]
    dga_best_plan = {
        rid: [cell for cell in route if _dga_valid_task(cell)]
        for rid, route in dga_best_plan.items()}


def _dga_handle_allocator_goal_arrival(arrived_cell):
    """Clear only the wrapper goal; allocator repair runs on the next choose."""
    global current_task_cell
    if current_task_cell is None:
        return False
    if (arrived_cell[0] != current_task_cell[0]
            or arrived_cell[1] != current_task_cell[1]):
        return False
    current_task_cell = None
    return True


def _dga_defer_collision_reallocation():
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


def _dga_complete_cell_arrival(cell_i):
    """Publish/observe an arrival before emitting allocator completion state."""
    global first_clue_seen

    publish_position()
    update_target_on_miss(cell_i)

    reached_allocator_goal = False
    if first_clue_seen:
        reached_allocator_goal = _dga_handle_allocator_goal_arrival(pos)

    if not found_target:
        potential_clue = (pos[0], pos[1])
        if potential_clue not in clues and at_intersection_and_white():
            if add_clue_if_new(pos[0], pos[1]):
                first_clue_seen = True
                publish_clue(pos[0], pos[1])
                update_mem_headroom()
                gc.collect()

    if not found_target:
        dga_flush_messages()
    return reached_allocator_goal


def _dga_reset_if_new_clue_information():
    global dga_clue_signature, dga_population, dga_best_plan, dga_best_fitness
    global dga_path, dga_pending_deltas, dga_received_entries
    global dga_received_solution_pool, dga_received_solutions
    global dga_received_latest_owner_prefix
    signature = tuple(sorted(set(clues)))
    if dga_clue_signature is None:
        dga_population = []
        dga_best_plan = {}
        dga_best_fitness = DGA_INF
        dga_path = []
        dga_pending_deltas = []
        dga_received_entries = {}
        dga_received_solution_pool = []
        dga_received_solutions = []
        dga_received_latest_owner_prefix = {}
    dga_clue_signature = signature


def _dga_release_current_task_for_replan():
    global dga_path
    dga_path = []


def _pick_task_cell_impl():
    global dga_received_better_solution, pending_collision_reallocation
    _dga_reset_if_new_clue_information()
    _dga_clear_invalid_or_completed_cells()
    if pending_collision_reallocation:
        pending_collision_reallocation = False
        _dga_release_current_task_for_replan()
    if not dga_path or dga_received_better_solution:
        _dga_run()
        dga_received_better_solution = False
    return dga_path[0] if dga_path else None


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
      (all delivered peer positions are blocked for the complete route)
    The target_p/prob_map reward is applied as a bonus in the node priority.
    Returns a path as a list: [start, ..., task_cell], or [] if failure.
    """
    # Simple energy tracking - no function call counting needed
    if start == task_cell:
        return [start]

    blocked = set(peer_pos.values())
    blocked.discard(start)
    if task_cell in blocked:
        return []

    frontier.clear()
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
            if (nx, ny) in blocked and (nx, ny) != task_cell:
                continue

            move_cost = 1.0
            turns = quarter_turns(cur_dir, (dx, dy))
            turn_cost = TURN_COST * turns
            visited_pen = cfg.VISITED_STEP_PENALTY if grid[i] == CELL_SEARCHED else 0.0
            base_cost = move_cost + turn_cost + visited_pen

            reward_bonus = prob_map[i] * REWARD_FACTOR
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
    global dga_population, dga_best_plan, dga_best_fitness, dga_path
    global dga_generation, dga_clue_signature, dga_pending_deltas
    global dga_last_sent_signatures, dga_received_entries
    global dga_received_solution_pool, dga_received_solutions
    global dga_received_latest_owner_prefix, dga_received_better_solution
    global dga_delta_counter, dga_probability_normalizer
    global dga_bad_prediction_count, dga_last_predicted_peer_first_task
    global dga_last_assessed_peer_solution, dga_dropped_peers, dga_rng
    global dga_backoff_rng
    dga_population = []
    dga_best_plan = {}
    dga_best_fitness = DGA_INF
    dga_path = []
    dga_generation = 0
    dga_clue_signature = None
    dga_pending_deltas = []
    dga_last_sent_signatures = {}
    dga_received_entries = {}
    dga_received_solution_pool = []
    dga_received_solutions = []
    dga_received_latest_owner_prefix = {}
    dga_received_better_solution = False
    dga_delta_counter = 0
    dga_probability_normalizer = 0.0
    dga_bad_prediction_count = {}
    dga_last_predicted_peer_first_task = {}
    dga_last_assessed_peer_solution = set()
    dga_dropped_peers = set()
    dga_rng = _new_dga_rng()
    dga_backoff_rng = _new_dga_backoff_rng()

def reset_search_state_for_next_trial():
    """Clear trial/world knowledge before a manually repositioned trial."""
    global first_clue_seen, found_target, target_location, current_task_cell
    global target_bump_stop, abort_signal
    global last_task_cell, collision_event_counted_since_move, METRIC_START_TIME_MS
    global peer_intent, peer_pos, peer_pos_yield, heading, pos
    global published_intent, blocked_goal_failures
    global temporary_invalid_task_until, pending_collision_reallocation

    for i in range(GRID_SIZE * GRID_SIZE):
        grid[i] = CELL_UNSEARCHED
        target_p[i] = 1.0 / (GRID_SIZE * GRID_SIZE)
        prob_map[i] = target_p[i]
    clues[:] = []
    forwarded_clues.clear()
    peer_intent = {}
    peer_pos = {}
    peer_pos_yield = {}
    published_intent = None
    blocked_goal_failures = {}
    temporary_invalid_task_until = {}
    pending_collision_reallocation = False
    first_clue_seen = False
    found_target = False
    abort_signal = False
    target_bump_stop = False
    target_location = None
    current_task_cell = None
    last_task_cell = None
    collision_event_counted_since_move = False
    METRIC_START_TIME_MS = None
    # The operator physically returns the robot to this east-facing pose.
    pos[0], pos[1] = START_POS
    heading = (START_HEADING[0], START_HEADING[1])
    refresh_probability_normalizer()
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
    global pending_collision_reallocation
    global busy_ms, mem_free_min
    try:
        while running and not found_target:
            busy_timer_reset()
            gc.collect()
            update_mem_headroom()
            _expire_temporary_invalid_tasks()

            blocked_retry_cells = set()
            try:
                prev_task_cell = current_task_cell
                previous_task_completed = (
                    prev_task_cell is not None
                    and grid[idx(prev_task_cell[0], prev_task_cell[1])] == CELL_SEARCHED
                )
                previous_task_invalidated = (
                    (prev_task_cell is not None and not previous_task_completed)
                    or (
                        prev_task_cell is None
                        and last_task_cell is not None
                        and grid[idx(last_task_cell[0], last_task_cell[1])] != CELL_SEARCHED
                    )
                )

                retained_goal = (
                    prev_task_cell is not None
                    and grid[idx(prev_task_cell[0], prev_task_cell[1])] == CELL_UNSEARCHED
                    and not _task_temporarily_invalid(prev_task_cell)
                )
                if retained_goal:
                    task_cell = prev_task_cell
                else:
                    current_task_cell = None
                    if not first_clue_seen:
                        task_cell = next_serpentine_task_cell_in_band()
                    else:
                        task_cell = pick_task_cell()
                        dga_flush_messages()

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
                        if previous_task_invalidated and first_clue_seen:
                            task_cell_replan_count += 1
                        last_task_cell = task_cell
                    # The active movement task stays internal. Committed DGA
                    # owner-path prefixes are sent separately as topic-3 deltas.
                    current_task_cell = task_cell

                blocked_retry_cells.clear()
                abandoned_goal = False
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
                        if first_clue_seen:
                            path_replan_count += 1
                        if _retry_original_goal_after_failed_alternate(
                            blocked_retry_cells
                        ):
                            continue
                        blocked_goal_failures.pop(task_cell, None)
                        current_task_cell = None
                        publish_intent()
                        break

                    nxt = path[1]
                    publish_intent(nxt[0], nxt[1])

                    busy_timer_pause()
                    turn_towards(tuple(pos), nxt)
                    busy_timer_resume()
                    if not running or found_target:
                        break
                    for _ in range(10):
                        uart_service()
                        busy_timer_pause()
                        time.sleep_ms(10)
                        busy_timer_resume()
                        if not running or found_target:
                            break
                    if not running or found_target:
                        break

                    if i_should_yield(nxt[0], nxt[1]):
                        if first_clue_seen:
                            path_replan_count += 1
                            if not collision_event_counted_since_move:
                                collision_prevention_count += 1
                                collision_event_counted_since_move = True

                        failure_count = blocked_goal_failures.get(task_cell, 0) + 1
                        blocked_goal_failures[task_cell] = failure_count
                        if first_clue_seen and failure_count >= 2:
                            publish_intent()
                            wait_ms = int(dga_backoff_rng.random() * 5000.0)
                            temporary_invalid_task_until[task_cell] = time.ticks_add(
                                time.ticks_ms(), max(wait_ms, 1))
                            blocked_goal_failures.pop(task_cell, None)
                            _dga_defer_collision_reallocation()
                            abandoned_goal = True

                            busy_timer_pause()
                            wait_until = time.ticks_add(time.ticks_ms(), wait_ms)
                            while (
                                running
                                and not found_target
                                and time.ticks_diff(wait_until, time.ticks_ms()) > 0
                            ):
                                uart_service()
                                remaining = time.ticks_diff(
                                    wait_until, time.ticks_ms())
                                time.sleep_ms(min(10, max(1, remaining)))
                            busy_timer_resume()
                            break

                        blocked_retry_cells.add(nxt)
                        continue
                    break

                if not running or found_target:
                    break
                if abandoned_goal:
                    continue
                if len(path) < 2:
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

                if found_target and target_bump_stop:
                    break

                pos[0], pos[1] = nxt[0], nxt[1]
                collision_event_counted_since_move = False
                blocked_goal_failures.clear()
                record_intersection(pos[0], pos[1])
                cell_i = idx(pos[0], pos[1])
                _dga_complete_cell_arrival(cell_i)

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
