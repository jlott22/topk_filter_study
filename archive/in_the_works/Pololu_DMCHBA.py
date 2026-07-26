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
# Pololu 3pi+ 2040 OLED — DMCHBA Coordinated Search (UART → ESP32 → MQTT)
# ===========================================================
# Memory-optimized active version. The original allocator is preserved at
# archive/Pololu_DMCHBA_unoptimized.py and regression-tested against this file.
#
# Runs on the Pololu 3pi+ 2040 OLED using MicroPython.
# Communication uses simple text frames over UART; an attached ESP32 relays
# those frames to MQTT topics.
#
# Behavior overview:
#   * Before any clue is found the robot sweeps its half of the grid in a
#     lawn‑mower pattern, nudged outward by a small centre‑ward cost.
#   * After a clue appears, a resource-bounded DMCHBA allocator selects
#     internal task cells using implicit shared state, matching-by-clone
#     Hungarian assignment, and a short committed path.
#   * Selected task cells are NOT communicated. Only next-step intent is
#     published for collision avoidance.
#   * Bump sensors detect the target; on a bump both robots halt and report.
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
ALGORITHM_NAME = "DMCHBA"
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

METRICS_LOG_FILE = "metrics-log-DMCHBA.txt"
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
CELL_OBSTACLE   = 1  # target or peer reservation
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
TARGET_DECAY_EXP = 1.0   # target correlation around clues


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

# Peer state used for implicit coordination and safety.
# DMCHBA does not communicate selected task cells/current task cells.
peer_intent = {}  # peer_id -> (x, y) next-step safety intent only
peer_pos = {}     # peer_id -> (x, y) last reported position (post-drop)
peer_pos_yield = {}  # peer_id -> (x, y) last reported position for collision checks
published_clues = set()  # each locally detected or forwarded clue is sent once
communicated_intent = None
current_task_cell = None  # local internal task cell only; never published
last_task_cell = None
collision_event_counted_since_move = False
blocked_goal_cell = None
blocked_goal_conflicts = 0
temporary_invalid_task_until = {}
pending_collision_reallocation = False

# -----------------------------
# DMCHBA allocator state
# -----------------------------
# Hardware note: the simulator-style Top-K prefilter bounds the task set before
# robots are cloned and the Hungarian assignment matrix is built.
DMCHBA_COMMITMENT_HORIZON = COMMITMENT_HORIZON
DMCHBA_PSEUDOTASK_COST = 1.0e9
DMCHBA_TIE_EPS = EPS
dmchba_path = []
dmchba_clue_signature = None
dmchba_last_assignment_signature = None

# Fixed allocator workspaces.  The logical Hungarian matrix is never
# materialized.  Instead, a small agent-by-task base-cost table is expanded
# virtually when the solver requests a clone-row/task-column cost.
DMCHBA_MAX_MATRIX_N = TOP_K_MAX_CELLS + NUM_ROBOTS - 1
DMCHBA_HUNGARIAN_INF = float("inf")
dmchba_candidate_ids = array('H', [0] * TOP_K_MAX_CELLS)
dmchba_agent_task_costs = [
    array('d', [0.0] * TOP_K_MAX_CELLS) for _ in range(NUM_ROBOTS)
]
dmchba_h_u = array('d', [0.0] * (DMCHBA_MAX_MATRIX_N + 1))
dmchba_h_v = array('d', [0.0] * (DMCHBA_MAX_MATRIX_N + 1))
dmchba_h_minv = array('d', [0.0] * (DMCHBA_MAX_MATRIX_N + 1))
dmchba_h_p = array('H', [0] * (DMCHBA_MAX_MATRIX_N + 1))
dmchba_h_way = array('H', [0] * (DMCHBA_MAX_MATRIX_N + 1))
dmchba_h_used = bytearray(DMCHBA_MAX_MATRIX_N + 1)
dmchba_h_assignment = array('h', [-1] * DMCHBA_MAX_MATRIX_N)
dmchba_assigned_ids = array('H', [0] * TOP_K_MAX_CELLS)


def _apply_top_k_capacity(capacity):
    global DMCHBA_MAX_MATRIX_N
    global dmchba_candidate_ids, dmchba_agent_task_costs
    global dmchba_h_u, dmchba_h_v, dmchba_h_minv
    global dmchba_h_p, dmchba_h_way, dmchba_h_used
    global dmchba_h_assignment, dmchba_assigned_ids

    matrix_n = capacity + NUM_ROBOTS - 1
    if (
        dmchba_candidate_ids is not None
        and len(dmchba_candidate_ids) == capacity
        and DMCHBA_MAX_MATRIX_N == matrix_n
    ):
        return

    dmchba_candidate_ids = None
    dmchba_agent_task_costs = None
    dmchba_h_u = None
    dmchba_h_v = None
    dmchba_h_minv = None
    dmchba_h_p = None
    dmchba_h_way = None
    dmchba_h_used = None
    dmchba_h_assignment = None
    dmchba_assigned_ids = None
    gc.collect()

    candidate_ids = array('H', [0] * capacity)
    agent_task_costs = [
        array('d', [0.0] * capacity) for _ in range(NUM_ROBOTS)
    ]
    h_u = array('d', [0.0] * (matrix_n + 1))
    h_v = array('d', [0.0] * (matrix_n + 1))
    h_minv = array('d', [0.0] * (matrix_n + 1))
    h_p = array('H', [0] * (matrix_n + 1))
    h_way = array('H', [0] * (matrix_n + 1))
    h_used = bytearray(matrix_n + 1)
    h_assignment = array('h', [-1] * matrix_n)
    assigned_ids = array('H', [0] * capacity)

    DMCHBA_MAX_MATRIX_N = matrix_n
    dmchba_candidate_ids = candidate_ids
    dmchba_agent_task_costs = agent_task_costs
    dmchba_h_u = h_u
    dmchba_h_v = h_v
    dmchba_h_minv = h_minv
    dmchba_h_p = h_p
    dmchba_h_way = h_way
    dmchba_h_used = h_used
    dmchba_h_assignment = h_assignment
    dmchba_assigned_ids = assigned_ids


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
# Format: "<topic#>.<payload>-"
# position/state = 1, next-step intent = 2, clue = 4, target/alert = 5,
# syncstate (not used in this code) = 6, hub command = 7
#
# Topic 3 task-cell/current-task messages are intentionally unused for DMCHBA.
# Selected task cells stay internal; collision avoidance uses topic 2 intent only.
# Examples:
#   001.3,4-  robot 00 position/state (x,y only)
#   002.7,8-  robot 00 next-step intent at (7,8)
#   004.5,2-  robot 00 clue at (5,2)
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
    This is not an allocator task claim or task-cell reservation.
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
    011.3,4-       # topic 1: position/state (x,y only) - previous pos treated as visited
    002.7,8-       # topic 2: next-step intent for collision avoidance
    004.5,2-       # topic 4: clue
    005.6,1-       # topic 5: target/alert
    996.1-         # topic 7: hub command

    Ignores:
      - other status fields we don't currently need
    """
    global pre_start_signal, peer_intent, peer_pos, current_task_cell
    global first_clue_seen, target_location, start_signal, found_target
    global move_forward_flag, communicated_intent

    # Minimal parsing: "<sender><topic>.<payload>"
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

    elif topic == "3": # deprecated task-cell/current-task message
        # DMCHBA does not use communicated task cells or task reservations.
        # Ignore this topic so old bridges cannot affect allocation behavior.
        return

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
    Recompute target_p from known clues, then copy it into prob_map.

    Pre-clue (no clues yet):
        - We leave target_p as-is (typically uniform) and just recompute value.
    Post-clue:
        - target_p[i] ∝ sum_k 1 / (1 + d(i, clue_k))**TARGET_DECAY_EXP
          for unsearched cells; 0 for visited.
    """
    has_clues = len(clues) > 0

    if has_clues:
        # Rebuild target_p from the clues with tunable target decay
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                i = idx(x, y)
                if grid[i] == CELL_SEARCHED:  # visited: target cannot be here (POD_target=1)
                    target_p[i] = 0.0
                    continue
                s = 0.0
                for (cx, cy) in clues:
                    d = manhattan(x, y, cx, cy)
                    s += 1.0 / ((1.0 + d) ** TARGET_DECAY_EXP)
                target_p[i] = s
        renorm(target_p)

    else:
        # With no clue evidence, the exact posterior is uniform over every
        # cell that has not yet been searched.
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

    # Whether or not we have clues, recompute the unified value map
    recompute_value_map()


def update_target_on_miss(i):
    """
    We have effectively searched cell i for the target (POD_target = 1)
    and did NOT find it. Set P_target(i) = 0 and renormalize.
    """
    # BeliefMap parity recomputes the complete posterior after every miss.
    # Repeated in-place renormalization is mathematically close but can drift
    # enough to change EPS-governed allocator ties.
    update_prob_map()


def i_should_yield(ix, iy):
    """Yield if a peer reserved or currently occupies (ix, iy)."""
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
    """Deterministic robot ID ordering, numeric when possible."""
    try:
        return int(rid)
    except (ValueError, TypeError):
        return 9999


def _cell_sort_key(cell):
    return (cell[0], cell[1])


def _dmchba_clue_signature():
    """Compact signature for event-triggered reassignment after clue changes."""
    if not clues:
        return ()
    return tuple(sorted(clues))


def _dmchba_searched_count():
    """Retained diagnostic helper used by the memory-equivalence harness."""
    count = 0
    for value in grid:
        if value == CELL_SEARCHED:
            count += 1
    return count


def _dmchba_assignment_signature(task_count=None):
    """
    Exact, compact signature of every assignment input.

    Candidate IDs are stored losslessly as little-endian uint16 bytes. This
    preserves the simulator's full ordered task tuple without retaining a
    second object-heavy tuple of cells on the RP2040.
    """
    if task_count is None:
        task_count = _dmchba_candidate_indices()
    packed_tasks = bytearray(task_count * 2)
    for task_index in range(task_count):
        cell_id = dmchba_candidate_ids[task_index]
        packed_tasks[task_index * 2] = cell_id & 0xff
        packed_tasks[task_index * 2 + 1] = cell_id >> 8
    team = tuple(
        (rid, cell[0], cell[1])
        for rid, cell in _dmchba_team_agents()
    )
    return (_dmchba_clue_signature(), bytes(packed_tasks), team)


def _dmchba_team_agents():
    """Return the locally known team state as (rid, (x,y)) tuples."""
    team = {ROBOT_ID: (pos[0], pos[1])}
    for rid, cell in peer_pos.items():
        if rid != ROBOT_ID and cell is not None:
            x, y = cell
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                team[rid] = (x, y)
    return [(rid, team[rid]) for rid in sorted(team.keys(), key=_rid_sort_key)]


def _dmchba_valid_task(cell):
    if cell is None:
        return False
    x, y = cell
    if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
        return False
    checker = globals().get("_task_temporarily_invalid")
    temporarily_invalid = (
        checker(cell)
        if checker is not None
        else cell in globals().get("temporary_invalid_task_until", {})
    )
    return (
        grid[idx(x, y)] == CELL_UNSEARCHED
        and not temporarily_invalid
    )


def _dmchba_pack_cell(x, y):
    return y * GRID_SIZE + x


def _dmchba_unpack_cell(cell_id):
    return (cell_id % GRID_SIZE, cell_id // GRID_SIZE)


def _dmchba_candidate_precedes(left_id, right_id):
    """Return True when left has the original candidate-sort priority."""
    lx = left_id % GRID_SIZE
    ly = left_id // GRID_SIZE
    rx = right_id % GRID_SIZE
    ry = right_id // GRID_SIZE
    lp = target_p[idx(lx, ly)]
    rp = target_p[idx(rx, ry)]
    if lp != rp:
        return lp > rp
    ld = manhattan(pos[0], pos[1], lx, ly)
    rd = manhattan(pos[0], pos[1], rx, ry)
    if ld != rd:
        return ld < rd
    if lx != rx:
        return lx < rx
    return ly < ry


def _dmchba_candidate_indices():
    """Fill the packed Top-K buffer with canonical conditional ordering."""
    started_us = time.ticks_us()
    expiry = globals().get("_expire_temporary_invalid_tasks")
    if expiry is not None:
        expiry()
    temporarily_invalid = globals().get(
        "temporary_invalid_task_until", {})
    count = 0
    ranked = False
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if (
                grid[idx(x, y)] != CELL_UNSEARCHED
                or (x, y) in temporarily_invalid
            ):
                continue
            cell_id = _dmchba_pack_cell(x, y)
            if count < TOP_K_MAX_CELLS:
                dmchba_candidate_ids[count] = cell_id
                count += 1
                continue
            if not ranked:
                for sort_index in range(1, count):
                    sort_id = dmchba_candidate_ids[sort_index]
                    insert_at = sort_index
                    while insert_at > 0 and _dmchba_candidate_precedes(
                            sort_id, dmchba_candidate_ids[insert_at - 1]):
                        dmchba_candidate_ids[insert_at] = (
                            dmchba_candidate_ids[insert_at - 1])
                        insert_at -= 1
                    dmchba_candidate_ids[insert_at] = sort_id
                ranked = True
            if _dmchba_candidate_precedes(
                    cell_id, dmchba_candidate_ids[count - 1]):
                insert_at = TOP_K_MAX_CELLS - 1
                while insert_at > 0 and _dmchba_candidate_precedes(
                        cell_id, dmchba_candidate_ids[insert_at - 1]):
                    dmchba_candidate_ids[insert_at] = dmchba_candidate_ids[insert_at - 1]
                    insert_at -= 1
                dmchba_candidate_ids[insert_at] = cell_id
    record_candidate_filter_time(started_us)
    return count


def _dmchba_cost(ref, cell):
    """Canonical shared probability-adjusted base cost."""
    x, y = cell
    probability = target_p[idx(x, y)] / allocation_probability_normalizer
    probability = max(0.0, min(1.0, probability))
    distance = manhattan(ref[0], ref[1], x, y)
    return distance + 8.0 * (1.0 - probability)


def _dmchba_prepare_agent_task_costs(team_agents, task_count):
    """Precompute only the non-clone agent-by-task portion of the costs."""
    for agent_index in range(len(team_agents)):
        ref = team_agents[agent_index][1]
        costs = dmchba_agent_task_costs[agent_index]
        for task_index in range(task_count):
            cell_id = dmchba_candidate_ids[task_index]
            x = cell_id % GRID_SIZE
            y = cell_id // GRID_SIZE
            costs[task_index] = _dmchba_cost(ref, (x, y))


def _dmchba_virtual_cost(row_index, col_index, clones_per_agent, task_count):
    if col_index >= task_count:
        return DMCHBA_PSEUDOTASK_COST + col_index * DMCHBA_TIE_EPS
    agent_index = row_index // clones_per_agent
    clone_index = row_index % clones_per_agent
    cell_id = dmchba_candidate_ids[col_index]
    return (
        dmchba_agent_task_costs[agent_index][col_index]
        + DMCHBA_TIE_EPS * (
            cell_id
            + clone_index * 0.001
            + row_index * 0.000001
        )
    )


def _hungarian_minimize_virtual(matrix_n, clones_per_agent, task_count):
    """
    Hungarian solver over the implicit clone matrix.

    The solver follows the same shortest-augmenting-path traversal as the
    original implementation, but all O(n) work arrays are fixed and reused.
    """
    n = matrix_n
    if n == 0:
        return dmchba_h_assignment
    if n > DMCHBA_MAX_MATRIX_N:
        raise MemoryError("DMCHBA matrix exceeds fixed workspace")

    h_u = dmchba_h_u
    h_v = dmchba_h_v
    h_minv = dmchba_h_minv
    h_p = dmchba_h_p
    h_way = dmchba_h_way
    h_used = dmchba_h_used
    h_assignment = dmchba_h_assignment
    agent_task_costs = dmchba_agent_task_costs

    for j in range(n + 1):
        h_u[j] = 0.0
        h_v[j] = 0.0
        h_p[j] = 0
        h_way[j] = 0
        h_minv[j] = DMCHBA_HUNGARIAN_INF
        h_used[j] = 0
        if j < n:
            h_assignment[j] = -1

    for i in range(1, n + 1):
        h_p[0] = i
        j0 = 0
        for j in range(n + 1):
            h_minv[j] = DMCHBA_HUNGARIAN_INF
            h_used[j] = 0
            h_way[j] = 0
        while True:
            h_used[j0] = 1
            i0 = h_p[j0]
            row_index = i0 - 1
            agent_index = row_index // clones_per_agent
            delta = DMCHBA_HUNGARIAN_INF
            j1 = 0
            for j in range(1, n + 1):
                if h_used[j]:
                    continue
                col_index = j - 1
                if col_index < task_count:
                    cost_value = _dmchba_virtual_cost(
                        row_index, col_index, clones_per_agent, task_count)
                else:
                    cost_value = (
                        DMCHBA_PSEUDOTASK_COST
                        + col_index * DMCHBA_TIE_EPS
                    )
                cur = cost_value - h_u[i0] - h_v[j]
                if cur < h_minv[j]:
                    h_minv[j] = cur
                    h_way[j] = j0
                if h_minv[j] < delta:
                    delta = h_minv[j]
                    j1 = j
            for j in range(0, n + 1):
                if h_used[j]:
                    h_u[h_p[j]] += delta
                    h_v[j] -= delta
                else:
                    h_minv[j] -= delta
            j0 = j1
            if h_p[j0] == 0:
                break
        while True:
            j1 = h_way[j0]
            h_p[j0] = h_p[j1]
            j0 = j1
            if j0 == 0:
                break

    for j in range(1, n + 1):
        if h_p[j] > 0:
            h_assignment[h_p[j] - 1] = j - 1
    return h_assignment


def _dmchba_order_assigned_ids(assigned_count):
    """Return only the behaviorally observable committed greedy prefix."""
    ordered = []
    ref_x = pos[0]
    ref_y = pos[1]
    remaining_count = assigned_count
    while remaining_count > 0 and len(ordered) < DMCHBA_COMMITMENT_HORIZON:
        best_i = 0
        best_score = -1000000000.0
        best_id = dmchba_assigned_ids[0]
        for i in range(remaining_count):
            cell_id = dmchba_assigned_ids[i]
            x = cell_id % GRID_SIZE
            y = cell_id // GRID_SIZE
            distance = manhattan(ref_x, ref_y, x, y)
            score = -_dmchba_cost((ref_x, ref_y), (x, y))
            if score > best_score + DMCHBA_TIE_EPS:
                best_i = i
                best_score = score
                best_id = cell_id
            elif abs(score - best_score) <= DMCHBA_TIE_EPS:
                best_x = best_id % GRID_SIZE
                best_y = best_id // GRID_SIZE
                best_distance = manhattan(
                    ref_x, ref_y, best_x, best_y)
                if (
                    distance < best_distance
                    or (
                        distance == best_distance
                        and (x < best_x or (x == best_x and y < best_y))
                    )
                ):
                    best_i = i
                    best_score = score
                    best_id = cell_id
        best_x = best_id % GRID_SIZE
        best_y = best_id // GRID_SIZE
        ordered.append((best_x, best_y))
        ref_x = best_x
        ref_y = best_y
        for i in range(best_i, remaining_count - 1):
            dmchba_assigned_ids[i] = dmchba_assigned_ids[i + 1]
        remaining_count -= 1
    return ordered


def _dmchba_drop_invalid_path_cells():
    """Remove searched/invalid cells from the committed DMCHBA path."""
    global dmchba_path
    if not dmchba_path:
        return
    new_path = []
    for cell in dmchba_path:
        if _dmchba_valid_task(cell):
            new_path.append(cell)
    dmchba_path = new_path


def _dmchba_should_reassign():
    """Return a reassignment reason or None."""
    global dmchba_clue_signature, dmchba_path
    _dmchba_drop_invalid_path_cells()

    clue_sig = _dmchba_clue_signature()
    if dmchba_clue_signature is None:
        dmchba_clue_signature = clue_sig
        dmchba_path = []
        return "clue_changed"
    if clue_sig != dmchba_clue_signature:
        # Later clues update the shared belief but do not discard an active
        # commitment. This is the simulator's event-triggered cadence.
        dmchba_clue_signature = clue_sig

    if not dmchba_path:
        sig = _dmchba_assignment_signature()
        if sig != dmchba_last_assignment_signature:
            return "path_exhausted"

    return None


def _dmchba_run_assignment_impl(reason):
    """Run bounded matching-by-clone Hungarian assignment and store our path."""
    global dmchba_path, dmchba_last_assignment_signature

    team_agents = _dmchba_team_agents()
    task_count = _dmchba_candidate_indices()
    dmchba_last_assignment_signature = _dmchba_assignment_signature(
        task_count)

    if task_count == 0 or not team_agents:
        dmchba_path = []
        return

    num_agents = len(team_agents)
    clones_per_agent = (task_count + num_agents - 1) // num_agents
    if clones_per_agent < 1:
        clones_per_agent = 1

    matrix_n = clones_per_agent * num_agents
    _dmchba_prepare_agent_task_costs(team_agents, task_count)
    row_to_col = _hungarian_minimize_virtual(
        matrix_n, clones_per_agent, task_count)
    assigned_count = 0
    for row_i in range(matrix_n):
        col_i = row_to_col[row_i]
        if col_i < 0 or col_i >= task_count:
            continue
        rid = team_agents[row_i // clones_per_agent][0]
        if rid == ROBOT_ID:
            dmchba_assigned_ids[assigned_count] = dmchba_candidate_ids[col_i]
            assigned_count += 1

    dmchba_path = _dmchba_order_assigned_ids(assigned_count)


def _dmchba_run_assignment(reason):
    started_us = time.ticks_us()
    filter_time_before_us = candidate_filter_time_us_total
    try:
        _dmchba_run_assignment_impl(reason)
    finally:
        record_allocator_solve_time(started_us, filter_time_before_us)


def _pick_task_cell_impl():
    """
    Select the next DMCHBA task cell.

    This uses the hardware-bounded version of the simulation DMCHBA:
    implicit shared state -> cloned robots -> square Hungarian assignment ->
    greedy local route order -> short committed path. No allocator-specific
    bid/claim/task-cell messages are sent. Collision avoidance remains separate
    and uses only next-step intent messages.
    """
    global dmchba_path, pending_collision_reallocation
    if pending_collision_reallocation:
        pending_collision_reallocation = False
        dmchba_path = []
        reason = "collision_avoidance"
    else:
        reason = _dmchba_should_reassign()
    if reason is not None:
        _dmchba_run_assignment(reason)

    _dmchba_drop_invalid_path_cells()
    if dmchba_path:
        return dmchba_path[0]
    return None


def pick_task_cell():
    started_us = time.ticks_us()
    try:
        return _pick_task_cell_impl()
    finally:
        record_allocator_time(started_us)


def _dmchba_defer_collision_reallocation():
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


def _dmchba_complete_cell_arrival(cell_i):
    """Publish and observe a move without clearing its protected intent."""
    global first_clue_seen, current_task_cell

    publish_position()
    grid[cell_i] = CELL_SEARCHED
    update_target_on_miss(cell_i)

    reached_allocator_goal = False
    if (
        current_task_cell is not None
        and pos[0] == current_task_cell[0]
        and pos[1] == current_task_cell[1]
    ):
        current_task_cell = None
        reached_allocator_goal = True

    if not found_target:
        potential_clue = (pos[0], pos[1])
        if potential_clue not in clues and at_intersection_and_white():
            clues.append(potential_clue)
            first_clue_seen = True
            update_prob_map()
            publish_clue(pos[0], pos[1])
            update_mem_headroom()
            gc.collect()

    return reached_allocator_goal


def next_serpentine_task_cell_in_band():
    """
    Pre-clue: return the next unsearched cell in this robot's row band
    following a serpentine (boustrophedon) pattern.

    Order within the band:
      row BAND_Y_MIN: x=0..GRID_SIZE-1
      row BAND_Y_MIN+1: x=GRID_SIZE-1..0
      row BAND_Y_MIN+2: x=0..GRID_SIZE-1
      ...
    We start from the *current* position in that ordering and pick the
    first later cell that is still CELL_UNSEARCHED. If none remain, return None.
    """
    # If somehow we're outside our band, clamp the logical "current row"
    cur_x, cur_y = pos[0], pos[1]
    if cur_y < BAND_Y_MIN:
        cur_y = BAND_Y_MIN
    elif cur_y > BAND_Y_MAX:
        cur_y = BAND_Y_MAX

    passed_current = False

    for y in range(BAND_Y_MIN, BAND_Y_MAX + 1):
        # serpentine direction for this row
        # even offset rows (relative to BAND_Y_MIN) go left->right
        # odd offset rows go right->left
        row_offset = y - BAND_Y_MIN
        if row_offset % 2 == 0:
            x_iter = range(0, GRID_SIZE)
        else:
            x_iter = range(GRID_SIZE - 1, -1, -1)

        for x in x_iter:
            # find where we are in the ordering
            if not passed_current:
                if x == cur_x and y == cur_y:
                    passed_current = True
                continue

            i = idx(x, y)
            if grid[i] == CELL_UNSEARCHED:
                return (x, y)

    # No unsearched cells after current position → check if there are any
    # earlier ones in the band (e.g., if we started mid-band)
    for y in range(BAND_Y_MIN, BAND_Y_MAX + 1):
        row_offset = y - BAND_Y_MIN
        if row_offset % 2 == 0:
            x_iter = range(0, GRID_SIZE)
        else:
            x_iter = range(GRID_SIZE - 1, -1, -1)

        for x in x_iter:
            if x == cur_x and y == cur_y:
                # we already checked "after" current; don't wrap forever
                return None
            i = idx(x, y)
            if grid[i] == CELL_UNSEARCHED:
                return (x, y)

    # Entire band fully searched
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
    The reward from prob_map is applied as a bonus in the node priority.
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
    global dmchba_path, dmchba_clue_signature, dmchba_last_assignment_signature
    dmchba_path = []
    dmchba_clue_signature = None
    dmchba_last_assignment_signature = None

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
    global dmchba_path, dmchba_last_assignment_signature
    global pending_collision_reallocation
    global busy_ms, mem_free_min
    try:
        while running and not found_target:
            busy_timer_reset()
            # free any unused memory from previous iteration to avoid
            # MicroPython allocation failures during long searches
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
                    task_cell = current_task_cell
                else:
                    current_task_cell = None
                    if not first_clue_seen:
                        task_cell = next_serpentine_task_cell_in_band()
                    else:
                        task_cell = pick_task_cell()
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
                    # DMCHBA task cells/current tasks are internal only; do not publish.
                    current_task_cell = task_cell

                path = []
                while True:
                    # Temporarily treat any blocked retry cells as obstacles for planning
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
                    # Maintain low memory usage between planning iterations
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
                            _dmchba_defer_collision_reallocation()
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
                _dmchba_complete_cell_arrival(cell_i)

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
