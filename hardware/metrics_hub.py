#!/usr/bin/env python3
"""Multi-trial MQTT controller and metrics collector for Pololu robots.

Compatible robot commands on hub/command:
  "CFG,..." = apply and acknowledge per-trial configuration.
  "CMD,PRESTART,N", "CMD,START,N", "CMD,RUN,N", and "CMD,ABORT,N" use
  the applied configuration sequence and require per-robot acknowledgments.
A JSON scenario is also published on hub/trial_task for operator/host tooling.
The operator manually selects one of five handpicked CSV scenarios before
every trial.
Robot allocation payloads on topic 3 are recorded verbatim. Topic 6 is
reserved for configuration and control acknowledgments.
"""

import argparse
import csv
import hashlib
import json
import os
import re
import sys
import threading
import time
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from typing import Dict, Iterable, List, Optional, Set, Tuple

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

BROKER_HOST = "192.168.1.10"
BROKER_PORT = 1883
HUB_COMMAND_TOPIC = "hub/command"
HUB_TASK_TOPIC = "hub/trial_task"
ROBOT_IDS = ["00", "01", "02", "03"]
HOME = {"00": (0, 0), "01": (0, 6), "02": (0, 12), "03": (0, 18)}
SCENARIO_FILE = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "simulator",
        "scenarios",
        "final_trial_500.csv",
    )
)
STUDY_MANIFEST_LOCK = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "results",
        "hardware_handpicked_5_scenario_manifest.json",
    )
)
OUT_DIR = "./hub_logs"
VALID_DIGITS = set("123456")
TOPIC_CATEGORY = {
    "1": "state", "2": "collision_intent", "3": "allocation_primary",
    "4": "clue", "5": "target", "6": "control",
}
PROTECTED = {"2", "5"}
CORE = {"1", "2", "4", "5"}
ALLOCATION = {"3"}
RATE_SCALE = 1_000_000
TRIAL_MODE = "clue_search"
LOGIC_REVISION = "dcta_parity_v1"
DEFAULT_COMMITMENT_HORIZON = 3
MANUAL_STOP_KEY = "m"
HANDPICKED_SOURCE_IDS = ("4", "53", "232", "394", "473")
HANDPICKED_TARGETS = (
    (5, 9),
    (7, 2),
    (3, 11),
    (5, 4),
    (13, 16),
)
HANDPICKED_COHORT_SHA256 = (
    "92ebcdc84dc259fc27fc6123bef9ca9f0488a874e84e405344e349aa2d07d393"
)


class ConfigurationError(RuntimeError):
    pass


class ManualTrialStop(RuntimeError):
    pass


class TerminalKeyReader:
    """Poll one terminal key without requiring Enter, restoring terminal state."""

    def __init__(self, enabled=True):
        self.enabled = bool(enabled)
        self.active = False
        self._mode = None
        self._fd = None
        self._saved_attributes = None
        self._msvcrt = None
        self._select = None
        self._termios = None

    def __enter__(self):
        if not self.enabled or not sys.stdin.isatty():
            return self
        try:
            if os.name == "nt":
                import msvcrt

                self._msvcrt = msvcrt
                self._mode = "windows"
            else:
                import select
                import termios
                import tty

                self._fd = sys.stdin.fileno()
                self._saved_attributes = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
                self._select = select
                self._termios = termios
                self._mode = "posix"
            self.active = True
        except (ImportError, OSError, ValueError):
            self._restore()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._restore()
        return False

    def _restore(self):
        if (
            self._mode == "posix"
            and self._termios is not None
            and self._saved_attributes is not None
            and self._fd is not None
        ):
            try:
                self._termios.tcsetattr(
                    self._fd,
                    self._termios.TCSADRAIN,
                    self._saved_attributes,
                )
            except (OSError, ValueError):
                pass
        self.active = False
        self._mode = None

    def poll(self):
        if not self.active:
            return None
        if self._mode == "windows":
            if not self._msvcrt.kbhit():
                return None
            key = self._msvcrt.getwch()
            if key in {"\x00", "\xe0"}:
                if self._msvcrt.kbhit():
                    self._msvcrt.getwch()
                return None
            return key.lower()
        readable, _, _ = self._select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        return sys.stdin.read(1).lower()


class Scenario:
    def __init__(self, trial_id, target, clues, source_trial_id=None):
        self.trial_id = str(trial_id)
        self.source_trial_id = str(
            trial_id if source_trial_id is None else source_trial_id
        )
        self.target = target
        self.clues = clues


class RobotState:
    def __init__(self, last_pos=None):
        self.last_pos = last_pos
        self.steps = 0
        self.pre_steps = 0
        self.post_steps = 0
        self.unique = 0
        self.revisits = 0
        self.messages = {d: 0 for d in "123456"}
        self.post_messages = {d: 0 for d in "123456"}


class Trial:
    def __init__(self, run_id, scenario, robots):
        self.run_id = run_id
        self.scenario = scenario
        self.robots = robots
        self.visits = {}
        self.messages = {d: 0 for d in "123456"}
        self.post_messages = {d: 0 for d in "123456"}
        self.clues = []
        self.unexpected_clues = []
        self.location_warnings = []
        self.events = []
        self.t0 = 0.0
        self.first_clue = None
        self.end_time = None
        self.reporter = ""
        self.reported_target = None
        self.active = False
        self.onboard_baseline = {}
        self.algorithm_verified = False
        self.trial_mode = TRIAL_MODE
        self.commitment_horizon = DEFAULT_COMMITMENT_HORIZON
        self.logic_revision = LOGIC_REVISION
        self.scenario_sha256 = ""
        self.top_k_rate = 1.0
        self.top_k_max_cells = 361
        self.drop_rate = 0.0
        self.config_sequence = 0
        self.memory_error = None
        self.status = "pending"
        self.failure_reason = ""
        self.control_phase = "pending"
        self.pending_start_events = []
        self.premature_target = None


def record_initial_robot_visits(
    trial: Trial,
    positions: Dict[str, Tuple[int, int]],
) -> None:
    """Match the simulator's per-robot accounting for recorded start cells."""

    for rid, robot in trial.robots.items():
        position = positions.get(rid)
        if position is None:
            continue
        if position not in trial.visits:
            robot.unique += 1
        trial.visits[position] = trial.visits.get(position, 0) + 1


def record_memory_error_result(trial: Trial, memory_error) -> None:
    """Record the operator result and identify a manually ended memory crash."""

    trial.memory_error = memory_error
    if trial.status == "manual_stop" and memory_error is True:
        trial.status = "memory_error_crash"
        trial.failure_reason += "; memory error confirmed by operator"


def rate_to_ppm(value, *, allow_zero: bool) -> int:
    try:
        rate = Decimal(str(value).strip())
    except (InvalidOperation, ValueError) as error:
        raise ValueError("rate must be a decimal number") from error
    if not rate.is_finite():
        raise ValueError("rate must be a finite decimal number")
    lower_valid = rate >= 0 if allow_zero else rate > 0
    if not lower_valid or rate > 1:
        interval = "[0, 1]" if allow_zero else "(0, 1]"
        raise ValueError(f"rate must be in {interval}")
    try:
        return int(
            (rate * RATE_SCALE).quantize(Decimal("1"), rounding=ROUND_HALF_UP)
        )
    except InvalidOperation as error:
        raise ValueError("rate must be a decimal number") from error


def ppm_to_rate(ppm: int) -> float:
    return float(Decimal(int(ppm)) / Decimal(RATE_SCALE))


def top_k_cells(grid_size: int, top_k_ppm: int) -> int:
    if grid_size <= 0:
        raise ValueError("grid size must be positive")
    cells = grid_size * grid_size
    return max(1, (cells * int(top_k_ppm) + RATE_SCALE // 2) // RATE_SCALE)


def parse_config_ack(payload: str):
    fields = payload.strip().split(",")
    if len(fields) != 11 or fields[0] != "CFGACK":
        return None
    try:
        sequence = int(fields[1])
        top_k_ppm = int(fields[3])
        top_k_max_cells = int(fields[4])
        drop_ppm = int(fields[5])
        commitment_horizon = int(fields[7])
    except ValueError:
        return None
    trial_mode = fields[6].strip()
    logic_revision = fields[8].strip()
    scenario_sha256 = fields[9].strip().lower()
    status = fields[10].strip().upper()
    if status not in {"OK", "INVALID", "MEMORY_ERROR"}:
        return None
    if (
        trial_mode != TRIAL_MODE
        or commitment_horizon <= 0
        or not logic_revision
        or re.fullmatch(r"[0-9a-f]{64}", scenario_sha256) is None
    ):
        return None
    return {
        "sequence": sequence,
        "algorithm": fields[2].strip().upper(),
        "top_k_ppm": top_k_ppm,
        "top_k_max_cells": top_k_max_cells,
        "drop_ppm": drop_ppm,
        "trial_mode": trial_mode,
        "commitment_horizon": commitment_horizon,
        "logic_revision": logic_revision,
        "scenario_sha256": scenario_sha256,
        "status": status,
    }


def parse_control_ack(payload: str):
    fields = payload.strip().split(",")
    if len(fields) != 4 or fields[0] != "CMDACK":
        return None
    try:
        sequence = int(fields[1])
    except ValueError:
        return None
    robot_id = fields[2].strip()
    state = fields[3].strip().upper()
    if (
        sequence <= 0
        or re.fullmatch(r"\d{2}", robot_id) is None
        or state not in {"READY", "STARTED", "RUNNING", "ABORTED"}
    ):
        return None
    return {
        "sequence": sequence,
        "robot_id": robot_id,
        "state": state,
    }


def csv_append(path: str, header: List[str], rows: Iterable[Iterable[object]]) -> None:
    rows = list(rows)
    if not rows:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    header_needed = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as stream:
        writer = csv.writer(stream)
        if header_needed:
            writer.writerow(header)
        writer.writerows(rows)


def parse_ids(value: str) -> List[str]:
    ids = [part.strip() for part in value.split(",") if part.strip()]
    if not ids or any(not re.fullmatch(r"\d{2}", rid) for rid in ids):
        raise argparse.ArgumentTypeError("use comma-separated two-digit robot IDs")
    return ids


def parse_topic(topic: str, ids: Iterable[str]) -> Optional[Tuple[str, str]]:
    if len(topic) == 3 and topic[:2] in ids and topic[2] in VALID_DIGITS:
        return topic[:2], topic[2]
    return None


def parse_coord(payload: str) -> Optional[Tuple[int, int]]:
    text = payload.strip()
    if text.endswith("-"):
        text = text[:-1].strip()
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*(-?\d+)\s*", text)
    return (int(match.group(1)), int(match.group(2))) if match else None


def _integer(row: Dict[str, str], *keys: str) -> Optional[int]:
    for key in keys:
        if row.get(key, "") != "":
            try:
                return int(row[key])
            except ValueError:
                return None
    return None


def _trial_id(row: Dict[str, str], row_number: int, default_index: int) -> str:
    raw = None
    for key in ("episode", "trial_id"):
        value = row.get(key)
        if value is not None and str(value).strip():
            raw = str(value).strip()
            break
    if raw is None:
        return str(int(default_index))
    if re.fullmatch(r"[+-]?\d+", raw) is None:
        raise ValueError(
            "scenario row {} has a non-integer trial ID".format(row_number)
        )
    return str(int(raw))


def _validate_cell(
    cell: Tuple[int, int],
    *,
    grid_size: int,
    label: str,
    row_number: int,
) -> None:
    if not (0 <= cell[0] < grid_size and 0 <= cell[1] < grid_size):
        raise ValueError(
            "{} {} {} is outside the {}x{} grid".format(
                label, row_number, cell, grid_size, grid_size
            )
        )


def load_scenarios(
    path: str,
    *,
    grid_size: int = 19,
    starts: Optional[Iterable[Tuple[int, int]]] = None,
    expected_clues: Optional[int] = None,
) -> List[Scenario]:
    if not path or not os.path.exists(path):
        raise ValueError("scenario file does not exist: {}".format(path))
    if grid_size <= 0:
        raise ValueError("grid size must be positive")
    start_cells = set(starts or ())
    result = []
    trial_ids: Set[str] = set()
    with open(path, newline="", encoding="utf-8-sig") as stream:
        lines = (line for line in stream if not line.lstrip().startswith("#"))
        reader = csv.DictReader(lines)
        if not reader.fieldnames:
            raise ValueError("scenario file has no CSV header")
        for index, row in enumerate(reader):
            row_number = index + 2
            if None in row:
                raise ValueError(
                    "scenario row {} has unexpected extra columns".format(
                        row_number
                    )
                )
            tx, ty = _integer(row, "target_x", "object_x"), _integer(row, "target_y", "object_y")
            if tx is None or ty is None:
                raise ValueError(
                    "scenario row {} has a missing or non-integer target".format(
                        row_number
                    )
                )
            target = (tx, ty)
            _validate_cell(
                target,
                grid_size=grid_size,
                label="target on row",
                row_number=row_number,
            )
            if target in start_cells:
                raise ValueError(
                    "scenario row {} target {} is on a robot start".format(
                        row_number, target
                    )
                )
            clues, number, saw_blank_clue = [], 1, False
            while f"clue{number}_x" in row or f"clue{number}_y" in row:
                raw_x = row.get(f"clue{number}_x", "")
                raw_y = row.get(f"clue{number}_y", "")
                if raw_x == "" and raw_y == "":
                    saw_blank_clue = True
                    number += 1
                    continue
                if saw_blank_clue:
                    raise ValueError(
                        "scenario row {} has non-contiguous clue coordinates".format(
                            row_number
                        )
                    )
                x = _integer(row, f"clue{number}_x")
                y = _integer(row, f"clue{number}_y")
                if x is None or y is None:
                    raise ValueError(
                        "scenario row {} has a partial or non-integer clue {}".format(
                            row_number, number
                        )
                    )
                clue = (x, y)
                _validate_cell(
                    clue,
                    grid_size=grid_size,
                    label="clue on row",
                    row_number=row_number,
                )
                clues.append(clue)
                number += 1
            if expected_clues is not None and len(clues) != expected_clues:
                raise ValueError(
                    "scenario row {} has {} clues; expected {}".format(
                        row_number, len(clues), expected_clues
                    )
                )
            if len(set(clues)) != len(clues):
                raise ValueError(
                    "scenario row {} contains duplicate clues".format(row_number)
                )
            if target in clues:
                raise ValueError(
                    "scenario row {} target is also a clue".format(row_number)
                )
            trial_id = _trial_id(row, row_number, index)
            if trial_id in trial_ids:
                raise ValueError("duplicate trial ID: {}".format(trial_id))
            trial_ids.add(trial_id)
            result.append(Scenario(trial_id, target, clues))
    if not result:
        raise ValueError("scenario file contains no scenarios")
    return result


def handpicked_scenarios(scenarios: Iterable[Scenario]) -> List[Scenario]:
    """Return the fixed hardware cohort, renumbered to study IDs 1 through 5."""

    by_source: Dict[str, Scenario] = {}
    for scenario in scenarios:
        source_id = str(scenario.source_trial_id)
        if source_id in by_source:
            raise ValueError("duplicate source trial ID: {}".format(source_id))
        by_source[source_id] = scenario

    missing = [
        source_id
        for source_id in HANDPICKED_SOURCE_IDS
        if source_id not in by_source
    ]
    if missing:
        raise ValueError(
            "scenario file is missing handpicked source trial IDs: {}".format(
                missing
            )
        )

    selected = []
    for index, (source_id, expected_target) in enumerate(
        zip(HANDPICKED_SOURCE_IDS, HANDPICKED_TARGETS),
        1,
    ):
        source = by_source[source_id]
        if source.target != expected_target:
            raise ValueError(
                "source trial {} target is {}, expected {}".format(
                    source_id,
                    source.target,
                    expected_target,
                )
            )
        selected.append(
            Scenario(
                str(index),
                source.target,
                list(source.clues),
                source_trial_id=source_id,
            )
        )
    cohort_hash = scenario_manifest_sha256(selected)
    if cohort_hash != HANDPICKED_COHORT_SHA256:
        raise ValueError(
            "handpicked cohort SHA-256 {} does not match expected {}".format(
                cohort_hash,
                HANDPICKED_COHORT_SHA256,
            )
        )
    return selected


def scenario_manifest_sha256(scenarios: Iterable[Scenario]) -> str:
    manifest = [
        {
            "trial_id": str(scenario.trial_id),
            "target": [int(scenario.target[0]), int(scenario.target[1])],
            "clues": [
                [int(clue[0]), int(clue[1])] for clue in scenario.clues
            ],
        }
        for scenario in scenarios
    ]
    payload = json.dumps(
        manifest, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def enforce_scenario_manifest_lock(
    path: str,
    scenarios: Iterable[Scenario],
    *,
    grid_size: int,
) -> str:
    """Create or verify the scenario selection shared by every condition."""

    selected = list(scenarios)
    manifest_hash = scenario_manifest_sha256(selected)
    record = {
        "schema": 2,
        "grid_size": int(grid_size),
        "logic_revision": LOGIC_REVISION,
        "scenario_sha256": manifest_hash,
        "trial_ids": [str(scenario.trial_id) for scenario in selected],
        "source_trial_ids": [
            str(scenario.source_trial_id) for scenario in selected
        ],
    }
    lock_path = os.path.abspath(path)
    parent = os.path.dirname(lock_path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    if os.path.exists(lock_path):
        try:
            with open(lock_path, encoding="utf-8") as stream:
                existing = json.load(stream)
        except (OSError, ValueError) as error:
            raise ValueError(
                "cannot read scenario manifest lock {}".format(lock_path)
            ) from error
        for field in (
            "schema",
            "grid_size",
            "logic_revision",
            "scenario_sha256",
            "trial_ids",
            "source_trial_ids",
        ):
            if existing.get(field) != record[field]:
                raise ValueError(
                    "selected scenarios do not match study manifest lock "
                    "{} (field {})".format(lock_path, field)
                )
        return manifest_hash

    try:
        with open(lock_path, "x", encoding="utf-8") as stream:
            json.dump(record, stream, sort_keys=True, indent=2)
            stream.write("\n")
    except FileExistsError:
        # A parallel condition created it first; verify that record.
        return enforce_scenario_manifest_lock(
            lock_path,
            selected,
            grid_size=grid_size,
        )
    return manifest_hash


def category_totals(counts: Dict[str, int]) -> Dict[str, int]:
    protected = sum(counts[d] for d in PROTECTED)
    return {
        "protected": protected,
        "unprotected": sum(counts.values()) - protected,
        "core": sum(counts[d] for d in CORE),
        "allocation": sum(counts[d] for d in ALLOCATION),
    }


def category_string(counts: Dict[str, int]) -> str:
    values: Dict[str, int] = {}
    for digit, count in counts.items():
        name = TOPIC_CATEGORY[digit]
        values[name] = values.get(name, 0) + count
    return ";".join(f"{key}:{values[key]}" for key in sorted(values))


def gini(values: Iterable[float]) -> float:
    data = sorted(float(value) for value in values if value >= 0)
    total, count = sum(data), len(data)
    if not data or total == 0:
        return 0.0
    weighted = sum((index + 1) * value for index, value in enumerate(data))
    return 2 * weighted / (count * total) - (count + 1) / count


def trial_task_payload(trial: Trial, algorithm: str) -> str:
    scenario = trial.scenario
    return json.dumps({
        "run_id": trial.run_id,
        "trial_id": scenario.trial_id,
        "source_trial_id": scenario.source_trial_id,
        "algorithm": algorithm,
        "algorithm_verified": trial.algorithm_verified,
        "comm_level": trial.drop_rate,
        "drop_rate": trial.drop_rate,
        "top_k_rate": trial.top_k_rate,
        "top_k_max_cells": trial.top_k_max_cells,
        "config_sequence": trial.config_sequence,
        "trial_mode": trial.trial_mode,
        "commitment_horizon": trial.commitment_horizon,
        "logic_revision": trial.logic_revision,
        "scenario_sha256": trial.scenario_sha256,
        "target": scenario.target,
        "clues": scenario.clues,
    }, separators=(",", ":"))


class Hub:
    def __init__(self, args: argparse.Namespace, scenarios: List[Scenario]):
        if mqtt is None:
            raise RuntimeError(
                "paho-mqtt is required; install it with: python3 -m pip install paho-mqtt"
            )
        self.args, self.scenarios, self.ids = args, scenarios, args.robot_ids
        self.scenario_by_id = {
            scenario.trial_id: scenario for scenario in scenarios
        }
        self.scenario_sha256 = scenario_manifest_sha256(scenarios)
        self.condition = threading.Condition()
        self.connected = threading.Event()
        self.positions: Dict[str, Tuple[int, int]] = {}
        self.last_message = time.monotonic()
        self.trial: Optional[Trial] = None
        self.commands: List[List[object]] = []
        self.config_sequence = 0
        self.config_acks: Dict[str, Dict[str, object]] = {}
        self.control_acks: Dict[str, Dict[str, object]] = {}
        self.control_expected_state = ""
        self.control_fault = ""
        self.config_ack_rows: List[List[object]] = []
        self.control_ack_rows: List[List[object]] = []
        self.connected_robots: Set[str] = set()
        self.printed_config_acks: Set[Tuple[object, ...]] = set()
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except (AttributeError, TypeError):
            self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc:
            print(f"[ERR] MQTT connection rc={rc}")
            return
        # Robot topics are flat strings such as "001", so MQTT's level-based
        # '+' wildcard cannot express a two-character prefix. Subscribe broadly
        # and reject unrelated topics in parse_topic().
        client.subscribe("#", qos=0)
        self.connected.set()

    def on_message(self, client, userdata, message):
        parsed = parse_topic(message.topic, self.ids)
        if not parsed:
            return
        rid, digit = parsed
        payload = message.payload.decode(errors="replace").strip()
        now, wall = time.monotonic(), time.time()
        coordinate = parse_coord(payload) if digit in {"1", "4", "5"} else None
        with self.condition:
            self.last_message = now
            trial = self.trial
            if rid not in self.connected_robots:
                self.connected_robots.add(rid)
                if trial is None or not trial.active:
                    print(f"[CONNECTED] robot={rid}")
            if digit == "6":
                acknowledgment = parse_config_ack(payload)
                if acknowledgment is not None:
                    acknowledgment["received_wall_time_s"] = wall
                    acknowledgment["robot_id"] = rid
                    self.config_ack_rows.append([
                        wall,
                        self.trial.run_id if self.trial else "",
                        self.trial.scenario.trial_id if self.trial else "",
                        (
                            self.trial.scenario.source_trial_id
                            if self.trial else ""
                        ),
                        rid,
                        acknowledgment["sequence"],
                        acknowledgment["algorithm"],
                        acknowledgment["top_k_ppm"],
                        acknowledgment["top_k_max_cells"],
                        acknowledgment["drop_ppm"],
                        acknowledgment["trial_mode"],
                        acknowledgment["commitment_horizon"],
                        acknowledgment["logic_revision"],
                        acknowledgment["scenario_sha256"],
                        acknowledgment["status"],
                    ])
                    if acknowledgment["sequence"] == self.config_sequence:
                        self.config_acks[rid] = acknowledgment
                        print_key = (
                            rid,
                            acknowledgment["sequence"],
                            acknowledgment["algorithm"],
                            acknowledgment["top_k_ppm"],
                            acknowledgment["top_k_max_cells"],
                            acknowledgment["drop_ppm"],
                            acknowledgment["trial_mode"],
                            acknowledgment["commitment_horizon"],
                            acknowledgment["logic_revision"],
                            acknowledgment["scenario_sha256"],
                            acknowledgment["status"],
                        )
                        if (
                            (trial is None or not trial.active)
                            and print_key not in self.printed_config_acks
                        ):
                            self.printed_config_acks.add(print_key)
                            print(
                                "[CONFIRMED] robot={} algorithm={} top_k={} "
                                "({} cells) drop_rate={} mode={} horizon={} "
                                "revision={} manifest={} status={}".format(
                                    rid,
                                    acknowledgment["algorithm"],
                                    ppm_to_rate(
                                        acknowledgment["top_k_ppm"]
                                    ),
                                    acknowledgment["top_k_max_cells"],
                                    ppm_to_rate(
                                        acknowledgment["drop_ppm"]
                                    ),
                                    acknowledgment["trial_mode"],
                                    acknowledgment["commitment_horizon"],
                                    acknowledgment["logic_revision"],
                                    acknowledgment["scenario_sha256"][:12],
                                    acknowledgment["status"],
                                )
                            )
                    self.condition.notify_all()
                    return
                control_ack = parse_control_ack(payload)
                if control_ack is not None:
                    self.control_ack_rows.append([
                        wall,
                        trial.run_id if trial else "",
                        trial.scenario.trial_id if trial else "",
                        trial.scenario.source_trial_id if trial else "",
                        rid,
                        control_ack["sequence"],
                        control_ack["robot_id"],
                        control_ack["state"],
                        control_ack["robot_id"] == rid,
                    ])
                    if control_ack["sequence"] == self.config_sequence:
                        if control_ack["robot_id"] != rid:
                            self.control_fault = (
                                "topic robot {} payload robot {}"
                            ).format(rid, control_ack["robot_id"])
                            print(
                                "[CONTROL ERROR] {}".format(
                                    self.control_fault
                                )
                            )
                        elif (
                            control_ack["state"]
                            == self.control_expected_state
                        ):
                            self.control_acks[rid] = control_ack
                        elif (
                            control_ack["state"] == "ABORTED"
                            and self.control_expected_state != "ABORTED"
                        ):
                            self.control_fault = (
                                "robot {} entered ABORTED while {} was expected"
                            ).format(rid, self.control_expected_state)
                            print(
                                "[CONTROL ERROR] {}".format(
                                    self.control_fault
                                )
                            )
                    self.condition.notify_all()
                    return
                # Topic 6 is reserved for configuration/control replies.
                # Unknown or damaged ACKs are audited and discarded here;
                # they must never fall through as allocator traffic.
                self.control_ack_rows.append([
                    wall,
                    trial.run_id if trial else "",
                    trial.scenario.trial_id if trial else "",
                    trial.scenario.source_trial_id if trial else "",
                    rid,
                    "",
                    "",
                    "INVALID",
                    False,
                ])
                print(
                    "[CONTROL ERROR] invalid topic-6 payload robot={} "
                    "payload={!r}".format(rid, payload)
                )
                self.condition.notify_all()
                return
            if digit == "1" and coordinate is not None:
                self.positions[rid] = coordinate
            if trial:
                relative = now - trial.t0 if trial.t0 else ""
                phase = (
                    "trial" if trial.active
                    else trial.control_phase
                    if trial.control_phase
                    in {"preparing", "arming", "starting", "aborting"}
                    else "return" if trial.end_time is not None
                    else "ready"
                )
                trial.events.append([
                    trial.run_id, trial.scenario.trial_id,
                    trial.scenario.source_trial_id, wall, relative, phase,
                    rid, digit, TOPIC_CATEGORY[digit], payload,
                    coordinate[0] if coordinate else "", coordinate[1] if coordinate else "",
                ])
                if trial.active:
                    self.collect(trial, rid, digit, coordinate, relative)
                elif (
                    trial.control_phase in {"preparing", "arming", "starting"}
                    and digit == "5"
                    and coordinate is not None
                ):
                    trial.premature_target = (rid, coordinate, relative)
                elif trial.control_phase == "starting":
                    if digit != "5":
                        trial.pending_start_events.append(
                            (rid, digit, coordinate, relative)
                        )
            self.condition.notify_all()

    def collect(self, trial: Trial, rid: str, digit: str,
                coordinate: Optional[Tuple[int, int]], relative: float):
        robot = trial.robots[rid]
        trial.messages[digit] += 1
        robot.messages[digit] += 1
        post = trial.first_clue is not None or digit == "4"
        if post:
            trial.post_messages[digit] += 1
            robot.post_messages[digit] += 1
        if digit == "1" and coordinate is not None and coordinate != robot.last_pos:
            self._record_logical_step(
                trial, robot, coordinate, update_last_pos=True
            )
        elif digit == "4" and coordinate is not None:
            if coordinate not in trial.clues:
                trial.clues.append(coordinate)
            if (
                coordinate not in trial.scenario.clues
                and coordinate not in trial.unexpected_clues
            ):
                trial.unexpected_clues.append(coordinate)
                warning = "unexpected clue {} for trial {}".format(
                    coordinate,
                    trial.scenario.trial_id,
                )
                trial.location_warnings.append(warning)
                print("[LOCATION WARNING] {}".format(warning))
            if trial.first_clue is None:
                trial.first_clue = relative
            print(f"[CLUE] robot={rid} cell={coordinate}")
        elif digit == "5" and coordinate is not None and trial.end_time is None:
            # The simulator first enters the target cell and then terminates.
            # A physical bump is reported before Pololu updates its pose, so
            # account for that final logical search step without falsifying the
            # robot's physical last_pos used for return-home behavior.
            if coordinate != robot.last_pos:
                self._record_logical_step(
                    trial, robot, coordinate, update_last_pos=False
                )
            trial.end_time, trial.reporter, trial.reported_target = relative, rid, coordinate
            trial.active = False
            print(f"[TARGET] t={relative:.3f}s robot={rid} cell={coordinate}")
            if coordinate != trial.scenario.target:
                warning = "reported target {} does not match expected {}".format(
                    coordinate,
                    trial.scenario.target,
                )
                trial.location_warnings.append(warning)
                print("[LOCATION WARNING] {}".format(warning))

    @staticmethod
    def _record_logical_step(
        trial: Trial,
        robot: RobotState,
        coordinate: Tuple[int, int],
        *,
        update_last_pos: bool,
    ) -> None:
        if update_last_pos:
            robot.last_pos = coordinate
        robot.steps += 1
        if trial.first_clue is None:
            robot.pre_steps += 1
        else:
            robot.post_steps += 1
        if coordinate in trial.visits:
            robot.revisits += 1
        else:
            robot.unique += 1
        trial.visits[coordinate] = trial.visits.get(coordinate, 0) + 1

    def publish(self, topic: str, payload: str, kind: str, trial: Trial):
        result = self.client.publish(topic, payload, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"publish failed rc={result.rc} topic={topic}")
        self.commands.append([
            time.time(),
            trial.run_id,
            trial.scenario.trial_id,
            trial.scenario.source_trial_id,
            kind,
            topic,
            payload,
        ])

    def prompt_scenario(self, previous_id: Optional[str]) -> Scenario:
        print("\nAvailable hardware scenarios:")
        for scenario in self.scenarios:
            print(
                "  trial {}: target={} (source episode {})".format(
                    scenario.trial_id,
                    scenario.target,
                    scenario.source_trial_id,
                )
            )
        while True:
            default = " [{}]".format(previous_id) if previous_id else ""
            raw = input("Trial ID{}: ".format(default)).strip()
            selected_id = previous_id if not raw else raw
            if selected_id in self.scenario_by_id:
                return self.scenario_by_id[selected_id]
            print(
                "[INVALID] choose one of {}".format(
                    ",".join(scenario.trial_id for scenario in self.scenarios)
                )
            )

    def prompt_rate(self, label: str, current: float, *, allow_zero: bool) -> Tuple[int, float]:
        while True:
            raw = input(f"{label} [{current:g}]: ").strip()
            value = current if not raw else raw
            try:
                ppm = rate_to_ppm(value, allow_zero=allow_zero)
                return ppm, ppm_to_rate(ppm)
            except ValueError as error:
                print(f"[INVALID] {error}")

    def prompt_memory_error(self) -> bool:
        while True:
            response = input("Did any robot have a memory error? [yes/no]: ").strip().lower()
            if response in {"y", "yes"}:
                return True
            if response in {"n", "no"}:
                return False
            print("[INVALID] enter yes or no")

    def configure_robots(
        self,
        trial: Trial,
        top_k_ppm: int,
        drop_ppm: int,
    ) -> None:
        expected_cells = top_k_cells(self.args.grid_size, top_k_ppm)
        expected_horizon = (
            1 if self.args.algorithm == "CBAA"
            else self.args.commitment_horizon
        )
        while True:
            with self.condition:
                self.config_sequence += 1
                sequence = self.config_sequence
                self.config_acks = {}
            trial.config_sequence = sequence
            payload = "CFG,{},{},{},{},{},{},{},{}".format(
                sequence,
                top_k_ppm,
                expected_cells,
                drop_ppm,
                self.args.trial_mode,
                expected_horizon,
                self.args.logic_revision,
                self.scenario_sha256,
            )
            self.publish(HUB_COMMAND_TOPIC, payload, "configuration", trial)
            print(
                "[CONFIG] sequence={} top_k={} ({} cells) drop_rate={}".format(
                    sequence,
                    ppm_to_rate(top_k_ppm),
                    expected_cells,
                    ppm_to_rate(drop_ppm),
                )
            )

            deadline = time.monotonic() + self.args.config_timeout
            next_retry = time.monotonic() + self.args.config_retry_seconds
            error = None
            while True:
                with self.condition:
                    if len(self.config_acks) == len(self.ids):
                        acknowledgments = dict(self.config_acks)
                        break
                    now = time.monotonic()
                    remaining = deadline - now
                    if remaining <= 0:
                        acknowledgments = dict(self.config_acks)
                        missing = sorted(set(self.ids) - set(acknowledgments))
                        error = "configuration acknowledgment timeout: {}".format(missing)
                        break
                    self.condition.wait(
                        min(remaining, max(0.0, next_retry - now))
                    )
                now = time.monotonic()
                if now >= next_retry:
                    self.publish(
                        HUB_COMMAND_TOPIC,
                        payload,
                        "configuration_retry",
                        trial,
                    )
                    next_retry = now + self.args.config_retry_seconds

            if error is None:
                for rid in self.ids:
                    acknowledgment = acknowledgments[rid]
                    if acknowledgment["status"] != "OK":
                        error = "robot {} configuration status {}".format(
                            rid, acknowledgment["status"]
                        )
                        break
                    if acknowledgment["algorithm"] != self.args.algorithm:
                        error = "robot {} reports algorithm {}, expected {}".format(
                            rid,
                            acknowledgment["algorithm"],
                            self.args.algorithm,
                        )
                        break
                    applied = (
                        acknowledgment["top_k_ppm"],
                        acknowledgment["top_k_max_cells"],
                        acknowledgment["drop_ppm"],
                        acknowledgment["trial_mode"],
                        acknowledgment["commitment_horizon"],
                        acknowledgment["logic_revision"],
                        acknowledgment["scenario_sha256"],
                    )
                    expected = (
                        top_k_ppm,
                        expected_cells,
                        drop_ppm,
                        self.args.trial_mode,
                        expected_horizon,
                        self.args.logic_revision,
                        self.scenario_sha256,
                    )
                    if applied != expected:
                        error = "robot {} applied {}, expected {}".format(
                            rid, applied, expected
                        )
                        break

            if error is None:
                trial.algorithm_verified = True
                trial.top_k_rate = ppm_to_rate(top_k_ppm)
                trial.top_k_max_cells = expected_cells
                trial.drop_rate = ppm_to_rate(drop_ppm)
                trial.trial_mode = self.args.trial_mode
                trial.commitment_horizon = expected_horizon
                trial.logic_revision = self.args.logic_revision
                trial.scenario_sha256 = self.scenario_sha256
                print(f"[CONFIG] verified robots={sorted(self.ids)}")
                return

            print(f"[CONFIG ERROR] {error}")
            input("Recover/restart the affected robots, return them home, then press Enter: ")
            self.wait_home()

    def transition_robots(
        self, trial: Trial, command: str, expected_state: str
    ) -> None:
        """Retry a sequenced command until every robot confirms application."""

        command = command.upper()
        expected_state = expected_state.upper()
        if command not in {"PRESTART", "START", "RUN", "ABORT"}:
            raise ValueError("invalid robot control command")
        if expected_state not in {
            "READY", "STARTED", "RUNNING", "ABORTED"
        }:
            raise ValueError("invalid robot control acknowledgment")
        payload = "CMD,{},{:d}".format(command, trial.config_sequence)
        publish_error = None
        boundary_error = None
        with self.condition:
            self.control_acks = {}
            self.control_expected_state = expected_state
            self.control_fault = ""
            if command == "PRESTART":
                trial.control_phase = "preparing"
            elif command == "START":
                trial.active = False
                trial.control_phase = "arming"
            elif command == "RUN":
                if trial.premature_target is not None:
                    rid, coordinate, _relative = trial.premature_target
                    boundary_error = (
                        "target {} reported by robot {} before RUNNING quorum"
                    ).format(coordinate, rid)
                else:
                    # This lock makes t0, the replay queue, and the initial
                    # nonblocking RUN publish one indivisible boundary to the
                    # MQTT callback. Retries never move this boundary.
                    trial.t0 = time.monotonic()
                    trial.active = False
                    trial.control_phase = "starting"
                    trial.pending_start_events = []
            else:
                trial.control_phase = "aborting"
            if boundary_error is None:
                try:
                    self.publish(
                        HUB_COMMAND_TOPIC,
                        payload,
                        "{}_command".format(command.lower()),
                        trial,
                    )
                except RuntimeError as error:
                    self.control_expected_state = ""
                    if command == "RUN":
                        trial.t0 = 0.0
                        trial.control_phase = "arming"
                        trial.pending_start_events = []
                    publish_error = error
            else:
                self.control_expected_state = ""
        if boundary_error is not None:
            raise ConfigurationError(boundary_error)
        if publish_error is not None:
            raise ConfigurationError(
                "{} command transport failure: {}".format(
                    command, publish_error
                )
            ) from publish_error

        timeout = getattr(
            self.args, "control_timeout", self.args.config_timeout
        )
        retry_seconds = getattr(
            self.args,
            "control_retry_seconds",
            self.args.config_retry_seconds,
        )
        deadline = time.monotonic() + timeout
        next_retry = time.monotonic() + retry_seconds
        error = None
        acknowledgments = {}
        while True:
            with self.condition:
                if self.control_fault:
                    error = self.control_fault
                    acknowledgments = dict(self.control_acks)
                    break
                if (
                    command != "ABORT"
                    and trial.premature_target is not None
                ):
                    rid, coordinate, _relative = trial.premature_target
                    error = (
                        "target {} reported by robot {} before RUNNING quorum"
                    ).format(coordinate, rid)
                    acknowledgments = dict(self.control_acks)
                    break
                if len(self.control_acks) == len(self.ids):
                    acknowledgments = dict(self.control_acks)
                    break
                now = time.monotonic()
                remaining = deadline - now
                if remaining <= 0:
                    acknowledgments = dict(self.control_acks)
                    missing = sorted(set(self.ids) - set(acknowledgments))
                    error = "{} acknowledgment timeout: {}".format(
                        expected_state, missing
                    )
                    break
                self.condition.wait(
                    min(remaining, max(0.0, next_retry - now))
                )
            now = time.monotonic()
            if now >= next_retry:
                try:
                    self.publish(
                        HUB_COMMAND_TOPIC,
                        payload,
                        "{}_retry".format(command.lower()),
                        trial,
                    )
                except RuntimeError as publish_error:
                    error = "{} retry transport failure: {}".format(
                        command, publish_error
                    )
                    break
                next_retry = now + retry_seconds

        with self.condition:
            self.control_expected_state = ""
            self.control_fault = ""
        if error is not None:
            raise ConfigurationError(error)
        for rid in self.ids:
            acknowledgment = acknowledgments[rid]
            if (
                acknowledgment["sequence"] != trial.config_sequence
                or acknowledgment["robot_id"] != rid
                or acknowledgment["state"] != expected_state
            ):
                raise ConfigurationError(
                    "robot {} returned invalid {} acknowledgment".format(
                        rid, expected_state
                    )
                )
        print(
            "[{}] confirmed robots={}".format(
                expected_state, sorted(self.ids)
            )
        )

    def activate_after_run_quorum(self, trial: Trial) -> None:
        """Activate and replay non-target frames captured during RUN quorum."""

        with self.condition:
            if {
                rid
                for rid, acknowledgment in self.control_acks.items()
                if (
                    acknowledgment["sequence"] == trial.config_sequence
                    and acknowledgment["robot_id"] == rid
                    and acknowledgment["state"] == "RUNNING"
                )
            } != set(self.ids):
                raise ConfigurationError("RUNNING quorum is incomplete")
            if trial.premature_target is not None:
                rid, coordinate, _relative = trial.premature_target
                raise ConfigurationError(
                    "target {} reported by robot {} before RUNNING quorum".format(
                        coordinate, rid
                    )
                )
            trial.active = True
            trial.control_phase = "active"
            pending = trial.pending_start_events
            trial.pending_start_events = []
            for rid, digit, coordinate, relative in pending:
                self.collect(trial, rid, digit, coordinate, relative)

    def wait_home(self):
        deadline = time.monotonic() + self.args.ready_timeout
        announced = set()
        with self.condition:
            while True:
                ready = {
                    rid
                    for rid in self.ids
                    if HOME.get(rid) is not None
                    and self.positions.get(rid) == HOME[rid]
                }
                for rid in sorted(ready - announced):
                    print(f"[HOME] robot={rid} position={HOME[rid]}")
                    announced.add(rid)
                if len(ready) == len(self.ids):
                    print("[READY] all robots home")
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"robots not home: {sorted(set(self.ids) - ready)}")
                self.condition.wait(min(0.5, remaining))

    def wait_quiet(self):
        deadline = time.monotonic() + self.args.quiet_timeout
        with self.condition:
            while time.monotonic() - self.last_message < self.args.quiet_window:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("robot topics did not become quiet")
                self.condition.wait(min(0.1, remaining))

    def wait_target(self, trial: Trial):
        deadline = time.monotonic() + self.args.trial_timeout
        key_reader = TerminalKeyReader(enabled=True)
        with key_reader:
            if key_reader.active:
                print(
                    "[ACTIVE] Press M (no Enter) to stop this trial and "
                    "record a suspected robot crash.",
                    flush=True,
                )
            else:
                print(
                    "[ACTIVE] Single-key input is unavailable; press Ctrl+C "
                    "to stop this trial.",
                    flush=True,
                )
            with self.condition:
                while trial.end_time is None:
                    if key_reader.poll() == MANUAL_STOP_KEY:
                        trial.active = False
                        trial.end_time = (
                            max(0.0, time.monotonic() - trial.t0)
                            if trial.t0 else 0.0
                        )
                        raise ManualTrialStop(
                            "operator ended active trial with M key"
                        )
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        trial.active = False
                        raise TimeoutError(f"trial {trial.run_id} timed out")
                    self.condition.wait(min(0.1, remaining))

    def run(self):
        os.makedirs(self.args.out_dir, exist_ok=True)
        self.client.connect(self.args.broker, self.args.port, keepalive=10)
        self.client.loop_start()
        if not self.connected.wait(self.args.connect_timeout):
            raise TimeoutError("MQTT connection timed out")
        try:
            current_top_k_ppm = rate_to_ppm(
                self.args.top_k_rate, allow_zero=False
            )
            current_drop_ppm = rate_to_ppm(
                self.args.drop_rate, allow_zero=True
            )
            previous_scenario_id = None
            for number in range(1, self.args.trials + 1):
                self.wait_home()
                scenario = self.prompt_scenario(previous_scenario_id)
                previous_scenario_id = scenario.trial_id
                current_top_k_ppm, current_top_k_rate = self.prompt_rate(
                    "Top-K rate",
                    ppm_to_rate(current_top_k_ppm),
                    allow_zero=False,
                )
                current_drop_ppm, current_drop_rate = self.prompt_rate(
                    "Drop rate",
                    ppm_to_rate(current_drop_ppm),
                    allow_zero=True,
                )

                run_id = f"{int(time.time())}-{number:04d}"
                robots = {rid: RobotState(last_pos=self.positions.get(rid)) for rid in self.ids}
                trial = Trial(run_id, scenario, robots)
                trial.top_k_rate = current_top_k_rate
                trial.top_k_max_cells = top_k_cells(
                    self.args.grid_size, current_top_k_ppm
                )
                trial.drop_rate = current_drop_rate
                record_initial_robot_visits(trial, self.positions)
                with self.condition:
                    self.trial = trial
                self.configure_robots(
                    trial, current_top_k_ppm, current_drop_ppm
                )
                trial.onboard_baseline = self.onboard_counts()
                task = trial_task_payload(trial, self.args.algorithm)
                self.publish(HUB_TASK_TOPIC, task, "trial_task", trial)
                print(
                    "\n[TASK {}/{}] id={} source_episode={}".format(
                        number,
                        self.args.trials,
                        scenario.trial_id,
                        scenario.source_trial_id,
                    )
                )
                print(f"  target={scenario.target} clues={scenario.clues}")
                input(
                    "Confirm the target/clues match this scenario, "
                    "then press Enter: "
                )
                fatal_control_error = None
                try:
                    self.transition_robots(trial, "PRESTART", "READY")
                    self.wait_quiet()
                    print(
                        "[ARM] trial={} algorithm={} top_k={} ({} cells) "
                        "drop_rate={}".format(
                            scenario.trial_id,
                            self.args.algorithm,
                            trial.top_k_rate,
                            trial.top_k_max_cells,
                            trial.drop_rate,
                        )
                    )
                    self.transition_robots(trial, "START", "STARTED")
                    print(
                        "[RUN] trial={} releasing all armed robots".format(
                            scenario.trial_id
                        )
                    )
                    self.transition_robots(trial, "RUN", "RUNNING")
                    self.activate_after_run_quorum(trial)
                    self.wait_target(trial)
                    trial.status = "completed"
                except ConfigurationError as error:
                    with self.condition:
                        trial.active = False
                        trial.end_time = (
                            max(0.0, time.monotonic() - trial.t0)
                            if trial.t0 else 0.0
                        )
                    trial.status = "control_handshake_failed"
                    trial.failure_reason = str(error)
                    fatal_control_error = error
                    print(f"[TRIAL ERROR] {error}")
                    try:
                        self.transition_robots(trial, "ABORT", "ABORTED")
                    except ConfigurationError as abort_error:
                        trial.failure_reason += "; abort: {}".format(
                            abort_error
                        )
                except TimeoutError as error:
                    with self.condition:
                        was_active = trial.active or trial.control_phase == "active"
                        trial.active = False
                        trial.end_time = (
                            max(0.0, time.monotonic() - trial.t0)
                            if trial.t0 else 0.0
                        )
                    trial.status = (
                        "target_timeout"
                        if was_active
                        else "control_handshake_failed"
                    )
                    trial.failure_reason = str(error)
                    if not was_active:
                        fatal_control_error = ConfigurationError(str(error))
                    print(f"[TRIAL ERROR] {error}")
                    try:
                        self.transition_robots(trial, "ABORT", "ABORTED")
                    except ConfigurationError as abort_error:
                        trial.failure_reason += "; abort: {}".format(
                            abort_error
                        )
                        fatal_control_error = abort_error
                except ManualTrialStop as error:
                    with self.condition:
                        trial.active = False
                        if trial.end_time is None:
                            trial.end_time = (
                                max(0.0, time.monotonic() - trial.t0)
                                if trial.t0 else 0.0
                            )
                    trial.status = "manual_stop"
                    trial.failure_reason = str(error)
                    print(f"[TRIAL STOP] {error}")
                    try:
                        self.transition_robots(trial, "ABORT", "ABORTED")
                    except ConfigurationError as abort_error:
                        trial.failure_reason += "; abort: {}".format(
                            abort_error
                        )
                except KeyboardInterrupt:
                    with self.condition:
                        trial.active = False
                        trial.end_time = (
                            max(0.0, time.monotonic() - trial.t0)
                            if trial.t0 else 0.0
                        )
                    trial.status = "operator_aborted"
                    trial.failure_reason = "operator interrupted active trial"
                    try:
                        self.transition_robots(trial, "ABORT", "ABORTED")
                    except ConfigurationError as abort_error:
                        trial.failure_reason += "; abort: {}".format(
                            abort_error
                        )
                finally:
                    time.sleep(self.args.drain_seconds)
                    memory_error = self.prompt_memory_error()
                    record_memory_error_result(trial, memory_error)
                    self.write_trial(trial)
                    self.import_onboard(trial)
                    print(f"[SAVED] run={run_id}")
                    with self.condition:
                        self.trial = None

                if fatal_control_error is not None:
                    raise fatal_control_error
                if trial.memory_error:
                    input("Recover/reset the robots, then press Enter to continue: ")
        finally:
            self.write_commands()
            self.write_config_acks()
            self.write_control_acks()
            self.client.loop_stop()
            self.client.disconnect()

    def write_trial(self, trial: Trial):
        prefix = os.path.join(self.args.out_dir, self.args.algorithm)
        total_steps = sum(r.steps for r in trial.robots.values())
        pre_steps = sum(r.pre_steps for r in trial.robots.values())
        post_steps = sum(r.post_steps for r in trial.robots.values())
        revisits = sum(r.revisits for r in trial.robots.values())
        unique = len(trial.visits)
        message_total = sum(trial.messages.values())
        categories = category_totals(trial.messages)
        post_total = sum(trial.post_messages.values())
        post_allocation = sum(trial.post_messages[d] for d in ALLOCATION)
        sys_header = [
            "run_id", "trial_id", "source_trial_id",
            "algorithm", "algorithm_verified",
            "comm_level", "drop_rate", "top_k_rate", "top_k_max_cells",
            "trial_mode", "commitment_horizon", "logic_revision",
            "scenario_sha256",
            "memory_error", "trial_status", "failure_reason",
            "config_sequence", "expected_target",
            "reported_target", "target_match", "target_reporter", "duration_s",
            "first_clue_s", "expected_clues", "observed_clues",
            "unexpected_clues", "location_warning", "total_team_steps",
            "steps_before_first_clue", "post_clue_steps_to_find",
            "unique_cells_searched", "system_revisits", "messages_sent_total",
            "messages_delivered_total", "protected_messages_sent_total",
            "unprotected_messages_sent_total", "core_messages_sent_total",
            "allocation_messages_sent_total", "post_clue_messages_sent_total",
            "post_clue_allocation_messages_sent_total", "messages_sent_by_topic",
            "workload_gini_unique_cells_contributed",
        ]
        reported = trial.reported_target
        sys_row = [
            trial.run_id, trial.scenario.trial_id,
            trial.scenario.source_trial_id, self.args.algorithm,
            int(trial.algorithm_verified), trial.drop_rate, trial.drop_rate,
            trial.top_k_rate, trial.top_k_max_cells,
            trial.trial_mode, trial.commitment_horizon,
            trial.logic_revision, trial.scenario_sha256,
            "" if trial.memory_error is None else int(trial.memory_error),
            trial.status, trial.failure_reason, trial.config_sequence,
            f"{trial.scenario.target[0]}/{trial.scenario.target[1]}",
            f"{reported[0]}/{reported[1]}" if reported else "",
            int(reported == trial.scenario.target), trial.reporter,
            trial.end_time, trial.first_clue if trial.first_clue is not None else "",
            ";".join(f"{x}/{y}" for x, y in trial.scenario.clues),
            ";".join(f"{x}/{y}" for x, y in trial.clues),
            ";".join(f"{x}/{y}" for x, y in trial.unexpected_clues),
            "; ".join(trial.location_warnings), total_steps,
            pre_steps, post_steps, unique, revisits, message_total,
            message_total * max(0, len(self.ids) - 1), categories["protected"],
            categories["unprotected"], categories["core"], categories["allocation"],
            post_total, post_allocation, category_string(trial.messages),
            gini(r.unique for r in trial.robots.values()),
        ]
        csv_append(prefix + "_sys.csv", sys_header, [sys_row])

        robot_header = [
            "run_id", "trial_id", "source_trial_id",
            "algorithm", "algorithm_verified",
            "comm_level", "drop_rate", "top_k_rate", "top_k_max_cells",
            "trial_mode", "commitment_horizon", "logic_revision",
            "scenario_sha256",
            "memory_error", "trial_status", "failure_reason",
            "config_sequence", "robot_id",
            "steps_total", "steps_before_first_clue", "steps_after_first_clue",
            "unique_cells_contributed", "revisits", "messages_sent",
            "protected_messages_sent", "unprotected_messages_sent",
            "core_messages_sent", "allocation_messages_sent",
            "post_clue_messages_sent", "post_clue_allocation_messages_sent",
            "messages_sent_by_topic", "messages_delivered_to_robot", "last_x", "last_y",
        ]
        robot_rows = []
        for rid, robot in trial.robots.items():
            totals = category_totals(robot.messages)
            delivered = sum(sum(trial.robots[other].messages.values()) for other in self.ids if other != rid)
            robot_rows.append([
                trial.run_id, trial.scenario.trial_id,
                trial.scenario.source_trial_id, self.args.algorithm,
                int(trial.algorithm_verified), trial.drop_rate, trial.drop_rate,
                trial.top_k_rate, trial.top_k_max_cells,
                trial.trial_mode, trial.commitment_horizon,
                trial.logic_revision, trial.scenario_sha256,
                "" if trial.memory_error is None else int(trial.memory_error),
                trial.status, trial.failure_reason, trial.config_sequence,
                rid, robot.steps, robot.pre_steps,
                robot.post_steps, robot.unique, robot.revisits,
                sum(robot.messages.values()), totals["protected"],
                totals["unprotected"], totals["core"], totals["allocation"],
                sum(robot.post_messages.values()),
                sum(robot.post_messages[d] for d in ALLOCATION),
                category_string(robot.messages), delivered,
                robot.last_pos[0] if robot.last_pos else "",
                robot.last_pos[1] if robot.last_pos else "",
            ])
        csv_append(prefix + "_robots.csv", robot_header, robot_rows)
        csv_append(prefix + "_events.csv", [
            "run_id", "trial_id", "source_trial_id",
            "wall_time_s", "relative_time_s", "phase",
            "robot_id", "topic_digit", "category", "payload", "x", "y",
        ], trial.events)

    def import_onboard(self, trial: Trial):
        if not self.args.robot_metrics_root:
            return
        rows, fields = [], set()
        for rid in self.ids:
            path = os.path.join(self.args.robot_metrics_root, rid, f"metrics-log-{self.args.algorithm}.txt")
            if not os.path.exists(path):
                print(f"[WARN] no mounted metrics for {rid}: {path}")
                continue
            baseline = trial.onboard_baseline.get(rid, 0)
            deadline = time.monotonic() + self.args.onboard_wait
            records = []
            while time.monotonic() <= deadline:
                with open(path, newline="") as stream:
                    records = list(csv.DictReader(stream))
                if len(records) > baseline:
                    break
                time.sleep(0.1)
            if len(records) <= baseline:
                print(f"[WARN] no new onboard metric row for robot {rid}")
                continue
            row = dict(records[-1])
            row.update({
                "run_id": trial.run_id,
                "trial_id": trial.scenario.trial_id,
                "source_trial_id": trial.scenario.source_trial_id,
                "source_robot_id": rid,
            })
            rows.append(row)
            fields.update(row)
        if not rows:
            return
        preferred = [
            "run_id", "trial_id", "source_trial_id", "source_robot_id"
        ]
        names = preferred + sorted(fields - set(preferred))
        path = os.path.join(self.args.out_dir, f"{self.args.algorithm}_onboard.csv")
        write_header = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, "a", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=names)
            if write_header:
                writer.writeheader()
            writer.writerows(rows)

    def onboard_counts(self) -> Dict[str, int]:
        if not self.args.robot_metrics_root:
            return {}
        counts = {}
        for rid in self.ids:
            path = os.path.join(
                self.args.robot_metrics_root,
                rid,
                f"metrics-log-{self.args.algorithm}.txt",
            )
            if not os.path.exists(path):
                counts[rid] = 0
                continue
            with open(path, newline="") as stream:
                counts[rid] = sum(1 for _ in csv.DictReader(stream))
        return counts

    def write_commands(self):
        csv_append(os.path.join(self.args.out_dir, f"{self.args.algorithm}_commands.csv"),
                   [
                       "wall_time_s", "run_id", "trial_id",
                       "source_trial_id", "kind", "topic", "payload",
                   ],
                   self.commands)

    def write_config_acks(self):
        csv_append(
            os.path.join(
                self.args.out_dir,
                f"{self.args.algorithm}_configuration_acks.csv",
            ),
            [
                "wall_time_s", "run_id", "trial_id", "source_trial_id",
                "robot_id",
                "config_sequence", "algorithm", "top_k_ppm",
                "top_k_max_cells", "drop_ppm", "trial_mode",
                "commitment_horizon", "logic_revision",
                "scenario_sha256", "status",
            ],
            self.config_ack_rows,
        )

    def write_control_acks(self):
        csv_append(
            os.path.join(
                self.args.out_dir,
                f"{self.args.algorithm}_control_acks.csv",
            ),
            [
                "wall_time_s", "run_id", "trial_id", "source_trial_id",
                "topic_robot_id",
                "config_sequence", "payload_robot_id", "state",
                "robot_id_match",
            ],
            self.control_ack_rows,
        )


def positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive integer") from error
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def parser():
    result = argparse.ArgumentParser(description="Pololu multi-trial metrics hub")
    result.add_argument("--algorithm")
    result.add_argument(
        "--drop-rate",
        type=float,
        default=0.0,
        help="peer-message drop probability from 0 to 1",
    )
    result.add_argument(
        "--comm-level",
        type=float,
        default=None,
        help="deprecated alias for --drop-rate",
    )
    result.add_argument("--grid-size", type=int, default=19)
    result.add_argument("--top-k-rate", type=float, default=1.0)
    result.add_argument(
        "--trial-mode",
        choices=(TRIAL_MODE,),
        default=TRIAL_MODE,
    )
    result.add_argument(
        "--commitment-horizon",
        type=int,
        default=DEFAULT_COMMITMENT_HORIZON,
    )
    result.add_argument("--logic-revision", default=LOGIC_REVISION)
    result.add_argument(
        "--expected-clues",
        type=int,
        default=4,
        help="fail if any selected scenario has a different clue count",
    )
    result.add_argument("--broker", default=BROKER_HOST)
    result.add_argument("--port", type=int, default=BROKER_PORT)
    result.add_argument("--robot-ids", type=parse_ids, default=ROBOT_IDS)
    result.add_argument("--scenario-file", default=SCENARIO_FILE)
    result.add_argument(
        "--expected-scenario-sha256",
        help="fail unless the selected scenario manifest has this SHA-256",
    )
    result.add_argument(
        "--scenario-manifest-lock",
        help=(
            "shared cross-condition manifest lock path; defaults to "
            "results/hardware_handpicked_5_scenario_manifest.json"
        ),
    )
    result.add_argument(
        "--trials",
        type=positive_int,
        default=1,
        help="number of manually operated trials",
    )
    result.add_argument("--out-dir", default=OUT_DIR)
    result.add_argument("--connect-timeout", type=float, default=15)
    result.add_argument("--ready-timeout", type=float, default=300)
    result.add_argument("--quiet-window", type=float, default=1)
    result.add_argument("--quiet-timeout", type=float, default=15)
    result.add_argument("--trial-timeout", type=float, default=3600)
    result.add_argument("--drain-seconds", type=float, default=1)
    result.add_argument("--robot-metrics-root")
    result.add_argument("--onboard-wait", type=float, default=5)
    result.add_argument("--config-timeout", type=float, default=15)
    result.add_argument("--config-retry-seconds", type=float, default=1)
    result.add_argument("--control-timeout", type=float, default=15)
    result.add_argument("--control-retry-seconds", type=float, default=1)
    return result


def main():
    args = parser().parse_args()
    if args.algorithm is None:
        args.algorithm = input("Algorithm (ACBBA/CBAA/DGA/DMCHBA/HIPC/PI): ")
    args.algorithm = re.sub(r"[^A-Za-z0-9_-]+", "_", args.algorithm.strip().upper())
    if not args.algorithm:
        raise SystemExit("[ERR] algorithm cannot be empty")
    if args.comm_level is not None:
        args.drop_rate = args.comm_level
    try:
        top_k_ppm = rate_to_ppm(args.top_k_rate, allow_zero=False)
        drop_ppm = rate_to_ppm(args.drop_rate, allow_zero=True)
    except ValueError as error:
        raise SystemExit(f"[ERR] {error}") from error
    if (
        args.config_timeout <= 0
        or args.config_retry_seconds <= 0
        or args.control_timeout <= 0
        or args.control_retry_seconds <= 0
        or args.expected_clues is not None
        and args.expected_clues < 0
    ):
        raise SystemExit("[ERR] invalid trial/configuration option")
    if args.grid_size != 19:
        raise SystemExit("[ERR] Top-K hardware study requires grid size 19")
    if args.robot_ids != ROBOT_IDS:
        raise SystemExit(
            "[ERR] Top-K hardware study requires robot IDs 00,01,02,03"
        )
    if (
        args.trial_mode != TRIAL_MODE
        or args.commitment_horizon != DEFAULT_COMMITMENT_HORIZON
        or args.logic_revision != LOGIC_REVISION
    ):
        raise SystemExit("[ERR] noncanonical trial logic configuration")
    args.top_k_rate = ppm_to_rate(top_k_ppm)
    args.drop_rate = ppm_to_rate(drop_ppm)
    args.comm_level = args.drop_rate
    args.top_k_max_cells = top_k_cells(args.grid_size, top_k_ppm)
    try:
        scenarios = handpicked_scenarios(load_scenarios(
            args.scenario_file,
            grid_size=args.grid_size,
            starts=(HOME[rid] for rid in args.robot_ids),
            expected_clues=args.expected_clues,
        ))
    except ValueError as error:
        raise SystemExit("[ERR] {}".format(error)) from error
    selected_sha256 = scenario_manifest_sha256(scenarios)
    expected_sha256 = (
        args.expected_scenario_sha256.strip().lower()
        if args.expected_scenario_sha256 is not None
        else None
    )
    if (
        expected_sha256 is not None
        and re.fullmatch(r"[0-9a-f]{64}", expected_sha256) is None
    ):
        raise SystemExit("[ERR] expected scenario SHA-256 is invalid")
    if expected_sha256 is not None and selected_sha256 != expected_sha256:
        raise SystemExit(
            "[ERR] selected scenario SHA-256 {} does not match expected {}".format(
                selected_sha256,
                expected_sha256,
            )
        )
    lock_path = args.scenario_manifest_lock or STUDY_MANIFEST_LOCK
    try:
        enforce_scenario_manifest_lock(
            lock_path,
            scenarios,
            grid_size=args.grid_size,
        )
    except ValueError as error:
        raise SystemExit("[ERR] {}".format(error)) from error
    print(
        "[CONFIG] algorithm={} trials={} robots={} top_k={} ({} cells) "
        "drop_rate={} mode={} horizon={} revision={} manifest={}".format(
            args.algorithm,
            args.trials,
            args.robot_ids,
            args.top_k_rate,
            args.top_k_max_cells,
            args.drop_rate,
            args.trial_mode,
            args.commitment_horizon,
            args.logic_revision,
            selected_sha256[:12],
        )
    )
    try:
        Hub(args, scenarios).run()
    except KeyboardInterrupt:
        print("\n[STOP] interrupted")
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        raise SystemExit(f"[ERR] {error}") from error


if __name__ == "__main__":
    main()
