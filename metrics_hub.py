#!/usr/bin/env python3
"""Multi-trial MQTT controller and metrics collector for Pololu robots.

Compatible robot commands on hub/command:
  "1" = pre-start, "2" = trial start.
A JSON scenario is also published on hub/trial_task for operator/host tooling.
Robot allocation payloads on topics 3 and 6 are recorded verbatim.
"""

import argparse
import csv
import json
import os
import re
import threading
import time
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
HOME = {"00": (0, 0), "01": (0, 5), "02": (0, 10), "03": (0, 15)}
SCENARIO_FILE = "./trials_d1_c4_g19.csv"
OUT_DIR = "./hub_logs"
VALID_DIGITS = set("123456")
TOPIC_CATEGORY = {
    "1": "state", "2": "collision_intent", "3": "allocation_primary",
    "4": "clue", "5": "target", "6": "allocation_secondary",
}
PROTECTED = {"2", "5"}
CORE = {"1", "2", "4", "5"}
ALLOCATION = {"3", "6"}


class Scenario:
    def __init__(self, trial_id, target, clues):
        self.trial_id = trial_id
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
        self.events = []
        self.t0 = 0.0
        self.first_clue = None
        self.end_time = None
        self.reporter = ""
        self.reported_target = None
        self.active = False
        self.onboard_baseline = {}


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


def load_scenarios(path: str) -> List[Scenario]:
    if not path or not os.path.exists(path):
        return []
    result = []
    with open(path, newline="", encoding="utf-8-sig") as stream:
        lines = (line for line in stream if not line.lstrip().startswith("#"))
        for index, row in enumerate(csv.DictReader(lines)):
            tx, ty = _integer(row, "target_x", "object_x"), _integer(row, "target_y", "object_y")
            if tx is None or ty is None:
                continue
            clues, number = [], 1
            while f"clue{number}_x" in row or f"clue{number}_y" in row:
                x, y = _integer(row, f"clue{number}_x"), _integer(row, f"clue{number}_y")
                if x is not None and y is not None:
                    clues.append((x, y))
                number += 1
            result.append(Scenario(str(row.get("trial_id", row.get("episode", index))), (tx, ty), clues))
    return result


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


class Hub:
    def __init__(self, args: argparse.Namespace, scenarios: List[Scenario]):
        if mqtt is None:
            raise RuntimeError(
                "paho-mqtt is required; install it with: python3 -m pip install paho-mqtt"
            )
        self.args, self.scenarios, self.ids = args, scenarios, args.robot_ids
        self.condition = threading.Condition()
        self.connected = threading.Event()
        self.positions: Dict[str, Tuple[int, int]] = {}
        self.last_message = time.monotonic()
        self.trial: Optional[Trial] = None
        self.commands: List[List[object]] = []
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
            if digit == "1" and coordinate is not None:
                self.positions[rid] = coordinate
            trial = self.trial
            if trial:
                relative = now - trial.t0 if trial.t0 else ""
                phase = "trial" if trial.active else ("return" if trial.end_time is not None else "ready")
                trial.events.append([
                    trial.run_id, trial.scenario.trial_id, wall, relative, phase,
                    rid, digit, TOPIC_CATEGORY[digit], payload,
                    coordinate[0] if coordinate else "", coordinate[1] if coordinate else "",
                ])
                if trial.active:
                    self.collect(trial, rid, digit, coordinate, relative)
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
        elif digit == "4" and coordinate is not None:
            if coordinate not in trial.clues:
                trial.clues.append(coordinate)
            if trial.first_clue is None:
                trial.first_clue = relative
            print(f"[CLUE] robot={rid} cell={coordinate}")
        elif digit == "5" and coordinate is not None and trial.end_time is None:
            trial.end_time, trial.reporter, trial.reported_target = relative, rid, coordinate
            trial.active = False
            print(f"[TARGET] t={relative:.3f}s robot={rid} cell={coordinate}")

    def publish(self, topic: str, payload: str, kind: str, trial: Trial):
        result = self.client.publish(topic, payload, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"publish failed rc={result.rc} topic={topic}")
        self.commands.append([time.time(), trial.run_id, trial.scenario.trial_id, kind, topic, payload])

    def wait_home(self):
        deadline = time.monotonic() + self.args.ready_timeout
        with self.condition:
            while True:
                ready = {rid for rid in self.ids if self.positions.get(rid) == HOME.get(rid)}
                if len(ready) == len(self.ids):
                    print(f"[READY] robots home: {sorted(ready)}")
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
        with self.condition:
            while trial.end_time is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    trial.active = False
                    raise TimeoutError(f"trial {trial.run_id} timed out")
                self.condition.wait(min(0.25, remaining))

    def run(self):
        os.makedirs(self.args.out_dir, exist_ok=True)
        self.client.connect(self.args.broker, self.args.port, keepalive=10)
        self.client.loop_start()
        if not self.connected.wait(self.args.connect_timeout):
            raise TimeoutError("MQTT connection timed out")
        try:
            for number, scenario in enumerate(self.scenarios, 1):
                self.wait_home()
                run_id = f"{int(time.time())}-{number:04d}"
                robots = {rid: RobotState(last_pos=self.positions.get(rid)) for rid in self.ids}
                trial = Trial(run_id, scenario, robots)
                for rid, position in self.positions.items():
                    if rid in robots:
                        trial.visits[position] = trial.visits.get(position, 0) + 1
                with self.condition:
                    self.trial = trial
                trial.onboard_baseline = self.onboard_counts()
                task = json.dumps({
                    "run_id": run_id, "trial_id": scenario.trial_id,
                    "algorithm": self.args.algorithm, "comm_level": self.args.comm_level,
                    "target": scenario.target, "clues": scenario.clues,
                }, separators=(",", ":"))
                self.publish(HUB_TASK_TOPIC, task, "trial_task", trial)
                print(f"\n[TASK {number}/{len(self.scenarios)}] id={scenario.trial_id}")
                print(f"  target={scenario.target} clues={scenario.clues}")
                if not self.args.auto:
                    input("Place the target/clues, then press Enter: ")
                self.publish(HUB_COMMAND_TOPIC, "1", "pre_start", trial)
                self.wait_quiet()
                with self.condition:
                    trial.t0, trial.active = time.monotonic(), True
                self.publish(HUB_COMMAND_TOPIC, "2", "start", trial)
                print('[CMD] hub/command "2"')
                self.wait_target(trial)
                time.sleep(self.args.drain_seconds)
                self.write_trial(trial)
                self.import_onboard(trial)
                print(f"[SAVED] run={run_id}")
                with self.condition:
                    self.trial = None
            self.write_commands()
        finally:
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
            "run_id", "trial_id", "algorithm", "comm_level", "expected_target",
            "reported_target", "target_match", "target_reporter", "duration_s",
            "first_clue_s", "observed_clues", "total_team_steps",
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
            trial.run_id, trial.scenario.trial_id, self.args.algorithm,
            self.args.comm_level, f"{trial.scenario.target[0]}/{trial.scenario.target[1]}",
            f"{reported[0]}/{reported[1]}" if reported else "",
            int(reported == trial.scenario.target), trial.reporter,
            trial.end_time, trial.first_clue if trial.first_clue is not None else "",
            ";".join(f"{x}/{y}" for x, y in trial.clues), total_steps,
            pre_steps, post_steps, unique, revisits, message_total,
            message_total * max(0, len(self.ids) - 1), categories["protected"],
            categories["unprotected"], categories["core"], categories["allocation"],
            post_total, post_allocation, category_string(trial.messages),
            gini(r.unique for r in trial.robots.values()),
        ]
        csv_append(prefix + "_sys.csv", sys_header, [sys_row])

        robot_header = [
            "run_id", "trial_id", "algorithm", "comm_level", "robot_id",
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
                trial.run_id, trial.scenario.trial_id, self.args.algorithm,
                self.args.comm_level, rid, robot.steps, robot.pre_steps,
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
            "run_id", "trial_id", "wall_time_s", "relative_time_s", "phase",
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
            row.update({"run_id": trial.run_id, "trial_id": trial.scenario.trial_id, "source_robot_id": rid})
            rows.append(row)
            fields.update(row)
        if not rows:
            return
        preferred = ["run_id", "trial_id", "source_robot_id"]
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
                   ["wall_time_s", "run_id", "trial_id", "kind", "topic", "payload"],
                   self.commands)


def parser():
    result = argparse.ArgumentParser(description="Pololu multi-trial metrics hub")
    result.add_argument("--algorithm")
    result.add_argument("--comm-level", type=float, default=0.0)
    result.add_argument("--broker", default=BROKER_HOST)
    result.add_argument("--port", type=int, default=BROKER_PORT)
    result.add_argument("--robot-ids", type=parse_ids, default=ROBOT_IDS)
    result.add_argument("--scenario-file", default=SCENARIO_FILE)
    result.add_argument("--start-index", type=int, default=0)
    result.add_argument("--trials", type=int, default=1, help="0 means all remaining")
    result.add_argument("--out-dir", default=OUT_DIR)
    result.add_argument("--auto", action="store_true")
    result.add_argument("--connect-timeout", type=float, default=15)
    result.add_argument("--ready-timeout", type=float, default=300)
    result.add_argument("--quiet-window", type=float, default=1)
    result.add_argument("--quiet-timeout", type=float, default=15)
    result.add_argument("--trial-timeout", type=float, default=3600)
    result.add_argument("--drain-seconds", type=float, default=1)
    result.add_argument("--robot-metrics-root")
    result.add_argument("--onboard-wait", type=float, default=5)
    return result


def main():
    args = parser().parse_args()
    if args.algorithm is None:
        args.algorithm = input("Algorithm (ACBBA/CBAA/DGA/DMCHBA/HIPC/PI): ")
    args.algorithm = re.sub(r"[^A-Za-z0-9_-]+", "_", args.algorithm.strip().upper())
    if not args.algorithm:
        raise SystemExit("[ERR] algorithm cannot be empty")
    if not 0 <= args.comm_level <= 1 or args.start_index < 0 or args.trials < 0:
        raise SystemExit("[ERR] invalid comm-level/start-index/trials")
    scenarios = load_scenarios(args.scenario_file)
    if not scenarios:
        scenarios = [Scenario(str(args.start_index), (-1, -1), [])]
    else:
        scenarios = scenarios[args.start_index:]
    if args.trials:
        scenarios = scenarios[:args.trials]
    if not scenarios:
        raise SystemExit("[ERR] no scenarios selected")
    print(f"[CONFIG] algorithm={args.algorithm} trials={len(scenarios)} robots={args.robot_ids}")
    try:
        Hub(args, scenarios).run()
    except KeyboardInterrupt:
        print("\n[STOP] interrupted")
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        raise SystemExit(f"[ERR] {error}") from error


if __name__ == "__main__":
    main()
