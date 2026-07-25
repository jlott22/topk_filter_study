"""Regressions for Pololu UART framing and sequenced trial control."""

from __future__ import annotations

import ast
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from hardware import metrics_hub


HARDWARE_DIR = Path(__file__).resolve().parent
POLULU_FILES = tuple(sorted(HARDWARE_DIR.glob("Pololu_*.py")))


def _extract(path, names, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    nodes = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    missing = set(names) - {node.name for node in nodes}
    if missing:
        raise AssertionError("{} lacks {}".format(path.name, sorted(missing)))
    module = ast.Module(body=nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(path), "exec"), namespace)
    return namespace


class _FakeTime:
    def __init__(self):
        self.now = 0

    def ticks_ms(self):
        return self.now

    @staticmethod
    def ticks_add(value, delta):
        return value + delta

    @staticmethod
    def ticks_diff(left, right):
        return left - right

    def sleep_ms(self, value):
        self.now += value
        time.sleep(0)


class _FakeUART:
    def __init__(self, actions=(), max_write=None, incoming=b""):
        self.actions = list(actions)
        self.max_write = max_write
        self.output = bytearray()
        self.incoming = bytearray(incoming)
        self.read_sizes = []
        self.read_called = False

    def write(self, data):
        raw = bytes(data)
        if self.actions:
            action = self.actions.pop(0)
            if action is None or action == 0:
                return action
            count = min(int(action), len(raw))
        elif self.max_write is not None:
            count = min(self.max_write, len(raw))
        else:
            count = len(raw)
        self.output.extend(raw[:count])
        time.sleep(0)
        return count

    def any(self):
        return len(self.incoming)

    def read(self, size=None):
        self.read_called = True
        self.read_sizes.append(size)
        if not self.incoming:
            return None
        count = len(self.incoming) if size is None else min(size, len(self.incoming))
        result = bytes(self.incoming[:count])
        del self.incoming[:count]
        return result


def _tx_namespace(uart=None, deadline=20):
    tx_buf = bytearray(256)
    return {
        "uart": uart or _FakeUART(),
        "time": _FakeTime(),
        "tx_buf": tx_buf,
        "tx_view": memoryview(tx_buf),
        "uart_tx_lock": threading.Lock(),
        "UART_WRITE_DEADLINE_MS": deadline,
        "TX_BUF_SIZE": len(tx_buf),
        "DELIM": ord("-"),
        "bytes_sent": 0,
        "metrics_frozen": False,
        "uart_tx_failed": False,
    }


class UARTWriteAllTests(unittest.TestCase):
    def _namespace(self, uart=None, deadline=20):
        namespace = _tx_namespace(uart, deadline)
        _extract(
            HARDWARE_DIR / "Pololu_DGA.py",
            {"_uart_write_all_locked", "_uart_send_text", "uart_send"},
            namespace,
        )
        return namespace

    def test_short_none_zero_and_full_writes_complete_one_frame(self):
        uart = _FakeUART(actions=(2, None, 0, 1))
        namespace = self._namespace(uart)

        length = namespace["_uart_send_text"]("3", "abcdef")

        self.assertEqual(bytes(uart.output), b"3.abcdef-")
        self.assertEqual(length, 9)
        self.assertEqual(namespace["bytes_sent"], 9)
        self.assertFalse(namespace["uart_tx_failed"])

    def test_timeout_is_visible_and_does_not_count_frame(self):
        uart = _FakeUART(actions=(None,) * 100)
        namespace = self._namespace(uart, deadline=3)

        with self.assertRaises(OSError):
            namespace["_uart_send_text"]("5", "1,2")

        self.assertTrue(namespace["uart_tx_failed"])
        self.assertEqual(namespace["bytes_sent"], 0)
        self.assertTrue(namespace["uart_tx_lock"].acquire(blocking=False))
        namespace["uart_tx_lock"].release()

    def test_binary_builder_rejects_embedded_frame_delimiter(self):
        namespace = self._namespace()
        namespace["tx_buf"][2:4] = b"-1"
        namespace["uart_tx_lock"].acquire()
        try:
            with self.assertRaises(ValueError):
                namespace["uart_send"]("5", 2)
        finally:
            namespace["uart_tx_lock"].release()
        self.assertEqual(namespace["bytes_sent"], 0)

    def test_concurrent_frames_are_atomic_under_short_writes(self):
        uart = _FakeUART(max_write=3)
        namespace = self._namespace(uart)
        barrier = threading.Barrier(3)
        errors = []

        def send(payload):
            try:
                barrier.wait()
                namespace["_uart_send_text"]("3", payload)
            except Exception as error:  # pragma: no cover - assertion reports
                errors.append(error)

        threads = [
            threading.Thread(target=send, args=("A" * 40,)),
            threading.Thread(target=send, args=("B" * 40,)),
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()

        self.assertEqual(errors, [])
        frames = [frame for frame in bytes(uart.output).split(b"-") if frame]
        self.assertCountEqual(frames, [b"3." + b"A" * 40, b"3." + b"B" * 40])
        self.assertEqual(namespace["bytes_sent"], 86)


class UARTReceiveTests(unittest.TestCase):
    def _parser_namespace(self):
        received = []
        namespace = {
            "DELIM": ord("-"),
            "MSG_BUF_SIZE": 256,
            "msg_buf": bytearray(256),
            "msg_len": 0,
            "rx_discarding_oversize": False,
            "bytes_received": 0,
            "start_signal": True,
            "control_state": "RUNNING",
            "metrics_frozen": False,
            "handle_msg": received.append,
        }
        _extract(
            HARDWARE_DIR / "Pololu_DGA.py",
            {
                "_msg_buf_ascii",
                "_trial_traffic_enabled",
                "_rx_feed_bytes",
            },
            namespace,
        )
        return namespace, received

    def test_36_frame_burst_survives_arbitrary_chunks(self):
        namespace, received = self._parser_namespace()
        expected = [
            "013.solution{},0,0,00,0,1,1,3,0,1".format(index)
            for index in range(36)
        ]
        burst = "".join(frame + "-" for frame in expected).encode("ascii")
        widths = (1, 7, 3, 29, 2, 64, 5, 11)
        offset = 0
        index = 0
        while offset < len(burst):
            width = widths[index % len(widths)]
            namespace["_rx_feed_bytes"](burst[offset : offset + width])
            offset += width
            index += 1

        self.assertEqual(received, expected)
        self.assertEqual(
            namespace["bytes_received"],
            sum(len(frame) + 1 for frame in expected),
        )

    def test_exact_maximum_is_valid_and_oversize_resynchronizes(self):
        namespace, received = self._parser_namespace()
        valid_max = b"V" * 256 + b"-"
        namespace["_rx_feed_bytes"](valid_max)
        self.assertEqual(received, ["V" * 256])

        namespace["_rx_feed_bytes"](b"X" * 257 + b"-013.ok-")
        self.assertEqual(received[-1], "013.ok")
        self.assertNotIn("X" * 256, received[1:])

    def test_uart_service_never_performs_an_empty_blocking_read(self):
        uart = _FakeUART()
        namespace = {
            "uart": uart,
            "bytes_received": 0,
            "metrics_frozen": False,
            "_rx_feed_bytes": lambda _data: None,
        }
        _extract(
            HARDWARE_DIR / "Pololu_DGA.py",
            {"uart_service"},
            namespace,
        )
        namespace["uart_service"]()
        self.assertFalse(uart.read_called)

    def test_uart_service_drains_in_bounded_chunks(self):
        incoming = b"Z" * 4096
        uart = _FakeUART(incoming=incoming)
        chunks = []
        namespace = {
            "uart": uart,
            "bytes_received": 0,
            "metrics_frozen": False,
            "_rx_feed_bytes": lambda data: chunks.append(bytes(data)),
        }
        _extract(
            HARDWARE_DIR / "Pololu_DGA.py",
            {"uart_service"},
            namespace,
        )
        namespace["uart_service"]()
        self.assertEqual(sum(map(len, chunks)), 4096)
        self.assertTrue(all(size <= 256 for size in uart.read_sizes))
        self.assertEqual(namespace["bytes_received"], 0)

    def test_control_frames_and_prestart_frames_do_not_count_received_bytes(self):
        namespace, received = self._parser_namespace()
        namespace["start_signal"] = False
        namespace["control_state"] = "BOOT"
        namespace["_rx_feed_bytes"](b"011.1,1-")
        namespace["start_signal"] = True
        namespace["control_state"] = "RUNNING"
        namespace["_rx_feed_bytes"](
            b"997.CMD,START,7-016.CMDACK,7,01,STARTED-011.2,2-"
        )
        self.assertEqual(namespace["bytes_received"], len("011.2,2") + 1)
        self.assertEqual(len(received), 4)


class PololuControlStateTests(unittest.TestCase):
    class _CountingDict(dict):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.clear_calls = 0

        def clear(self):
            self.clear_calls += 1
            super().clear()

    def test_control_transitions_are_sequenced_idempotent_and_clear_once(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                peer_pos = self._CountingDict({"01": (1, 1)})
                peer_yield = self._CountingDict({"01": (1, 1)})
                peer_intent = self._CountingDict({"01": (2, 1)})
                acknowledgments = []
                freezes = []
                resets = []
                fake_time = _FakeTime()
                fake_time.now = 1234
                namespace = {
                    "ROBOT_ID": "03",
                    "applied_config_sequence": 7,
                    "control_state": "CONFIGURED",
                    "pre_start_signal": False,
                    "start_signal": False,
                    "abort_signal": False,
                    "trial_active": False,
                    "METRIC_START_TIME_MS": None,
                    "found_target": False,
                    "move_forward_flag": True,
                    "peer_pos": peer_pos,
                    "peer_pos_yield": peer_yield,
                    "peer_intent": peer_intent,
                    "published_intent": ((0, 0), (1, 0)),
                    "communicated_intent": ((0, 0), (1, 0)),
                    "_send_command_ack": (
                        lambda sequence, state: acknowledgments.append(
                            (sequence, state)
                        )
                    ),
                    "time": fake_time,
                    "reset_trial_metrics": lambda: resets.append(True),
                    "freeze_trial_metrics": lambda: freezes.append(True),
                }
                _extract(
                    path,
                    {
                        "_clear_start_transport_caches",
                        "_handle_control_command",
                    },
                    namespace,
                )

                self.assertFalse(
                    namespace["_handle_control_command"]("CMD,PRESTART,6")
                )
                self.assertFalse(
                    namespace["_handle_control_command"]("CMD,START,7")
                )
                self.assertEqual(acknowledgments, [])

                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,PRESTART,7")
                )
                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,PRESTART,7")
                )
                self.assertEqual(namespace["control_state"], "READY")
                self.assertTrue(namespace["pre_start_signal"])

                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,START,7")
                )
                self.assertEqual(namespace["control_state"], "STARTED")
                self.assertFalse(namespace["start_signal"])
                self.assertFalse(namespace["trial_active"])
                self.assertEqual(len(resets), 1)
                self.assertEqual(peer_pos.clear_calls, 1)
                self.assertEqual(peer_yield.clear_calls, 1)
                self.assertEqual(peer_intent.clear_calls, 1)

                peer_pos["02"] = (4, 4)
                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,START,7")
                )
                self.assertEqual(peer_pos, {"02": (4, 4)})
                self.assertEqual(peer_pos.clear_calls, 1)
                self.assertEqual(len(resets), 1)
                self.assertFalse(
                    namespace["_handle_control_command"]("CMD,PRESTART,7")
                )

                fake_time.now = 5678
                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,RUN,7")
                )
                self.assertEqual(namespace["control_state"], "RUNNING")
                self.assertTrue(namespace["start_signal"])
                self.assertTrue(namespace["trial_active"])
                self.assertEqual(namespace["METRIC_START_TIME_MS"], 5678)
                self.assertEqual(peer_pos, {"02": (4, 4)})
                self.assertEqual(peer_pos.clear_calls, 1)
                self.assertEqual(len(resets), 1)

                fake_time.now = 9999
                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,RUN,7")
                )
                self.assertEqual(namespace["METRIC_START_TIME_MS"], 5678)
                self.assertEqual(peer_pos, {"02": (4, 4)})
                self.assertEqual(peer_pos.clear_calls, 1)
                self.assertEqual(len(resets), 1)

                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,ABORT,7")
                )
                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,ABORT,7")
                )
                self.assertEqual(namespace["control_state"], "ABORTED")
                self.assertFalse(namespace["start_signal"])
                self.assertFalse(namespace["pre_start_signal"])
                self.assertTrue(namespace["abort_signal"])
                self.assertTrue(namespace["found_target"])
                self.assertEqual(len(freezes), 1)
                self.assertEqual(
                    acknowledgments,
                    [
                        (7, "READY"),
                        (7, "READY"),
                        (7, "STARTED"),
                        (7, "STARTED"),
                        (7, "RUNNING"),
                        (7, "RUNNING"),
                        (7, "ABORTED"),
                        (7, "ABORTED"),
                    ],
                )

    def test_armed_target_prevents_late_run_release(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                acknowledgments = []
                namespace = {
                    "ROBOT_ID": "03",
                    "applied_config_sequence": 7,
                    "control_state": "READY",
                    "pre_start_signal": True,
                    "start_signal": False,
                    "abort_signal": False,
                    "trial_active": False,
                    "METRIC_START_TIME_MS": None,
                    "found_target": False,
                    "move_forward_flag": True,
                    "peer_pos": {},
                    "peer_pos_yield": {},
                    "peer_intent": {},
                    "published_intent": None,
                    "communicated_intent": None,
                    "time": _FakeTime(),
                    "reset_trial_metrics": lambda: None,
                    "freeze_trial_metrics": lambda: None,
                    "_send_command_ack": (
                        lambda sequence, state: acknowledgments.append(
                            (sequence, state)
                        )
                    ),
                }
                _extract(
                    path,
                    {
                        "_clear_start_transport_caches",
                        "_handle_control_command",
                    },
                    namespace,
                )
                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,START,7")
                )
                namespace["found_target"] = True
                namespace["move_forward_flag"] = False

                self.assertTrue(
                    namespace["_handle_control_command"]("CMD,RUN,7")
                )
                self.assertEqual(namespace["control_state"], "ABORTED")
                self.assertFalse(namespace["start_signal"])
                self.assertFalse(namespace["trial_active"])
                self.assertTrue(namespace["abort_signal"])
                self.assertEqual(
                    acknowledgments,
                    [(7, "STARTED"), (7, "ABORTED")],
                )

    def test_command_ack_wire_shape_and_topic(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                sent = []
                namespace = {
                    "ROBOT_ID": "03",
                    "_uart_send_text": (
                        lambda topic, payload, count_bytes=True: sent.append(
                            (topic, payload, count_bytes)
                        )
                    ),
                }
                _extract(path, {"_send_command_ack"}, namespace)
                namespace["_send_command_ack"](42, "RUNNING")
                self.assertEqual(
                    sent, [("6", "CMDACK,42,03,RUNNING", False)]
                )


class HubControlBarrierTests(unittest.TestCase):
    @staticmethod
    def _hub(ids=("00",)):
        hub = metrics_hub.Hub.__new__(metrics_hub.Hub)
        hub.ids = list(ids)
        hub.condition = threading.Condition()
        hub.positions = {}
        hub.last_message = time.monotonic()
        hub.trial = None
        hub.connected_robots = set()
        hub.config_sequence = 7
        hub.config_acks = {}
        hub.control_acks = {}
        hub.control_expected_state = ""
        hub.control_fault = ""
        hub.config_ack_rows = []
        hub.control_ack_rows = []
        hub.printed_config_acks = set()
        hub.args = SimpleNamespace(
            config_timeout=0.2,
            config_retry_seconds=0.01,
            control_timeout=0.2,
            control_retry_seconds=0.01,
        )
        return hub

    @staticmethod
    def _message(topic, payload):
        return SimpleNamespace(topic=topic, payload=payload.encode("ascii"))

    def test_control_ack_parser_and_identity_sequence_rejection(self):
        self.assertEqual(
            metrics_hub.parse_control_ack("CMDACK,7,03,RUNNING"),
            {"sequence": 7, "robot_id": "03", "state": "RUNNING"},
        )
        self.assertIsNone(metrics_hub.parse_control_ack("CMDACK,0,03,STARTED"))
        self.assertIsNone(metrics_hub.parse_control_ack("CMDACK,7,3,STARTED"))

        hub = self._hub()
        hub.control_expected_state = "STARTED"
        hub.on_message(None, None, self._message("006", "CMDACK,7,01,STARTED"))
        self.assertEqual(hub.control_acks, {})
        self.assertIn("topic robot", hub.control_fault)
        hub.control_fault = ""
        hub.on_message(None, None, self._message("006", "CMDACK,6,00,STARTED"))
        self.assertEqual(hub.control_acks, {})
        hub.on_message(None, None, self._message("006", "CMDACK,7,00,READY"))
        self.assertEqual(hub.control_acks, {})
        hub.on_message(None, None, self._message("006", "CMDACK,7,00,STARTED"))
        self.assertEqual(set(hub.control_acks), {"00"})

    def test_malformed_topic_six_is_audited_without_metric_contamination(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState()}
        )
        trial.config_sequence = 7
        trial.t0 = time.monotonic()
        trial.active = True
        trial.control_phase = "active"
        hub.trial = trial

        hub.on_message(None, None, self._message("006", "damaged-ack"))

        self.assertEqual(trial.messages["6"], 0)
        self.assertEqual(trial.robots["00"].messages["6"], 0)
        self.assertEqual(trial.events, [])
        self.assertEqual(hub.control_ack_rows[-1][6], "INVALID")

    def test_transition_retries_until_every_robot_acknowledges(self):
        hub = self._hub(("00", "01"))
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {rid: metrics_hub.RobotState() for rid in hub.ids}
        )
        trial.config_sequence = 7
        publishes = []

        def publish(_topic, payload, kind, _trial):
            publishes.append((payload, kind))
            if len(publishes) == 2:
                with hub.condition:
                    for rid in hub.ids:
                        hub.control_acks[rid] = {
                            "sequence": 7,
                            "robot_id": rid,
                            "state": "READY",
                        }
                    hub.condition.notify_all()

        hub.publish = publish
        hub.transition_robots(trial, "PRESTART", "READY")
        self.assertGreaterEqual(len(publishes), 2)
        self.assertTrue(all(item[0] == "CMD,PRESTART,7" for item in publishes))

    def test_run_timestamp_is_set_at_initial_publish_not_start_or_retry(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState()}
        )
        trial.config_sequence = 7
        trial.pending_start_events = [("stale",)]
        observed = []

        def publish(_topic, payload, _kind, _trial):
            observed.append((payload, trial.t0, list(trial.pending_start_events)))
            if ",START," in payload:
                state = "STARTED"
            elif sum(item[0] == "CMD,RUN,7" for item in observed) >= 2:
                state = "RUNNING"
            else:
                return
            with hub.condition:
                hub.control_acks["00"] = {
                    "sequence": 7,
                    "robot_id": "00",
                    "state": state,
                }
                hub.condition.notify_all()

        hub.publish = publish
        hub.transition_robots(trial, "START", "STARTED")
        self.assertEqual(observed[0][0], "CMD,START,7")
        self.assertEqual(observed[0][1], 0.0)
        self.assertEqual(observed[0][2], [("stale",)])

        hub.transition_robots(trial, "RUN", "RUNNING")
        self.assertEqual(observed[1][0], "CMD,RUN,7")
        self.assertGreater(observed[1][1], 0)
        self.assertEqual(observed[1][2], [])
        self.assertEqual(observed[2][0], "CMD,RUN,7")
        self.assertEqual(observed[2][1], observed[1][1])
        self.assertEqual(observed[2][2], [])
        timestamp = trial.t0
        self.assertEqual(timestamp, observed[1][1])

    def test_run_boundary_blocks_callbacks_until_publish_returns(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState()}
        )
        trial.config_sequence = 7
        publish_entered = threading.Event()
        release_publish = threading.Event()
        callback_entered = threading.Event()
        errors = []

        def publish(*_args):
            publish_entered.set()
            if not release_publish.wait(1):
                raise RuntimeError("test publish release timeout")
            hub.control_acks["00"] = {
                "sequence": 7,
                "robot_id": "00",
                "state": "RUNNING",
            }
            hub.condition.notify_all()

        def transition():
            try:
                hub.transition_robots(trial, "RUN", "RUNNING")
            except Exception as error:  # pragma: no cover - assertion reports
                errors.append(error)

        def callback():
            with hub.condition:
                callback_entered.set()

        hub.publish = publish
        transition_thread = threading.Thread(target=transition)
        transition_thread.start()
        self.assertTrue(publish_entered.wait(1))
        callback_thread = threading.Thread(target=callback)
        callback_thread.start()
        self.assertFalse(callback_entered.wait(0.03))
        release_publish.set()
        transition_thread.join(1)
        callback_thread.join(1)
        self.assertEqual(errors, [])
        self.assertTrue(callback_entered.is_set())

    def test_command_publish_failure_becomes_configuration_error(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState()}
        )
        trial.config_sequence = 7

        def publish(*_args):
            raise RuntimeError("mqtt rc=4")

        hub.publish = publish
        with self.assertRaises(metrics_hub.ConfigurationError):
            hub.transition_robots(trial, "PRESTART", "READY")
        self.assertEqual(hub.control_expected_state, "")

    def test_run_window_position_replays_after_quorum(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState(last_pos=(0, 0))}
        )
        trial.config_sequence = 7
        trial.t0 = time.monotonic()
        trial.control_phase = "starting"
        hub.trial = trial

        hub.on_message(None, None, self._message("001", "1,0"))
        self.assertEqual(trial.robots["00"].steps, 0)
        self.assertEqual(len(trial.pending_start_events), 1)
        hub.control_acks = {
            "00": {"sequence": 7, "robot_id": "00", "state": "RUNNING"}
        }
        hub.activate_after_run_quorum(trial)
        self.assertTrue(trial.active)
        self.assertEqual(trial.robots["00"].steps, 1)
        self.assertEqual(trial.messages["1"], 1)

    def test_run_window_target_quarantines_trial(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState(last_pos=(0, 0))}
        )
        trial.config_sequence = 7
        trial.t0 = time.monotonic()
        trial.control_phase = "starting"
        hub.trial = trial

        hub.on_message(None, None, self._message("005", "5,5"))
        hub.control_acks = {
            "00": {"sequence": 7, "robot_id": "00", "state": "RUNNING"}
        }
        with self.assertRaises(metrics_hub.ConfigurationError):
            hub.activate_after_run_quorum(trial)
        self.assertFalse(trial.active)
        self.assertIsNone(trial.end_time)
        self.assertEqual(trial.messages["5"], 0)

    def test_target_while_armed_blocks_initial_run_publish(self):
        hub = self._hub()
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState(last_pos=(0, 0))}
        )
        trial.config_sequence = 7
        trial.control_phase = "arming"
        hub.trial = trial
        publishes = []
        hub.publish = lambda *args: publishes.append(args)

        hub.on_message(None, None, self._message("005", "5,5"))
        with self.assertRaisesRegex(
            metrics_hub.ConfigurationError, "before RUNNING quorum"
        ):
            hub.transition_robots(trial, "RUN", "RUNNING")

        self.assertEqual(publishes, [])
        self.assertEqual(trial.t0, 0.0)
        self.assertFalse(trial.active)

    def test_missing_running_ack_invalidates_release(self):
        hub = self._hub()
        hub.args.control_timeout = 0.03
        hub.args.control_retry_seconds = 0.01
        scenario = metrics_hub.Scenario("1", (5, 5), ())
        trial = metrics_hub.Trial(
            "run", scenario, {"00": metrics_hub.RobotState()}
        )
        trial.config_sequence = 7
        trial.control_phase = "arming"
        publishes = []
        hub.publish = (
            lambda _topic, payload, kind, _trial:
            publishes.append((payload, kind))
        )

        with self.assertRaisesRegex(
            metrics_hub.ConfigurationError,
            "RUNNING acknowledgment timeout",
        ):
            hub.transition_robots(trial, "RUN", "RUNNING")

        self.assertTrue(publishes)
        self.assertTrue(
            all(payload == "CMD,RUN,7" for payload, _kind in publishes)
        )
        self.assertGreater(trial.t0, 0.0)
        self.assertFalse(trial.active)

    def test_run_loop_orders_full_started_quorum_before_run(self):
        source = Path(metrics_hub.__file__).read_text(encoding="utf-8")
        tree = ast.parse(source, filename=metrics_hub.__file__)
        hub_class = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "Hub"
        )
        run_method = next(
            node
            for node in hub_class.body
            if isinstance(node, ast.FunctionDef) and node.name == "run"
        )
        transitions = sorted(
            (
                node.lineno,
                node.args[1].value,
                node.args[2].value,
            )
            for node in ast.walk(run_method)
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "transition_robots"
                and len(node.args) >= 3
                and isinstance(node.args[1], ast.Constant)
                and isinstance(node.args[2], ast.Constant)
            )
        )
        non_abort = [
            (command, state)
            for _line, command, state in transitions
            if command != "ABORT"
        ]
        self.assertEqual(
            non_abort,
            [
                ("PRESTART", "READY"),
                ("START", "STARTED"),
                ("RUN", "RUNNING"),
            ],
        )


class TransportStructureTests(unittest.TestCase):
    def test_every_program_has_explicit_buffers_single_write_site_and_locked_builders(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                source = path.read_text(encoding="utf-8")
                tree = ast.parse(source, filename=str(path))
                uart_call = next(
                    node.value
                    for node in tree.body
                    if isinstance(node, ast.Assign)
                    and any(
                        isinstance(target, ast.Name) and target.id == "uart"
                        for target in node.targets
                    )
                    and isinstance(node.value, ast.Call)
                )
                keywords = {
                    keyword.arg: ast.literal_eval(keyword.value)
                    for keyword in uart_call.keywords
                    if keyword.arg in {
                        "rxbuf", "txbuf", "timeout", "timeout_char"
                    }
                }
                self.assertEqual(
                    keywords,
                    {
                        "rxbuf": 4096,
                        "txbuf": 1024,
                        "timeout": 1000,
                        "timeout_char": 10,
                    },
                )
                write_sites = [
                    node
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == "uart"
                    and node.func.attr == "write"
                ]
                self.assertEqual(len(write_sites), 1)
                parent = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "_uart_write_all_locked"
                )
                self.assertIn(write_sites[0], list(ast.walk(parent)))
                self.assertNotIn("RB_SIZE", source)

                for name in (
                    "publish_position",
                    "publish_clue",
                    "publish_target",
                    "publish_intent",
                ):
                    function = next(
                        node
                        for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == name
                    )
                    segment = ast.get_source_segment(source, function)
                    self.assertIn("uart_tx_lock.acquire()", segment)
                    self.assertIn("finally:", segment)
                    self.assertIn("uart_tx_lock.release()", segment)
                    first_mutation = min(
                        position
                        for position in (
                            segment.find("_write_int(tx_buf"),
                            segment.find("tx_buf["),
                        )
                        if position >= 0
                    )
                    self.assertLess(
                        segment.find("uart_tx_lock.acquire()"), first_mutation
                    )

    def test_target_stop_is_safe_even_if_publish_raises(self):
        for path in POLULU_FILES:
            with self.subTest(file=path.name):
                source = path.read_text(encoding="utf-8")
                tree = ast.parse(source, filename=str(path))
                function = next(
                    node
                    for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "stop_and_alert_target"
                )
                segment = ast.get_source_segment(source, function)
                self.assertLess(
                    segment.find("move_forward_flag = False"),
                    segment.find("publish_target(next_x, next_y)"),
                )
                self.assertIn("finally:", segment)
                self.assertIn("freeze_trial_metrics(detected_at_ms)", segment)
                self.assertIn("motors_off()", segment)
