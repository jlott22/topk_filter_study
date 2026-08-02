from __future__ import annotations

import random
import unittest

from benchmark_sim.comms.bus import MessageBus
from benchmark_sim.comms.message import Message
from benchmark_sim.comms.models import GilbertElliotModel, make_comm_model


class _Receiver:
    def __init__(self, rid: str) -> None:
        self.rid = rid
        self.pos = (0, 0)
        self.messages: list[Message] = []

    def receive_message(self, message: Message) -> None:
        self.messages.append(message)


class GilbertElliottModelTests(unittest.TestCase):
    def test_factory_preserves_requested_stationary_delivery_rate_with_bursts(self) -> None:
        model = make_comm_model("gilbert_elliot", 0.75)

        self.assertIsInstance(model, GilbertElliotModel)
        self.assertAlmostEqual(model.p_good_to_good, 0.95)
        self.assertAlmostEqual(model.p_bad_to_bad, 0.85)
        self.assertAlmostEqual(model.initial_good_prob, 0.75)
        self.assertAlmostEqual(model.stationary_delivery_prob, 0.75)
        self.assertAlmostEqual(model.state_correlation, 0.8)

    def test_seeded_sequence_has_expected_rate_and_positive_lag_correlation(self) -> None:
        model = make_comm_model("gilbert_elliot", 0.75)
        rng = random.Random(314159)
        message = Message("00", "robot/00/state", {}, 0.0)
        outcomes = [
            int(model.should_deliver(message, (0, 0), (1, 0), rng, ("00", "01")))
            for _ in range(200_000)
        ]

        mean = sum(outcomes) / len(outcomes)
        centered_products = sum(
            (left - mean) * (right - mean)
            for left, right in zip(outcomes, outcomes[1:])
        )
        variance_sum = sum((value - mean) ** 2 for value in outcomes[:-1])
        lag_one_correlation = centered_products / variance_sum

        self.assertAlmostEqual(mean, 0.75, delta=0.01)
        self.assertAlmostEqual(lag_one_correlation, 0.8, delta=0.02)

    def test_directed_links_keep_separate_states(self) -> None:
        model = make_comm_model("gilbert_elliot", 0.75)
        rng = random.Random(7)
        message = Message("00", "robot/00/state", {}, 0.0)

        model.should_deliver(message, (0, 0), (1, 0), rng, ("00", "01"))
        model.should_deliver(message, (0, 0), (2, 0), rng, ("00", "02"))

        self.assertEqual(set(model.states), {("00", "01"), ("00", "02")})

    def test_protected_messages_bypass_bad_link_without_advancing_it(self) -> None:
        model = make_comm_model("gilbert_elliot", 0.75)
        model.states[("00", "01")] = False
        bus = MessageBus(model=model, delay_s=0.0, delay_jitter_s=0.0, rng=random.Random(1))
        sender = _Receiver("00")
        receiver = _Receiver("01")
        bus.register(sender)
        bus.register(receiver)

        bus.publish("00", "robot/00/target", {"loc": [1, 1]}, 0.0)
        bus.pump(0.0)

        self.assertEqual(len(receiver.messages), 1)
        self.assertFalse(model.states[("00", "01")])
        self.assertEqual(bus.counters.protected_delivered_total, 1)
        self.assertEqual(bus.counters.dropped_total, 0)

    def test_invalid_factory_parameters_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            make_comm_model("gilbert_elliot", -0.1)
        with self.assertRaises(ValueError):
            make_comm_model("gilbert_elliot", 1.1)
        with self.assertRaises(ValueError):
            make_comm_model("gilbert_elliot", 0.75, state_correlation=1.0)


if __name__ == "__main__":
    unittest.main()
