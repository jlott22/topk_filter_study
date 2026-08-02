from __future__ import annotations

import heapq
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Protocol, Tuple

from benchmark_sim.core.types import Cell
from benchmark_sim.metrics.counters import MessageCounters
from .message import Message
from .models import CommunicationModel

CORE_MESSAGE_TOPICS = {"state", "clue", "target", "collision_intent"}


class Receiver(Protocol):
    rid: str
    pos: Cell

    def receive_message(self, message: Message) -> None: ...


@dataclass(order=True)
class PendingDelivery:
    deliver_at_s: float
    order: int
    receiver: str = field(compare=False)
    message: Message = field(compare=False)


class MessageBus:
    def __init__(
        self,
        model: CommunicationModel,
        delay_s: float = 0.04,
        delay_jitter_s: float = 0.0,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.model = model
        self.delay_s = max(0.0, delay_s)
        self.delay_jitter_s = max(0.0, delay_jitter_s)
        self.rng = rng or random.Random()
        self.receivers: Dict[str, Receiver] = {}
        self.pending: List[PendingDelivery] = []
        self.counters = MessageCounters()
        self._order = 0

    def register(self, receiver: Receiver) -> None:
        self.receivers[receiver.rid] = receiver

    def publish(self, sender: str, topic: str, payload: dict, now_s: float, post_clue: bool = False) -> None:
        if sender not in self.receivers:
            raise KeyError(f"Sender {sender} is not registered")
        msg = Message(sender=sender, topic=topic, payload=dict(payload), created_at_s=now_s)
        self.counters.sent(
            sender,
            topic=msg.category,
            protected=msg.protected,
            core=msg.category in CORE_MESSAGE_TOPICS,
            post_clue=post_clue,
        )
        sender_pos = self.receivers[sender].pos
        for rid, receiver in self.receivers.items():
            if rid == sender:
                continue
            link_key = (sender, rid)
            deliver = True if msg.protected else self.model.should_deliver(msg, sender_pos, receiver.pos, self.rng, link_key)
            if not deliver:
                self.counters.dropped(rid)
                continue
            delay = self.delay_s
            if self.delay_jitter_s > 0:
                delay += self.rng.uniform(-self.delay_jitter_s, self.delay_jitter_s)
                delay = max(0.0, delay)
            # Protected safety/target topics still respect delay, but are never dropped.
            deliver_at = now_s + delay
            self._order += 1
            heapq.heappush(self.pending, PendingDelivery(deliver_at, self._order, rid, msg))

    def pump(self, now_s: float) -> None:
        while self.pending and self.pending[0].deliver_at_s <= now_s + 1e-12:
            item = heapq.heappop(self.pending)
            receiver = self.receivers.get(item.receiver)
            if receiver is None:
                continue
            delivered_msg = Message(
                sender=item.message.sender,
                topic=item.message.topic,
                payload=item.message.payload,
                created_at_s=item.message.created_at_s,
                delivered_at_s=item.deliver_at_s,
            )
            receiver.receive_message(delivered_msg)
            self.counters.delivered(item.receiver, protected=delivered_msg.protected)
