"""Numeric observation channel with explicit consumer-side fault injection."""

from collections import deque


class SensorChannel:
    def __init__(self, stale_s=2.0, latency_s=0.0):
        self.stale_s = stale_s
        self.latency_s = latency_s
        self.pending = deque()
        self.mode = "normal"
        self.last_stamp = None
        self.last_fresh_wall = None
        self.last = None
        self.latencies = []

    def receive(self, stamp, value, now):
        if self.mode == "drop":
            return
        if self.mode == "freeze" and self.last is not None:
            stamp, value = self.last
        self.pending.append((now + self.latency_s, now, stamp, value))

    def deliver(self, now):
        while self.pending and self.pending[0][0] <= now:
            _, arrival, stamp, value = self.pending.popleft()
            if self.last_stamp is None or stamp > self.last_stamp:
                self.last_stamp = stamp
                self.last_fresh_wall = now
                self.last = (stamp, value)
                self.latencies.append(now - arrival)

    def healthy(self, now):
        return (
            self.last_fresh_wall is not None
            and now - self.last_fresh_wall < self.stale_s
        )
