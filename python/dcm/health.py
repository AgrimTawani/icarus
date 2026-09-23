"""Deterministic DCM health state and senior fallback directives.

The pattern is inspired by AerialClaw's repeated-failure detection in
``brain/agent_loop.py`` (MIT, XDEI Group), but this implementation removes
model self-recovery and mutable skills. Only measured runtime outcomes change
state, and every fallback still travels through Icarus's typed Drone API.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_POLICY = ROOT / "config/dcm/runtime.yaml"
HEALTH_STATES = frozenset({
    "healthy", "late", "invalid", "unavailable", "stale-input", "circuit-open",
})


def load_policy(path=DEFAULT_POLICY):
    policy = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if policy.get("schema") != "icarus.dcm.runtime-policy.v1":
        raise ValueError("unsupported DCM runtime policy")
    threshold = policy.get("circuit_breaker", {}).get("consecutive_failures")
    if type(threshold) is not int or threshold < 1:
        raise ValueError("invalid DCM circuit-breaker threshold")
    recovery = policy.get("circuit_breaker", {}).get(
        "recovery_probe_after_decisions")
    if type(recovery) is not int or recovery < 1:
        raise ValueError("invalid DCM recovery-probe interval")
    return policy


@dataclass
class HealthTransition:
    previous: str
    current: str
    consecutive_failures: int
    circuit_opened: bool
    fallback_action: str | None


class DcmHealthMachine:
    def __init__(self, policy=None):
        self.policy = policy or load_policy()
        self.state = "healthy"
        self.consecutive_failures = 0
        self.suppressed_decisions = 0

    def allow_inference(self):
        """Hold an open circuit shut, then permit one controlled probe."""
        if self.state != "circuit-open":
            return True
        recovery_after = self.policy["circuit_breaker"][
            "recovery_probe_after_decisions"]
        if self.suppressed_decisions >= recovery_after:
            self.suppressed_decisions = 0
            return True
        self.suppressed_decisions += 1
        return False

    def circuit_fallback(self, airborne=False):
        if not airborne:
            return self.policy["fallback"].get("landed_action")
        return (self.policy["fallback"]["opening_action_airborne"]
                if self.suppressed_decisions == 1 else
                self.policy["fallback"]["circuit_open_action_airborne"])

    def transition(self, outcome, airborne=False):
        previous = self.state
        opened = False
        fallback = None
        if outcome in ("valid", "executed", "none"):
            self.state = "healthy"
            self.consecutive_failures = 0
            self.suppressed_decisions = 0
        else:
            mapped = self.policy["failure_states"].get(outcome, "unavailable")
            if mapped not in HEALTH_STATES:
                raise ValueError("runtime policy names an unknown health state")
            self.state = mapped
            self.consecutive_failures += 1
            threshold = self.policy["circuit_breaker"]["consecutive_failures"]
            if self.consecutive_failures >= threshold:
                self.state = "circuit-open"
                opened = previous != "circuit-open"
                if airborne:
                    key = ("opening_action_airborne" if opened
                           else "circuit_open_action_airborne")
                    fallback = self.policy["fallback"][key]
        return HealthTransition(previous, self.state,
                                self.consecutive_failures, opened, fallback)

    def record(self):
        return {"state": self.state,
                "consecutive_failures": self.consecutive_failures,
                "policy_schema": self.policy["schema"]}
