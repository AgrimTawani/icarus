"""A deterministic no-LLM controller, scored through the same harness.

Both the Phase 11 and Phase 12 exit gates require comparing a model against a
scripted controller. This is that controller: a small rule engine that reads
the same curated observation, obeys the same contract and emits the same JSON,
so the only difference between it and a model is the thing being measured.

It deliberately does **not** look at the recorded action. A runtime that
replayed the baseline would score perfectly by construction and tell you
nothing; this one decides from state, exactly as a model must.

Its purpose is to be a floor, not a champion. If a language model cannot beat
roughly thirty lines of if-statements on these missions, that is the single
most useful thing an evaluation can report, and it should be discoverable
before anything is built on the assumption that the model earns its place.
"""

import json
import re

from python.dcm.contract import ACTIONS

# Phrases that indicate the operator asked for a particular altitude. The
# baseline cannot interpret language, so it extracts a number and otherwise
# falls back to a default; that limitation is the point of comparing it.
_ALTITUDE = re.compile(r"(\d+(?:\.\d+)?)\s*(?:m\b|metre|meter)", re.IGNORECASE)
DEFAULT_ALTITUDE_M = 3.0
DEFAULT_HOLD_MS = 5_000


def _number(value):
    if type(value) in (int, float):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _requested_altitude(mission, actions=ACTIONS):
    spec = actions["takeoff"]["target_altitude_agl_m"]
    match = _ALTITUDE.search(mission or "")
    altitude = float(match.group(1)) if match else DEFAULT_ALTITUDE_M
    # Clamp rather than propose out of bounds: the baseline should fail by
    # choosing badly, never by being rejected on a technicality.
    return min(max(altitude, spec["minimum"]), spec["maximum"])


class ScriptedRuntime:
    """The deterministic baseline, shaped as a ModelRuntime.

    Implements the same interface as a model adapter, so `observe-dcm`,
    `evaluate-dcm` and `dcm-fly` drive it unchanged.
    """

    name = "scripted-baseline-v1"

    def __init__(self, hold_ms=DEFAULT_HOLD_MS):
        self.hold_ms = hold_ms

    def propose(self, observation):
        return json.dumps(self.decide(observation))

    def decide(self, observation):
        """One action from state alone. Never reads the recorded action."""
        state = observation.get("state") or {}
        perception = observation.get("perception") or {}
        mission = observation.get("mission") or ""
        completed = [entry.get("action")
                     for entry in observation.get("actions_completed") or []]

        phase = state.get("flight_phase") or ""
        armed = state.get("armed")
        landed = state.get("landed")
        altitude = _number(state.get("relative_altitude_m")) or 0.0
        airborne = phase == "FLIGHT_PHASE_AIRBORNE" or altitude > 0.5

        # Safety first, and before anything the mission asked for. A frontal
        # hazard or unhealthy perception outranks every other consideration.
        if perception.get("path_ahead_clear") is False:
            return self._action("hold", {"duration_ms": self.hold_ms})
        if perception.get("overall") == "HEALTH_LEVEL_UNAVAILABLE" and airborne:
            return self._action("hold", {"duration_ms": self.hold_ms})
        if state.get("geofence_breached") is True:
            return self._action("return_home", {})
        battery = (state.get("battery") or {}).get("remaining_percent")
        battery = _number(battery)
        if battery is not None and battery < 30.0 and airborne:
            return self._action("return_home", {})

        # Mission sequence. The order is the ordinary shape of every mission
        # in the corpus: arm, climb, do the thing, come down.
        if not armed and not airborne:
            if "arm" in completed and "takeoff" not in completed:
                # Armed once already and still on the ground: climbing is the
                # only way the mission progresses.
                return self._action(
                    "takeoff",
                    {"target_altitude_agl_m": _requested_altitude(mission)})
            return self._action("arm", {})

        if armed and not airborne and landed is not False:
            return self._action(
                "takeoff",
                {"target_altitude_agl_m": _requested_altitude(mission)})

        if airborne:
            # The work is done once the aircraft has held at altitude, so the
            # flight ends. This is the decision a 4B model got wrong six or
            # seven times in nine, and it is four lines here.
            if "hold" in completed or "goto" in completed or "orbit" in completed:
                return self._action("land", {})
            return self._action("hold", {"duration_ms": self.hold_ms})

        return self._action("none", {})

    @staticmethod
    def _action(name, arguments):
        return {"action": name, "arguments": arguments}
