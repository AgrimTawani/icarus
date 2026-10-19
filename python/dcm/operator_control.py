"""Immediate operator commands; independent of mission inference."""

import threading
import time
import uuid

OPERATOR_COMMANDS = {
    "/rtl": "rtl",
    "/land": "land",
    "/hold": "hold",
    "/stop": "hold",
    "/abort": "abort",
    # Exact plain safety phrases.  This is safety-command recognition, not
    # plan grounding: a match can only stop the aircraft, land it or bring it
    # home through the guarded action path, never plan or change a mission.
    "rtl": "rtl",
    "return home": "rtl",
    "return to home": "rtl",
    "go home": "rtl",
    "land": "land",
    "land now": "land",
    "stop": "hold",
    "hold": "hold",
    "abort": "abort",
}
OPERATOR_COMMAND_ACTIONS = {
    "rtl": "return_home",
    "abort": "return_home",
    "land": "land",
    "hold": "hold",
}
_COMMAND_RECOVERY = {"rtl": "rtl", "abort": "rtl", "land": "land", "hold": "hold"}
_COMMAND_TEXT = {
    "return_home": "returning home",
    "land": "landing here",
    "hold": "holding position",
}
MAX_OPERATOR_COMMANDS = 4
# A failed runtime target objective may add a hold and its fallback.
RUNTIME_FALLBACK_ACTIONS = 2
FLIGHT_INPUT_NOTICE = "Mission running — only /rtl /land /hold /abort are accepted now"


def operator_safety_command(text):
    """Return rtl/land/hold/abort for an exact safety command, else None."""
    normalized = " ".join(str(text or "").lower().split()).strip(" .!")
    return OPERATOR_COMMANDS.get(normalized)


class OperatorInterrupt:
    """Thread-safe operator interrupt shared by the console and the executor.

    While a mission runs (``arm``..``disarm``) the console thread
    passes every typed line to ``submit``.  A safety command is recorded,
    the running Drone API action is cancelled (CancelAction), and the
    executor then sends the matching return_home/land/hold through the normal
    guarded action path and ends the mission.  A later, different command
    (for example /land during an /rtl) supersedes the earlier one the same
    way.  Any other text is refused by the console and discarded, so nothing
    typed mid-flight can become the next mission.
    """

    def __init__(self, echo=print):
        self._lock = threading.Lock()
        self._echo = echo
        self._client = None
        self._running = None
        self.active = False
        self.safety_active = False
        self.commands = []
        self.cancellations = []

    def arm(self, client):
        with self._lock:
            self._client = client
            self.active = True

    def enable_safety(self, client):
        """Accept safety commands before execution, including model waits."""
        with self._lock:
            self._client = client
            self.safety_active = True

    def disable_safety(self):
        with self._lock:
            self.safety_active = False
            return len(self.commands)

    def disarm(self):
        with self._lock:
            self.active = False
            self._running = None

    def sequence(self):
        with self._lock:
            return len(self.commands)

    def latest(self):
        with self._lock:
            return dict(self.commands[-1]) if self.commands else None

    def record(self):
        with self._lock:
            return {
                "interrupted": bool(self.commands),
                "command": (self.commands[-1]["command"] if self.commands else None),
                "commands": [dict(item) for item in self.commands],
                "cancellations": [dict(item) for item in self.cancellations],
            }

    def submit(self, text):
        """Handle one line typed during flight; return its command or None."""
        command = operator_safety_command(text)
        if command is None:
            return None
        action = OPERATOR_COMMAND_ACTIONS[command]
        with self._lock:
            if not (self.active or self.safety_active):
                return None
            current = self.commands[-1]["command"] if self.commands else None
            duplicate = (
                current is not None and OPERATOR_COMMAND_ACTIONS[current] == action
            )
            full = len(self.commands) >= MAX_OPERATOR_COMMANDS
            running = None
            if not duplicate and not full:
                self.commands.append(
                    {
                        "command": command,
                        "text": " ".join(str(text).split())[:80],
                        "action": action,
                        "unix_ms": int(time.time() * 1000),
                    }
                )
                running = self._running
        if duplicate:
            self._echo(f"Operator command {command!r} is already being handled.")
        elif full:
            self._echo(
                "Operator command limit reached; the current safety action continues."
            )
        else:
            self._echo(
                f"Operator command {command!r} received: stopping the "
                f"mission and {_COMMAND_TEXT[action]}."
            )
            if running is not None:
                self._cancel(running)
        return command

    def track(self, receipt, since):
        """Register the action being waited on; cancel it if a command
        arrived after it was sent (``since`` is the sequence at send time)."""
        with self._lock:
            self._running = receipt
            late = len(self.commands) > since
        if late:
            self._cancel(receipt)

    def untrack(self, receipt):
        with self._lock:
            if self._running is receipt:
                self._running = None

    def _cancel(self, receipt):
        from icarus.v1 import action_pb2

        entry = {
            "action_id": getattr(receipt, "action_id", "") or "",
            "unix_ms": int(time.time() * 1000),
            "requested": False,
        }
        try:
            client = self._client
            request = action_pb2.CancelActionRequest(
                context=client.context(f"cancel-{uuid.uuid4().hex[:8]}"),
                action_id=entry["action_id"],
            )
            reply = client.action_api.CancelAction(request, timeout=5)
            entry["requested"] = True
            entry["message"] = str(getattr(reply, "message", "") or "")
        except Exception as failure:  # noqa: BLE001 - the safety action still follows
            entry["error"] = f"{type(failure).__name__}: {failure}"[:200]
            self._echo(
                f"      cancel request failed ({failure}); the safety "
                "action is still sent through the guarded path"
            )
        with self._lock:
            self.cancellations.append(entry)
