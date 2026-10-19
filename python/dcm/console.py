"""Single terminal reader with immediate operator interrupts."""

import os
import queue
import sys
import threading

from python.dcm.operator_control import FLIGHT_INPUT_NOTICE, operator_safety_command


class OperatorConsole:
    """The single stdin reader of a flight session.

    One daemon thread reads every line. Safety commands bypass the normal
    input queue during planning and confirmation too. While a mission is flying
    (its OperatorInterrupt is armed) each line goes to the interrupt: a
    safety command stops the flight, anything else is refused and discarded,
    so text typed mid-flight can never become the next mission.  Otherwise
    lines queue, in order, for whoever reads next (the mission prompt, the
    per-call confirmation or a clarification answer).  On an interactive
    terminal a line typed before the current prompt was shown is discarded
    rather than answering it.
    """

    def __init__(self, stream=None, echo=print):
        self._stream = stream if stream is not None else sys.stdin
        self._echo = echo
        self._lines = queue.Queue()
        self._lock = threading.Lock()
        self._interrupt = None
        try:
            self.interactive = bool(self._stream.isatty())
        except (AttributeError, ValueError, OSError):
            self.interactive = False
        self._thread = None
        if self.interactive and not _foreground(self._stream):
            # A background job reading its terminal would be stopped by
            # SIGTTIN; behave as end of input (confirmation then declines).
            self._lines.put(None)
            return
        self._thread = threading.Thread(
            target=self._read, daemon=True, name="icarus-operator-console"
        )
        self._thread.start()

    def _read(self):
        while True:
            try:
                line = self._stream.readline()
            except Exception:  # noqa: BLE001 - an unreadable stdin is EOF
                line = ""
            if not line:
                self._lines.put(None)
                return
            self.route(line)

    def attach(self, interrupt):
        with self._lock:
            self._interrupt = interrupt

    def route(self, line):
        with self._lock:
            interrupt = self._interrupt
        if (
            interrupt is not None
            and interrupt.safety_active
            and operator_safety_command(line)
        ):
            interrupt.submit(line)
            return
        if interrupt is not None and interrupt.active:
            text = line.strip()
            if text and interrupt.submit(text) is None:
                self._echo(FLIGHT_INPUT_NOTICE + f" (discarded: {text!r})")
            return
        self._lines.put(line)

    def readline(self):
        """Return the next queued line ('' at end of input)."""
        while True:
            with self._lock:
                interrupt = self._interrupt
            if (
                interrupt is not None
                and interrupt.safety_active
                and interrupt.sequence()
            ):
                # Wake a confirmation wait without queueing a safety command
                # that could become a later mission or clarification answer.
                return "\n"
            try:
                line = self._lines.get(timeout=0.5)
            except queue.Empty:
                continue
            if line is None:
                self._lines.put(None)  # end of input stays sticky
                return ""
            return line

    def discard_pending(self):
        """Drop lines typed before the current prompt (interactive only)."""
        if not self.interactive:
            return 0
        dropped = 0
        kept_eof = False
        while True:
            try:
                line = self._lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                kept_eof = True
                break
            dropped += 1
        if kept_eof:
            self._lines.put(None)
        return dropped


def _foreground(stream):
    """Whether this process may read its terminal without being stopped."""
    try:
        descriptor = stream.fileno()
        return os.tcgetpgrp(descriptor) == os.getpgrp()
    except (AttributeError, ValueError, OSError):
        return True
