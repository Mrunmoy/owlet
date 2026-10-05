"""High-level client for an owlet pan-tilt head over a serial port."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Callable, Iterator

import serial
from serial.tools import list_ports

from .protocol import (
    AxisCalibration,
    Capabilities,
    decode,
    Diagnostics,
    encode,
    HeadEvent,
    parse_calibration,
    parse_caps,
    parse_diag,
    parse_event,
    parse_position,
    parse_trouble,
    Position,
    PROTOCOL_VERSION,
    ProtocolError,
    Sentence,
    Trouble,
)

_AXES = ("pan", "tilt")
_log = logging.getLogger("owlet")


def _axis(name: str) -> str:
    a = name.lower()
    if a not in _AXES:
        raise ValueError(f"axis must be 'pan' or 'tilt', not {name!r}")
    return a

ESPRESSIF_VID = 0x303A
RASPBERRY_PI_VID = 0x2E8A  # Pico stepper head (pico-sdk USB CDC)
OWLET_VIDS = frozenset({ESPRESSIF_VID, RASPBERRY_PI_VID})

# Sentences that are replies to a request (POS is excluded: it is also streamed).
REPLY_NAMES = frozenset({"ACK", "NAK", "ERR", "PONG", "CLOCK", "SPEED", "WATCHDOG", "LIMITS"})


def is_reply(s: Sentence) -> bool:
    """True if `s` answers some request ($ERR,BUS is unsolicited)."""
    return s.name in REPLY_NAMES and not (s.name == "ERR" and s.fields[:1] == ("BUS",))


class OwletError(Exception):
    """The device refused a request (NAK/ERR) or did not answer."""

    def __init__(self, message: str, code: str | None = None):
        super().__init__(message)
        self.code = code


def find_ports() -> list[str]:
    """Serial devices that look like an owlet head (ESP32 or Raspberry Pi Pico USB)."""
    return sorted(p.device for p in list_ports.comports() if p.vid in OWLET_VIDS)


class Signal:
    """Functions to call when something happens.

    >>> head.limit_hit.connect(lambda axis, end: print(axis, "hit", end))

    A handler that raises is logged and does not stop the others.
    """

    def __init__(self, on_connect: Callable[[], None] | None = None):
        self._handlers: list[Callable[..., None]] = []
        self._on_connect = on_connect

    def connect(self, handler: Callable[..., None]) -> Callable[..., None]:
        """Call ``handler`` on every emission. Returns it, so this works as a decorator."""
        self._handlers.append(handler)
        if self._on_connect is not None:
            self._on_connect()
        return handler

    def disconnect(self, handler: Callable[..., None]) -> None:
        self._handlers.remove(handler)

    def emit(self, *args) -> None:
        for handler in list(self._handlers):
            try:
                handler(*args)
            except Exception:  # noqa: BLE001 - one bad handler must not break the link
                _log.exception("owlet event handler failed")


class Owlet:
    """Talk to an owlet head.

    >>> with Owlet() as head:          # auto-detects the port
    ...     head.enable()
    ...     head.move(30, -15, wait=True)
    ...     print(head.position())

    The head also reports things by itself. Connect a function to hear them:

    >>> head.limit_hit.connect(lambda axis, end: print(axis, "hit its", end, "switch"))
    >>> head.arrived.connect(lambda pan, tilt: print("there:", pan, tilt))

    Signals: ``arrived(pan, tilt)``, ``watchdog_expired()``, ``bus_fault()``,
    ``bus_ok()``, ``limit_hit(axis, end)``, ``stalled(axis)``,
    ``fault_changed(axis, faults)``, ``driver_online(axis)``,
    ``driver_offline(axis)``, ``position_lost(axis)``, ``homed()``,
    ``homing_failed(axis, code)``, ``events_lost(count)``, and ``event(HeadEvent)``
    for every ``$EVT`` line whatever its kind.

    Connecting the first handler starts a background reader, so handlers
    run without your program asking. They run on that reader's thread, or
    on yours when the line arrives during one of your own calls. With
    ``dispatch="manual"`` no thread is started: handlers run only during
    your calls and ``process_events()``.
    """

    def __init__(self, port: str | None = None, timeout: float = 1.0,
                 connect: bool = True, transport=None, dispatch: str = "auto"):
        """``transport`` replaces the serial port with any object offering
        pyserial's ``read``/``write``/``reset_input_buffer``/``close``
        (a socket bridge, a test fake, …), and optionally ``in_waiting``."""
        if dispatch not in ("auto", "manual"):
            raise ValueError("dispatch must be 'auto' or 'manual'")
        self._dispatch = dispatch
        if transport is not None:
            self.port = port or "<transport>"
            self._serial = transport
        elif port is None:
            ports = find_ports()
            if len(ports) != 1:
                raise OwletError(
                    "could not auto-detect the owlet port "
                    f"(found {ports or 'none'}); pass port=..."
                )
            port = ports[0]
        if transport is None:
            self.port = port
            self._serial = serial.Serial(port, 115200, timeout=0.05)
        self.timeout = timeout
        # Requests sent with send() whose replies haven't been seen yet. A
        # frame-level $ERR can only be blamed on the current request when
        # nothing else is outstanding.
        self._outstanding = 0
        self._outstanding_lock = threading.Lock()
        self._rx = b""
        self._pending: deque[Sentence] = deque()
        self.on_event: Callable[[Sentence], None] | None = None
        # One reader at a time: a request holds this from its write to its
        # reply, so the background reader can never take that reply.
        self._io = threading.RLock()
        self._closing = threading.Event()
        self._listener: threading.Thread | None = None
        self._listener_holds = 0  # readers that need every line themselves (a Tracker)
        self._arrivals = 0
        self._homings = 0          # homing outcomes seen
        self._homing_failure: tuple[str | None, str] | None = None
        start = self._start_listener
        self.arrived = Signal(start)
        self.watchdog_expired = Signal(start)
        self.bus_fault = Signal(start)
        self.bus_ok = Signal(start)
        self.limit_hit = Signal(start)
        self.stalled = Signal(start)
        self.fault_changed = Signal(start)
        self.driver_online = Signal(start)
        self.driver_offline = Signal(start)
        self.position_lost = Signal(start)
        self.homed = Signal(start)
        self.homing_failed = Signal(start)
        self.events_lost = Signal(start)
        self.event = Signal(start)
        self.protocol_version: int | None = None
        self.firmware_version: str | None = None
        if connect:
            self._handshake()

    # ── lifecycle ─────────────────────────────────────────────────────────

    def close(self) -> None:
        self._closing.set()
        listener = self._listener
        if listener is not None and listener is not threading.current_thread():
            listener.join(timeout=1.0)
        self._serial.close()

    def __enter__(self) -> "Owlet":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _handshake(self) -> None:
        # The ROM banner or a $READY may be in flight; PING until a PONG
        # comes back instead of trusting the first bytes on the wire.
        self._serial.reset_input_buffer()
        last: Exception | None = None
        for _ in range(3):
            try:
                proto, fw = self.ping()
                break
            except OwletError as e:
                last = e
        else:
            raise OwletError(f"no answer from owlet on {self.port}: {last}")
        if proto != PROTOCOL_VERSION:
            raise OwletError(f"device speaks protocol v{proto}, client v{PROTOCOL_VERSION}")

    # ── commands ──────────────────────────────────────────────────────────

    def ping(self) -> tuple[int, str]:
        s = self.request("PING", expect="PONG")
        self.protocol_version, self.firmware_version = int(s.fields[0]), s.fields[1]
        return self.protocol_version, self.firmware_version

    def enable(self, pan: float | None = None, tilt: float | None = None) -> None:
        """Energise the servos. With a pose they snap there (if disabled)."""
        if (pan is None) != (tilt is None):
            raise ValueError("give both pan and tilt, or neither")
        args = () if pan is None else (pan, tilt)
        self._ack("ENABLE", *args)

    def disable(self) -> None:
        self._ack("DISABLE")

    def move(self, pan: float, tilt: float, wait: bool = False,
             timeout: float | None = None) -> Position | None:
        return self._move("MOVE", (pan, tilt), wait, timeout)

    def nudge(self, dpan: float, dtilt: float, wait: bool = False,
              timeout: float | None = None) -> Position | None:
        return self._move("NUDGE", (dpan, dtilt), wait, timeout)

    def home(self, wait: bool = False, timeout: float | None = None) -> Position | None:
        return self._move("HOME", (), wait, timeout)

    def _move(self, name: str, args: tuple, wait: bool, timeout: float | None) -> Position | None:
        with self._io:
            before = self._arrivals  # an arrival may come in with the reply itself
            self._ack(name, *args)
            return self.wait_arrived(timeout, _since=before) if wait else None

    def stop(self) -> None:
        self._ack("STOP")

    def seek_home(self, wait: bool = False, timeout: float = 180.0) -> None:
        """Find the reference position from the limit switches. The head
        moves by itself. Refused with code ``UNS`` on a head without switches.

        With ``wait`` it returns once the head is homed, and raises if homing
        failed. Without, connect to ``homed`` and ``homing_failed``.
        """
        with self._io:
            before = self._homings
            self._ack("SEEK")
            if not wait:
                return
            deadline = time.monotonic() + timeout
            while self._homings == before:
                s = self._next_sentence(deadline)
                if s is None:
                    raise OwletError("timed out waiting for homing to finish")
                if s.name != "EVT":
                    self._dispatch_event(s)
            if self._homing_failure is not None:
                axis, code = self._homing_failure
                raise OwletError(f"homing failed on {axis} (reason {code})", "HOMEFAIL")

    def speed(self, deg_per_sec: float | None = None) -> float:
        if deg_per_sec is None:
            return float(self.request("SPEED", expect="SPEED").fields[0])
        self._ack("SPEED", deg_per_sec)
        return float(deg_per_sec)

    def position(self) -> Position:
        return parse_position(self.request("POS", expect="POS"))

    def limits(self) -> tuple[float, float, float, float]:
        f = self.request("LIMITS", expect="LIMITS").fields
        return float(f[0]), float(f[1]), float(f[2]), float(f[3])

    # ── capabilities and calibration (protocol v3) ────────────────────────

    def caps(self) -> Capabilities:
        """Motion capabilities: speed, velocity, watchdog and stream ranges."""
        return parse_caps(self.request("CAPS", expect="CAPS"))

    def calibration(self, axis: str) -> AxisCalibration:
        """The calibration stored on the device for 'pan' or 'tilt'."""
        a = _axis(axis)
        return parse_calibration(self.request("CAL", a.upper(), expect="CAL"))

    def diag(self, axis: str) -> Diagnostics:
        """What the motor driver of 'pan' or 'tilt' reports: faults, load, stall, current.

        ``available`` is False on a head whose drivers have no data link (the
        servo head) or while a driver is not answering (its motor supply is off).
        Firmware before 0.4.0 does not know the command and raises OwletError.
        """
        a = _axis(axis)
        return parse_diag(self.request("DIAG", a.upper(), expect="DIAG"))

    def set_calibration(self, axis: str, min_us: int, center_us: int, max_us: int,
                        min_deg: float, max_deg: float) -> None:
        """Replace an axis's calibration (in RAM; ``save_calibration`` persists).

        Changing the pulses requires the servos to be disabled (error code
        ``ENA``); changing only the limits works at any time.
        """
        self._ack("CAL", _axis(axis).upper(), int(min_us), int(center_us), int(max_us),
                  min_deg, max_deg)

    def set_limits(self, axis: str, min_deg: float, max_deg: float) -> None:
        """Change only the soft limits of an axis (allowed while enabled)."""
        c = self.calibration(axis)
        self.set_calibration(axis, c.min_us, c.center_us, c.max_us, min_deg, max_deg)

    def set_reversed(self, axis: str, reversed_: bool) -> None:
        """Make + go the other way (servos must be disabled). Does nothing if
        the axis is already in that state.

        Swaps the pulse endpoints *and mirrors the limits*, so the soft limits
        keep protecting the same physical end stops.
        """
        c = self.calibration(axis)
        if c.reversed != reversed_:
            self.set_calibration(axis, c.max_us, c.center_us, c.min_us, -c.max_deg, -c.min_deg)

    def save_calibration(self) -> None:
        """Store both axes' calibration in the device's flash."""
        self._ack("CAL", "SAVE")

    def reset_calibration(self) -> None:
        """Restore factory calibration and erase the saved copy (servos must be
        disabled)."""
        self._ack("CAL", "RESET")

    def stream(self, hz: int) -> None:
        self._ack("STREAM", int(hz))

    def velocity(self, pan_deg_s: float, tilt_deg_s: float) -> None:
        """Velocity mode: turn at signed rates until told otherwise.

        The device stops by itself unless this is refreshed within the
        watchdog period (see ``watchdog``). For a high-rate control loop use
        ``owlet.tracking.Tracker.set_rates``, which doesn't wait for each ACK.
        """
        self._ack("VEL", pan_deg_s, tilt_deg_s)

    def watchdog(self, ms: int | None = None) -> int:
        """Get or set the velocity watchdog (0 = off, else 50–5000 ms)."""
        if ms is None:
            return int(self.request("WATCHDOG", expect="WATCHDOG").fields[0])
        self._ack("WATCHDOG", int(ms))
        return int(ms)

    def clock(self) -> int:
        """Device uptime in ms (wraps at 2**32)."""
        return int(self.request("CLOCK", expect="CLOCK").fields[0])

    def watch(self, hz: int = 10) -> Iterator[Position]:
        """Yield streamed positions until the caller stops iterating."""
        self.stream(hz)
        try:
            while True:
                s = self._next_sentence(deadline=time.monotonic() + max(1.0, 3.0 / hz))
                if s is None:
                    raise OwletError("position stream stalled")
                if s.name == "POS":
                    yield parse_position(s)
                else:
                    self._dispatch_event(s)
        finally:
            self.stream(0)

    def wait_arrived(self, timeout: float | None = None, _since: int | None = None) -> Position:
        """Block until the device reports $ARRIVED; returns the final position."""
        deadline = time.monotonic() + (timeout if timeout is not None else 30.0)
        with self._io:
            since = self._arrivals if _since is None else _since
            while self._arrivals == since:
                s = self._next_sentence(deadline)
                if s is None:
                    raise OwletError("timed out waiting for ARRIVED")
                if s.name != "ARRIVED":
                    self._dispatch_event(s)
            return self.position()

    def events(self, enabled: bool) -> None:
        """Turn the head's ``$EVT`` lines on or off (they are on at power-up)."""
        self._ack("EVENTS", 1 if enabled else 0)

    def stats(self) -> dict[str, int]:
        """The head's health counters: events ``posted`` and ``dropped``, the
        event queue's ``high_water`` mark, control-loop passes that ran
        ``late``, and the longest gap between two passes, ``worst_ms``."""
        f = [int(x) for x in self.request("STAT", expect="STAT").fields]
        f += [0] * (5 - len(f))  # firmware 0.5.0 development builds sent three
        return {"posted": f[0], "dropped": f[1], "high_water": f[2], "late": f[3], "worst_ms": f[4]}

    def trouble_codes(self) -> list[Trouble]:
        """Everything that has gone wrong since power-up (or the last clear),
        one entry per kind of trouble per axis, oldest first."""
        with self._io:
            count = int(self.request("DTC", expect="DTC").fields[0])
            return [parse_trouble(self.request("DTC", i, expect="DTC")) for i in range(count)]

    def clear_trouble_codes(self) -> None:
        self._ack("DTC", "CLEAR")

    def process_events(self, timeout: float = 0.0) -> int:
        """Read what the head has sent and run the handlers for it.

        Returns the number of lines handled. Only needed with
        ``dispatch="manual"``; otherwise the background reader does this.
        """
        deadline = time.monotonic() + timeout
        handled = 0
        with self._io:
            while True:
                if not self._pending and not self._input_waiting() and time.monotonic() >= deadline:
                    return handled
                s = self._next_sentence(max(deadline, time.monotonic() + 0.001))
                if s is None:
                    return handled
                self._handle_unsolicited(s)
                handled += 1

    # ── background reader ─────────────────────────────────────────────────

    def hold_listener(self) -> None:
        """Keep the background reader off the port, for a reader that needs
        every line itself. Pair with ``release_listener()``."""
        with self._outstanding_lock:
            self._listener_holds += 1

    def release_listener(self) -> None:
        with self._outstanding_lock:
            self._listener_holds = max(0, self._listener_holds - 1)

    def _start_listener(self) -> None:
        if self._dispatch != "auto" or self._listener is not None or self._closing.is_set():
            return
        self._listener = threading.Thread(target=self._listen, name="owlet-events", daemon=True)
        self._listener.start()

    def _input_waiting(self) -> bool:
        # A transport that cannot say is assumed to have something.
        return bool(getattr(self._serial, "in_waiting", 1))

    def _listen(self) -> None:
        while not self._closing.is_set():
            try:
                idle = self._listener_holds > 0 or not (self._pending or self._input_waiting())
                if idle:
                    self._closing.wait(0.005)
                    continue
                with self._io:
                    s = self._next_sentence(time.monotonic() + 0.02)
                    if s is not None:
                        self._handle_unsolicited(s)
            except Exception:  # noqa: BLE001 - the port went away; requests will report it
                if not self._closing.is_set():
                    _log.exception("owlet event reader stopped")
                return

    def _handle_unsolicited(self, s: Sentence) -> None:
        if is_reply(s):
            self.take_outstanding()  # a late answer to an earlier send()
        self._dispatch_event(s)

    # ── low level ─────────────────────────────────────────────────────────

    def send(self, name: str, *fields: object) -> None:
        """Send a request without waiting for its reply (fire-and-forget)."""
        with self._outstanding_lock:
            self._serial.write(encode(name, *fields))
            self._outstanding += 1

    def await_reply(self, name: str) -> Sentence:
        """Wait for the ACK/NAK of an earlier ``send(name, …)``, matched by
        name, and settle the books: replies arrive in order, so everything
        sent before it has now been answered (or was lost)."""
        deadline = time.monotonic() + self.timeout
        while True:
            s = self._next_sentence(deadline)
            if s is None:
                raise OwletError(f"no reply to {name}")
            if s.name in ("ACK", "NAK") and s.fields[:1] == (name,):
                with self._outstanding_lock:
                    self._outstanding = 0
                if s.name == "NAK":
                    code = s.fields[1] if len(s.fields) > 1 else None
                    raise OwletError(f"{name} refused: {code}", code)
                return s
            self._dispatch_event(s)

    def take_outstanding(self) -> bool:
        """Account one reply to an earlier ``send()``. Returns False if none
        was outstanding. Used by readers that consume replies themselves."""
        with self._outstanding_lock:
            if self._outstanding > 0:
                self._outstanding -= 1
                return True
            return False

    def read_sentence(self, timeout: float) -> Sentence | None:
        """Next valid sentence from the device, or None after ``timeout`` s."""
        return self._next_sentence(time.monotonic() + timeout)

    def request(self, name: str, *fields: object, expect: str = "ACK") -> Sentence:
        """Send a request and return its reply, raising on NAK/ERR.

        Replies arrive in request order. So while earlier ``send()``s are
        unanswered, the next reply-type line belongs to the oldest of them,
        whatever its name, and goes to ``on_event``. Only after that is a
        reply matched to this request. Unsolicited lines (POS stream,
        ARRIVED, TIMEOUT, ERR,BUS, READY) also go to ``on_event``.
        """
        with self._io:
            return self._request(name, fields, expect)

    def _request(self, name: str, fields: tuple, expect: str) -> Sentence:
        self._serial.write(encode(name, *fields))
        deadline = time.monotonic() + self.timeout
        while True:
            s = self._next_sentence(deadline)
            if s is None:
                with self._outstanding_lock:
                    self._outstanding = 0  # lost replies: resynchronise the books
                raise OwletError(f"no reply to {name}")
            if is_reply(s) and self.take_outstanding():
                self._dispatch_event(s)  # late reply to an earlier send()
                continue
            if s.name == "NAK" and s.fields[:1] == (name,):
                code = s.fields[1] if len(s.fields) > 1 else None
                raise OwletError(f"{name} refused: {code}", code)
            if s.name == "ERR" and is_reply(s):
                err = s.fields[0] if s.fields else None
                raise OwletError(f"{name} rejected: {err}", err)
            if s.name == expect and (expect != "ACK" or s.fields[:1] == (name,)):
                return s
            self._dispatch_event(s)

    def _ack(self, name: str, *fields: object) -> None:
        self.request(name, *fields, expect="ACK")

    def _dispatch_event(self, s: Sentence) -> None:
        if self.on_event is not None:
            self.on_event(s)

    def _emit(self, s: Sentence) -> None:
        """Run the handlers for a line the head sent by itself. Every line
        read passes through here exactly once, whoever is reading."""
        try:
            if s.name == "ARRIVED":
                self._arrivals += 1
                self.arrived.emit(float(s.fields[0]), float(s.fields[1]))
            elif s.name == "TIMEOUT":
                self.watchdog_expired.emit()
            elif s.name == "ERR" and s.fields[:1] == ("BUS",):
                self.bus_fault.emit()
            elif s.name == "EVT":
                e: HeadEvent = parse_event(s)
                self.event.emit(e)
                if e.name in ("HOMED", "HOMEFAIL"):
                    self._homing_failure = None if e.name == "HOMED" else (e.axis, e.arg)
                    self._homings += 1
                signal, args = {
                    "BUSOK": (self.bus_ok, ()),
                    "LIMIT": (self.limit_hit, (e.axis, e.arg.lower())),
                    "STALL": (self.stalled, (e.axis,)),
                    "FAULT": (self.fault_changed, (e.axis, int(e.arg) if e.arg.isdigit() else 0)),
                    "ONLINE": (self.driver_online, (e.axis,)),
                    "OFFLINE": (self.driver_offline, (e.axis,)),
                    "LOST": (self.position_lost, (e.axis,)),
                    "HOMED": (self.homed, ()),
                    "HOMEFAIL": (self.homing_failed, (e.axis, e.arg)),
                    "OVERFLOW": (self.events_lost, (int(e.arg) if e.arg.isdigit() else 0,)),
                }.get(e.name, (None, ()))
                if signal is not None:
                    signal.emit(*args)
        except (ProtocolError, ValueError, IndexError):
            pass  # a malformed line is not worth breaking the link for

    def _next_sentence(self, deadline: float) -> Sentence | None:
        with self._io:
            s = self._read_sentence(deadline)
            if s is not None:
                self._emit(s)
            return s

    def _read_sentence(self, deadline: float) -> Sentence | None:
        while time.monotonic() < deadline:
            if self._pending:
                return self._pending.popleft()
            # pyserial's read(n) waits for all n bytes or the timeout, so asking
            # for a fixed size would hold every short reply back by the whole
            # timeout. Take what is already buffered (at least one byte).
            chunk = self._serial.read(max(1, getattr(self._serial, "in_waiting", 0)))
            if not chunk:
                continue
            self._rx += chunk
            *lines, self._rx = self._rx.split(b"\n")
            for raw in lines:
                text = raw.decode("ascii", errors="replace").strip()
                start = text.find("$")
                if start < 0:
                    continue  # boot banner or other noise
                try:
                    self._pending.append(decode(text[start:]))
                except ProtocolError:
                    continue
        return None
