"""High-level client for an owlet pan-tilt head over a serial port."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Iterator

import serial
from serial.tools import list_ports

from .protocol import (
    PROTOCOL_VERSION,
    AxisCalibration,
    Capabilities,
    Position,
    ProtocolError,
    Sentence,
    decode,
    encode,
    parse_calibration,
    parse_caps,
    parse_position,
)

_AXES = ("pan", "tilt")


def _axis(name: str) -> str:
    a = name.lower()
    if a not in _AXES:
        raise ValueError(f"axis must be 'pan' or 'tilt', not {name!r}")
    return a

ESPRESSIF_VID = 0x303A

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
    """Serial devices that look like an ESP32-S3 USB-Serial-JTAG port."""
    return sorted(p.device for p in list_ports.comports() if p.vid == ESPRESSIF_VID)


class Owlet:
    """Talk to an owlet head.

    >>> with Owlet() as head:          # auto-detects the port
    ...     head.enable()
    ...     head.move(30, -15, wait=True)
    ...     print(head.position())
    """

    def __init__(self, port: str | None = None, timeout: float = 1.0,
                 connect: bool = True, transport=None):
        """``transport`` replaces the serial port with any object offering
        pyserial's ``read``/``write``/``reset_input_buffer``/``close``
        (a socket bridge, a test fake, …), and optionally ``in_waiting``."""
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
        self.protocol_version: int | None = None
        self.firmware_version: str | None = None
        if connect:
            self._handshake()

    # ── lifecycle ─────────────────────────────────────────────────────────

    def close(self) -> None:
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
        self._ack("MOVE", pan, tilt)
        return self.wait_arrived(timeout) if wait else None

    def nudge(self, dpan: float, dtilt: float, wait: bool = False,
              timeout: float | None = None) -> Position | None:
        self._ack("NUDGE", dpan, dtilt)
        return self.wait_arrived(timeout) if wait else None

    def home(self, wait: bool = False, timeout: float | None = None) -> Position | None:
        self._ack("HOME")
        return self.wait_arrived(timeout) if wait else None

    def stop(self) -> None:
        self._ack("STOP")

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

    def wait_arrived(self, timeout: float | None = None) -> Position:
        """Block until the device reports $ARRIVED; returns the final position."""
        deadline = time.monotonic() + (timeout if timeout is not None else 30.0)
        while True:
            s = self._next_sentence(deadline)
            if s is None:
                raise OwletError("timed out waiting for ARRIVED")
            if s.name == "ARRIVED":
                return self.position()
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

    def _next_sentence(self, deadline: float) -> Sentence | None:
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
