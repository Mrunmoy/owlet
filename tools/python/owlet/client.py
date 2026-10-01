"""High-level client for an owlet pan-tilt head over a serial port."""

from __future__ import annotations

import time
from collections import deque
from typing import Callable, Iterator

import serial
from serial.tools import list_ports

from .protocol import (
    PROTOCOL_VERSION,
    Position,
    ProtocolError,
    Sentence,
    decode,
    encode,
    parse_position,
)

ESPRESSIF_VID = 0x303A

# Lines the device sends on its own, never as the reply to a request.
_EVENTS = {"READY", "ARRIVED"}


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
                 connect: bool = True):
        if port is None:
            ports = find_ports()
            if len(ports) != 1:
                raise OwletError(
                    "could not auto-detect the owlet port "
                    f"(found {ports or 'none'}); pass port=..."
                )
            port = ports[0]
        self.port = port
        self.timeout = timeout
        self._serial = serial.Serial(port, 115200, timeout=0.05)
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

    def stream(self, hz: int) -> None:
        self._ack("STREAM", int(hz))

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
        self._serial.write(encode(name, *fields))

    def request(self, name: str, *fields: object, expect: str = "ACK") -> Sentence:
        """Send a request and return its reply, raising on NAK/ERR."""
        self.send(name, *fields)
        deadline = time.monotonic() + self.timeout
        while True:
            s = self._next_sentence(deadline)
            if s is None:
                raise OwletError(f"no reply to {name}")
            if s.name == "NAK":
                code = s.fields[1] if len(s.fields) > 1 else None
                raise OwletError(f"{name} refused: {code}", code)
            if s.name == "ERR":
                raise OwletError(f"{name} rejected: {s.fields[0]}", s.fields[0])
            if s.name == expect and (expect != "ACK" or s.fields[:1] == (name,)):
                return s
            # Streamed POS, ARRIVED, READY, ... — not our reply.
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
            chunk = self._serial.read(256)
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
