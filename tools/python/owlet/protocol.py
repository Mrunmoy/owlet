"""owlet wire protocol v1: framing, checksum and reply decoding.

Pure functions, no I/O. See docs/DESIGN.md §5 for the specification.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import reduce

PROTOCOL_VERSION = 3
MAX_LINE_LEN = 82

_NAME_RE = re.compile(r"^[A-Z]{1,10}$")
_FRAME_RE = re.compile(r"^\$([^$*]+)\*([0-9A-Fa-f]{2})$")


class ProtocolError(Exception):
    """A line that is not a valid owlet frame."""


class ChecksumError(ProtocolError):
    pass


class SyntaxError_(ProtocolError):  # noqa: N801 - avoid shadowing builtin
    pass


@dataclass(frozen=True)
class Sentence:
    name: str
    fields: tuple[str, ...]


@dataclass(frozen=True)
class Position:
    """A $POS report. Angles in degrees; state is D (disabled), M or H.

    ``t_ms`` is the device uptime (ms, wraps at 2**32) at which the pose was
    computed. Use ``owlet.tracking.DeviceClock`` to map it to host time.
    """

    pan: float
    tilt: float
    target_pan: float
    target_tilt: float
    state: str
    t_ms: int

    @property
    def enabled(self) -> bool:
        return self.state != "D"

    @property
    def moving(self) -> bool:
        return self.state == "M"


@dataclass(frozen=True)
class Capabilities:
    """What the head can do ($CAPS)."""

    speed_min: float        # slew speed range for position moves, deg/s
    speed_max: float
    velocity_max: float     # |rate| limit in velocity mode, deg/s
    watchdog_min_ms: int    # non-zero watchdog range (0 = off is also allowed)
    watchdog_max_ms: int
    stream_max_hz: int


@dataclass(frozen=True)
class AxisCalibration:
    """One axis's calibration ($CAL,<AXIS>)."""

    axis: str               # "pan" or "tilt"
    min_us: int             # pulse at -90 degrees
    center_us: int          # pulse at 0 degrees
    max_us: int             # pulse at +90 degrees
    min_deg: float          # soft limits
    max_deg: float
    reversed: bool
    source: str             # "factory", "saved" or "modified"


@dataclass(frozen=True)
class Diagnostics:
    """What one axis's motor driver reports about itself ($DIAG,<AXIS>)."""

    axis: str               # "pan" or "tilt"
    available: bool         # False: no data link to the driver, or it is not answering
    faults: int             # bit mask, see FAULT_NAMES
    stalled: bool           # load at or beyond the stall threshold while moving
    load: int               # driver's load measure (TMC2209: 0..510, high = running free)
    current_ma: int         # motor run current the driver is set to, RMS

    FAULT_NAMES = {1: "over-temperature warning", 2: "over-temperature shutdown",
                   4: "short on coil A", 8: "short on coil B",
                   16: "coil A open", 32: "coil B open"}

    @property
    def fault_names(self) -> list[str]:
        return [name for bit, name in self.FAULT_NAMES.items() if self.faults & bit]


_SOURCES = {"F": "factory", "S": "saved", "M": "modified"}


def parse_diag(sentence: Sentence) -> Diagnostics:
    f = sentence.fields
    if sentence.name != "DIAG" or len(f) != 6 or f[1] not in ("U", "N"):
        raise ProtocolError(f"not a DIAG report: {sentence}")
    try:
        return Diagnostics(f[0].lower(), f[1] == "U", int(f[2]), f[3] == "1", int(f[4]), int(f[5]))
    except ValueError as e:
        raise ProtocolError(f"bad DIAG field in {sentence}") from e


def parse_caps(sentence: Sentence) -> Capabilities:
    f = sentence.fields
    if sentence.name != "CAPS" or len(f) != 6:
        raise ProtocolError(f"not a CAPS report: {sentence}")
    try:
        return Capabilities(float(f[0]), float(f[1]), float(f[2]), int(f[3]), int(f[4]), int(f[5]))
    except ValueError as e:
        raise ProtocolError(f"bad CAPS field in {sentence}") from e


def parse_calibration(sentence: Sentence) -> AxisCalibration:
    f = sentence.fields
    if sentence.name != "CAL" or len(f) != 8 or f[6] not in ("R", "N") or f[7] not in _SOURCES:
        raise ProtocolError(f"not a CAL report: {sentence}")
    try:
        return AxisCalibration(f[0].lower(), int(f[1]), int(f[2]), int(f[3]),
                               float(f[4]), float(f[5]), f[6] == "R", _SOURCES[f[7]])
    except ValueError as e:
        raise ProtocolError(f"bad CAL field in {sentence}") from e


def checksum(body: str) -> int:
    return reduce(lambda ck, c: ck ^ ord(c), body, 0)


def format_number(value: float) -> str:
    """Format a number the way the device accepts it (≤ 2 decimals)."""
    text = f"{round(float(value), 2):.2f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def encode(name: str, *fields: object) -> bytes:
    """Build a complete frame, CRLF included."""
    if not _NAME_RE.match(name):
        raise ValueError(f"invalid command name: {name!r}")
    parts = [name]
    for f in fields:
        parts.append(format_number(f) if isinstance(f, (int, float)) else str(f))
    body = ",".join(parts)
    line = f"${body}*{checksum(body):02X}\r\n"
    if len(line) > MAX_LINE_LEN:
        raise ValueError(f"frame longer than {MAX_LINE_LEN} bytes")
    return line.encode("ascii")


def decode(line: str) -> Sentence:
    """Decode one frame (CR/LF optional). Raises ProtocolError subclasses."""
    frame = line.rstrip("\r\n")
    m = _FRAME_RE.match(frame)
    if not m:
        raise SyntaxError_(f"not a frame: {frame!r}")
    body, ck = m.group(1), int(m.group(2), 16)
    if any(not (0x20 <= ord(c) <= 0x7E) for c in body):
        raise SyntaxError_(f"non-printable byte in {frame!r}")
    if checksum(body) != ck:
        raise ChecksumError(f"checksum mismatch in {frame!r}")
    name, *fields = body.split(",")
    if not _NAME_RE.match(name):
        raise SyntaxError_(f"bad sentence name in {frame!r}")
    return Sentence(name, tuple(fields))


def parse_position(sentence: Sentence) -> Position:
    if sentence.name != "POS" or len(sentence.fields) != 6:
        raise ProtocolError(f"not a v2 POS report: {sentence}")
    p, t, tp, tt, state, ms = sentence.fields
    try:
        return Position(float(p), float(t), float(tp), float(tt), state, int(ms))
    except ValueError as e:
        raise ProtocolError(f"bad POS field in {sentence}") from e
