"""owlet wire protocol v1: framing, checksum and reply decoding.

Pure functions, no I/O. See docs/DESIGN.md §5 for the specification.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import reduce

PROTOCOL_VERSION = 2
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
