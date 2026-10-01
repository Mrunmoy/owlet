"""Scripted stand-in for pyserial.Serial, injected via Owlet(transport=...)."""

import threading
import time

from owlet.protocol import decode, encode


def frame(body: str) -> bytes:
    name, *fields = body.split(",")
    return encode(name, *fields)


class FakeSerial:
    """Answers requests through `responder(sentence) -> [reply bodies]`."""

    def __init__(self, responder=None):
        self.rx = bytearray()
        self.written: list[str] = []
        self.responder = responder or default_responder
        self.read_error: Exception | None = None
        self._lock = threading.Lock()

    # pyserial surface used by Owlet
    def write(self, data: bytes) -> int:
        text = data.decode()
        with self._lock:
            self.written.append(text.strip())
        for body in self.responder(decode(text)):
            self.inject(body)
        return len(data)

    def read(self, n: int) -> bytes:
        if self.read_error is not None:
            raise self.read_error
        with self._lock:
            chunk = bytes(self.rx[:n])
            del self.rx[:n]
        if not chunk:
            time.sleep(0.002)
        return chunk

    def reset_input_buffer(self) -> None:
        with self._lock:
            self.rx.clear()

    def close(self) -> None:
        pass

    # test helpers
    def inject(self, body: str) -> None:
        with self._lock:
            self.rx += frame(body)

    def sent(self, name: str) -> list[str]:
        with self._lock:
            return [w for w in self.written if w.startswith("$" + name)]


_t0 = time.monotonic()


def default_responder(s):
    ms = int((time.monotonic() - _t0) * 1000)
    if s.name == "PING":
        return ["PONG,2,test"]
    if s.name == "CLOCK":
        return [f"CLOCK,{ms}"]
    if s.name == "POS":
        return [f"POS,0.0,0.0,0.0,0.0,H,{ms}"]
    if s.name in ("WATCHDOG", "STREAM", "VEL", "STOP", "ENABLE", "DISABLE", "MOVE"):
        if s.name == "WATCHDOG" and not s.fields:
            return ["WATCHDOG,500"]
        return [f"ACK,{s.name}"]
    return ["ERR,UNK"]
