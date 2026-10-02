"""Building blocks for closed-loop tracking (e.g. a camera + object detector).

    with Owlet() as head:
        head.enable()
        ctl = PixelController(fov_h_deg=70, fov_v_deg=43)
        with Tracker(head) as tr:
            while True:
                frame, t_capture = grab()                    # host time.monotonic()
                box = detect(frame)                          # your detector
                if box is None:
                    tr.set_rates(0, 0)
                    continue
                pan, tilt = ctl.update(box.cx, box.cy, frame.w, frame.h,
                                       pose_at_capture=tr.pose_at(t_capture),
                                       pose_now=tr.pose_at(time.monotonic()))
                tr.set_rates(pan, tilt)

If the loop stalls, the device's watchdog stops the head on its own.
"""

from __future__ import annotations

import math
import random
import threading
import time
from collections import deque
from typing import Callable, NamedTuple

from .client import Owlet, OwletError, is_reply
from .protocol import Position, ProtocolError, parse_position

_WRAP = 2**32


class Pose(NamedTuple):
    pan: float
    tilt: float


class DeviceClock:
    """Maps the device's 32-bit millisecond clock onto host ``time.monotonic()``.

    Each sample is a ``$CLOCK`` round trip (host t0, device ms, host t1). The
    offset comes from the sample with the smallest round-trip time among the
    last ``window`` samples: its midpoint is the tightest bound on when the
    device read its clock, and the window lets crystal drift be followed.
    """

    def __init__(self, monotonic: Callable[[], float] = time.monotonic, window: int = 8):
        self._monotonic = monotonic
        self._last_raw: int | None = None
        self._last_ext = 0
        self._samples: deque[tuple[float, float]] = deque(maxlen=window)  # (rtt, offset)
        self.offset_s: float | None = None

    def unwrap(self, ms: int) -> int:
        """Extend a wrapping 32-bit ms value, using the signed difference from
        the previous value (so late samples from before a wrap stay put)."""
        if self._last_raw is None:
            ext = ms
        else:
            delta = (ms - self._last_raw + _WRAP // 2) % _WRAP - _WRAP // 2
            ext = self._last_ext + delta
        self._last_raw, self._last_ext = ms, ext
        return ext

    def add_sample(self, t0: float, ms: int, t1: float) -> float:
        """Record one round trip; returns its RTT in seconds."""
        rtt = t1 - t0
        self._samples.append((rtt, (t0 + t1) / 2.0 - self.unwrap(ms) / 1000.0))
        self.offset_s = min(self._samples)[1]
        return rtt

    def sync(self, query: Callable[[], int], samples: int = 8,
             before_each: Callable[[], None] | None = None) -> float:
        """Take ``samples`` round trips now; returns the best RTT seen."""
        best = math.inf
        for _ in range(samples):
            if before_each is not None:
                before_each()
            t0 = self._monotonic()
            ms = query()
            best = min(best, self.add_sample(t0, ms, self._monotonic()))
        return best

    def ext_to_host(self, ext_ms: int) -> float:
        if self.offset_s is None:
            raise RuntimeError("DeviceClock.sync() has not been run")
        return ext_ms / 1000.0 + self.offset_s

    def to_host(self, ms: int) -> float:
        if self.offset_s is None:
            raise RuntimeError("DeviceClock.sync() has not been run")
        return self.ext_to_host(self.unwrap(ms))


class PoseHistory:
    """Recent (host time, pose) samples with linear interpolation."""

    def __init__(self, maxlen: int = 256):
        self._samples: deque[tuple[float, float, float]] = deque(maxlen=maxlen)
        self._lock = threading.Lock()  # the Tracker's reader appends while you query

    def __len__(self) -> int:
        return len(self._samples)

    def add(self, t: float, pan: float, tilt: float) -> None:
        with self._lock:
            if self._samples and t <= self._samples[-1][0]:
                return  # out of order or duplicate: keep the series monotonic
            self._samples.append((t, pan, tilt))

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    @property
    def latest(self) -> Pose | None:
        with self._lock:
            return Pose(*self._samples[-1][1:]) if self._samples else None

    @property
    def latest_time(self) -> float | None:
        with self._lock:
            return self._samples[-1][0] if self._samples else None

    def at(self, t: float) -> Pose | None:
        """Pose at host time ``t``; clamped to the oldest/newest sample."""
        with self._lock:
            return self._at(t)

    def _at(self, t: float) -> Pose | None:
        s = self._samples
        if not s:
            return None
        if t <= s[0][0]:
            return Pose(s[0][1], s[0][2])
        if t >= s[-1][0]:
            return Pose(s[-1][1], s[-1][2])
        # Recent times are the common query, so search from the newest end.
        for i in range(len(s) - 1, 0, -1):
            t0, p0, q0 = s[i - 1]
            t1, p1, q1 = s[i]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0)
                return Pose(p0 + a * (p1 - p0), q0 + a * (q1 - q0))
        return Pose(s[-1][1], s[-1][2])  # unreachable with monotonic samples


class PixelController:
    """Turns a target's pixel position into pan/tilt rates (deg/s).

    Uses pinhole geometry per axis (exact on the image's centre lines, and a
    close approximation off-axis), rather than a linear pixels-per-degree. A proportional law (rate = gain × angular error)
    suits velocity mode: the head slows as it centres the target.
    """

    def __init__(self, fov_h_deg: float, fov_v_deg: float, gain: float = 3.0,
                 deadband_deg: float = 0.3, max_rate: float = 180.0):
        for name, fov in (("fov_h_deg", fov_h_deg), ("fov_v_deg", fov_v_deg)):
            if not 0.0 < fov < 180.0:
                raise ValueError(f"{name} must be between 0 and 180 degrees")
        self._tan_h = math.tan(math.radians(fov_h_deg) / 2.0)
        self._tan_v = math.tan(math.radians(fov_v_deg) / 2.0)
        self.gain = gain
        self.deadband_deg = deadband_deg
        self.max_rate = max_rate

    def angle_offset(self, x: float, y: float, width: float, height: float) -> tuple[float, float]:
        """Angles (deg) from the image centre to pixel (x, y); owlet signs."""
        if width <= 0 or height <= 0:
            raise ValueError("frame width and height must be positive")
        nx = (x - width / 2.0) / (width / 2.0)    # -1 … 1
        ny = (y - height / 2.0) / (height / 2.0)
        dpan = math.degrees(math.atan(nx * self._tan_h))
        dtilt = -math.degrees(math.atan(ny * self._tan_v))  # image y grows downward
        return dpan, dtilt

    def update(self, x: float, y: float, width: float, height: float,
               pose_at_capture: Pose | None = None,
               pose_now: Pose | None = None) -> tuple[float, float]:
        """Rates to command now. Pass both poses to cancel detector latency:
        the error is corrected by how far the head has moved since capture."""
        dpan, dtilt = self.angle_offset(x, y, width, height)
        if pose_at_capture is not None and pose_now is not None:
            dpan += pose_at_capture.pan - pose_now.pan
            dtilt += pose_at_capture.tilt - pose_now.tilt
        return self._rate(dpan), self._rate(dtilt)

    def _rate(self, err_deg: float) -> float:
        if abs(err_deg) < self.deadband_deg:
            return 0.0
        return max(-self.max_rate, min(self.max_rate, self.gain * err_deg))


_MAX_RATE = 500.0  # deg/s, the device's velocity limit
_CLOCK_REPLY_TIMEOUT_S = 0.5


class Tracker:
    """Owns an ``Owlet`` link for a tracking session.

    - syncs the device clock (and keeps re-syncing it), sets the watchdog and
      starts the POS stream;
    - a background reader keeps ``latest``, ``history``, ``timeouts`` and
      ``last_error`` up to date;
    - ``set_rates`` sends VEL without waiting for the ACK, so a 30–60 Hz
      control loop never blocks on the serial round trip.

    Failures never pass silently: if the reader dies, or the device reboots
    (it comes back limp), ``failed`` becomes True, ``last_error`` says why,
    and ``set_rates`` raises. Start a new Tracker after fixing the cause.

    While a Tracker is running, don't call the head's request methods: the
    tracker's reader owns the input stream.
    """

    def __init__(self, head: Owlet, stream_hz: int = 50, watchdog_ms: int = 300,
                 history: int = 256, clock_samples: int = 32, resync_s: float = 1.0,
                 clock_window: int = 32, clock_jitter_s: float = 0.012):
        """Clock defaults (measured on the reference head, 2026-10-02): one
        round trip lands anywhere in the device's 10 ms loop, so it takes ~32
        samples to be sure of a short one (8 left the offset up to 4 ms off).
        The offset comes from the best of the last ``clock_window`` samples,
        and the crystals drift ~13 ppm apart, so the window must stay short:
        32 samples × 1 s resync ≈ 0.4 ms of drift at most.

        ``clock_jitter_s`` delays each clock sample by a random 0…that many
        seconds. The device serves input once per control loop (10 ms on
        owlet), so samples sent straight after a reply, or straight after a
        streamed $POS, all land just after a loop tick: every round trip is
        then a whole loop long and the offset is biased by about half of it.
        Random phases let the lowest-round-trip sample be a short one. Keep it
        a little over the device's loop period."""
        if not 50 <= watchdog_ms <= 5000:
            raise ValueError("watchdog_ms must be 50–5000: tracking always needs the watchdog")
        if not 0.0 <= clock_jitter_s <= 1.0:
            raise ValueError("clock_jitter_s must be 0–1 s")
        if clock_window < 1:
            raise ValueError("clock_window must be at least 1")
        self._head = head
        self._stream_hz = stream_hz
        self._watchdog_ms = watchdog_ms
        self._clock_samples = clock_samples
        self._resync_s = resync_s
        self._clock_jitter_s = clock_jitter_s
        self.clock = DeviceClock(window=clock_window)
        self.history = PoseHistory(history)
        self.latest: Position | None = None
        self.timeouts = 0
        self.failed = False
        self.last_error: OwletError | None = None
        self._lock = threading.Lock()  # serialises writes and start/stop
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_ext_ms: int | None = None
        self._clock_t0: float | None = None
        self._next_resync = 0.0
        self.clock_misses = 0
        self._max_rate = _MAX_RATE

    def __enter__(self) -> "Tracker":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            # A fresh session: nothing from a previous one may leak in.
            self.history.clear()
            self.latest = None
            self.timeouts = 0
            self.last_error = None
            self._last_ext_ms = None
            self._clock_t0 = None
            self.clock.sync(self._head.clock, self._clock_samples,
                            before_each=self._clock_jitter)
            self._head.watchdog(self._watchdog_ms)
            # Clamp to what *this* head reports, not to an assumed constant.
            self._max_rate = min(_MAX_RATE, self._head.caps().velocity_max)
            self._head.stream(self._stream_hz)
            self.failed = False
            self._next_resync = time.monotonic() + self._resync_s
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, name="owlet-tracker",
                                            daemon=True)
            self._thread.start()

    def stop(self) -> None:
        """Stop velocity mode and streaming and hand the link back.

        Best effort: problems are recorded in ``last_error``, never raised, so
        cleanup after it (e.g. ``head.disable()``) still runs.
        """
        with self._lock:
            if not self._running:
                return
            self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            # Replies arrive in request order, so once STREAM's reply is in,
            # every earlier fire-and-forget request has been answered (or its
            # reply was lost) and no further $POS can follow: the link is
            # clean for synchronous use. Matching by name, not by count, means
            # a reply lost during tracking can't make this wait time out.
            self._head.send("VEL", 0, 0)
            self._head.send("STREAM", 0)
            self._head.await_reply("STREAM")
        except Exception as e:  # noqa: BLE001 - cleanup must not raise
            self.last_error = OwletError(f"tracker stop: {e}")

    def set_rates(self, pan_deg_s: float, tilt_deg_s: float) -> None:
        """Command turn rates now (fire-and-forget; also feeds the watchdog).

        Rates are clamped to the head's reported velocity limit ($CAPS): an
        out-of-range VEL would be refused and the *previous* rate would keep
        running until the watchdog.
        """
        if math.isnan(pan_deg_s) or math.isnan(tilt_deg_s):
            raise ValueError("rates must be numbers")
        pan = max(-self._max_rate, min(self._max_rate, pan_deg_s))
        tilt = max(-self._max_rate, min(self._max_rate, tilt_deg_s))
        with self._lock:
            if self.failed:
                raise OwletError(f"tracker failed: {self.last_error}",
                                 getattr(self.last_error, "code", None))
            if not self._running:
                raise OwletError("tracker is not running")
            self._head.send("VEL", pan, tilt)

    def pose_at(self, host_time: float) -> Pose | None:
        """Interpolated head pose at a ``time.monotonic()`` timestamp."""
        return self.history.at(host_time)

    def _clock_jitter(self) -> None:
        if self._clock_jitter_s > 0.0:
            time.sleep(random.uniform(0.0, self._clock_jitter_s))

    # ── reader thread ──────────────────────────────────────────────────────

    def _fail(self, message: str, code: str | None) -> None:
        self.last_error = OwletError(message, code)
        self.failed = True

    def _read_loop(self) -> None:
        try:
            while self._running and not self.failed:
                now = time.monotonic()
                if self._clock_t0 is not None and now - self._clock_t0 > _CLOCK_REPLY_TIMEOUT_S:
                    # Reply late or lost: stop waiting and try again later. The
                    # reply's count is left alone: if it arrives late it settles
                    # itself, and stop() resynchronises the books anyway.
                    self._clock_t0 = None
                    self.clock_misses += 1
                    self._next_resync = now + self._resync_s
                if self._clock_t0 is None and now >= self._next_resync:
                    # This loop wakes on each streamed $POS, i.e. just after a
                    # device loop tick: send at a random phase instead.
                    self._clock_jitter()
                    with self._lock:
                        if not self._running:
                            break
                        self._clock_t0 = time.monotonic()
                        self._head.send("CLOCK")
                s = self._head.read_sentence(timeout=0.05)
                if s is not None:
                    self._handle(s)
        except Exception as e:  # noqa: BLE001 - must surface, not vanish
            self._fail(f"tracker reader stopped: {e}", "IO")

    def _handle(self, s) -> None:
        if is_reply(s):
            self._head.take_outstanding()  # answers one of our fire-and-forget sends
        if s.name == "POS":
            try:
                p = parse_position(s)
            except ProtocolError:
                return
            ext = self.clock.unwrap(p.t_ms)
            if self._last_ext_ms is not None and ext < self._last_ext_ms - 1000:
                self._fail("device clock went backwards: device rebooted", "REBOOT")
                return
            self._last_ext_ms = ext
            self.latest = p
            self.history.add(self.clock.ext_to_host(ext), p.pan, p.tilt)
        elif s.name == "CLOCK" and self._clock_t0 is not None and s.fields:
            self.clock.add_sample(self._clock_t0, int(s.fields[0]), time.monotonic())
            self._clock_t0 = None
            self._next_resync = time.monotonic() + self._resync_s
        elif s.name == "READY":
            self._fail("device rebooted (servos are limp; re-enable and restart tracking)",
                       "REBOOT")
        elif s.name == "TIMEOUT":
            self.timeouts += 1
        elif s.name == "NAK":
            code = s.fields[1] if len(s.fields) > 1 else None
            self.last_error = OwletError(",".join(s.fields), code)
        elif s.name == "ERR":
            code = s.fields[0] if s.fields else None
            self.last_error = OwletError("ERR," + ",".join(s.fields), code)
