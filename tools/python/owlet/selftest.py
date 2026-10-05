"""Motor self-test: exercise a head inside its own limits and say what works.

    owlet selftest            # asks before moving and at the end
    owlet selftest --yes      # no questions (for scripts)

It reads the head's capabilities and soft limits first and scales everything
from them, so the same test suits the servo head, the stepper head and the
simulator. Nothing is hard-coded to one mechanism. The head is always left
disabled, whatever happens.

Without home switches the head cannot know its absolute position: zero is
wherever it was when enabled. The test therefore ends by asking a person
whether the head is back on its marks, which is the only evidence of lost
steps there is. When switches are fitted, that question becomes a measurement
(see the "home switches" check).
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import Callable, Sequence

from .client import Owlet, OwletError

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"

# Driver fault bits ($DIAG) that mean "stop and look", versus worth a mention.
_SERIOUS_FAULTS = 2 | 4 | 8          # over-temperature shutdown, short on a coil
_OPEN_COIL_FAULTS = 16 | 32          # unreliable at low speed and current: reported, not failed


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str = ""


class SelfTest:
    """Runs the checks against an open :class:`Owlet` and collects the results."""

    POSITION_TOLERANCE_DEG = 0.2
    MIN_SPAN_DEG = 2.0

    def __init__(self, head: Owlet, *, span: float = 0.5,
                 speeds: Sequence[float] = (0.25, 0.5, 1.0),
                 axes: Sequence[str] = ("pan", "tilt"),
                 ask: Callable[[str], bool] | None = None,
                 report: Callable[[Check], None] | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        """``span`` and ``speeds`` are fractions of the head's own limits and top
        speed. ``ask`` puts a yes/no question to a person; without it the test
        never waits for one and marks those checks as not done. ``clock`` and
        ``sleep`` exist so tests can run on simulated time."""
        if not 0.0 < span <= 1.0:
            raise ValueError("span must be a fraction in (0, 1]")
        if not speeds or any(not 0.0 < s <= 1.0 for s in speeds):
            raise ValueError("speeds must be fractions in (0, 1]")
        self.head = head
        self.span = span
        self.speeds = tuple(speeds)
        self.axes = tuple(axes)
        self.ask = ask
        self._report = report
        self._clock = clock
        self._sleep = sleep
        self.checks: list[Check] = []
        self._has_diag = True

    # ── running ───────────────────────────────────────────────────────────

    def run(self) -> list[Check]:
        try:
            self._run()
        except OwletError as e:
            self._add("self-test", FAIL, f"stopped: {e}")
        finally:
            try:
                self.head.stop()
            except OwletError:
                pass  # refused when already disabled: nothing to stop
            try:
                self.head.disable()
                self._add("disable", PASS, "motors off")
            except OwletError as e:
                self._add("disable", FAIL, f"could not disable: {e}")
        return self.checks

    @property
    def passed(self) -> bool:
        return not any(c.status == FAIL for c in self.checks)

    def _run(self) -> None:
        proto, firmware = self.head.ping()
        self._add("link", PASS, f"owlet {firmware}, protocol v{proto}, on {self.head.port}")

        caps = self.head.caps()
        pmin, pmax, tmin, tmax = self.head.limits()
        self._add("ranges", PASS,
                  f"pan {pmin:g}..{pmax:g}, tilt {tmin:g}..{tmax:g} deg; "
                  f"speed up to {caps.speed_max:g} deg/s")
        reach = {"pan": self.span * min(abs(pmin), abs(pmax)),
                 "tilt": self.span * min(abs(tmin), abs(tmax))}

        self._check_drivers("drivers at rest")

        if self.ask is not None and not self.ask(
                "The head will now move: pan up to "
                f"±{reach['pan']:.0f} and tilt up to ±{reach['tilt']:.0f} degrees, at up to "
                f"{max(self.speeds) * caps.speed_max:.0f} deg/s. Is it centred and clear to move?"):
            self._add("motion", SKIP, "not confirmed, so nothing was moved")
            return

        self.head.enable()
        state = self.head.position().state
        self._add("enable", PASS if state == "H" else FAIL, f"state {state} after enable")

        for axis in self.axes:
            if reach[axis] < self.MIN_SPAN_DEG:
                self._add(f"{axis} sweep", SKIP, "its limits leave no room to move")
                continue
            for fraction in self.speeds:
                speed = max(caps.speed_min, fraction * caps.speed_max)
                self._sweep(f"{axis} sweep at {speed:g} deg/s", speed,
                            [self._pose(axis, reach[axis]), self._pose(axis, -reach[axis]), (0.0, 0.0)])

        if all(reach[a] >= self.MIN_SPAN_DEG for a in ("pan", "tilt")) and len(self.axes) == 2:
            speed = max(caps.speed_min, 0.5 * caps.speed_max)
            p, t = reach["pan"], reach["tilt"]
            self._sweep(f"both axes together at {speed:g} deg/s", speed,
                        [(p, t), (-p, -t), (p, -t), (-p, t), (0.0, 0.0)])

        self._check_watchdog(caps, reach)
        self._check_limit_is_enforced(pmax)
        self._check_drivers("drivers after the run")
        self._finish_at_home()
        self._add("home switches", SKIP,
                  "none fitted: zero is the pose at enable, so lost steps can only be seen by eye")

    # ── checks ────────────────────────────────────────────────────────────

    def _sweep(self, name: str, speed: float, poses: Sequence[tuple[float, float]]) -> None:
        self.head.speed(speed)
        times, loads, problems = [], [], []
        here = self.head.position()
        at = (here.pan, here.tilt)
        for pose in poses:
            distance = max(abs(pose[0] - at[0]), abs(pose[1] - at[1]))
            cruise = distance / speed
            elapsed, final, seen = self._move_and_watch(pose, timeout=cruise * 3.0 + 5.0)
            loads += seen
            if final is None:
                problems.append(f"never arrived at {pose[0]:g}, {pose[1]:g}")
                break
            times.append(elapsed)
            if (abs(final.pan - pose[0]) > self.POSITION_TOLERANCE_DEG or
                    abs(final.tilt - pose[1]) > self.POSITION_TOLERANCE_DEG):
                problems.append(f"stopped at {final.pan:g}, {final.tilt:g}, not {pose[0]:g}, {pose[1]:g}")
            # A head may take longer than distance / speed (it ramps), never much less.
            if distance > 1.0 and elapsed < 0.7 * cruise - 0.1:
                problems.append(f"{distance:g} deg took {elapsed:.2f} s: faster than the set speed allows")
            at = pose
        detail = "legs " + ", ".join(f"{t:.2f} s" for t in times) if times else ""
        if loads:
            detail += f"; load {min(loads)}..{max(loads)}, median {int(statistics.median(loads))}"
        if problems:
            self._add(name, FAIL, "; ".join(problems) + (f" ({detail})" if detail else ""))
        else:
            self._add(name, PASS, detail)

    def _move_and_watch(self, pose: tuple[float, float], timeout: float):
        """Start a move and watch it: (seconds taken, final position or None, load samples)."""
        loads: list[int] = []
        start = self._clock()
        self.head.move(pose[0], pose[1])
        while self._clock() - start < timeout:
            position = self.head.position()
            if position.state != "M":
                return self._clock() - start, position, loads
            for axis, moving in (("pan", position.pan != position.target_pan),
                                 ("tilt", position.tilt != position.target_tilt)):
                if moving:
                    d = self._diag(axis)
                    if d is not None and d.available:
                        loads.append(d.load)
            self._sleep(0.02)
        self.head.stop()
        return self._clock() - start, None, loads

    def _check_watchdog(self, caps, reach: dict[str, float]) -> None:
        axis = next((a for a in self.axes if reach[a] >= self.MIN_SPAN_DEG), None)
        if axis is None:
            self._add("velocity mode and watchdog", SKIP, "no axis has room to move")
            return
        period_ms = max(caps.watchdog_min_ms, 300)
        rate = min(20.0, 0.2 * caps.velocity_max, reach[axis])   # slow: it may run until the limit
        before = self.head.watchdog()
        start = self.head.position()
        self.head.watchdog(period_ms)
        try:
            self.head.velocity(*self._pose(axis, rate))
            self._sleep(period_ms / 1000.0 + 1.0)   # send nothing: the head must stop by itself
            after = self.head.position()
        finally:
            self.head.stop()
            self.head.watchdog(before)
        moved = abs((after.pan - start.pan) if axis == "pan" else (after.tilt - start.tilt))
        expected = rate * period_ms / 1000.0
        if after.state == "M":
            self._add("velocity mode and watchdog", FAIL,
                      f"still moving {period_ms + 1000} ms after the last command: the watchdog did not stop it")
        elif moved < 0.2 * expected:
            self._add("velocity mode and watchdog", FAIL,
                      f"moved {moved:.1f} deg, expected about {expected:.1f}: velocity mode did not run")
        else:
            self._add("velocity mode and watchdog", PASS,
                      f"ran at {rate:g} deg/s and stopped itself after {period_ms} ms ({moved:.1f} deg)")
        self.head.move(0.0, 0.0, wait=True, timeout=30.0)

    def _check_limit_is_enforced(self, pan_max: float) -> None:
        before = self.head.position()
        try:
            self.head.move(pan_max + 5.0, 0.0)
        except OwletError as e:
            after = self.head.position()
            if e.code == "RNG" and after.target_pan == before.target_pan:
                self._add("soft limit", PASS, f"a move to {pan_max + 5:g} deg was refused")
            else:
                self._add("soft limit", FAIL, f"unexpected answer to an out-of-range move: {e}")
            return
        self.head.stop()
        self._add("soft limit", FAIL, f"a move to {pan_max + 5:g} deg, beyond the limit, was accepted")

    def _check_drivers(self, name: str) -> None:
        notes, status = [], PASS
        for axis in ("pan", "tilt"):
            d = self._diag(axis)
            if d is None:
                self._add(name, SKIP, "this firmware has no $DIAG")
                return
            if not d.available:
                notes.append(f"{axis}: no driver data")
                status = SKIP if status == PASS else status
                continue
            notes.append(f"{axis}: {d.current_ma} mA" + (f", {', '.join(d.fault_names)}" if d.faults else ""))
            if d.faults & _SERIOUS_FAULTS:
                status = FAIL
            elif d.faults & ~_OPEN_COIL_FAULTS and status != FAIL:
                status = WARN
        self._add(name, status, "; ".join(notes))

    def _finish_at_home(self) -> None:
        final = self.head.move(0.0, 0.0, wait=True, timeout=30.0)
        if abs(final.pan) > self.POSITION_TOLERANCE_DEG or abs(final.tilt) > self.POSITION_TOLERANCE_DEG:
            self._add("return home", FAIL, f"reports {final.pan:g}, {final.tilt:g} instead of 0, 0")
        elif self.ask is None:
            self._add("return home", PASS, "reports 0, 0 (not checked by eye)")
        elif self.ask("Is the head back exactly on its centre marks?"):
            self._add("return home", PASS, "reports 0, 0 and is on its marks")
        else:
            self._add("return home", FAIL,
                      "reports 0, 0 but is off its marks: steps were lost, or something slipped")

    # ── helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _pose(axis: str, value: float) -> tuple[float, float]:
        return (value, 0.0) if axis == "pan" else (0.0, value)

    def _diag(self, axis: str):
        if not self._has_diag:
            return None
        try:
            return self.head.diag(axis)
        except OwletError:
            self._has_diag = False   # older firmware: ask once, not on every sample
            return None

    def _add(self, name: str, status: str, detail: str = "") -> None:
        check = Check(name, status, detail)
        self.checks.append(check)
        if self._report is not None:
            self._report(check)


def format_check(check: Check) -> str:
    return f"  {check.status:4s}  {check.name}" + (f": {check.detail}" if check.detail else "")


def summary(checks: Sequence[Check]) -> str:
    counts = {s: sum(1 for c in checks if c.status == s) for s in (PASS, FAIL, WARN, SKIP)}
    verdict = "FAILED" if counts[FAIL] else "passed"
    return (f"self-test {verdict}: {counts[PASS]} passed, {counts[FAIL]} failed, "
            f"{counts[WARN]} warnings, {counts[SKIP]} skipped")
