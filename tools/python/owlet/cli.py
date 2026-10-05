"""Command-line interface: ``owlet [-p PORT] <command> ...``."""

from __future__ import annotations

import argparse
import shlex
import sys

from .client import Owlet, OwletError, find_ports
from .protocol import Position, Sentence


def _fmt(p: Position) -> str:
    state = {"D": "disabled", "M": "moving", "H": "holding"}.get(p.state, p.state)
    return (f"pan {p.pan:6.1f}  tilt {p.tilt:6.1f}   "
            f"target {p.target_pan:6.1f} {p.target_tilt:6.1f}   {state}")


def _shell(head: Owlet) -> None:
    """Tiny REPL: type commands as words, e.g. `move 30 -15`."""
    head.on_event = lambda s: print(f"  <- {s.name} {','.join(s.fields)}")
    print(f"owlet {head.firmware_version} on {head.port}. "
          "Type commands like `enable`, `move 30 -15`, `pos`; Ctrl-D quits.")
    while True:
        try:
            line = input("owlet> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        words = shlex.split(line)
        name = words[0].upper()
        try:
            expect = name if name in ("POS", "LIMITS") else (
                "PONG" if name == "PING" else
                "SPEED" if name == "SPEED" and len(words) == 1 else "ACK")
            reply: Sentence = head.request(name, *words[1:], expect=expect)
            print(f"  {reply.name} {','.join(reply.fields)}")
        except OwletError as e:
            print(f"  ! {e}")


def _fmt_cal(c) -> str:
    rev = "reversed" if c.reversed else "normal"
    return (f"{c.axis:<4}  pulses {c.min_us}/{c.center_us}/{c.max_us} us   "
            f"limits {c.min_deg:.1f}..{c.max_deg:.1f} deg   {rev}   ({c.source})")


def _cal(head: Owlet, args) -> int:
    changing = args.limits is not None or args.pulses is not None or args.reverse is not None
    if args.target in ("save", "reset") and changing:
        print(f"`cal {args.target}` takes no --limits/--pulses/--reverse", file=sys.stderr)
        return 2
    if args.target == "save":
        head.save_calibration()
        print("calibration saved on the device")
        return 0
    if args.target == "reset":
        head.reset_calibration()
        print("calibration reset to factory values")
        return 0
    axes = [args.target] if args.target else ["pan", "tilt"]
    if changing:
        if args.target is None:
            print("choose an axis to change: owlet cal pan|tilt ...", file=sys.stderr)
            return 2
        c = head.calibration(args.target)
        mn, ct, mx = args.pulses if args.pulses else (c.min_us, c.center_us, c.max_us)
        if args.reverse is not None and (mn > mx) != args.reverse:
            mn, mx = mx, mn
        lo, hi = c.min_deg, c.max_deg
        if (mn > mx) != c.reversed:
            # The direction really changes: mirror the limits so they keep
            # guarding the same physical end stops.
            lo, hi = -hi, -lo
        if args.limits:
            lo, hi = args.limits  # explicit limits are the final ones
        head.set_calibration(args.target, mn, ct, mx, lo, hi)
    for a in axes:
        print(_fmt_cal(head.calibration(a)))
    if changing:
        print("(not saved yet: run `owlet cal save` to keep it)")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="owlet", description="Drive an owlet pan-tilt head.")
    p.add_argument("-p", "--port", help="serial port (default: auto-detect the ESP32-S3)")
    p.add_argument("-t", "--timeout", type=float, default=1.0,
                   help="seconds to wait for each reply (default 1.0)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="list candidate owlet serial ports")
    sub.add_parser("ping", help="check the link and show versions")
    en = sub.add_parser("enable", help="energise servos (optionally at a pose)")
    en.add_argument("pose", nargs="*", type=float, metavar="DEG")
    sub.add_parser("disable", help="servos off (limp)")
    for name, help_ in (("move", "absolute move"), ("nudge", "relative move")):
        m = sub.add_parser(name, help=help_)
        m.add_argument("pan", type=float)
        m.add_argument("tilt", type=float)
        m.add_argument("-w", "--wait", action="store_true", help="wait until arrived")
    h = sub.add_parser("home", help="move to 0,0")
    h.add_argument("-w", "--wait", action="store_true")
    sub.add_parser("stop", help="freeze at the current position")
    sp = sub.add_parser("speed", help="get or set slew speed (deg/s)")
    sp.add_argument("value", nargs="?", type=float)
    sub.add_parser("pos", help="print current position")
    sub.add_parser("limits", help="print soft limits")
    v = sub.add_parser("vel", help="velocity mode: turn at PAN TILT deg/s (watchdog applies)")
    v.add_argument("pan", type=float)
    v.add_argument("tilt", type=float)
    wd = sub.add_parser("watchdog", help="get or set the velocity watchdog (ms, 0 = off)")
    wd.add_argument("ms", nargs="?", type=int)
    sub.add_parser("clock", help="print the device clock (ms since boot)")
    sub.add_parser("caps", help="print the head's capabilities")
    sub.add_parser("diag", help="print what the motor drivers report: faults, load, current")
    st = sub.add_parser("selftest", help="exercise the motors inside the head's limits and report")
    st.add_argument("-y", "--yes", action="store_true",
                    help="don't ask before moving or for a visual check at the end")
    st.add_argument("--span", type=float, default=0.5, metavar="FRACTION",
                    help="how far to travel, as a fraction of each soft limit (default 0.5)")
    st.add_argument("--speeds", type=float, nargs="+", default=[0.25, 0.5, 1.0], metavar="FRACTION",
                    help="speeds to sweep at, as fractions of the head's top speed")
    st.add_argument("--axis", choices=["pan", "tilt", "both"], default="both")
    cal = sub.add_parser("cal", help="show or change the calibration stored on the device")
    cal.add_argument("target", nargs="?", choices=["pan", "tilt", "save", "reset"],
                     help="axis to show/change, or save/reset (default: show both axes)")
    cal.add_argument("--limits", nargs=2, type=float, metavar=("MIN", "MAX"),
                     help="soft limits in degrees")
    cal.add_argument("--pulses", nargs=3, type=int, metavar=("MIN_US", "CENTER_US", "MAX_US"),
                     help="pulse widths at -90/0/+90 degrees (servos must be disabled)")
    rev = cal.add_mutually_exclusive_group()
    rev.add_argument("--reverse", dest="reverse", action="store_const", const=True,
                     help="make + go the other way (servos must be disabled)")
    rev.add_argument("--no-reverse", dest="reverse", action="store_const", const=False)
    w = sub.add_parser("watch", help="stream positions until Ctrl-C")
    w.add_argument("--hz", type=int, default=10)
    sub.add_parser("shell", help="interactive prompt")
    return p


def _selftest(head: Owlet, args) -> int:
    from .selftest import SelfTest, format_check, summary

    def ask(question: str) -> bool:
        return input(f"\n{question} [y/N] ").strip().lower() in ("y", "yes")

    axes = ("pan", "tilt") if args.axis == "both" else (args.axis,)
    try:
        test = SelfTest(head, span=args.span, speeds=args.speeds, axes=axes,
                        ask=None if args.yes else ask,
                        report=lambda c: print(format_check(c), flush=True))
    except ValueError as e:
        print(f"owlet: {e}", file=sys.stderr)
        return 2
    print(summary(test.run()))
    return 0 if test.passed else 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.cmd == "ports":
        for port in find_ports():
            print(port)
        return 0

    try:
        with Owlet(args.port, timeout=args.timeout) as head:
            if args.cmd == "ping":
                print(f"owlet {head.firmware_version} "
                      f"(protocol v{head.protocol_version}) on {head.port}")
            elif args.cmd == "enable":
                if len(args.pose) not in (0, 2):
                    print("enable takes no pose or PAN TILT", file=sys.stderr)
                    return 2
                head.enable(*args.pose)
            elif args.cmd == "disable":
                head.disable()
            elif args.cmd in ("move", "nudge"):
                fn = head.move if args.cmd == "move" else head.nudge
                final = fn(args.pan, args.tilt, wait=args.wait)
                if final:
                    print(_fmt(final))
            elif args.cmd == "home":
                final = head.home(wait=args.wait)
                if final:
                    print(_fmt(final))
            elif args.cmd == "stop":
                head.stop()
            elif args.cmd == "speed":
                print(f"{head.speed(args.value):.1f} deg/s")
            elif args.cmd == "pos":
                print(_fmt(head.position()))
            elif args.cmd == "limits":
                pmin, pmax, tmin, tmax = head.limits()
                print(f"pan {pmin:.1f}..{pmax:.1f}   tilt {tmin:.1f}..{tmax:.1f}")
            elif args.cmd == "vel":
                head.velocity(args.pan, args.tilt)
            elif args.cmd == "watchdog":
                print(f"{head.watchdog(args.ms)} ms")
            elif args.cmd == "clock":
                print(f"{head.clock()} ms")
            elif args.cmd == "caps":
                c = head.caps()
                print(f"speed {c.speed_min:.1f}..{c.speed_max:.1f} deg/s   "
                      f"velocity max {c.velocity_max:.1f} deg/s")
                print(f"watchdog {c.watchdog_min_ms}..{c.watchdog_max_ms} ms (0 = off)   "
                      f"stream max {c.stream_max_hz} Hz")
            elif args.cmd == "diag":
                for axis in ("pan", "tilt"):
                    d = head.diag(axis)
                    if not d.available:
                        print(f"{axis:5s} no driver data (no data link, or the motor supply is off)")
                        continue
                    faults = ", ".join(d.fault_names) or "none"
                    print(f"{axis:5s} current {d.current_ma} mA   load {d.load}   "
                          f"{'STALLED' if d.stalled else 'not stalled'}   faults: {faults}")
            elif args.cmd == "selftest":
                return _selftest(head, args)
            elif args.cmd == "cal":
                return _cal(head, args)
            elif args.cmd == "watch":
                try:
                    for p in head.watch(args.hz):
                        print(_fmt(p), flush=True)
                except KeyboardInterrupt:
                    pass
            elif args.cmd == "shell":
                _shell(head)
    except OwletError as e:
        print(f"owlet: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
