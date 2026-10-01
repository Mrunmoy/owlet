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
    w = sub.add_parser("watch", help="stream positions until Ctrl-C")
    w.add_argument("--hz", type=int, default=10)
    sub.add_parser("shell", help="interactive prompt")
    return p


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
