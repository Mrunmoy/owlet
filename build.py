#!/usr/bin/env python3
"""Build orchestrator for owlet firmware.

Wraps the host-side test build (CMake + Ninja + GTest) and the Raspberry Pi
Pico firmware build so that a freshly cloned repo only needs:

    nix develop
    ./build.py --host -t

and, once hardware is ready:

    ./build.py --pico --flash           # finds the Pico and loads it with picotool
"""

from __future__ import annotations

import argparse
import glob
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
HOST_BUILD_DIR = REPO_ROOT / "build" / "host"
PICO_APP_DIR = REPO_ROOT / "apps" / "pico-owlet"
PICO_BUILD_DIR = PICO_APP_DIR / "build"


def run(cmd: str, cwd: Path | None = None) -> None:
    print(f"+ {cmd}", flush=True)
    rc = subprocess.call(cmd, shell=True, cwd=str(cwd) if cwd else None)
    if rc != 0:
        sys.exit(rc)


def build_host(run_tests: bool, clean: bool) -> None:
    if clean and HOST_BUILD_DIR.exists():
        run(f"rm -rf {shlex.quote(str(HOST_BUILD_DIR))}")
    run(
        f"cmake -S {shlex.quote(str(REPO_ROOT))} "
        f"-B {shlex.quote(str(HOST_BUILD_DIR))} "
        f"-G Ninja -DOWLET_BUILD_TESTS=ON"
    )
    run(f"cmake --build {shlex.quote(str(HOST_BUILD_DIR))} --parallel")
    if run_tests:
        run(
            f"ctest --test-dir {shlex.quote(str(HOST_BUILD_DIR))} "
            f"--output-on-failure"
        )


RASPBERRY_PI_VID = "2e8a"
PICO_BOOTSEL_PIDS = ("0003",)   # RP2040 ROM bootloader (the RPI-RP2 drive)


def pico_usb_state() -> str:
    """'bootsel', 'running' (some firmware is up) or 'absent', from the USB bus."""
    out = subprocess.run(["lsusb", "-d", f"{RASPBERRY_PI_VID}:"],
                         capture_output=True, text=True).stdout.lower()
    if any(f"{RASPBERRY_PI_VID}:{pid}" in out for pid in PICO_BOOTSEL_PIDS):
        return "bootsel"
    return "running" if RASPBERRY_PI_VID in out else "absent"


def wait_for_pico(states: tuple[str, ...], seconds: float) -> str:
    deadline = time.monotonic() + seconds
    while True:
        state = pico_usb_state()
        if state in states or time.monotonic() >= deadline:
            return state
        time.sleep(0.25)


def pico_serial_ports() -> list[str]:
    return sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi*") +
                  glob.glob("/dev/serial/by-id/usb-MicroPython*"))


def ask_pico_for_bootsel() -> None:
    """Get a running Pico into the bootloader without touching the button.

    Tried in order, each harmless if the firmware doesn't understand it:
      1. picotool's reboot request (owlet and anything else built on
         pico-sdk stdio_usb answers it);
      2. opening the serial port at 1200 baud (the same firmware's fallback);
      3. MicroPython's machine.bootloader() typed at its prompt.
    """
    if subprocess.call("picotool reboot -f -u", shell=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
        if wait_for_pico(("bootsel",), 5) == "bootsel":
            return
    try:
        import serial  # pyserial, provided by the dev shell
    except ImportError:
        return
    for port in pico_serial_ports():
        try:
            serial.Serial(port, 1200).close()
        except (OSError, serial.SerialException):
            continue
        if wait_for_pico(("bootsel",), 3) == "bootsel":
            return
    for port in pico_serial_ports():
        try:
            with serial.Serial(port, 115200, timeout=0.2) as repl:
                repl.write(b"\x03\x03\r\nimport machine\r\nmachine.bootloader()\r\n")
        except (OSError, serial.SerialException):
            continue
        if wait_for_pico(("bootsel",), 3) == "bootsel":
            return


def flash_pico(uf2: Path, wait_seconds: float) -> None:
    state = pico_usb_state()
    if state == "absent" and wait_seconds > 0:
        print(f"waiting up to {wait_seconds:.0f} s for a Pico on USB...", flush=True)
        state = wait_for_pico(("bootsel", "running"), wait_seconds)
        time.sleep(1.0)  # let the serial port appear
    if state == "absent":
        sys.exit("no Raspberry Pi Pico found on USB. Plug it in and run this again.")
    if state == "running":
        print("asking the running firmware to reboot into the bootloader...", flush=True)
        ask_pico_for_bootsel()
        state = wait_for_pico(("bootsel",), 2)
    if state != "bootsel":
        sys.exit(
            "the Pico is running firmware that ignores every reboot request.\n"
            "This one time, hold BOOTSEL while plugging it in, then run this again.\n"
            "Once owlet is on the board, flashing never needs the button.")
    run(f"picotool load -x {shlex.quote(str(uf2))}")
    if wait_for_pico(("running",), 10) == "running":
        time.sleep(1.0)
        ports = pico_serial_ports()
        print("owlet is running" + (f" on {ports[0]}" if ports else ""))


def build_pico(flash: bool, clean: bool, wait_seconds: float) -> None:
    if not os.environ.get("PICO_SDK_PATH"):
        sys.exit("PICO_SDK_PATH is not set; run inside `nix develop`")
    if clean and PICO_BUILD_DIR.exists():
        run(f"rm -rf {shlex.quote(str(PICO_BUILD_DIR))}")
    run(
        f"cmake -S {shlex.quote(str(PICO_APP_DIR))} "
        f"-B {shlex.quote(str(PICO_BUILD_DIR))} -G Ninja -DCMAKE_BUILD_TYPE=Release"
    )
    run(f"cmake --build {shlex.quote(str(PICO_BUILD_DIR))} --parallel")
    uf2 = PICO_BUILD_DIR / "pico_owlet.uf2"
    print(f"firmware: {uf2}")
    if flash:
        flash_pico(uf2, wait_seconds)


def build_docs() -> None:
    """API reference from the headers. Any Doxygen warning fails the build."""
    (REPO_ROOT / "build" / "docs").mkdir(parents=True, exist_ok=True)
    run("doxygen Doxyfile", cwd=REPO_ROOT)
    print(f"docs: {REPO_ROOT / 'build' / 'docs' / 'html' / 'index.html'}")


def main() -> None:
    p = argparse.ArgumentParser(description="owlet build orchestrator")
    p.add_argument("--host", action="store_true",
                   help="build host-side unit tests")
    p.add_argument("--pico", action="store_true",
                   help="build Raspberry Pi Pico firmware")
    p.add_argument("--docs", action="store_true",
                   help="build the API reference with Doxygen into build/docs/html")
    p.add_argument("-t", "--test", action="store_true",
                   help="run host tests after building")
    p.add_argument("--flash", action="store_true",
                   help="load the Pico firmware just built (implies --pico). The board "
                        "is found and rebooted into its bootloader automatically")
    p.add_argument("--wait", metavar="SECONDS", type=float, default=0,
                   help="with --flash: wait this long for the board to be plugged in")
    p.add_argument("--clean", action="store_true",
                   help="clean build output before building")
    args = p.parse_args()

    if args.flash:
        args.pico = True

    if not (args.host or args.pico or args.docs):
        p.error("specify --host, --pico, --docs, or a combination")

    if args.docs:
        build_docs()
    if args.host:
        build_host(run_tests=args.test, clean=args.clean)
    if args.pico:
        build_pico(flash=args.flash, clean=args.clean, wait_seconds=args.wait)


if __name__ == "__main__":
    main()
