#!/usr/bin/env python3
"""Build orchestrator for owlet firmware.

Wraps the host-side test build (CMake + Ninja + GTest) and the ESP-IDF
firmware build so that a freshly cloned repo only needs:

    nix develop
    ./build.py --host -t

and, once hardware is ready:

    ./build.py --esp --flash            # auto-detects the Espressif USB port
"""

from __future__ import annotations

import argparse
import glob
import os
import shlex
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
HOST_BUILD_DIR = REPO_ROOT / "build" / "host"
ESP_APP_DIR = REPO_ROOT / "apps" / "esp32-owlet"


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


def find_esp_port() -> str:
    """Return the single Espressif USB-Serial-JTAG port, by stable by-id path."""
    ports = sorted(glob.glob("/dev/serial/by-id/usb-Espressif*"))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        sys.exit("no Espressif USB device found; pass the port explicitly: --flash PORT")
    sys.exit("several Espressif devices found; pick one with --flash PORT:\n  "
             + "\n  ".join(ports))


def build_esp(flash_port: str | None, clean: bool) -> None:
    if not ESP_APP_DIR.exists():
        sys.exit(f"ESP app directory missing: {ESP_APP_DIR}")
    if clean:
        run("idf.py fullclean", cwd=ESP_APP_DIR)
    # Target is pinned via sdkconfig.defaults; no need for `idf.py set-target`
    # which would fullclean on every call.
    run("idf.py build", cwd=ESP_APP_DIR)
    if flash_port:
        port = find_esp_port() if flash_port == "auto" else flash_port
        # No `monitor`: the USB port carries the protocol, not logs. Use the
        # owlet CLI to talk to it; logs are on UART0 (GP43).
        run(f"idf.py -p {shlex.quote(port)} flash", cwd=ESP_APP_DIR)


def main() -> None:
    p = argparse.ArgumentParser(description="owlet build orchestrator")
    p.add_argument("--host", action="store_true",
                   help="build host-side unit tests")
    p.add_argument("--esp", action="store_true",
                   help="build ESP32-S3 firmware")
    p.add_argument("-t", "--test", action="store_true",
                   help="run host tests after building")
    p.add_argument("--flash", metavar="PORT", nargs="?", const="auto",
                   help="flash ESP firmware (PORT defaults to the auto-detected "
                        "Espressif device)")
    p.add_argument("--clean", action="store_true",
                   help="clean build output before building")
    args = p.parse_args()

    if args.flash:
        args.esp = True

    if not (args.host or args.esp):
        p.error("specify --host, --esp, or both")

    if args.host:
        build_host(run_tests=args.test, clean=args.clean)
    if args.esp:
        build_esp(flash_port=args.flash, clean=args.clean)


if __name__ == "__main__":
    main()
