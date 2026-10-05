#!/usr/bin/env bash
# Check that this machine is ready to build, flash and talk to an owlet head.
# Run it inside `nix develop`:
#
#   tools/dev/doctor.sh
#
# Missing tools are errors (exit 1). Things that only matter once hardware is
# connected, such as USB permissions, are warnings.

set -u

errors=0
warnings=0

ok()   { printf "  OK    %-22s %s\n" "$1" "$2"; }
bad()  { printf "  FAIL  %-22s %s\n" "$1" "$2"; errors=$((errors + 1)); }
warn() { printf "  WARN  %-22s %s\n" "$1" "$2"; warnings=$((warnings + 1)); }

need() {    # need <command> <what it is for>
    if command -v "$1" >/dev/null 2>&1; then
        ok "$1" "$2"
    else
        bad "$1" "missing ($2)"
    fi
}

echo "Build tools"
need cmake              "build system"
need ninja              "build system"
need g++                "host tests"
need python3            "client, tests, build.py"
need clang-format       "code formatting"

echo
echo "Stepper head (Raspberry Pi Pico)"
need arm-none-eabi-gcc  "Pico compiler"
need picotool           "Pico flashing"
if [[ -z "${PICO_SDK_PATH:-}" ]]; then
    bad "PICO_SDK_PATH" "not set"
elif [[ ! -f "$PICO_SDK_PATH/lib/tinyusb/src/tusb.h" ]]; then
    bad "PICO_SDK_PATH" "SDK has no TinyUSB (submodules missing): $PICO_SDK_PATH"
else
    ok "PICO_SDK_PATH" "pico-sdk with TinyUSB"
fi

echo
echo "Servo head (ESP32-S3)"
need idf.py             "ESP-IDF build"
need esptool.py         "ESP flashing"

echo
echo "Client"
if python3 -c "import serial" 2>/dev/null; then
    ok "pyserial" "serial port access"
else
    bad "pyserial" "missing (python module 'serial')"
fi
if command -v owlet >/dev/null 2>&1; then
    ok "owlet" "command-line client on PATH"
else
    bad "owlet" "not on PATH (tools/bin should be, inside nix develop)"
fi

echo
echo "This machine"
if [[ "$(uname -s)" == "Linux" ]]; then
    if grep -qs 'idVendor}=="2e8a"' /etc/udev/rules.d/*.rules 2>/dev/null; then
        ok "USB permissions" "a udev rule for the Pico is installed"
    else
        warn "USB permissions" "no udev rule for the Pico: run  sudo tools/dev/install-udev.sh"
    fi
fi
ports="$(owlet ports 2>/dev/null | tr '\n' ' ')"
if [[ -n "${ports// /}" ]]; then
    ok "connected heads" "$ports"
else
    warn "connected heads" "none found (fine if nothing is plugged in)"
fi

echo
if (( errors > 0 )); then
    echo "$errors problem(s). Are you inside 'nix develop'?" >&2
    exit 1
fi
if (( warnings > 0 )); then
    echo "Ready to build. $warnings warning(s) above matter once you connect hardware."
else
    echo "Everything is in place."
fi
