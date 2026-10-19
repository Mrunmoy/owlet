# Getting started

From a fresh clone to a moving head. Every command here is meant to be copied as it is.

- [The short version](#the-short-version)
- [1. What you need](#1-what-you-need)
- [2. Set up the PC (once)](#2-set-up-the-pc-once)
- [3. Prove the software works, with no hardware](#3-prove-the-software-works-with-no-hardware)
- [4. Put the firmware on the head](#4-put-the-firmware-on-the-head)
- [5. First contact](#5-first-contact)
- [6. Everyday use](#6-everyday-use)
- [7. When something is wrong](#7-when-something-is-wrong)
- [8. Orientation for contributors](#8-orientation-for-contributors)

## The short version

```bash
git clone https://github.com/Mrunmoy/owlet.git && cd owlet
nix develop                        # all tools; first run downloads them
sudo tools/dev/install-udev.sh     # Linux, once: USB access without sudo
tools/dev/doctor.sh                # is this machine ready?
./build.py --host -t               # build and run every test on the PC
./build.py --pico --flash          # build the firmware and put it on the Pico
owlet ping                         # is it alive?
owlet selftest                     # exercise both motors and report
```

If each line succeeds you are done. The rest of this page explains each step and what to do when one does not.

## 1. What you need

**A PC** running Linux (what the project is developed on) or macOS, with [Nix](https://nixos.org/download) and [flakes enabled](https://nixos.wiki/wiki/Flakes). Nix supplies every tool at a pinned version: compilers for the PC and the Pico, the pico-sdk, picotool, Python and its packages. Nothing else needs installing, and nothing is installed system-wide.

**Or a Raspberry Pi.** The reference head runs from a Pi 3B+ on Debian 13 (arm64), with its tools from apt instead of Nix. Install `build-essential cmake ninja-build pkg-config gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib libusb-1.0-0-dev picotool python3-serial` with apt, clone [pico-sdk](https://github.com/raspberrypi/pico-sdk) 2.2.0 with its submodules, and set `PICO_SDK_PATH` to it. Then `./build.py --host -t`, `./build.py --pico` and `./build.py --pico --flash` run directly. A udev rule for vendor ID `2e8a` lets picotool run without sudo. Expect about 4.5 minutes for the tests and 40 s for the firmware on a Pi 3.

**The head**, or nothing if you only want to run the simulator:

| | |
|---|---|
| Board | Raspberry Pi Pico (RP2040) |
| Motors | 2× NEMA 17 on TMC2209 drivers, 3:1 belts |
| Motor supply | 12 V, 2 A |
| Parts and wiring | [README](../README.md#hardware), [`wiring/pico-tmc2209/`](wiring/pico-tmc2209/README.md), [`mechanical/stepper-head/`](../mechanical/stepper-head/README.md) |

A USB data cable from the board to the PC carries both the firmware and the commands.

## 2. Set up the PC (once)

```bash
git clone https://github.com/Mrunmoy/owlet.git
cd owlet
nix develop
```

The first `nix develop` downloads the toolchains and takes several minutes. After that it starts instantly. **Run every other command on this page inside that shell**, from the repository's top directory.

On Linux, give your user access to the Pico's USB port. Without this, flashing the Pico fails with a permission error:

```bash
sudo tools/dev/install-udev.sh
```

Then check the machine:

```bash
tools/dev/doctor.sh
```

It lists every tool and says `Everything is in place.` or names what is missing. Warnings about USB permissions or `connected heads: none found` only matter once you plug the head in.

## 3. Prove the software works, with no hardware

```bash
./build.py --host -t
```

This builds the firmware's logic for the PC and runs every test: the C++ unit and integration tests, and the Python client tests, some of which drive the real firmware core through a simulated head. It ends with `100% tests passed`.

You can also drive that simulated head by hand. It is the same core the Pico runs, with the reference head's speeds and limits, behind a fake serial port:

```bash
build/host/apps/host-sim/owlet_sim --link /tmp/owlet &
owlet -p /tmp/owlet ping
owlet -p /tmp/owlet enable
owlet -p /tmp/owlet move 30 -15 --wait
owlet -p /tmp/owlet selftest --yes
kill %1
```

## 4. Put the firmware on the head

Plug the Pico into the PC with a USB data cable.

```bash
./build.py --pico --flash
```

No button is needed. The script finds the board and asks whatever is running on it to reboot into its bootloader: owlet does, and so does MicroPython. A brand-new Pico is already in its bootloader. Only a board running some other firmware that ignores the request needs its `BOOTSEL` button held while you plug it in, once. `--wait 60` makes the script wait for you to plug the board in.

It ends by telling you the serial port the head is on. The first build takes a few minutes; later ones take seconds.

> Flashing restarts the firmware, and the head takes wherever it is at start-up as its home position. Send it home (`owlet home --wait`) before you flash, or centre it by hand afterwards.

## 5. First contact

The motors are **off** after power-up or flashing. Nothing moves until you enable them.

```bash
owlet ping          # owlet 0.5.0 (protocol v3) on /dev/ttyACM0
owlet caps          # what this head can do
owlet limits        # how far each axis may travel
```

The head has no position sensor yet, so put both axes in their zero pose by hand before enabling (centred, or on the reference rig: pan centred and tilt pointing straight up). Wherever it is when enabled becomes 0, 0. A pen mark across each joint makes this quick. Switch the 12 V supply on first.

Then let the head test itself. It asks before it moves:

```bash
owlet selftest
```

It sweeps each axis at three speeds, then both together, checks the safety stops, and prints a pass or fail for each step. See the [README](../README.md#self-test) for a sample report. If it passes and the head is back on its marks, the build is good.

By hand, the essentials are:

```bash
owlet enable
owlet move 30 -15 --wait
owlet pos
owlet home --wait
owlet disable
```

## 6. Everyday use

| I want to… | Command |
|---|---|
| see every command | `owlet --help` |
| type commands interactively | `owlet shell` |
| watch the position live | `owlet watch` |
| check driver current, load and faults | `owlet diag` |
| change how far an axis may travel, and keep it | `owlet cal pan --limits -80 80` then `owlet cal save` |
| use it from Python | see [Python library](../README.md#python-library) |
| use it from another language | see [the wire protocol](../README.md#any-other-language-the-wire-protocol) |
| rebuild and reflash after a change | `./build.py --host -t && ./build.py --pico --flash` |

Positive pan turns the camera to its own right; positive tilt points it up.

## 7. When something is wrong

| Symptom | Likely cause and fix |
|---|---|
| `nix: command not found`, or flakes are refused | Install Nix and enable flakes (links in section 1). |
| A build cannot find a file you just added | Nix only sees files git knows about: `git add` it. |
| `doctor.sh` reports missing tools | You are outside `nix develop`. |
| `could not auto-detect the owlet port (found none)` | Board unplugged, a charge-only cable, or no USB permission (`sudo tools/dev/install-udev.sh`, then replug). |
| `could not auto-detect … (found [two ports])` | More than one head is connected: add `-p /dev/ttyACM1`. |
| `no answer from owlet` right after it worked | Another program has the port open. Only one client can talk to a head at a time. |
| Pico flash: `ignores every reboot request` | It runs firmware that cannot be asked. Hold `BOOTSEL` while plugging in, once. |
| `owlet: MOVE refused: DIS` | The head is disabled: `owlet enable` first. |
| `owlet: MOVE refused: RNG` | The target is outside the soft limits: `owlet limits`. |
| `owlet diag` says `no driver data` | The 12 V motor supply is off, or the drivers' UART wires are missing. The head still works without them. |
| An axis holds firmly but never turns | Its `STEP` wire is not reaching the driver. Check it against the [wiring diagram](wiring/pico-tmc2209/wiring.png); swapped `STEP`/`DIR` gives exactly this. |
| An axis turns the wrong way | Flip `kPanReversed` / `kTiltReversed` in `apps/pico-owlet/board.hpp` and reflash. |
| The head ends up off its marks | Steps were lost: lower the speed or acceleration in `apps/pico-owlet/board.hpp`, or check for something binding. Then re-centre and re-enable. |
| The Pico's on-board LED is not flashing once a second | The firmware is not running: reflash. |

## 8. Orientation for contributors

For anyone, or anything, about to change the code.

**Where things are**

| Path | What it holds |
|---|---|
| `drivers/owlet_hal/` | Interfaces only: what the core needs from hardware |
| `drivers/owlet_stepper/` | The stepper axis and the TMC2209 driver, written against those interfaces |
| `firmware/common/owlet_core/` | Motion policy, control loop, calibration: no hardware, no protocol |
| `firmware/common/owlet_text/` | The text protocol |
| `platform/owlet_pico/` | The only code that touches the pico-sdk |
| `apps/pico-owlet/`, `apps/host-sim/` | `main.cpp` files that build objects and connect them, and nothing else. The head's pins, motors, speeds and limits are in `apps/pico-owlet/board.hpp` |
| `tests/`, `tools/python/tests/` | C++ and Python tests |
| `tools/python/owlet/` | The PC client and `owlet` command |
| `docs/DESIGN.md` | Architecture, protocol reference, behaviour |

**The three places to extend it**, each a class or a folder, with nothing else changing: a motor type (`hal::IAxis`), a board (`platform/owlet_<board>/` plus a wiring-only `main.cpp`), a transport (`hal::IByteSource` / `IByteSink`). [`DESIGN.md` §4.7](DESIGN.md) walks through each.

**Rules the code keeps**

- Everything under `drivers/` and `firmware/common/` builds and is tested on the PC: no vendor headers, no heap, no exceptions.
- Tests come first. A change is not finished until `./build.py --host -t` passes and `./build.py --pico` still builds.
- Behaviour lives in the tested libraries. A `main.cpp` makes no decisions.
- A change to the protocol touches four places together: `owlet_text`, the Python client, `docs/DESIGN.md` §5 and `docs/protocol_vectors.txt`.
- New C++ files are formatted with the repository's `.clang-format`. Naming and layout follow the existing code.
- Every hardware check records what was and was not verified in `docs/CALIBRATION.md`.

**Working with real hardware**

- The motors start disabled and stay that way until `enable`. Do not weaken that.
- Send the head home before flashing.
- One client at a time on a serial port.
- A reported position is what was commanded, not a measurement: the head has no position feedback. Only eyes or a camera can tell whether the head really moved.
- When a test will move the head, say so first and keep the first moves small and slow.
