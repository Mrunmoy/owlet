<h1 align="center">🦉 owlet</h1>

<p align="center">
  <b>A USB pan-tilt head you can drive from any computer, in any language, built for camera tracking.</b><br>
  Raspberry Pi Pico + two belt-driven NEMA 17 steppers · one USB cable · a plain-text protocol with checksums
</p>

<p align="center">
  <a href="LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-green.svg"></a>
  <img alt="Firmware: pico-sdk 2.2" src="https://img.shields.io/badge/firmware-pico--sdk%202.2-red.svg">
  <img alt="MCU: RP2040" src="https://img.shields.io/badge/MCU-RP2040-blue.svg">
  <img alt="C++17" src="https://img.shields.io/badge/C%2B%2B-17-00599C.svg">
  <img alt="Python 3.9+" src="https://img.shields.io/badge/python-3.9%2B-3776AB.svg">
  <img alt="Status: alpha" src="https://img.shields.io/badge/status-alpha-orange.svg">
</p>

```console
$ owlet enable
$ owlet move 30 -15 --wait
pan   30.0  tilt  -15.0   target   30.0  -15.0   holding
```

<p align="center"><i>Photos and video of the head in action: to come.</i></p>

---

## Contents

- [Features](#features)
- [How it works](#how-it-works)
- [Object tracking](#object-tracking)
- [Hardware](#hardware)
- [Quick start](#quick-start)
- [Usage](#usage)
  - [Command-line tool](#command-line-tool)
  - [Python library](#python-library)
  - [Any other language: the wire protocol](#any-other-language-the-wire-protocol)
  - [No hardware? Use the simulator](#no-hardware-use-the-simulator)
- [Protocol reference](#protocol-reference)
- [Extending owlet](#extending-owlet)
- [Using owlet as a git submodule](#using-owlet-as-a-git-submodule)
- [Configuration and calibration](#configuration-and-calibration)
- [Safety](#safety)
- [Building from source](#building-from-source)
- [Project layout](#project-layout)
- [Troubleshooting](#troubleshooting)
- [Project status and roadmap](#project-status-and-roadmap)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Features

- **Plug in and go.** The board's own USB port shows up as a standard serial port (`/dev/ttyACM*`, `COMx`, `/dev/cu.usbmodem*`). No drivers, no USB-TTL adapter.
- **Human-readable, checksummed protocol.** One line per message, like `$MOVE,30,-15*3B`. You can type them in a terminal or generate them from any language, and every frame is integrity-checked.
- **Know where it is, and when.** Poll the position with `$POS` or stream it at up to 50 Hz. Every report is timestamped with the device clock, and the client maps those timestamps onto your PC's clock, so you can ask "where was the camera pointing when this frame was taken?"
- **It tells you; you don't have to ask.** The head reports what it notices as it happens: a move arriving, the watchdog firing, a motor driver raising a fault. In Python you connect a function to each (`head.arrived.connect(...)`) and it is called when the time comes. Inside the firmware the same idea holds: whatever notices a change announces it once, and nothing polls.
- **It can say what went wrong.** `owlet dtc` lists every kind of trouble since power-up, with how often, when, and what to check.
- **Self-describing.** The head reports its capabilities (`$CAPS`), its calibration (`$CAL`) and what its motor drivers say about themselves (`$DIAG`: current, load, faults), so clients adapt instead of probing. Set the limits once from the PC; they are stored on the device.
- **Made for tracking.** Velocity mode (`$VEL`) turns at the rates your controller asks for, and a **watchdog** stops the head if your program stops talking to it. A ready-made `Tracker` and pixel-to-rate controller plug straight into a YOLO/OpenCV loop.
- **Smooth, speed-limited motion.** Motion is timed from the actual elapsed time, so it doesn't speed up or slow down with loop timing. The steppers ramp up and down at a set acceleration, land exactly on the target and count every step; the step pulses themselves come from the Pico's PIO, so nothing the CPU does can jitter them.
- **Safe by default.** The motors are limp at power-up and nothing moves until you explicitly enable it.
- **Swap the motors, the board or the link.** The core talks to three small interfaces: an axis (`IAxis`), a platform (clock, serial port, storage) and a byte stream. The head that ships is a Raspberry Pi Pico with belt-driven steppers. Another motor type or board is one driver class or one platform folder, not a rewrite.
- **Pluggable protocols (SOLID).** The firmware core knows nothing about any wire format. Protocols are `IProtocol` plug-ins injected at start-up, several can share the USB port, and you can add your own (Pelco-D, G-code, binary, …) without touching owlet's code.
- **Submodule-friendly.** CMake `owlet::` targets that build on a PC and under the pico-sdk, and a `pip`-installable client.
- **Batteries-included PC side.** A `pip`-installable Python library and an `owlet` command with port auto-detection, a live position view, and an interactive shell.
- **Hardware-free development.** A host simulator runs the *real* firmware logic behind a virtual serial port.
- **Efficient.** No heap, no exceptions, fixed buffers, integer formatting, and input handed to protocols a chunk at a time. The firmware is about 142 KB.
- **Tested.** 412 automated tests: 287 C++ unit and integration tests, plus 125 Python tests, including end-to-end runs against the simulator that check latency and clock accuracy against known values, and an opt-in suite that runs on a real head. Every finding from an adversarial review has a regression test.

## How it works

```mermaid
flowchart LR
    PC["PC / Raspberry Pi<br/>owlet CLI · Python · any language"]
    PICO["Raspberry Pi Pico<br/>owlet firmware"]
    DPAN["TMC2209<br/>pan driver"]
    DTILT["TMC2209<br/>tilt driver"]
    PAN(["Pan stepper<br/>3:1 belt"])
    TILT(["Tilt stepper<br/>3:1 belt"])

    PC <-->|"USB serial · $MOVE,30,-15*3B ⇄ $ACK,MOVE*74"| PICO
    PICO -->|"STEP / DIR from PIO · UART for current and status"| DPAN
    PICO -->|"STEP / DIR from PIO · UART for current and status"| DTILT
    DPAN --> PAN
    DTILT --> TILT
```

The firmware runs a fixed-rate loop at 500 Hz. Each pass it advances both axes (position move or velocity mode, plus the watchdog), delivers whatever that step announced (an arrival, a timeout, a fault) to the parts that react to it, sends the streamed position if one is due, then reads and answers any complete command lines from USB. All of that logic is platform-independent C++ that is unit-tested on a normal PC. The board-specific code only connects it to the real USB port, clock and motor outputs. See [Hardware](#hardware).

## Object tracking

owlet is designed to be the motion half of a camera tracker: the PC runs the detector, and owlet turns the camera.

```mermaid
flowchart LR
    CAM["Webcam frame<br/>+ capture time"] --> DET["Detector<br/>e.g. YOLO"]
    DET -->|"target pixel"| CTL["PixelController<br/>pixel → angle → rate"]
    TR["Tracker<br/>pose history · clock sync"] -->|"pose at capture, pose now"| CTL
    CTL -->|"pan / tilt °/s"| TR
    TR -->|"$VEL · no waiting for ACK"| HEAD["owlet head"]
    HEAD -->|"$POS · 50 Hz, timestamped"| TR
    HEAD -.->|"no $VEL for 300 ms → stop, $TIMEOUT"| HEAD
```

```python
import time
from owlet import Owlet, PixelController, Tracker

ctl = PixelController(fov_h_deg=70, fov_v_deg=43, gain=3.0)    # your camera's field of view

with Owlet() as head:
    head.enable()
    with Tracker(head, stream_hz=50, watchdog_ms=300) as tr:   # syncs clocks, streams pose
        while True:
            frame, t_capture = grab_frame()                     # t_capture = time.monotonic()
            target = detect(frame)                              # e.g. YOLO → pixel centre
            if target is None:
                tr.set_rates(0, 0)
                continue
            pan, tilt = ctl.update(target.x, target.y, frame.width, frame.height,
                                   pose_at_capture=tr.pose_at(t_capture),
                                   pose_now=tr.pose_at(time.monotonic()))
            tr.set_rates(pan, tilt)                             # $VEL, no waiting for the ACK
```

| Piece | What it does |
|-------|--------------|
| `$VEL,<pan>,<tilt>` | Turn at signed rates (°/s). A new command replaces the old one immediately; nothing queues. |
| Watchdog (`$WATCHDOG`) | If no `VEL` arrives within the timeout (default 500 ms), the head stops by itself and reports `$TIMEOUT,VEL`. A crashed tracker can't leave the head spinning. |
| Timestamped `$POS` | Each streamed pose carries the device time it was computed. |
| `Tracker` | Background reader that syncs the device clock (minimum round-trip sample of 32 randomly spaced `$CLOCK`s, refreshed every second; within ~1 ms on the reference head), keeps a pose history and sends rates fire-and-forget. If the link breaks or the device reboots, `tr.failed` is set, `tr.last_error` says why, and `set_rates()` raises rather than failing silently. |
| `PixelController` | Pixel offset → angle (pinhole geometry) → rate (gain, deadband, rate limit). Given the pose at capture and the pose now, it subtracts the motion that happened while the detector was running. |

A complete webcam + YOLO script is in [`tools/python/examples/track_object.py`](tools/python/examples/track_object.py):

```bash
pip install ./tools/python opencv-python ultralytics
python tools/python/examples/track_object.py --target person --fov-h 70 --fov-v 43 --show
```

## Hardware

| | |
|---|---|
| Board | Raspberry Pi Pico (RP2040) |
| Motors | 2× NEMA 17 pancake steppers on TMC2209 drivers, each through a 3:1 GT2 belt |
| Position | counted in steps: where the head is at `enable` is taken as the pose you give |
| Smallest step | 0.0375° |
| Top speed | 240 °/s at 1440 °/s² (ran clean at 360 °/s and 1920 °/s²) |
| Travel | set per rig, up to pan ±180° and tilt ±135° (factory ±90° / ±60°) |
| Status | verified on hardware with firmware 0.5.0: self-test up to 240 °/s, pan ±180° and tilt ±135° swept slowly by eye |

### Bill of materials

The full build guide is in [`mechanical/stepper-head/`](mechanical/stepper-head/README.md): the printable STL files, a bill of materials with shopping links, the fasteners, the tools you need and the assembly order. In short:

- Raspberry Pi Pico, two TMC2209 StepStick drivers, a 12 V 2 A supply
- Two NEMA 17 pancake steppers, GT2 20T and 60T pulleys, two 200 mm closed belts (3:1 per axis)
- Four 625ZZ bearings, 5 mm rod, M3 screws and nuts
- Optional: four lever endstop switches (supported in the firmware, not yet fitted to the reference head)

### Wiring

![Pico + two TMC2209 wiring](docs/wiring/pico-tmc2209/wiring.png)

| Pico pin | Goes to | |
|----------|---------|---|
| `GP2` / `GP3` | Pan driver `STEP` / `DIR` | |
| `GP6` / `GP7` | Tilt driver `DIR` / `STEP` | Note the order: the reverse of the pan pair |
| `GP8` | `EN` on both drivers | Active low; the firmware keeps it high (motors off) until `enable` |
| `GP12` / `GP13` | `MS1` / `MS2` on both drivers | Microstep select, set by the firmware (1/16) |
| `GP4` + `GP5` | `PDN/UART` on the pan driver | TMC2209 UART, one private line per driver: the firmware sets the motor current over it and reads load and faults (`owlet diag`) |
| `GP0` + `GP1` | `PDN/UART` on the tilt driver | As above |
| `GP22` | Activity LED cathode (`3V3` → resistor → LED → `GP22`) | The pin sinks the LED; lit while the head is moving |
| `3V3(OUT)` | `VIO` on both drivers | |
| `GND` | Driver grounds and supply negative | |

`GP10`/`GP11` (pan) and `GP14`/`GP15` (tilt) are set aside for the limit switches. 12 V goes to `VM` on the drivers only, never to the Pico. Pin tables, the driver pinout and a staged bring-up order are in [`docs/wiring/pico-tmc2209/`](docs/wiring/pico-tmc2209/README.md).

## Quick start

You need [Nix](https://nixos.org/download) with [flakes enabled](https://nixos.wiki/wiki/Flakes). It supplies every tool at a pinned version, so nothing else needs installing.

```bash
git clone https://github.com/Mrunmoy/owlet.git && cd owlet
nix develop                        # all tools; first run downloads them
sudo tools/dev/install-udev.sh     # Linux, once: USB access without sudo
tools/dev/doctor.sh                # is this machine ready?
./build.py --host -t               # build and run every test on the PC
./build.py --pico --flash          # build the firmware and load it onto the Pico
owlet ping                         # owlet 0.5.0 (protocol v3) on /dev/ttyACM0
owlet selftest                     # exercise both motors and report
```

That is the whole path from a clone to a tested head. **[`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md)** goes through it step by step: what each command does, how to try everything with no hardware, and what to do when a step fails.

A few things worth knowing straight away:

- **Flashing the Pico needs no button.** The script asks the running firmware to reboot into its bootloader. Only a board running firmware that ignores the request needs `BOOTSEL` held while plugging in, once.
- **The motors boot limp**, so `owlet enable` is always the first step. See [Safety](#safety).
- **The head has no position sensor yet.** Put it in its zero pose by hand before enabling (on the reference rig: pan centred, tilt pointing straight up): wherever it is then becomes 0, 0. `owlet enable 20 0` instead tells it "you are at pan 20 right now"; a stepper never jumps on enable.
- **`owlet` is already on your `PATH` inside `nix develop`.** Don't `pip install` there: Nix's Python is read-only. Outside Nix, any machine with Python 3.9+ can use the client with `pip install ./tools/python`.

```bash
owlet enable                # motors energise and hold
owlet move 20 0 --wait      # pan 20° to the camera's right, and wait until it is there
owlet pos                   # where is it?
owlet disable               # motors go limp
```

## Usage

### Command-line tool

```text
owlet [-p PORT] [-t TIMEOUT] <command> [args]
```

| Command | What it does |
|---------|--------------|
| `owlet ports` | List connected boards that look like an owlet |
| `owlet ping` | Check the link; print firmware and protocol versions |
| `owlet enable [PAN TILT]` | Energise the motors, optionally at a given pose |
| `owlet disable` | Turn the motors off (limp) |
| `owlet move PAN TILT [-w]` | Absolute move in degrees; `-w` waits for arrival |
| `owlet nudge DPAN DTILT [-w]` | Relative move, clamped to the limits |
| `owlet home [-w]` | Move to 0, 0 |
| `owlet stop` | Stop where it is right now |
| `owlet speed [DEG_PER_S]` | Get or set the slew speed (default 90 °/s; the range is the head's, see `owlet caps`) |
| `owlet pos` | Print the current and target position |
| `owlet limits` | Print the soft limits per axis |
| `owlet vel PAN TILT` | Velocity mode in °/s (stops after the watchdog period unless repeated) |
| `owlet watchdog [MS]` | Get or set the velocity watchdog (0 = off, 50–5000 ms) |
| `owlet clock` | Device clock in ms |
| `owlet caps` | What the head can do: speed, velocity, watchdog and stream ranges |
| `owlet diag` | What the motor drivers report: current, load, faults |
| `owlet dtc [--clear]` | What has gone wrong since power-up, how often, and what to check |
| `owlet stat` | Health counters: events raised and lost, late control-loop passes |
| `owlet seek` | Find the reference position from the limit switches (a head that has them) |
| `owlet selftest [-y]` | Exercise both motors inside the head's own limits and print a pass/fail report. Asks before moving; `-y` skips the questions |
| `owlet cal [pan\|tilt]` | Show the calibration stored on the device |
| `owlet cal pan --limits -85 90` | Change soft limits (works while enabled) |
| `owlet cal save` / `owlet cal reset` | Store the calibration in flash / restore factory values |
| `owlet watch [--hz N]` | Live position stream until Ctrl-C |
| `owlet shell` | Interactive prompt: type `move 30 -15`, `speed 45`, `pos`, … |

The port is auto-detected by the Raspberry Pi USB vendor ID. Use `-p /dev/ttyACM1`, `-p COM5`, etc. to choose one explicitly. `$EVENTS` has no subcommand; turn the `$EVT` lines off from Python with `head.events(False)`.

#### Self-test

`owlet selftest` is the quickest way to know a head is healthy after building, rewiring or reflashing it. It reads the head's limits and top speed and scales itself from them, so it suits any head and the simulator:

An example run on the reference head with its factory limits (the exact lines depend on the head):

```console
$ owlet selftest
  PASS  link: owlet 0.5.0, protocol v3, on /dev/ttyACM0
  PASS  ranges: pan -90..90, tilt -60..60 deg; speed up to 240 deg/s
  PASS  drivers at rest: pan: 458 mA; tilt: 458 mA

The head will now move: pan up to ±45 and tilt up to ±30 degrees, at up to 240 deg/s. Is it centred and clear to move? [y/N] y
  PASS  enable: state H after enable
  PASS  pan sweep at 60 deg/s: legs 0.85 s, 1.61 s, 0.86 s; load 28..66, median 58
  …
  PASS  velocity mode and watchdog: ran at 20 deg/s and stopped itself after 300 ms (6.9 deg)
  PASS  soft limit: a move to 95 deg was refused
  PASS  return home: reports 0, 0 and is on its marks
  SKIP  home switches: none fitted: zero is the pose at enable, so lost steps can only be seen by eye
  PASS  disable: motors off
self-test passed: 12 passed, 0 failed, 0 warnings, 1 skipped
```

It sweeps each axis at three speeds, then both together, checks that velocity mode stops by itself when commands stop, that a move past a soft limit is refused, and that the drivers report no faults. The head is left disabled whatever happens, and the exit code is non-zero if anything failed. `--span` and `--speeds` set how far and how fast as fractions of the head's own limits; `--axis` picks one axis.

Without home switches the head cannot measure where it is, so the last question, whether it is back on its centre marks, is the only check for lost steps.

### Python library

```python
from owlet import Owlet, OwletError

with Owlet() as head:                       # or Owlet("/dev/ttyACM1"), Owlet("COM5")
    head.enable()
    head.speed(60)                          # °/s

    head.move(45, 10, wait=True)            # blocks until $ARRIVED
    print(head.position())                  # Position(pan=45.0, tilt=10.0, ..., state='H', t_ms=...)

    for p in head.watch(hz=20):             # live stream; streaming stops when you break
        print(p.pan, p.tilt, p.moving)
        if not p.moving:
            break

    try:
        head.move(120, 0)                   # outside ±90°
    except OwletError as e:
        print(e.code)                       # "RNG"

    head.disable()
```

The only dependency is [`pyserial`](https://pypi.org/project/pyserial/).

**Being told instead of asking.** The head reports what it notices, and your program can connect a function to each kind of report:

```python
def on_fault(axis, faults):
    print(axis, "driver reports", faults)

with Owlet() as head:
    head.arrived.connect(lambda pan, tilt: print("arrived at", pan, tilt))
    head.fault_changed.connect(on_fault)
    head.watchdog_expired.connect(lambda: print("no command in time; head stopped"))

    head.enable()
    head.move(30, -15)          # returns at once; the arrival is reported when it happens
    ...                         # your program gets on with its own work
```

Connecting the first function starts a background reader, so the functions run without your program reading anything. A program that connects none behaves exactly as before, with no extra thread. `Owlet(dispatch="manual")` never starts the thread: functions then run during your own calls and whenever you call `head.process_events()`, which suits a GUI loop.

| Signal | Called with | When |
|--------|-------------|------|
| `arrived` | `pan, tilt` | a position move finished |
| `watchdog_expired` | | velocity mode was stopped for lack of a command |
| `bus_fault` / `bus_ok` | | commands to the motors started / stopped failing |
| `driver_online` / `driver_offline` | `axis` | a motor driver started / stopped answering |
| `fault_changed` | `axis, faults` | a motor driver's fault bits changed (see `$DIAG`) |
| `limit_hit` | `axis, end` | a limit switch tripped (`"min"` or `"max"`) |
| `position_lost` | `axis` | the head no longer knows where that axis is |
| `stalled` | `axis` | the motor stopped following its steps |
| `homed` / `homing_failed` | / `axis, code` | homing finished or gave up |
| `events_lost` | `count` | the head could not report everything; re-read its state |

The limit, position, stall and homing reports are defined in the protocol and the client now; the head starts sending them when limit switches and stall detection are fitted.

### Any other language: the wire protocol

Every message is one ASCII line:

```
$<NAME>[,<field>...]*<CHECKSUM>\r\n
```

The checksum is the **XOR of every character between `$` and `*`**, written as two hex digits. Here it is in a few languages:

```python
cs = 0
for c in body: cs ^= ord(c)          # body = "MOVE,30,-15"  →  0x3B
```
```c
uint8_t cs = 0; for (const char* p = body; *p; ++p) cs ^= (uint8_t)*p;
```
```javascript
const cs = [...body].reduce((a, c) => a ^ c.charCodeAt(0), 0);
```

An example session:

```mermaid
sequenceDiagram
    participant PC
    participant H as owlet
    PC->>H: $ENABLE*01
    H-->>PC: $ACK,ENABLE*64
    PC->>H: $MOVE,30,-15*3B
    H-->>PC: $ACK,MOVE*74
    Note over H: slews at the set speed
    H-->>PC: $ARRIVED,30.0,-15.0*75
    PC->>H: $POS*4C
    H-->>PC: $POS,30.0,-15.0,30.0,-15.0,H,48211*3A
    PC->>H: $VEL,-12.5,30*69
    Note right of PC: velocity mode: repeat within the watchdog period
    H-->>PC: $ACK,VEL*3A
    Note over PC,H: the PC stops sending VEL
    H-->>PC: $TIMEOUT,VEL*28
```

Serial settings don't matter on native USB: any baud rate works, except 1200, which the pico-sdk treats as a request to reboot into the bootloader.

[`docs/protocol_vectors.txt`](docs/protocol_vectors.txt) lists valid and invalid frames you can use to test your own implementation.

### No hardware? Use the simulator

`owlet_sim` runs the real firmware logic (protocol, motion, events, and a simulated head with the reference head's gearing, speeds and limits) behind a Linux pseudo-terminal:

```bash
nix develop
./build.py --host
build/host/apps/host-sim/owlet_sim --link /tmp/owlet &

owlet -p /tmp/owlet enable
owlet -p /tmp/owlet move 20 20 --wait
owlet -p /tmp/owlet watch
```

`--cal-file PATH` keeps the simulator's calibration in a file across restarts, as the head keeps it in flash. Started with `--inject`, the simulator also raises events it has no hardware to produce, from lines typed on its standard input (`limit pan max`, `fault tilt 6`, `offline pan`, `stall tilt`, …). That is how a program's handlers can be tried without a head.

## Protocol reference

### Requests

| Request | Reply | Notes |
|---------|-------|-------|
| `$PING` | `$PONG,<proto>,<fw>` | e.g. `$PONG,3,0.5.0` |
| `$ENABLE` | `$ACK,ENABLE` | Energise at the last known position (0, 0 after boot) |
| `$ENABLE,<pan>,<tilt>` | `$ACK,ENABLE` | Energise at a pose; if already enabled, moves there smoothly instead |
| `$DISABLE` | `$ACK,DISABLE` | Motors limp |
| `$MOVE,<pan>,<tilt>` | `$ACK,MOVE` | Absolute move; outside the limits → `NAK RNG` |
| `$NUDGE,<dpan>,<dtilt>` | `$ACK,NUDGE` | Relative to the current target; clamped to the limits |
| `$HOME` | `$ACK,HOME` | Same as `MOVE,0,0` |
| `$STOP` | `$ACK,STOP` | Stop at the current position |
| `$SPEED` | `$SPEED,<°/s>` | Query the slew speed |
| `$SPEED,<°/s>` | `$ACK,SPEED` | Applies to both axes. The allowed range is the head's own: read it with `$CAPS` |
| `$POS` | `$POS,<pan>,<tilt>,<tpan>,<ttilt>,<state>,<ms>` | `state`: `D` disabled, `M` moving, `H` holding; `ms`: device time of the pose |
| `$STREAM,<hz>` | `$ACK,STREAM` | 0 = off, 1–50 Hz periodic `$POS` |
| `$LIMITS` | `$LIMITS,<pmin>,<pmax>,<tmin>,<tmax>` | Soft limits in degrees |
| `$VEL,<pan°/s>,<tilt°/s>` | `$ACK,VEL` | Velocity mode, each up to the head's limit either way (`$CAPS`); stops at soft limits; ended by any position command |
| `$WATCHDOG` / `$WATCHDOG,<ms>` | `$WATCHDOG,<ms>` / `$ACK,WATCHDOG` | Velocity watchdog: 0 = off, 50–5000 ms, default 500 |
| `$CLOCK` | `$CLOCK,<ms>` | Device uptime in ms (wraps after ~49.7 days) |
| `$CAPS` | `$CAPS,<spdmin>,<spdmax>,<velmax>,<wdmin>,<wdmax>,<hzmax>` | Capabilities, e.g. `$CAPS,1.0,240.0,240.0,50,5000,50` |
| `$DIAG,<AXIS>` | `$DIAG,<AXIS>,<U\|N>,<faults>,<stalled>,<load>,<mA>` | What the motor driver reports. `N` = no driver data (the motor supply is off, or the driver's UART wire is missing). Faults are a bit mask; see [`docs/DESIGN.md` §5.7](docs/DESIGN.md) |
| `$EVENTS,<0\|1>` | `$ACK,EVENTS` | Turn the `$EVT` lines off or on. On at power-up |
| `$SEEK` | `$ACK,SEEK` | Find the reference position from the limit switches. The head moves by itself; the outcome is `$EVT,HOMED` or `$EVT,HOMEFAIL`. `NAK,SEEK,UNS` on a head without switches |
| `$DTC` | `$DTC,<count>` | How many kinds of trouble have been recorded since power-up |
| `$DTC,<index>` | `$DTC,<index>,<code>,<PAN\|TILT\|->,<times>,<detail>,<first_ms>,<last_ms>` | One of them. Codes are listed under [When something goes wrong](#when-something-goes-wrong) |
| `$DTC,CLEAR` | `$ACK,DTC` | Forget them |
| `$STAT` | `$STAT,<posted>,<dropped>,<high>,<late>,<worst_ms>` | Health counters: events raised, events lost, the most that were ever waiting; control-loop passes that ran late, and the longest gap between two |
| `$CAL,<AXIS>` | `$CAL,<AXIS>,<min_us>,<ctr_us>,<max_us>,<min°>,<max°>,<R\|N>,<F\|S\|M>` | Calibration of `PAN`/`TILT`: reversed or not; factory / saved / modified |
| `$CAL,<AXIS>,0,0,0,<min°>,<max°>` | `$ACK,CAL` | Set the soft limits, within the travel compiled into the firmware (`NAK,CAL,RNG` beyond it). The three drive numbers are always 0 on this head. Limits can change any time |
| `$CAL,SAVE` / `$CAL,RESET` | `$ACK,CAL` | Persist to flash / restore factory values and erase (reset needs disabled). `SAVE` is refused with `NAK,CAL,MOV` while the head is moving |

Angles are degrees from center: **pan + = right, tilt + = up**. Numbers may have an optional sign and up to 2 decimal places. `MOVE`, `NUDGE`, `HOME`, `STOP` and `VEL` are refused with `NAK …,DIS` until the head is enabled. This is protocol **v3** (since owlet 0.3.0); clients should check the version in `$PONG`.

### Messages owlet sends on its own

| Message | When |
|---------|------|
| `$READY,<proto>,<fw>` | Once after boot |
| `$ARRIVED,<pan>,<tilt>` | Exactly once for every accepted `MOVE`/`NUDGE`/`HOME` (cancelled by `STOP`/`DISABLE`) |
| `$POS,…` | At the `STREAM` rate |
| `$ERR,BUS` | A motor update failed to reach the hardware |
| `$TIMEOUT,VEL` | The velocity watchdog stopped the head |
| `$EVT,<name>,<PAN\|TILT\|->,<arg>` | Something else the head noticed (firmware 0.5.0 and later). `$EVENTS,0` silences these |

| `$EVT` name | `arg` | Meaning |
|-------------|-------|---------|
| `ONLINE` / `OFFLINE` | 0 | That axis's motor driver started / stopped answering |
| `FAULT` | fault bits | That driver's faults changed (same bits as `$DIAG`; 0 = cleared) |
| `BUSOK` | 0 | Motor updates are getting through again after `$ERR,BUS` |
| `OVERFLOW` | count | That many events could not be reported. Re-read the head's state |
| `LIMIT` | `MIN` / `MAX` | A limit switch tripped |
| `LOST` | 0 | The head no longer knows where that axis is |
| `STALL` | 0 | The motor stopped following its steps |
| `HOMED` / `HOMEFAIL` | 0 / reason | Homing finished / gave up |

`LIMIT`, `LOST`, `STALL`, `HOMED` and `HOMEFAIL` are reserved for limit switches and stall detection, which are not fitted yet. A client should ignore `$EVT` names it does not know.

### When something goes wrong

The head keeps a short list of what has gone wrong since it was powered: what, on which axis, how many times, and when. Ask for it at any time:

```
$ owlet dtc
11 WATCHDOG: velocity mode was stopped because no command arrived in time
     2 times, last 0.4 s ago. Check: the program sending rates stalled or the link dropped; normal if you stopped sending.
21 DRIVER-FAULT on tilt: the motor driver raised a fault (short on coil A)
     once, last 12.0 s ago. Check: the fault names given; a short or open coil means motor wiring.
```

`owlet dtc --clear` empties the list. From Python it is `head.trouble_codes()` and `head.clear_trouble_codes()`. The list is kept in memory only, so a power cycle also clears it.

| Code | Name | Meaning | Detail field |
|------|------|---------|--------------|
| 10 | `MOTOR-BUS` | A motor update did not reach the hardware | |
| 11 | `WATCHDOG` | Velocity mode was stopped for lack of a command | |
| 20 | `DRIVER-OFFLINE` | A motor driver stopped answering | |
| 21 | `DRIVER-FAULT` | A motor driver raised a fault | its fault bits, as in `$DIAG` |
| 30 | `LIMIT` | A limit switch tripped outside homing | 0 = min end, 1 = max end |
| 31 | `POSITION-LOST` | The head no longer knows where the axis is | |
| 32 | `STALL` | The motor stopped following its steps | |
| 40 | `HOMING-FAILED` | Homing gave up | the reason |
| 50 | `EVENTS-LOST` | The head could not report everything it noticed | how many, last time |

A code's number never changes meaning. Codes 30 to 40 are recorded once limit switches and stall detection are fitted. A driver that was never powered is not trouble by this list's reckoning, because nothing changed; `owlet diag` shows that.

### Errors

| Reply | Meaning |
|-------|---------|
| `$NAK,<CMD>,FMT` | Wrong number of fields or a malformed number |
| `$NAK,<CMD>,RNG` | Value out of range |
| `$NAK,<CMD>,LOST` | The head does not know where it is (it ran into a limit switch, or homing failed). `SEEK`, or `DISABLE` then `ENABLE,<pan>,<tilt>` at a known pose |
| `$NAK,<CMD>,BSY` | Homing is under way. `STOP` aborts it |
| `$NAK,<CMD>,UNS` | This head cannot do that |
| `$NAK,<CMD>,DIS` | The motors are disabled; send `ENABLE` first |
| `$NAK,<CMD>,BUS` | The motor hardware didn't respond (check wiring and power) |
| `$NAK,CAL,STO` | Saving to (or erasing from) flash failed |
| `$ERR,CHK` | Checksum mismatch |
| `$ERR,SYN` | Not a valid `$…*HH` frame |
| `$ERR,LEN` | Line longer than 82 bytes |
| `$ERR,UNK` | Unknown command |

Every valid request gets exactly one reply, in order. The full specification, including edge cases and the reasoning behind them, is in [`docs/DESIGN.md` §5](docs/DESIGN.md#5-native-wire-protocol-v3).

> **About "position":** the head does not measure where the shafts really are. It counts steps, which is exact as long as no step is lost. If a motor is blocked or overloaded, the reported and the real position will differ; limit switches are what let the head find out and recover.

## Extending owlet

The firmware is organised around small interfaces, so you can extend it by **adding** classes rather than editing existing ones:

```mermaid
flowchart TB
    LOOP["ControlLoop<br/>fixed-rate scheduler"]
    SRC[/"IByteSource · USB, UART, TCP…"/] --> LOOP
    LOOP --> PROTO{{"IProtocol"}}
    PROTO --- TEXT["TextProtocol<br/>built in"]
    PROTO --- MINE["Your protocol<br/>Pelco-D, G-code, binary…"]
    PROTO --- MUX["ProtocolMux<br/>several on one port"]
    TEXT & MINE --> HEAD{{"IPanTilt"}}
    TEXT & MINE --> SINK[/"IByteSink · replies"/]
    HEAD --- PT["PanTilt<br/>or your own mechanism"]
    PT --> AXIS{{"IAxis · one motor"}}
    AXIS --- STEP["StepperAxis"] --> GEN{{"IStepGenerator"}}
    GEN --- PIO["Pico PIO<br/>or another chip's timer"]
    AXIS --- OTHER["Your motor<br/>DC + encoder, …"]
```

| Interface | Implement it to… |
|-----------|-------------------|
| `owlet::core::IProtocol` | add a wire protocol (Pelco-D for PTZ software, G-code, a compact binary format, …) |
| `owlet::hal::IAxis` (+ `IAxisCalibration`) | add a **motor type**. `StepperAxis` is the one built in; a DC motor with an encoder would be another. The motion policy, calibration, every protocol and the client work with it unchanged |
| `owlet::hal::IStepGenerator` | emit step/dir pulses on another chip (the Pico uses a PIO state machine per motor) |
| `owlet::core::EventDelegate` | react to what the head notices without anything polling for it. Derive from it and override only the reactions you want: `onArrived()`, `onLimitHit(axis, end)`, `onDriverFaultChanged(axis, faults)`, … The trouble log and the wire protocol are both delegates; a status display or a logger would be another |
| `owlet::hal::IClock` · `ISerialPort` · `IDigitalOutput` · `core::ICalibrationStore` | port to **another board**: implement these few in a `platform/owlet_<board>/` folder and add a wiring-only `main.cpp`. `platform/owlet_pico/` is about 525 lines of `.cpp`; the full stepper feature set also needs `IHalfDuplexLink`, `IDigitalInput` and `IInterruptLock` |
| `owlet::hal::IByteSource` / `IByteSink` | carry the protocol over **another transport** (UART, TCP, BLE, Wi-Fi). The loop and the protocols only ever see these two |
| `owlet::core::IPanTilt` | replace the whole two-axis policy (a different kinematic arrangement) under every existing protocol |

A protocol is about 100 lines. It only ever sees `IPanTilt` (what it may ask the head to do) and `IByteSink` (where replies go):

```cpp
class MyProtocol final : public owlet::core::IProtocol {
public:
    MyProtocol(owlet::hal::IByteSink& out, owlet::core::IPanTilt& head);
    bool claims(uint8_t byte, bool atLineStart) const override;   // e.g. byte == 0xFF
    owlet::core::ConsumeResult consume(const uint8_t* data, std::size_t len) override;
    void begin(uint32_t nowMs) override;
    void onTick(uint32_t nowMs) override;
};
```

Then, in the app's `main.cpp` (`apps/pico-owlet/`), run it next to the built-in protocol on the same USB port:

```cpp
static owlet::text::TextProtocol    native(usb, head, calibrator, clock, owlet::core::kFirmwareVersion);
static MyProtocol                   mine(usb, head);
static owlet::core::ProtocolMux<2>  protocols({&native, &mine});
static owlet::core::ControlLoop     loop(usb, head, protocols, &events);
```

A protocol that wants to be told when a move arrives, the watchdog fires or a motor stops answering also derives from `owlet::core::EventDelegate`, overrides the reactions it wants, and is attached to the event bus:

```cpp
class MyProtocol final : public owlet::core::IProtocol, public owlet::core::EventDelegate {
    void onArrived() override;                                   // tell my client the move finished
    void onLimitHit(uint8_t axis, owlet::core::LimitEnd end) override;
    …
};

events.attach(mine);
```

The names follow one rule throughout the firmware, so a class's header says what it does: **`handleXxx`** carries out a command that was asked for, **`notifyXxx`** announces something that happened, and **`onXxx`** reacts to an announcement.

The multiplexer offers the first byte of each frame to each protocol in order. Native frames start with `$`, so anything else is free for your protocol. The contract and an example test setup are in [`docs/DESIGN.md` §4.3–4.4](docs/DESIGN.md#43-the-iprotocol-contract).

## Using owlet as a git submodule

```bash
git submodule add https://github.com/Mrunmoy/owlet.git third_party/owlet
```

- **Python (PC side):** `pip install -e third_party/owlet/tools/python`, then `from owlet import Owlet, Tracker, PixelController`.
- **Host C++ (tests, simulators):** `add_subdirectory(third_party/owlet)` and link `owlet::core`, `owlet::text`, `owlet::stepper` or `owlet::hal`. owlet's own tests and tools are built only when it's the top-level project.
- **pico-sdk firmware:** `add_subdirectory` owlet's `drivers`, `firmware/common` and `platform/owlet_pico` after `pico_sdk_init()` and link `owlet::pico`, as `apps/pico-owlet/CMakeLists.txt` does, then wire things up as `apps/pico-owlet/main.cpp` does.

## Configuration and calibration

The soft limits live **on the device**, in flash. Every client can read them (`$CAL`, `$LIMITS`, `$CAPS`), and changing them never needs a rebuild:

```bash
owlet cal                                  # show both axes
owlet cal pan --limits -180 180            # this mechanism turns all the way round
owlet cal tilt --limits -135 135
owlet cal save                             # store it in the Pico's flash (kept across resets)
```

```
pan   limits -180.0..180.0 deg   (saved)
tilt  limits -135.0..135.0 deg   (saved)
```

- Commands outside the limits are refused (`MOVE`) or clamped (`NUDGE`, velocity mode). Limits can change while the head is enabled; a head left outside new limits moves back inside them.
- The widest limits the firmware accepts are compiled in: pan ±180° and tilt ±135°. The factory limits are ±90° and ±60°; `owlet cal reset` goes back to them.
- Everything else about the head is a property of the build, in [`apps/pico-owlet/board.hpp`](apps/pico-owlet/board.hpp): pins, steps per degree (motor, microsteps, belt ratio), direction (`kPanReversed`, `kTiltReversed`), speed, acceleration, motor current and the travel bounds.
- **Zero** is the pose the head is in when it is enabled. Put it there by hand first, or tell it where it is: `owlet disable`, then `owlet enable <pan> <tilt>`.

How the limits were found on the reference head, and what has been measured, is in [`docs/CALIBRATION.md`](docs/CALIBRATION.md).

## Safety

- **Limp at boot.** Every motor is off after power-up or reset. Nothing moves until a computer sends `ENABLE`.
- **Enabling never moves the head.** It only names the pose the head is already in (`ENABLE,<pan>,<tilt>`, or 0, 0).
- **No silent queuing.** Move commands sent while disabled are refused, not saved for later, so enabling never triggers an unexpected motion.
- **Soft limits** are enforced in firmware. Set them inside your mechanism's hard stops.
- **Limit switches, where fitted, outrank everything.** A switch closing cuts that motor's step pulses from its interrupt, without waiting for the control loop. The head then holds still and refuses to move until it has found its reference again, because it no longer knows where it is.

## Building from source

Everything below runs inside `nix develop`.

```bash
./build.py --host -t          # host build + all tests (C++ and Python)
./build.py --host --clean -t  # same, from scratch
./build.py --pico             # firmware only (apps/pico-owlet/build/pico_owlet.uf2)
./build.py --pico --flash     # firmware + load with picotool (--wait SECONDS for a board still booting)
./build.py --docs             # API reference from the headers, in build/docs/html (Doxygen)
```

Run one test suite or test:

```bash
ctest --test-dir build/host -R TextProtocol --output-on-failure
build/host/tests/owlet_stepper_axis_test --gtest_filter='StepperAxis.*'
cd tools/python && python -m unittest tests.test_protocol
```

On a connected head (this **moves the motors**, within ±20° of centre at ≤ 30 °/s, and leaves them limp):

```bash
cd tools/python/tests && OWLET_HW=1 python -m unittest test_hardware -v    # OWLET_PORT=... to pick the port
```

It checks what the simulator can't: request round trip and `$POS` freshness on a real USB link, clock-offset precision, velocity mode at the commanded rate, the watchdog trip, and a continuous `Tracker` run. Normal test runs skip it.

> Nix flakes only see files that git knows about. If you add a file and the build can't find it, run `git add` on it first.

**Without Nix:** the host tests only need CMake, Ninja and a C++17 compiler (GoogleTest is downloaded automatically). The firmware needs `gcc-arm-none-eabi`, `picotool` and pico-sdk 2.2.0 with its submodules, with `PICO_SDK_PATH` pointing at it; [`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md) lists the packages for a Raspberry Pi.

### Debug logs

The USB port carries **only** protocol traffic, and the firmware prints no logs. Ask the head instead: `owlet dtc` (what went wrong), `owlet stat` (event and loop counters) and `owlet diag` (what the drivers report).

## Project layout

| Path | What's there |
|------|--------------|
| `apps/pico-owlet/` | pico-sdk firmware project: `main.cpp` wires it up, `board.hpp` describes the head (pins, motors, speeds, limits) |
| `apps/host-sim/` | `owlet_sim`: the firmware core on a Linux pseudo-terminal |
| `drivers/owlet_hal/` | Hardware interfaces: axis, axis calibration, axis diagnostics and listeners, step generator, half-duplex link, digital input (with edge listener) and output, delay, serial port, clock, interrupt lock, pollable |
| `drivers/owlet_stepper/` | Stepper axis: exact step counting, acceleration-limited motion, limit-switch guard; TMC2209 over UART and standalone; shared enable line |
| `firmware/common/owlet_core/` | Interfaces (`IPanTilt`, `IProtocol`, `ICalibration`), `PanTilt`, `ControlLoop`, `ProtocolMux`, `Calibrator`; the event bus, trouble log and homing |
| `firmware/common/owlet_text/` | The native text protocol: codec + `TextProtocol` |
| `platform/owlet_pico/` | pico-sdk implementations: USB CDC, clock, GPIO, PIO step generator, flash calibration store |
| `mechanical/stepper-head/` | Printable belt-driven head for two NEMA 17 motors: OpenSCAD source and STLs |
| `tests/` | GoogleTest suites |
| `tools/python/` | Client library and `owlet` CLI (`tools/bin/owlet` runs it in `nix develop`) |
| `tools/dev/` | `doctor.sh` (checks the toolchain), the udev rule and its installer |
| `datasheets/` | Index and fetch script for the chips' datasheets (the PDFs are not tracked) |
| `docs/` | Design, calibration and hardware test record, getting started, protocol test vectors, wiring diagrams; `docs/design/` holds the event-driven design |
| `build.py` · `flake.nix` | Build/test/flash entry point · reproducible toolchain |

Code in `drivers/` and `firmware/common/` is plain C++17 with no vendor headers and no dynamic memory, and it builds and runs on a PC. Only `platform/` and `apps/pico-owlet/` touch the pico-sdk. Read [`docs/DESIGN.md`](docs/DESIGN.md) for the architecture and the reasoning behind it.

## Troubleshooting

| Symptom | Likely cause and fix |
|---------|-------------------|
| `could not auto-detect the owlet port` | No board, or several boards, found. Run `owlet ports`, then pass `-p PORT`. |
| `Permission denied: '/dev/ttyACM…'` (Linux) | Add yourself to the group that owns the device (`ls -l /dev/ttyACM*`, usually `dialout` or `plugdev`), then log out and back in. |
| `no answer from owlet` | Wrong port, firmware not flashed, or something else (e.g. a serial monitor) has the port open. |
| `MOVE refused: DIS` | The motors are disabled. Run `owlet enable` first. |
| `MOVE refused: LOST` | The head ran into a limit switch or failed to home, so it no longer trusts its position. Run `owlet seek`, or `owlet disable` and then `owlet enable PAN TILT` at a pose you know. `owlet dtc` says what happened. |
| Something stopped and you don't know why | `owlet dtc`. |
| `owlet diag` says `no driver data` | The 12 V motor supply is off, or a driver's UART wire is missing. The head still runs without them, on the drivers' potentiometers. |
| An axis holds firmly but never turns | Its `STEP` wire is not reaching the driver; swapped `STEP`/`DIR` gives exactly this. |
| An axis moves the wrong way | Flip `kPanReversed` / `kTiltReversed` in `apps/pico-owlet/board.hpp`, or swap the two wires of one motor coil. |
| The head ends up off its marks | Steps were lost: lower the speed or acceleration in `board.hpp`, or look for something binding. Then put it back in its zero pose and re-enable. |

## Project status and roadmap

**v0.5.0 firmware, v0.5.0 Python client, alpha.** The firmware, protocol, PC client, tracking support, on-device calibration and simulator are complete and tested. On the reference head, motion, directions, ranges, speed and acceleration limits, velocity mode, the watchdog and calibration storage were verified with firmware 0.4.0. Firmware 0.5.0 makes the head report changes as events; on the Pico its arrival, watchdog and event-counter paths were checked with the motors disconnected, and the driver reports (`ONLINE`, `FAULT`) have host tests only so far. With the motors connected, 0.5.0 passed `owlet selftest` on 2026-10-09, and the same day the reference head was swept slowly through pan ±180° and tilt ±135°, which are now its saved limits ([`docs/CALIBRATION.md`](docs/CALIBRATION.md)).

- [x] Firmware: USB protocol, speed-limited motion, boot-limp safety (first built on an ESP32-S3 with hobby servos; that head was retired on 2026-10-19)
- [x] Python library and CLI with port auto-detection
- [x] Host simulator and end-to-end tests
- [x] First motion on hardware
- [x] Pluggable protocol architecture (`IProtocol`, `ProtocolMux`)
- [x] Tracking support: velocity mode, watchdog, timestamped pose, clock sync, `Tracker`, `PixelController`
- [x] On-device calibration and capabilities (`$CAL`, `$CAPS`)
- [x] Verify velocity mode, watchdog, clock sync and `Tracker` on hardware
- [x] Motor-neutral axis interface (`IAxis`): the core knows no motor type
- [x] Stepper axis with acceleration ramps, and a Raspberry Pi Pico platform
- [x] First motion of the head: both axes swept and back on their marks
- [x] Directions confirmed (pan + = camera's right, tilt + = up)
- [x] On 12 V: speed and acceleration limits measured and set
- [x] Wider travel (pan ±180°, tilt ±135°) swept and saved
- [x] Motor current set over the TMC2209 UART instead of the potentiometer; driver load and faults readable (`owlet diag`)
- [ ] Stall detection: characterise the load value against speed and pick thresholds
- [x] Event-driven firmware and client: the head announces changes (`$EVT`), Python handlers, trouble codes (`owlet dtc`)
- [x] Limit switches and homing in firmware: interrupt cut-off, lost-position rule, `owlet seek` (tested in simulation)
- [ ] Fit the limit switches to the head and prove homing on it
- [ ] Driver events (`ONLINE`, `FAULT`) checked on powered drivers
- [ ] Track a real object with the webcam + YOLO example
- [ ] Verify the client on Windows and macOS
- [ ] Separate speed per axis
- [x] Mechanical design files for the head ([`mechanical/stepper-head/`](mechanical/stepper-head/))

What has been measured and verified on the reference head is recorded in [`docs/CALIBRATION.md`](docs/CALIBRATION.md).

## Contributing

Issues and pull requests are welcome. Before opening a PR:

1. Run `./build.py --host -t`; every test must pass.
2. Add or update tests for any behaviour you change. All device behaviour lives in `firmware/common/` so it can be tested on a PC.
3. If you change the native protocol, update the firmware (`owlet_text`), the Python client, and `docs/DESIGN.md` §5 together.
4. Keep firmware code free of dynamic allocation, exceptions and RTTI, following the existing style (C++17, Allman braces, `m_` member prefix).
5. Name by intent: `handleXxx` for a command, `notifyXxx` for an announcement, `onXxx` for a reaction to one.

## License

owlet is released under the [MIT License](LICENSE). That covers the firmware, the PC client, the documentation and the mechanical design files (OpenSCAD sources and STLs).

## Acknowledgements

- owlet grew out of **clops**, a one-eyed robot. Its hardware-interface layer and build system were carried over.
- Toolchain packaging by [nixpkgs-esp-dev](https://github.com/mirrexagon/nixpkgs-esp-dev).
