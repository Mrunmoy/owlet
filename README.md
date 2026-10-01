<h1 align="center">🦉 owlet</h1>

<p align="center">
  <b>A USB pan-tilt head you can drive from any computer, in any language.</b><br>
  ESP32-S3 · PCA9685 · two 180° servos · one USB cable · a plain-text protocol with checksums
</p>

<p align="center">
  <a href="LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-green.svg"></a>
  <img alt="Firmware: ESP-IDF 5.5" src="https://img.shields.io/badge/firmware-ESP--IDF%205.5-red.svg">
  <img alt="MCU: ESP32-S3" src="https://img.shields.io/badge/MCU-ESP32--S3-blue.svg">
  <img alt="C++17" src="https://img.shields.io/badge/C%2B%2B-17-00599C.svg">
  <img alt="Python 3.9+" src="https://img.shields.io/badge/python-3.9%2B-3776AB.svg">
  <img alt="Status: alpha" src="https://img.shields.io/badge/status-alpha-orange.svg">
</p>

```console
$ owlet enable
$ owlet move 30 -15 --wait
pan   30.0  tilt  -15.0   target   30.0  -15.0   holding
```

---

## Contents

- [Features](#features)
- [How it works](#how-it-works)
- [Hardware](#hardware)
- [Quick start](#quick-start)
- [Usage](#usage)
  - [Command-line tool](#command-line-tool)
  - [Python library](#python-library)
  - [Any other language: the wire protocol](#any-other-language-the-wire-protocol)
  - [No hardware? Use the simulator](#no-hardware-use-the-simulator)
- [Protocol reference](#protocol-reference)
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

- **Plug in and go.** The ESP32-S3's built-in USB shows up as a standard serial port (`/dev/ttyACM*`, `COMx`, `/dev/cu.usbmodem*`). No drivers, no USB-TTL adapter.
- **Human-readable, checksummed protocol.** NMEA-style lines like `$MOVE,30,-15*3B`. You can type them in a terminal or generate them from any language, and every frame is integrity-checked.
- **Know where it is.** Poll the position with `$POS` or stream it at up to 50 Hz. The head also reports `$ARRIVED` when a move finishes.
- **Smooth, speed-limited motion.** Constant-velocity slew from 1 to 500 °/s, timed from the actual elapsed time so it doesn't speed up or slow down with loop timing.
- **Safe by default.** The servos are limp at power-up and nothing moves until you explicitly enable it.
- **Batteries-included PC side.** A `pip`-installable Python library and an `owlet` command with port auto-detection, a live position view, and an interactive shell.
- **Hardware-free development.** A host simulator runs the *real* firmware logic behind a virtual serial port.
- **Tested.** 126 automated tests: C++ unit and integration tests, plus Python end-to-end tests against the simulator.

## How it works

```
┌──────────────┐   USB (CDC serial)    ┌────────────────┐   I²C   ┌──────────┐   PWM   ┌───────────┐
│  PC / Pi /   │ ───────────────────▶ │  ESP32-S3 Zero │ ──────▶ │ PCA9685  │ ──────▶ │ pan servo │
│  anything    │ ◀─────────────────── │  owlet firmware│         │ 16-ch PWM│ ──────▶ │ tilt servo│
└──────────────┘  $MOVE,30,-15*3B      └────────────────┘         └──────────┘         └───────────┘
                  $ACK,MOVE*74
```

The firmware runs a 100 Hz loop. Each pass it advances both servos toward their targets at the configured speed, then reads and answers any complete command lines from USB. All of that logic is platform-independent C++ that is unit-tested on a normal PC. The ESP32-specific code only connects it to the real USB port and I²C bus.

## Hardware

### Bill of materials

| Qty | Part | Notes |
|----:|------|-------|
| 1 | [Waveshare ESP32-S3 Zero](https://www.waveshare.com/wiki/ESP32-S3-Zero) | Any ESP32-S3 board with native USB should work if you adjust the pins |
| 1 | PCA9685 16-channel PWM board | The common blue breakout, I²C address `0x40` |
| 2 | 180° hobby servos (MG90S/SG90 class) | Channel 0 = pan (base rotate), channel 1 = tilt |
| 1 | Pan-tilt bracket / printed mechanism | |
| 1 | USB-C data cable | |
| — | *Optional:* separate 5 V supply for servos | Recommended once the head carries a load |
| — | *Optional:* USB-TTL adapter | Only for reading firmware debug logs |

### Wiring

```
ESP32-S3 Zero          PCA9685
─────────────          ───────
GND            ─────── GND
3V3            ─────── VCC        (logic supply)
5V             ─────── V+         (servo supply; or a separate 5 V source with common GND)
GP1            ─────── SDA
GP2            ─────── SCL

PCA9685 channel 0  →  pan servo  (base rotate)
PCA9685 channel 1  →  tilt servo

GP43 (UART0 TX) ────→ optional USB-TTL adapter RX, 115200 baud, for debug logs
```

> **Power note:** two small servos can run off the ESP's USB 5 V for bench testing. Under load, or if the board resets when the servos move (a brown-out), power `V+` from a separate 5 V supply and keep the grounds connected.

## Quick start

### 1. Flash the firmware

You need [Nix](https://nixos.org/download) with [flakes enabled](https://nixos.wiki/wiki/Flakes). It provides ESP-IDF, the compilers and Python, so nothing else needs to be installed.

```bash
git clone https://github.com/Mrunmoy/owlet.git
cd owlet
nix develop                 # first run downloads the toolchain; later runs are instant
./build.py --esp --flash    # build and flash; the board's port is detected automatically
```

With more than one Espressif board connected, pass the port: `./build.py --esp --flash /dev/ttyACM1`.

### 2. Install the PC client

Any machine with Python 3.9+ (Linux, macOS or Windows):

```bash
pip install ./tools/python
```

### 3. Say hello

```bash
owlet ping                  # owlet 0.1.0 (protocol v1) on /dev/ttyACM1
owlet enable                # servos energise at center (0, 0)
owlet move 20 0 --wait      # base turns 20° and the command waits until it's there
owlet pos                   # where is it?
owlet disable               # servos go limp
```

The servos boot **limp**, so `enable` is always the first step. See [Safety](#safety).

## Usage

### Command-line tool

```text
owlet [-p PORT] [-t TIMEOUT] <command> [args]
```

| Command | What it does |
|---------|--------------|
| `owlet ports` | List connected boards that look like an owlet |
| `owlet ping` | Check the link; print firmware and protocol versions |
| `owlet enable [PAN TILT]` | Energise the servos, optionally at a given pose |
| `owlet disable` | Turn the servos off (limp) |
| `owlet move PAN TILT [-w]` | Absolute move in degrees; `-w` waits for arrival |
| `owlet nudge DPAN DTILT [-w]` | Relative move, clamped to the limits |
| `owlet home [-w]` | Move to 0, 0 |
| `owlet stop` | Stop where it is right now |
| `owlet speed [DEG_PER_S]` | Get or set the slew speed (1–500 °/s, default 90) |
| `owlet pos` | Print the current and target position |
| `owlet limits` | Print the soft limits per axis |
| `owlet watch [--hz N]` | Live position stream until Ctrl-C |
| `owlet shell` | Interactive prompt: type `move 30 -15`, `speed 45`, `pos`, … |

The port is auto-detected by Espressif's USB vendor ID. Use `-p /dev/ttyACM1`, `-p COM5`, etc. to choose one explicitly.

### Python library

```python
from owlet import Owlet, OwletError

with Owlet() as head:                       # or Owlet("/dev/ttyACM1"), Owlet("COM5")
    head.enable()
    head.speed(60)                          # °/s

    head.move(45, 10, wait=True)            # blocks until $ARRIVED
    print(head.position())                  # Position(pan=45.0, tilt=10.0, ..., state='H')

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

An example session (`→` is sent to owlet, `←` comes back):

```
→ $ENABLE*01
← $ACK,ENABLE*64
→ $MOVE,30,-15*3B
← $ACK,MOVE*74
← $ARRIVED,30.0,-15.0*75            (sent when the move completes)
→ $POS*4C
← $POS,30.0,-15.0,30.0,-15.0,H*28
```

Serial settings don't matter on native USB (any baud rate works). Open the port normally and **don't toggle DTR/RTS**, because that's the chip's auto-reset signal.

[`docs/protocol_vectors.txt`](docs/protocol_vectors.txt) lists valid and invalid frames you can use to test your own implementation.

### No hardware? Use the simulator

`owlet_sim` runs the real firmware logic (protocol, motion and a simulated PCA9685) behind a Linux pseudo-terminal:

```bash
nix develop
./build.py --host
build/host/apps/host-sim/owlet_sim --link /tmp/owlet &

owlet -p /tmp/owlet enable
owlet -p /tmp/owlet move 20 20 --wait
owlet -p /tmp/owlet watch
```

## Protocol reference

### Requests

| Request | Reply | Notes |
|---------|-------|-------|
| `$PING` | `$PONG,<proto>,<fw>` | e.g. `$PONG,1,0.1.0` |
| `$ENABLE` | `$ACK,ENABLE` | Energise at the last known position (0, 0 after boot) |
| `$ENABLE,<pan>,<tilt>` | `$ACK,ENABLE` | Energise at a pose; if already enabled, moves there smoothly instead |
| `$DISABLE` | `$ACK,DISABLE` | Servos limp |
| `$MOVE,<pan>,<tilt>` | `$ACK,MOVE` | Absolute move; outside the limits → `NAK RNG` |
| `$NUDGE,<dpan>,<dtilt>` | `$ACK,NUDGE` | Relative to the current target; clamped to the limits |
| `$HOME` | `$ACK,HOME` | Same as `MOVE,0,0` |
| `$STOP` | `$ACK,STOP` | Stop at the current position |
| `$SPEED` | `$SPEED,<°/s>` | Query the slew speed |
| `$SPEED,<°/s>` | `$ACK,SPEED` | 1–500, applies to both axes |
| `$POS` | `$POS,<pan>,<tilt>,<tpan>,<ttilt>,<state>` | `state`: `D` disabled, `M` moving, `H` holding |
| `$STREAM,<hz>` | `$ACK,STREAM` | 0 = off, 1–50 Hz periodic `$POS` |
| `$LIMITS` | `$LIMITS,<pmin>,<pmax>,<tmin>,<tmax>` | Soft limits in degrees |

Angles are degrees from center: **pan + = right, tilt + = up**. Numbers may have an optional sign and up to 2 decimal places. `MOVE`, `NUDGE`, `HOME` and `STOP` are refused with `NAK …,DIS` until the head is enabled.

### Messages owlet sends on its own

| Message | When |
|---------|------|
| `$READY,<proto>,<fw>` | Once after boot |
| `$ARRIVED,<pan>,<tilt>` | Exactly once for every accepted `MOVE`/`NUDGE`/`HOME` (cancelled by `STOP`/`DISABLE`) |
| `$POS,…` | At the `STREAM` rate |
| `$ERR,BUS` | A servo update failed to reach the PCA9685 |

### Errors

| Reply | Meaning |
|-------|---------|
| `$NAK,<CMD>,FMT` | Wrong number of fields or a malformed number |
| `$NAK,<CMD>,RNG` | Value out of range |
| `$NAK,<CMD>,DIS` | Servos are disabled; send `ENABLE` first |
| `$NAK,<CMD>,BUS` | PCA9685 didn't respond (check wiring and power) |
| `$ERR,CHK` | Checksum mismatch |
| `$ERR,SYN` | Not a valid `$…*HH` frame |
| `$ERR,LEN` | Line longer than 82 bytes |
| `$ERR,UNK` | Unknown command |

Every valid request gets exactly one reply, in order. The full specification, including edge cases and the reasoning behind them, is in [`docs/DESIGN.md` §5](docs/DESIGN.md#5-wire-protocol-v1).

> **About "position":** hobby servos don't report where they actually are. owlet reports the position it has *commanded*, rate-limited the same way the servo moves. At normal speeds the real servo follows it closely. If a servo is blocked or overloaded, the two will differ.

## Configuration and calibration

Settings live in ESP-IDF's Kconfig, under the **owlet configuration** menu (`idf.py menuconfig` inside `apps/esp32-owlet`). You can also put them in a local `apps/esp32-owlet/sdkconfig.defaults.local` file (copy the [`.example`](apps/esp32-owlet/sdkconfig.defaults.local.example)). That file is git-ignored, so your per-build values stay out of the repo.

| Setting | Default | Meaning |
|---------|---------|---------|
| `OWLET_I2C_SDA_GPIO` / `OWLET_I2C_SCL_GPIO` | 1 / 2 | I²C pins |
| `OWLET_PAN_CHANNEL` / `OWLET_TILT_CHANNEL` | 0 / 1 | PCA9685 channels |
| `OWLET_<AXIS>_PULSE_MIN_US` | 500 | Pulse at −90° |
| `OWLET_<AXIS>_PULSE_CENTER_US` | 1500 | Pulse at 0° |
| `OWLET_<AXIS>_PULSE_MAX_US` | 2500 | Pulse at +90° |
| `OWLET_<AXIS>_LIMIT_MIN_DEG` / `_MAX_DEG` | −90 / 90 | Soft limits; commands outside them are refused |

- **Axis moves the wrong way?** Swap its `PULSE_MIN_US` and `PULSE_MAX_US`.
- **Mechanism collides before ±90°?** Narrow the soft limits.
- **0° isn't quite centered?** Adjust `PULSE_CENTER_US`. The mapping bends at center, so this doesn't disturb the endpoints.

Many servos reach their physical end stop before 500 or 2500 µs. Measure yours with the step-by-step procedure in [`docs/CALIBRATION.md`](docs/CALIBRATION.md).

### Mounting the servo horns

1. Power the board and run `owlet enable 0 0`. Both servos will hold at center.
2. Fit the horns so the head faces straight ahead and level. Don't twist the shafts by hand while they're energised.
3. Run `owlet disable` when you're done.

## Safety

- **Limp at boot.** Every servo output is off after power-up or reset. Nothing moves until a computer sends `ENABLE`.
- **The first `ENABLE` jumps.** A hobby servo moves to its first commanded position at full speed, because nothing can control its speed before it knows where it is. Use `ENABLE,<pan>,<tilt>` with the pose the head is actually in to keep that jump small.
- **No silent queuing.** Move commands sent while disabled are refused, not saved for later, so enabling never triggers an unexpected motion.
- **Soft limits** are enforced in firmware. Set them inside your mechanism's hard stops.

## Building from source

Everything below runs inside `nix develop`.

```bash
./build.py --host -t          # host build + all tests (C++ and Python)
./build.py --host --clean -t  # same, from scratch
./build.py --esp              # firmware only
./build.py --esp --flash      # firmware + flash
```

Run one test suite or test:

```bash
ctest --test-dir build/host -R NmeaProtocol --output-on-failure
build/host/tests/owlet_servo_test --gtest_filter='Servo.Slew*'
cd tools/python && python -m unittest tests.test_protocol
```

> Nix flakes only see files that git knows about. If you add a file and the build can't find it, run `git add` on it first.

**Without Nix:** `apps/esp32-owlet` is a standard ESP-IDF v5.5 project, so with ESP-IDF installed natively, `cd apps/esp32-owlet && idf.py build flash` should work. This path isn't regularly tested. The host tests only need CMake, Ninja and a C++17 compiler (GoogleTest is downloaded automatically).

### Debug logs

The USB port carries **only** protocol traffic. Firmware logs go to UART0: connect a USB-TTL adapter's RX to **GP43** (and GND), then open it at 115200 baud.

## Project layout

```
owlet/
├── apps/
│   ├── esp32-owlet/        ESP-IDF firmware project (entry point and Kconfig)
│   └── host-sim/           owlet_sim: the firmware core on a Linux pseudo-terminal
├── drivers/
│   ├── owlet_hal/          hardware interfaces (I²C, delay, servo channel, serial port)
│   ├── owlet_pca9685/      PCA9685 PWM driver
│   └── owlet_servo/        servo model: angle↔pulse, speed-limited motion
├── firmware/common/
│   ├── owlet_core/         interfaces (IPanTilt, IProtocol), PanTilt, ControlLoop, ProtocolMux
│   └── owlet_nmea/         the native protocol: codec + NmeaProtocol
├── platform/owlet_esp32/   ESP-IDF implementations of the hardware interfaces
├── simulators/             virtual I²C bus + register-level PCA9685 model
├── tests/                  GoogleTest suites
├── tools/python/           pip-installable client library and `owlet` CLI
├── docs/                   design, protocol, calibration, dev journal
├── build.py                build/test/flash entry point
└── flake.nix               reproducible toolchain
```

Code in `drivers/` and `firmware/common/` is plain C++17 with no ESP-IDF headers and no dynamic memory, and it builds and runs on a PC. Only `platform/` and `apps/esp32-owlet/` touch ESP-IDF. Read [`docs/DESIGN.md`](docs/DESIGN.md) for the architecture and the reasoning behind it.

## Troubleshooting

| Symptom | Likely cause and fix |
|---------|-------------------|
| `could not auto-detect the owlet port` | No board, or several boards, found. Run `owlet ports`, then pass `-p PORT`. |
| `Permission denied: '/dev/ttyACM…'` (Linux) | Add yourself to the group that owns the device (`ls -l /dev/ttyACM*`, usually `dialout` or `plugdev`), then log out and back in. |
| `no answer from owlet` | Wrong port, firmware not flashed, or something else (e.g. a serial monitor) has the port open. |
| `MOVE refused: DIS` | Servos are disabled. Run `owlet enable` first. |
| `ENABLE refused: BUS` | The PCA9685 isn't answering. Check SDA/SCL (GP1/GP2), 3V3 to `VCC`, and the common GND. |
| Board resets when the servos move | 5 V brown-out. Power the servos (`V+`) from a separate supply. |
| An axis moves the wrong way | Swap that axis's min/max pulse ([Configuration](#configuration-and-calibration)). |
| Servo buzzes at the ends of travel | It's pressing on its end stop. Calibrate the endpoints or narrow the limits. |

## Project status and roadmap

**v0.1.0, alpha.** The firmware, protocol, PC client and simulator are complete and tested. Motion has been verified on real hardware.

- [x] Firmware: USB protocol, speed-limited motion, boot-limp safety
- [x] Python library and CLI with port auto-detection
- [x] Host simulator and end-to-end tests
- [x] First motion on hardware
- [ ] Servo calibration for the reference mechanism ([`docs/CALIBRATION.md`](docs/CALIBRATION.md))
- [ ] Verify the client on Windows and macOS
- [ ] Separate speed per axis
- [ ] Mechanical design files for the pan-tilt bracket

Progress, decisions and test evidence are logged in [`docs/DEV_JOURNAL.md`](docs/DEV_JOURNAL.md).

## Contributing

Issues and pull requests are welcome. Before opening a PR:

1. Run `./build.py --host -t`; every test must pass.
2. Add or update tests for any behaviour you change. All device behaviour lives in `firmware/common/` so it can be tested on a PC.
3. If you change the native protocol, update the firmware (`owlet_nmea`), the Python client, and `docs/DESIGN.md` §5 together.
4. Keep firmware code free of dynamic allocation, exceptions and RTTI, following the existing style (C++17, Allman braces, `m_` member prefix).

## License

owlet is released under the [MIT License](LICENSE).

## Acknowledgements

- owlet grew out of **clops**, a one-eyed robot built on the same ESP32-S3 + PCA9685 stack. Its HAL, PCA9685 driver and simulator were carried over.
- Toolchain packaging by [nixpkgs-esp-dev](https://github.com/mirrexagon/nixpkgs-esp-dev).
- Framing inspired by NMEA 0183.
