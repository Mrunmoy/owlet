# owlet — Design Document

*Last updated: 2026-10-01*

## 1. Goals

- A two-axis **pan-tilt head** that any PC can drive over a single USB cable.
- A **documented, human-readable wire protocol** with integrity checking, so it can be scripted from any language or typed by hand.
- The PC can **command a pose** and **know the current pose** at any time, either by polling or by streaming.
- Reuse the proven pieces from the [clops](../../clops) one-eyed robot: the HAL interfaces, PCA9685 driver, simulators and build system.
- Keep **all behaviour host-testable**. On clops the real behaviour lived in an ESP-only `main.cpp` with no tests; owlet keeps `main.cpp` as pure wiring.

## 2. Non-goals

- No WiFi / BLE / web UI. The USB CDC link is the only control surface.
- No closed-loop position feedback (hobby servos don't report position; see §6.4).
- No trajectory planning beyond a constant-velocity slew per axis.

## 3. Hardware

| Component                | Details                                                    |
|--------------------------|------------------------------------------------------------|
| MCU                      | Waveshare ESP32-S3 Zero (native USB on the USB-C connector) |
| Servo controller         | PCA9685, I2C address 0x40, 50 Hz                           |
| Actuators                | 2× 180° hobby servos (MG90S-class): ±90° from center       |
| Power                    | ESP USB-C 5 V → PCA9685 V+ (bench); separate 5 V supply recommended for loaded heads |

### 3.1 Pin map

| ESP32-S3 Zero | PCA9685 / other | Function                              |
|---------------|-----------------|---------------------------------------|
| `GND`         | `GND`           | common ground                         |
| `3V3`         | `VCC`           | logic supply                          |
| `5V`          | `V+`            | servo rail                            |
| `GP1`         | `SDA`           | I2C data                              |
| `GP2`         | `SCL`           | I2C clock                             |
| USB-C         | PC              | **protocol** (USB-Serial-JTAG, CDC-ACM) |
| `GP43` (TX0)  | optional USB-TTL RX | debug log console (UART0, 115200)  |

PCA9685 channels: **0 = pan**, **1 = tilt**.

### 3.2 Why the logs move off USB

On the ESP32-S3 the USB-C port is the USB-Serial-JTAG peripheral, which enumerates as a standard CDC-ACM serial port (`/dev/ttyACM*` on Linux, `COMx` on Windows, `/dev/cu.usbmodem*` on macOS). No driver or adapter is needed. By default ESP-IDF sends its log console to this port, which would interleave `I (123) ...` lines with protocol frames. owlet points the primary console at UART0 (GP43/GP44) and disables the secondary console, so the USB port carries only protocol traffic. If you want logs, attach a USB-TTL adapter to GP43.

The ROM bootloader still prints a short banner on USB at reset. The protocol framer ignores everything that doesn't start with `$`, and the firmware announces itself with `$READY` once it is ready, so clients should wait for that.

## 4. Software architecture

```
┌──────────────────────────────────────────────────────────────┐
│ apps/esp32-owlet/main      composition root (wiring only)     │
│ apps/host-sim              same core on a Linux pty           │
└──────────────┬───────────────────────────────────────────────┘
               │
┌──────────────┴───────────────┐  ┌─────────────────────────────┐
│ firmware/common/owlet_core   │  │ firmware/common/            │
│   CommandProcessor           │──│   owlet_protocol            │
│   PanTilt                    │  │   Checksum, LineFramer,     │
└──────────────┬───────────────┘  │   Request parser, Reply fmt │
               │                  └─────────────────────────────┘
┌──────────────┴───────────────┐
│ drivers/owlet_servo  Servo   │  angle↔pulse, deg/s slew, engage/release
└──────────────┬───────────────┘
┌──────────────┴───────────────┐
│ drivers/owlet_pca9685        │  register-level PWM driver
└──────────────┬───────────────┘
┌──────────────┴───────────────┐
│ drivers/owlet_hal            │  II2CBus, IDelayProvider, IServoChannel, ISerialPort
└──────────────┬───────────────┘
┌──────────────┴───────────────┐
│ platform/owlet_esp32         │  Esp32I2CBus, Esp32Delay, Esp32UsbSerial
└──────────────────────────────┘
```

Dependency rules:

- Layers depend only downward.
- `drivers/` and `firmware/common/` are C++17 with no ESP-IDF headers and no heap. They build on the host for tests.
- `platform/owlet_esp32/` and `apps/esp32-owlet/` are the only places ESP-IDF headers may appear.
- The `owlet_` directory prefix keeps ESP-IDF component names (= directory basename) unique.
- The core is clocked through `tick(nowMs)`. It never reads a clock or sleeps by itself, so tests use a fake clock.

### 4.1 Control loop

`app_main` runs one loop at 100 Hz:

1. Drain all available bytes from `ISerialPort` into `CommandProcessor::feed()`. Complete frames are parsed and executed immediately, and replies are written back.
2. `CommandProcessor::tick(nowMs)` advances both servos' slew by the elapsed time, emits `$ARRIVED` when a move completes, and emits `$POS` at the streaming rate.

There's one task and one owner of all state, so nothing needs a mutex.

## 5. Wire protocol (v1)

### 5.1 Framing

```
$<BODY>*<CK>\r\n
```

- `BODY` = `NAME[,field]*`. `NAME` is upper-case ASCII letters. Fields are printable ASCII excluding `$`, `*` and `,`.
- `CK` = XOR of every byte of `BODY` (between `$` and `*`, both excluded), as two hex digits. The device always emits upper-case; it accepts either case.
- The line ends with `\n`, and a preceding `\r` is optional. The device always sends `\r\n`.
- The maximum frame length is **82 bytes**, including `$` and `\r\n` (same as NMEA 0183).
- Bytes outside a frame are ignored. A `$` in the middle of a frame discards the partial frame and starts a new one.
- The checksum is **mandatory** in both directions.

Example: `$MOVE,30,-15*3B\r\n`, where `0x3B` is the XOR of `MOVE,30,-15`.

### 5.2 Numbers

Angles and speeds are decimal with an optional sign and up to 2 fractional digits: `30`, `-15.5`, `+0.25`. Exponents, `nan`, `inf` and empty strings are rejected. The device reports angles with exactly one decimal (`-15.5`, `0.0`).

The coordinate frame is: **pan +** = right, **tilt +** = up, **0,0** = calibrated center. Viewed from behind the head, "right" is the head's right. Mounting differences are handled by calibration (an inverted axis swaps min/max pulse), never by the protocol.

### 5.3 Requests (PC → device)

| Request                     | Success reply                         | Notes |
|-----------------------------|---------------------------------------|-------|
| `$PING`                     | `$PONG,<proto>,<fw>`                  | e.g. `$PONG,1,0.1.0` |
| `$ENABLE`                   | `$ACK,ENABLE`                         | Energise both servos at their last known position (initially 0,0). No-op if already enabled. |
| `$ENABLE,<pan>,<tilt>`      | `$ACK,ENABLE`                         | Energise at the given pose (snap). If already enabled, this is a slewed move and earns an `$ARRIVED`. |
| `$DISABLE`                  | `$ACK,DISABLE`                        | PWM off; servos go limp. Target is kept. |
| `$MOVE,<pan>,<tilt>`        | `$ACK,MOVE`                           | Absolute move at the current speed. Out-of-limit → `RNG`. |
| `$NUDGE,<dpan>,<dtilt>`     | `$ACK,NUDGE`                          | Relative to the current **target**, clamped to limits (for jogging). |
| `$HOME`                     | `$ACK,HOME`                           | `MOVE,0,0`. |
| `$STOP`                     | `$ACK,STOP`                           | Target := current position. |
| `$SPEED`                    | `$SPEED,<deg/s>`                      | Query slew speed. |
| `$SPEED,<deg/s>`            | `$ACK,SPEED`                          | 1–500 deg/s, both axes. Default 90. |
| `$POS`                      | `$POS,<pan>,<tilt>,<tpan>,<ttilt>,<st>` | See §5.5. |
| `$STREAM,<hz>`              | `$ACK,STREAM`                         | 0 = off, 1–50 Hz periodic `$POS`. |
| `$LIMITS`                   | `$LIMITS,<pmin>,<pmax>,<tmin>,<tmax>` | Soft limits in degrees. |

`MOVE`, `NUDGE`, `HOME` and `STOP` require the servos to be enabled; otherwise the reply is `$NAK,<CMD>,DIS`. This means a PC can't make the head snap by queuing a move while it's limp. The only way to choose the energise pose is the explicit `ENABLE,<pan>,<tilt>`.

### 5.4 Errors

| Reply              | Meaning |
|--------------------|---------|
| `$NAK,<CMD>,FMT`   | Wrong number of fields or a malformed number |
| `$NAK,<CMD>,RNG`   | Value outside the allowed range |
| `$NAK,<CMD>,DIS`   | Servos disabled; send `ENABLE` first |
| `$NAK,<CMD>,BUS`   | PCA9685 write failed (I2C error) |
| `$ERR,CHK`         | Checksum mismatch: the frame was discarded unread |
| `$ERR,LEN`         | Frame exceeded 82 bytes |
| `$ERR,SYN`         | Frame is not `$...*HH` |
| `$ERR,UNK`         | Valid frame, unknown command name |
| `$ERR,BUS`         | Unsolicited: a slew step failed to reach the PCA9685. Sent once per fault; the step is retried every tick |

Frame-level problems use `$ERR` because the command name can't be trusted. Every well-formed request gets exactly one reply, in order, so a client can match replies to requests without sequence numbers.

### 5.5 Position reporting

```
$POS,<pan>,<tilt>,<target_pan>,<target_tilt>,<state>
```

`state` is `D` (disabled), `M` (moving: slew in progress) or `H` (holding at target).

Unsolicited messages (device → PC):

| Message                 | When |
|-------------------------|------|
| `$READY,<proto>,<fw>`   | Once after boot, when the core is ready |
| `$ARRIVED,<pan>,<tilt>` | Exactly once per accepted `MOVE`/`NUDGE`/`HOME`, even a zero-length one, so clients can always wait on it. `STOP` and `DISABLE` cancel it. |
| `$POS,...`              | At the `STREAM` rate |
| `$ERR,BUS`              | When a slew write fails (see §5.4) |

When streaming is on, a streamed `$POS` and the reply to a `$POS` request look the same; both are current, so either answers the query.

Clients must tolerate unsolicited lines interleaved with replies. A reply is identified by its name (`ACK`/`NAK`/`PONG`/...), not its position in the stream.

## 6. Behaviour

### 6.1 Servo mapping

Each axis has a calibration of `minPulseUs`, `centerPulseUs`, `maxPulseUs` and `minDeg`/`maxDeg`, defaulting to 500/1500/2500 µs ↔ −90/0/+90°. The mapping is **piecewise-linear through center**, so an asymmetric servo stays accurate at 0°. If `minPulseUs > maxPulseUs` the axis is reversed. The defaults are configurable per axis in Kconfig and get measured during bring-up.

### 6.2 Slew

Each axis moves toward its target at `speed` deg/s, based on elapsed time (`tick(nowMs)`) rather than a per-tick step, so loop jitter doesn't change the speed. Each step is capped at 100 ms worth of motion, so a starved loop resumes smoothly instead of lurching. Both axes use the same speed and slew independently, which means a diagonal move finishes the shorter axis first.

### 6.3 Boot and enable (safety)

At boot every PCA9685 channel is full-OFF and the head is limp. This is the clops lesson: a servo's first pulse after power-up snaps at full speed, because no slew applies before the device knows where it is. Nothing moves until a PC sends `ENABLE`. Clients should send `ENABLE` with the pose they believe the head is in (or `0,0` after a manual center) to keep the snap small. A bare `ENABLE` after a mid-move `DISABLE` energises at the last position estimate rather than the old target, because that's where the horn most likely is. If the PCA9685 is missing or unpowered, the firmware still runs the protocol: `ENABLE` replies `NAK,ENABLE,BUS`, which the PC can see without a debug UART.

### 6.4 What "position" means

Hobby servos have no feedback. The reported position is the **commanded pulse** after slew limiting, converted back to degrees. At reasonable speeds (≤ ~200 deg/s unloaded) the real horn tracks it closely. When the servo is stalled, overloaded or limp, the report and reality diverge. This is documented in the protocol rather than hidden: `state=D` says the number is only the last commanded pose.

## 7. PC side

`tools/python/` is a pure-Python package (`pip install ./tools/python`; only needs `pyserial`):

- `owlet.protocol`: encode, decode and checksum (no I/O; unit-tested against `docs/protocol_vectors.txt`, the same file the C++ tests read).
- `owlet.client.Owlet`: high-level API (`ping()`, `enable()`, `move()`, `position()`, `stream()`, ...) that waits for `$READY` and matches replies by name.
- `owlet` / `python -m owlet`: a CLI (`owlet move 30 -15 --wait`, `owlet pos`, `owlet watch`, `owlet shell`). The port is auto-detected by Espressif's USB VID (`0x303A`).

Opening the port: on Linux (verified on hardware, 2026-10-01), opening the USB-Serial-JTAG port with pyserial's defaults (DTR/RTS asserted) does **not** reset the chip. Toggling DTR/RTS individually can, because that is the auto-reset signal `esptool` uses. Clients shouldn't touch the modem lines.

`apps/host-sim/owlet_sim` runs the real `CommandProcessor` + `PanTilt` + `Servo` + `PCA9685` stack against the PCA9685 simulator, behind a Linux pseudo-terminal. You can drive it with the CLI without any hardware, and the end-to-end test uses it.

## 8. Testing strategy

- **Unit (GTest):** Servo, Checksum, LineFramer, request parsing, reply formatting, PanTilt and CommandProcessor (fake serial + fake clock).
- **Integration (GTest):** the real driver stack over `VirtualI2CBus` + `PCA9685Simulator`, asserting on the PWM counts the chip would output.
- **Cross-language:** C++ and Python both validate `docs/protocol_vectors.txt`.
- **End-to-end:** the Python client drives `owlet_sim` over a pty.
- **On-target:** manual, recorded in the dev journal.

## 9. Open questions

| # | Question | Plan |
|---|----------|------|
| 1 | Real pulse endpoints for ±90° on the actual servos (500/2500 µs is nominal; many clones stop short or hit the end stop) | Measure in P7, then update the Kconfig defaults and `docs/CALIBRATION.md`. |
| 2 | Should a mandatory checksum be relaxed for hand-typing in a terminal (e.g. accept `*XX` as "no checksum")? | Start strict; the CLI covers interactive use. Revisit if people ask. |
| 3 | Per-axis speed? | Not in v1. Add `SPEED,<pan>,<tilt>` later if needed. |
