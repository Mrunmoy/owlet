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

### 4.1 Layers and interfaces

```
 apps/esp32-owlet, apps/host-sim          composition roots: construct + inject, no logic
        │
        ├── ControlLoop ──────────────► ITickable (motion)      IByteSource (input)
        │        │
        │        ▼
        │   IProtocol ◄── ProtocolMux<N>   (composite: several protocols, one port)
        │        ▲
        │        ├── nmea::NmeaProtocol    owlet native "$…*HH" protocol  (owlet_nmea)
        │        └── your protocol         implement IProtocol, inject it (Pelco-D, G-code, binary, …)
        │                 │
        │                 ▼  (protocols depend only on these abstractions)
        │            IPanTilt  ─────────── IByteSink (output)
        │                 ▲
        ├── PanTilt : IPanTilt, ITickable           owlet_core
        │       └── Servo ── IServoChannel           owlet_servo
        │                        ▲
        ├── PCA9685::Channel ────┘                   owlet_pca9685 ── II2CBus, IDelayProvider
        └── Esp32UsbSerial : IByteSource, IByteSink  owlet_esp32
```

| Interface | Owner | Role |
|-----------|-------|------|
| `hal::IByteSource` / `hal::IByteSink` | `owlet_hal` | Input and output halves of a byte stream. `ISerialPort` combines them. Protocols only receive a sink; the loop only receives a source. |
| `hal::IServoChannel` | `owlet_hal` | One PWM output: the seam for PCA9685, LEDC or anything else. |
| `core::IPanTilt` | `owlet_core` | Everything a protocol may ask of the head: enable/disable, move/nudge/home/stop, speed, pose, limits, state. A stepper-based head would implement this and reuse every protocol unchanged. |
| `core::ITickable` | `owlet_core` | `tick(dtMs) → TickStatus`: advance motion and report events (bus fault, watchdog trip). Only the loop calls it, never a protocol. |
| `core::IProtocol` | `owlet_core` | A wire protocol: `claims()` / `consume()` / `begin()` / `onTick()` (see §4.3). |

The SOLID principles map onto this as follows:

- **Single responsibility:** `ControlLoop` schedules; protocols translate bytes to `IPanTilt` calls; `PanTilt` enforces policy; `Servo` maps and slews; drivers talk to silicon.
- **Open/closed:** adding a protocol or a motion backend means adding a class. No existing class changes.
- **Liskov substitution:** every `IProtocol` works standalone *or* inside `ProtocolMux`, and every `IPanTilt` works under every protocol.
- **Interface segregation:** protocols can't tick motion or read the port; the loop can't issue moves.
- **Dependency inversion:** `owlet_core` depends on no protocol. Protocols depend on `IPanTilt`/`IByteSink`, never on `PanTilt` or a platform type.

Rules:

- `drivers/` and `firmware/common/` are C++17 with no ESP-IDF headers, no heap, no exceptions and no RTTI. They build on the host for tests.
- `platform/owlet_esp32/` and `apps/esp32-owlet/` are the only places ESP-IDF headers may appear.
- The `owlet_` directory prefix keeps ESP-IDF component names (= directory basename) unique.
- Nothing in the core reads a clock or sleeps. Time comes in through `service(nowMs)`, so tests use a fake clock.

### 4.2 Control loop

`ControlLoop::service(nowMs)` runs at 100 Hz on one task:

1. `ITickable::tick(dt)` advances both servos, with `dt` capped at 100 ms so a starved loop resumes smoothly instead of lurching.
2. `IProtocol::onTick(nowMs, status)` lets the protocol emit time-driven output: arrival events, position streams, watchdog and bus-fault reports.
3. Bytes are drained from `IByteSource` in 64-byte chunks and handed to `IProtocol::consume()`. Replies are written synchronously to the protocol's `IByteSink`.

Motion is ticked before input is read, so a reply is never preceded by an event caused by the request it answers. One task owns all state, so nothing needs a mutex.

### 4.3 The `IProtocol` contract

```cpp
struct ConsumeResult { std::size_t consumed; bool released; };

class IProtocol {
    virtual bool claims(uint8_t byte, bool atLineStart) const = 0;  // may I own the stream from here?
    virtual ConsumeResult consume(const uint8_t* data, std::size_t len) = 0;
    virtual void begin(uint32_t nowMs) = 0;                          // boot banner
    virtual void onTick(uint32_t nowMs, const TickStatus& status) = 0;
};
```

- `consume()` must consume at least one byte when `len > 0`, and reports `released = true` at the end of each frame (for line protocols, at CR/LF).
- `claims()` is only used by `ProtocolMux`. Native frames are claimed on `$` anywhere, which keeps NMEA's resync behaviour. A line protocol such as G-code would claim only at line start (`atLineStart`), and a binary protocol such as Pelco-D would claim its `0xFF` sync byte anywhere.
- `ProtocolMux<N>` offers each frame's first byte to its protocols in order. The first one that claims it owns the stream until it releases. Bytes nobody claims are dropped up to the end of the line, which is how the ROM boot banner is discarded. The mux feeds whole chunks to the owner, so each chunk costs one virtual call rather than one per byte.
- The mux is itself an `IProtocol` (composite), so a single protocol and a mux are interchangeable at the composition root.

### 4.4 Writing your own protocol

1. Create a component (e.g. `firmware/common/my_proto/`) that depends on `owlet_core` only.
2. Implement `IProtocol`. Take `IPanTilt&` and `IByteSink&` in the constructor.
3. In `apps/esp32-owlet/main/main.cpp`, construct it and add it to the `ProtocolMux` (or pass it to `ControlLoop` directly if it should be the only protocol).
4. Unit-test it on the host with a real `PanTilt` over `FakeServoChannel`, or with your own `IPanTilt` fake. See `tests/test_nmea_protocol.cpp` for the pattern.

### 4.5 Efficiency

- No dynamic allocation anywhere in firmware code. Every buffer is a fixed `std::array` sized at compile time (frame 80 B, reply line 82 B, read chunk 64 B).
- Virtual dispatch happens once per chunk of input and once per tick per protocol, never per byte inside a protocol.
- Number formatting is integer fixed-point, so neither target nor host uses `printf` float formatting.
- `ProtocolMux<N>` takes its protocol count as a template parameter and stores plain pointers in a `std::array`.

### 4.6 Using owlet as a submodule

- **Host CMake:** `add_subdirectory(owlet)` gives the `owlet::hal`, `owlet::pca9685`, `owlet::servo`, `owlet::core` and `owlet::nmea` targets. Tests and the simulator are built only when owlet is the top-level project (`OWLET_BUILD_TESTS` defaults to `PROJECT_IS_TOP_LEVEL`).
- **ESP-IDF:** `include(owlet/cmake/owlet-idf-components.cmake)` and append `${OWLET_IDF_COMPONENT_DIRS}` to `EXTRA_COMPONENT_DIRS`. The bundled app uses this same list.
- **Python:** `pip install ./owlet/tools/python`, or `pip install -e` for development.

## 5. Native wire protocol (v2)

This is owlet's built-in protocol. It is designed for **closed-loop control from a PC**, such as camera-based object tracking: commands replace the current goal immediately instead of queuing, velocity mode with a safety watchdog maps directly onto a tracking controller's output, and timestamped position streaming lets the PC know where the head pointed when each camera frame was taken. Other protocols can be added as plug-ins (§4.4).

Version 2 (owlet 0.2.0) added `VEL`, `WATCHDOG`, `CLOCK`, `$TIMEOUT`, and the timestamp field on `$POS`. Clients must check the version in `$PONG`.

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
| `$PING`                     | `$PONG,<proto>,<fw>`                  | e.g. `$PONG,2,0.2.0` |
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
| `$VEL,<pan°/s>,<tilt°/s>`   | `$ACK,VEL`                            | Velocity mode: turn continuously at these signed rates (each −500…500). The head stops at soft limits without error. Replaces any position move (no `$ARRIVED`); any `MOVE`/`NUDGE`/`HOME`/`STOP`/`DISABLE` ends it. Guarded by the watchdog. |
| `$WATCHDOG`                 | `$WATCHDOG,<ms>`                      | Query the velocity watchdog timeout. |
| `$WATCHDOG,<ms>`            | `$ACK,WATCHDOG`                       | 0 = off, otherwise 50–5000 ms. Default 500. |
| `$CLOCK`                    | `$CLOCK,<ms>`                         | Device uptime in ms (wraps after ~49.7 days), for clock sync. |

`MOVE`, `NUDGE`, `HOME`, `STOP` and `VEL` require the servos to be enabled; otherwise the reply is `$NAK,<CMD>,DIS`. This means a PC can't make the head snap by queuing a move while it's limp. The only way to choose the energise pose is the explicit `ENABLE,<pan>,<tilt>`.

**Velocity watchdog.** In velocity mode the PC must refresh `VEL` at least once per watchdog period. If it doesn't, because the tracking program crashed, hung or lost the cable, the device stops both axes where they are (staying enabled) and sends `$TIMEOUT,VEL` once. Position moves aren't guarded: they end by themselves.

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
$POS,<pan>,<tilt>,<target_pan>,<target_tilt>,<state>,<ms>
```

`state` is `D` (disabled), `M` (moving: slew or velocity mode in progress) or `H` (holding). `ms` is the device uptime at which the pose was sampled, the same clock as `$CLOCK`. In velocity mode the target equals the position.

**Latency compensation.** A tracking loop needs to know where the camera pointed *when a frame was captured*, not when the detection finished. The client estimates the device-to-host clock offset with `$CLOCK` round trips (taking the minimum-RTT sample), stamps every streamed `$POS` with the host time, and interpolates the pose at any host timestamp (§7).

Unsolicited messages (device → PC):

| Message                 | When |
|-------------------------|------|
| `$READY,<proto>,<fw>`   | Once after boot, when the core is ready |
| `$TIMEOUT,VEL`          | The velocity watchdog stopped the head |
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
- `owlet.tracking`: the pieces of a tracking loop.
  - `Tracker(head)`: background reader with a 50 Hz `$POS` stream, clock sync, `set_rates()` (sends `VEL` without waiting for each ACK), `pose_at(host_time)` interpolation and a watchdog-friendly refresh.
  - `PixelController`: turns a detection's pixel offset into pan/tilt rates using the camera's field of view, a gain, a deadband and a rate limit.
  - `examples/track_object.py`: a webcam + YOLO loop built on these. It is optional and needs `opencv-python` and `ultralytics`, which the library itself doesn't depend on.
- `owlet` / `python -m owlet`: a CLI (`owlet move 30 -15 --wait`, `owlet pos`, `owlet watch`, `owlet shell`). The port is auto-detected by Espressif's USB VID (`0x303A`).

Opening the port: on Linux (verified on hardware, 2026-10-01), opening the USB-Serial-JTAG port with pyserial's defaults (DTR/RTS asserted) does **not** reset the chip. Toggling DTR/RTS individually can, because that is the auto-reset signal `esptool` uses. Clients shouldn't touch the modem lines.

`apps/host-sim/owlet_sim` runs the real `ControlLoop` + `NmeaProtocol` + `PanTilt` + `Servo` + `PCA9685` stack against the PCA9685 simulator, behind a Linux pseudo-terminal. You can drive it with the CLI without any hardware, and the end-to-end test uses it.

## 8. Testing strategy

- **Unit (GTest):** Servo, the NMEA codec (checksum, framer, parser, writer), PanTilt, ProtocolMux and ControlLoop (with fake protocols, tickers and ports), and NmeaProtocol (fake serial + fake clock).
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
