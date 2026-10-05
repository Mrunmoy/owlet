# platform/owlet_pico

pico-sdk implementations of the HAL interfaces declared in `drivers/owlet_hal/`, for the Raspberry Pi Pico (RP2040) stepper head. Together with `apps/pico-owlet/` this is the only place pico-sdk headers may appear.

- `PicoClock`: `IClock` on the 64-bit microsecond timer.
- `PicoDelay`: `IDelayProvider` on `sleep_us` / `sleep_ms`.
- `PicoUsbSerial`: `ISerialPort` on the native USB port as CDC-ACM (`/dev/ttyACM*` / `COMx`), through the SDK's `stdio_usb` driver with newline translation off. Nothing else writes to the port.
- `PicoDigitalOutput`: `IDigitalOutput` on a GPIO, with the polarity (active high or low) set here. The pin is driven to its off level before it becomes an output.
- `PicoStepProgram` + `PicoStepGenerator`: `IStepGenerator` on PIO. One ten-instruction program per PIO block, one state machine per motor.
- `PicoFlashCalibrationStore`: `ICalibrationStore` in the last sector of the on-board flash.

## Step pulses

A burst is two FIFO words: the pulse spacing, and the step count with the direction in bit 0. The state machine sets `DIR`, then emits the pulses on a 5 MHz clock (0.2 µs resolution, 0.8 µs `STEP` high time). Once a burst is queued, nothing the CPU does can move its pulses. `emit()` refuses a burst while an earlier one is still waiting, so at most one is ever queued and commands take effect within a control period or two.

The program is assembled at start-up from the SDK's `pio_encode_*` helpers, so the build needs no `pioasm` step.

## Build

This is an INTERFACE library: its sources compile as part of the application, so they see the same SDK configuration (stdio, board) the app does. The app sets `PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000` so a host that stops reading cannot stall the control loop for more than 5 ms per write.

`PICO_SDK_PATH` must point at an SDK **with submodules** (TinyUSB). The Nix flake provides one; the plain nixpkgs `pico-sdk` package does not.

## Flashing

`./build.py --pico --flash` needs no button. It asks the running firmware to reboot into the bootloader (picotool's reboot request, which `stdio_usb` answers; then a 1200-baud open; then MicroPython's `machine.bootloader()`), loads the UF2 and restarts it.

## Verified on hardware (2026-10-05)

On a USB-C Pico clone: enumerates as `2e8a:000a`, the client auto-detects it, `ping`/`caps`/`limits`/`pos`/`cal` answer, velocity mode runs and the watchdog stops it, and a re-flash with no button press works.

With motors attached (5 V supply): both axes move smoothly and quietly; pan swept ±90° and tilt ±60° and returned to the centre mark, so the step count, direction pin and pulse timing are right on both state machines. The activity LED (wired to sink into GP22) lights for exactly the duration of a move.

**Still to check:** that a calibration save survives a power cycle; behaviour above 30 °/s and on 12 V.
