# platform/owlet_esp32

> **The servo head is untested on the current firmware.** It was last run on real hardware with firmware 0.3.0. Everything added since (the motor-neutral axis layer, events, trouble codes, limit switches) builds for the ESP32-S3 and passes the PC tests and the simulator, which use the servo code, but it has not been flashed to a servo head again. Treat this path as unverified until someone re-runs it: `./build.py --esp --flash`, then `owlet selftest`. The stepper head is the one in active use.

ESP-IDF implementations of the HAL interfaces declared in `drivers/owlet_hal/`. This is the only HAL layer that may include ESP-IDF headers. It is registered as the ESP-IDF component `owlet_esp32` and used by `apps/esp32-owlet/`.

- `Esp32I2CBus`: `II2CBus` on the ESP-IDF 5.x `i2c_master` driver (GP1 = SDA, GP2 = SCL by default).
- `Esp32Delay`: `IDelayProvider` using `esp_rom_delay_us` / `vTaskDelay`.
- `Esp32UsbSerial`: `ISerialPort` on the USB-Serial-JTAG peripheral. The PC sees it as `/dev/ttyACM*` / `COMx`.

## Why the directory is prefixed `owlet_`

ESP-IDF names each component after its directory. Plain names like `hal` or `esp32` collide with IDF built-ins, so every shared directory in this repo carries an `owlet_` prefix.
