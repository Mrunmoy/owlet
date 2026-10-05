#pragma once

#include "owlet/hal/ISerialPort.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet::platform::esp32
{

/// ISerialPort over the ESP32-S3's built-in USB-Serial-JTAG peripheral, which
/// the PC sees as a plain CDC-ACM port. The console must be routed elsewhere
/// (sdkconfig: CONFIG_ESP_CONSOLE_UART_DEFAULT + SECONDARY_NONE) or log lines
/// would interleave with protocol frames.
class Esp32UsbSerial final : public ::owlet::hal::ISerialPort
{
public:
    static constexpr uint32_t kRxBufferBytes = 512;
    /// Enough for a full 50 Hz POS stream plus replies between loop turns.
    static constexpr uint32_t kTxBufferBytes = 2048;
    static constexpr uint32_t kWriteTimeoutMs = 5;

    Esp32UsbSerial() = default;

    Esp32UsbSerial(const Esp32UsbSerial&)            = delete;
    Esp32UsbSerial& operator=(const Esp32UsbSerial&) = delete;

    [[nodiscard]] bool init();

    [[nodiscard]] std::size_t read(uint8_t* out, std::size_t maxLen) override;
    [[nodiscard]] bool write(const char* data, std::size_t len) override;
};

} // namespace owlet::platform::esp32
