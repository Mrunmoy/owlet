#pragma once

#include "owlet/hal/ISerialPort.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet::platform::pico
{

// ISerialPort on the Pico's native USB as a CDC-ACM device (pico-sdk
// stdio_usb underneath). The port carries the protocol and nothing else:
// newline translation is off and nothing in the firmware printf()s to it.
//
// Writes never stall the control loop for long: if the host stops reading,
// a write gives up after PICO_STDIO_USB_STDOUT_TIMEOUT_US (set by the app's
// build) and the line is dropped, which the protocol already tolerates.
class PicoUsbSerial final : public ::owlet::hal::ISerialPort
{
public:
    [[nodiscard]] bool init();

    [[nodiscard]] std::size_t read(uint8_t* out, std::size_t maxLen) override;
    // False when no host has the port open: the bytes are dropped.
    [[nodiscard]] bool write(const char* data, std::size_t len) override;
};

} // namespace owlet::platform::pico
