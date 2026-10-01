#pragma once

#include <cstddef>
#include <cstdint>

namespace owlet::hal
{

// Byte-stream transport for the command protocol. On ESP32-S3 this is the
// USB-Serial-JTAG CDC port; on the host simulator it is a pseudo-terminal;
// in tests it is an in-memory fake.
class ISerialPort
{
public:
    virtual ~ISerialPort() = default;

    // Non-blocking. Copies up to `maxLen` already-received bytes into `out`
    // and returns how many were copied (0 when nothing is pending).
    [[nodiscard]] virtual std::size_t read(uint8_t* out, std::size_t maxLen) = 0;

    // Queue `len` bytes for transmission. Returns false if the bytes could not
    // all be accepted (e.g. host not connected and TX buffer full); the
    // protocol layer treats that as a dropped line, not a fatal error.
    [[nodiscard]] virtual bool write(const char* data, std::size_t len) = 0;
};

} // namespace owlet::hal
