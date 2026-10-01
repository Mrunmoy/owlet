#pragma once

#include <cstddef>
#include <cstdint>

namespace owlet::hal
{

// Input half of a byte stream (USB CDC, UART, pty, test fake).
class IByteSource
{
public:
    virtual ~IByteSource() = default;

    // Non-blocking. Copies up to `maxLen` already-received bytes into `out`
    // and returns how many were copied (0 when nothing is pending).
    [[nodiscard]] virtual std::size_t read(uint8_t* out, std::size_t maxLen) = 0;
};

} // namespace owlet::hal
