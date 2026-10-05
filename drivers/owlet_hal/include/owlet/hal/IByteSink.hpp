#pragma once

#include <cstddef>

namespace owlet::hal
{

/// Output half of a byte stream. Protocols get only this half, so they can
/// reply but can't consume input meant for another protocol.
class IByteSink
{
public:
    virtual ~IByteSink() = default;

    /// Queue `len` bytes for transmission. Returns false if they could not all
    /// be accepted (e.g. no host connected); callers treat that as a dropped
    /// line, not a fatal error.
    [[nodiscard]] virtual bool write(const char* data, std::size_t len) = 0;
};

} // namespace owlet::hal
