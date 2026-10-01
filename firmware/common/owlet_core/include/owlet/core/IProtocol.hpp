#pragma once

#include "owlet/core/ITickable.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet::core
{

struct ConsumeResult
{
    std::size_t consumed;  // bytes taken; must be > 0 whenever input was offered
    bool        released;  // true at the end of a frame: the stream is free again
};

// A wire protocol: turns bytes into IPanTilt calls and writes replies to the
// IByteSink it was constructed with. Implement this to plug your own protocol
// into owlet (see docs/DESIGN.md §4.3–4.4).
class IProtocol
{
public:
    virtual ~IProtocol() = default;

    // Used by ProtocolMux to pick an owner for the next frame: may this
    // protocol take the stream starting at `byte`? `atLineStart` is true after
    // CR/LF or at power-up, for line-oriented protocols that only start there.
    [[nodiscard]] virtual bool claims(uint8_t byte, bool atLineStart) const = 0;

    // Feed bytes. Return how many were consumed and whether the frame ended.
    [[nodiscard]] virtual ConsumeResult consume(const uint8_t* data, std::size_t len) = 0;

    // Called once when the system is ready (e.g. to announce itself).
    virtual void begin(uint32_t nowMs) = 0;

    // Called every loop iteration after motion has advanced.
    virtual void onTick(uint32_t nowMs, const TickStatus& status) = 0;
};

} // namespace owlet::core
