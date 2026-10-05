#pragma once



#include <cstddef>
#include <cstdint>

namespace owlet::core
{

/// consumed > 0              : took these bytes (released=true at end of frame)
/// consumed == 0, released    : "not mine" — give this byte to another protocol
/// consumed == 0, !released   : not allowed (the caller drops the byte)
struct ConsumeResult
{
    std::size_t consumed;
    bool        released;
};

/// A wire protocol: turns bytes into IPanTilt calls and writes replies to the
/// IByteSink it was constructed with. Implement this to plug your own protocol
/// into owlet (see docs/DESIGN.md §4.3–4.4).
class IProtocol
{
public:
    virtual ~IProtocol() = default;

    /// Used by ProtocolMux to pick an owner for the next frame: may this
    /// protocol take the stream starting at `byte`? `atLineStart` is true after
    /// CR/LF or at power-up, for line-oriented protocols that only start there.
    [[nodiscard]] virtual bool claims(uint8_t byte, bool atLineStart) const = 0;

    /// Feed bytes. Return how many were consumed and whether the frame ended.
    [[nodiscard]] virtual ConsumeResult consume(const uint8_t* data, std::size_t len) = 0;

    /// Called once when the system is ready (e.g. to announce itself).
    virtual void begin(uint32_t nowMs) = 0;

    /// Called every loop iteration, after motion has advanced and its events
    /// have been delivered, for output that is driven by time alone.
    virtual void onTick(uint32_t nowMs) = 0;
};

} // namespace owlet::core
