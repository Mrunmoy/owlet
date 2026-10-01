#pragma once

#include "owlet/core/IProtocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::core
{

// Runs several protocols on one byte stream.
//
// The first byte of each frame is offered to the protocols in order; the
// first that claims it owns the stream until it reports `released`. Bytes no
// protocol claims are dropped up to the end of the line (boot banners, line
// noise). The mux is itself an IProtocol, so it can stand anywhere a single
// protocol can, including inside another mux.
//
//   NmeaProtocol   native(usb, head, version);
//   MyProtocol     mine(usb, head);
//   ProtocolMux<2> mux({&native, &mine});
//   ControlLoop    loop(usb, head, mux);
template <std::size_t N>
class ProtocolMux final : public IProtocol
{
public:
    static_assert(N > 0, "ProtocolMux needs at least one protocol");

    explicit ProtocolMux(const std::array<IProtocol*, N>& protocols) : m_protocols(protocols) {}

    ProtocolMux(const ProtocolMux&)            = delete;
    ProtocolMux& operator=(const ProtocolMux&) = delete;

    [[nodiscard]] bool claims(uint8_t byte, bool atLineStart) const override
    {
        return find(byte, atLineStart) != nullptr;
    }

    [[nodiscard]] ConsumeResult consume(const uint8_t* data, std::size_t len) override
    {
        std::size_t i = 0;
        while (i < len)
        {
            if (m_owner == nullptr)
            {
                m_owner = find(data[i], m_atLineStart);
                if (m_owner == nullptr)
                {
                    // Unclaimed: drop this byte; only a line break makes the
                    // next byte a line start again.
                    m_atLineStart = (data[i] == '\r' || data[i] == '\n');
                    ++i;
                    continue;
                }
            }

            const ConsumeResult r = m_owner->consume(data + i, len - i);
            if (r.consumed == 0)
            {
                // Contract violation by the owner; drop the byte rather than spin.
                m_owner = nullptr;
                ++i;
                continue;
            }
            i += r.consumed;
            if (r.released)
            {
                m_owner       = nullptr;
                m_atLineStart = true;
                // Report frame boundaries upward so a parent mux can re-arbitrate.
                return {i, true};
            }
        }
        return {i, false};
    }

    void begin(uint32_t nowMs) override
    {
        for (IProtocol* p : m_protocols)
        {
            p->begin(nowMs);
        }
    }

    void onTick(uint32_t nowMs, const TickStatus& status) override
    {
        for (IProtocol* p : m_protocols)
        {
            p->onTick(nowMs, status);
        }
    }

private:
    [[nodiscard]] IProtocol* find(uint8_t byte, bool atLineStart) const
    {
        for (IProtocol* p : m_protocols)
        {
            if (p->claims(byte, atLineStart))
            {
                return p;
            }
        }
        return nullptr;
    }

    std::array<IProtocol*, N> m_protocols;
    IProtocol*                m_owner{nullptr};
    bool                      m_atLineStart{true};
};

} // namespace owlet::core
