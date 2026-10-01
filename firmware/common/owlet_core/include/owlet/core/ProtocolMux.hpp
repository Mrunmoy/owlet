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
// first that claims it owns the stream until it reports `released`. An owner
// may also hand back a byte it doesn't want ({0, released}), which is then
// offered to the others. Bytes nobody claims are returned to the caller, which
// drops them (boot banners, line noise). The mux is itself an IProtocol, so it can stand anywhere a single
// protocol can, including inside another mux.
//
//   TextProtocol   native(usb, head, version);
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
        // A parent mux is about to hand us this byte: adopt its view of the
        // line so nested arbitration matches the outer one.
        m_hintPending   = true;
        m_hintLineStart = atLineStart;
        return find(byte, atLineStart) != nullptr;
    }

    [[nodiscard]] ConsumeResult consume(const uint8_t* data, std::size_t len) override
    {
        if (m_hintPending)
        {
            m_atLineStart = m_hintLineStart;
            m_hintPending = false;
        }
        std::size_t i = 0;
        while (i < len)
        {
            bool fresh = false;
            if (m_owner == nullptr)
            {
                m_owner = find(data[i], m_atLineStart);
                if (m_owner == nullptr)
                {
                    // Nobody here wants it: give it back. At top level the
                    // loop drops it; a parent mux may offer it elsewhere.
                    m_atLineStart = isEol(data[i]);
                    return {i, true};
                }
                fresh = true;
            }

            const ConsumeResult r = m_owner->consume(data + i, len - i);
            if (r.consumed == 0)
            {
                m_owner = nullptr;
                if (r.released && !fresh)
                {
                    m_atLineStart = false;  // mid-frame hand-back: re-offer this byte
                    continue;
                }
                // A protocol that claims a byte and then takes nothing would
                // spin forever: drop the byte instead.
                m_atLineStart = isEol(data[i]);
                ++i;
                continue;
            }
            i += r.consumed;
            if (r.released)
            {
                m_owner       = nullptr;
                m_atLineStart = isEol(data[i - 1]);
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
    [[nodiscard]] static bool isEol(uint8_t b) { return b == '\r' || b == '\n'; }

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
    mutable bool              m_hintPending{false};   // set by claims(), used by consume()
    mutable bool              m_hintLineStart{false};
};

} // namespace owlet::core
