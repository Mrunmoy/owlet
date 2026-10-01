#include "owlet/core/ControlLoop.hpp"

#include <array>

namespace owlet::core
{

ControlLoop::ControlLoop(::owlet::hal::IByteSource& input, ITickable& motion, IProtocol& protocol)
    : m_input(input), m_motion(motion), m_protocol(protocol)
{
}

void ControlLoop::begin(uint32_t nowMs)
{
    m_lastTickMs = nowMs;
    m_protocol.begin(nowMs);
}

void ControlLoop::service(uint32_t nowMs)
{
    uint32_t dt  = nowMs - m_lastTickMs;  // unsigned: correct across the millis wrap
    m_lastTickMs = nowMs;
    if (dt > kMaxTickMs)
    {
        dt = kMaxTickMs;
    }

    const TickStatus status = m_motion.tick(dt);
    m_protocol.onTick(nowMs, status);

    std::array<uint8_t, kReadChunk> chunk{};
    for (std::size_t n = m_input.read(chunk.data(), chunk.size()); n != 0;
         n             = m_input.read(chunk.data(), chunk.size()))
    {
        std::size_t at = 0;
        while (at < n)
        {
            const ConsumeResult r = m_protocol.consume(chunk.data() + at, n - at);
            // A protocol that takes nothing would stall the loop; skip a byte.
            at += (r.consumed != 0) ? r.consumed : 1;
        }
    }
}

} // namespace owlet::core
