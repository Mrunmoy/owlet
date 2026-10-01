#pragma once

#include "owlet/core/IProtocol.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/hal/IByteSource.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet::core
{

// Protocol-agnostic scheduler. Each service() call:
//   1. advances motion by the real elapsed time (the motion layer caps its
//      own step; timeouts such as the watchdog need wall time),
//   2. lets the protocol emit time-driven output for that step,
//   3. feeds pending input to the protocol, at most kMaxChunksPerService
//      chunks, so a flood can't stall motion or the watchdog.
// Motion runs before input so a reply is never preceded by an event its own
// request caused.
class ControlLoop
{
public:
    static constexpr std::size_t kReadChunk           = 64;
    static constexpr std::size_t kMaxChunksPerService = 4;  // 256 B per 10 ms ≈ 25 KB/s

    ControlLoop(::owlet::hal::IByteSource& input, ITickable& motion, IProtocol& protocol);

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    void begin(uint32_t nowMs);
    void service(uint32_t nowMs);

private:
    ::owlet::hal::IByteSource& m_input;
    ITickable&                 m_motion;
    IProtocol&                 m_protocol;
    uint32_t                   m_lastTickMs{0};
};

} // namespace owlet::core
