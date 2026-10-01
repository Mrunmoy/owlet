#pragma once

#include "owlet/core/IProtocol.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/hal/IByteSource.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet::core
{

// Protocol-agnostic scheduler. Each service() call:
//   1. advances motion by the elapsed time (capped),
//   2. lets the protocol emit time-driven output for that step,
//   3. drains pending input into the protocol.
// Motion runs before input so a reply is never preceded by an event its own
// request caused.
class ControlLoop
{
public:
    // A starved loop resumes with at most this much motion per step.
    static constexpr uint32_t    kMaxTickMs = 100;
    static constexpr std::size_t kReadChunk = 64;

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
