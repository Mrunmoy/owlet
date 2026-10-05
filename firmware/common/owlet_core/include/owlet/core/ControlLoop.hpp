#pragma once

#include "owlet/core/EventBus.hpp"
#include "owlet/core/IProtocol.hpp"
#include "owlet/core/LoopStats.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/hal/IByteSource.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet::core
{

/// Protocol-agnostic scheduler. Each service() call:
///   1. advances motion by the real elapsed time (the motion layer caps its
///      own step; timeouts such as the watchdog need wall time),
///   2. delivers the events that step and any interrupt raised,
///   3. lets the protocol emit time-driven output,
///   4. feeds pending input to the protocol, at most kMaxChunksPerService
///      chunks, so a flood can't stall motion or the watchdog.
/// Motion runs before input so a reply is never preceded by an event its own
/// request caused.
class ControlLoop
{
public:
    static constexpr std::size_t kReadChunk           = 64;
    static constexpr std::size_t kMaxChunksPerService = 4;  // 256 B per 10 ms ≈ 25 KB/s

    /// `events` is optional: a loop without one delivers no events.
    ControlLoop(::owlet::hal::IByteSource& input,
                ITickable&                 motion,
                IProtocol&                 protocol,
                EventBus*                  events = nullptr);

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    /// Has the loop record how well it keeps time.
    /// @param stats    Where to record it; must outlive the loop.
    /// @param periodMs The period service() is meant to be called at.
    void watchTiming(LoopStats& stats, uint32_t periodMs)
    {
        m_stats    = &stats;
        m_periodMs = periodMs;
    }

    void begin(uint32_t nowMs);
    void service(uint32_t nowMs);

private:
    ::owlet::hal::IByteSource& m_input;
    ITickable&                 m_motion;
    IProtocol&                 m_protocol;
    EventBus*                  m_events;
    LoopStats*                 m_stats{nullptr};
    uint32_t                   m_periodMs{0};
    uint32_t                   m_lastTickMs{0};
};

} // namespace owlet::core
