#pragma once

#include "owlet/hal/IDigitalOutput.hpp"
#include "owlet/hal/IPollable.hpp"

#include <cstdint>

namespace owlet::core
{

/// A short flash once per period: proof at a glance that the board is powered
/// and the control loop is running. A loop that hangs leaves the light stuck.
class Heartbeat final : public ::owlet::hal::IPollable
{
public:
    static constexpr uint32_t kDefaultPeriodMs = 1000;
    static constexpr uint32_t kDefaultOnMs = 100;

    explicit Heartbeat(::owlet::hal::IDigitalOutput& indicator,
                       uint32_t periodMs = kDefaultPeriodMs,
                       uint32_t onMs = kDefaultOnMs)
        : m_indicator(indicator), m_periodMs(periodMs != 0 ? periodMs : kDefaultPeriodMs),
          m_onMs(onMs)
    {
    }

    Heartbeat(const Heartbeat&) = delete;
    Heartbeat& operator=(const Heartbeat&) = delete;

    void poll(uint32_t dtMs) override
    {
        // Reduce dt first, so a huge value can't overflow the sum.
        m_phaseMs = (m_phaseMs + dtMs % m_periodMs) % m_periodMs;
        const bool lit = m_phaseMs < m_onMs;
        if (lit != m_lit || !m_started)
        {
            m_started = true;
            m_lit = lit;
            m_indicator.set(lit);
        }
    }

private:
    ::owlet::hal::IDigitalOutput& m_indicator;
    uint32_t m_periodMs;
    uint32_t m_onMs;
    uint32_t m_phaseMs{0};
    bool m_lit{false};
    bool m_started{false};
};

} // namespace owlet::core
