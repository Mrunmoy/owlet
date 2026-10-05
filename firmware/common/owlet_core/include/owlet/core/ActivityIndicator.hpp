#pragma once

#include "owlet/core/IPanTilt.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/hal/IDigitalOutput.hpp"

#include <cstdint>

namespace owlet::core
{

// Lights an indicator while the head is moving. It wraps the motion layer as
// an ITickable, so it slots between ControlLoop and any head without either
// of them knowing, and it writes the output only when the state changes.
class ActivityIndicator final : public ITickable
{
public:
    ActivityIndicator(ITickable& motion,
                      const IPanTilt& head,
                      ::owlet::hal::IDigitalOutput& indicator)
        : m_motion(motion), m_head(head), m_indicator(indicator)
    {
    }

    ActivityIndicator(const ActivityIndicator&) = delete;
    ActivityIndicator& operator=(const ActivityIndicator&) = delete;

    [[nodiscard]] TickStatus tick(uint32_t dtMs) override
    {
        const TickStatus status = m_motion.tick(dtMs);
        const bool active = m_head.state() == IPanTilt::State::Moving;
        if (active != m_active)
        {
            m_active = active;
            m_indicator.set(active);
        }
        return status;
    }

private:
    ITickable& m_motion;
    const IPanTilt& m_head;
    ::owlet::hal::IDigitalOutput& m_indicator;
    bool m_active{false};
};

} // namespace owlet::core
