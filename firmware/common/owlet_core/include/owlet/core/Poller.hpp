#pragma once

#include "owlet/core/ITickable.hpp"
#include "owlet/hal/IPollable.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::core
{

/// Gives background housekeeping (hal::IPollable) a slice of every loop pass,
/// after motion. Like ActivityIndicator it wraps the motion layer as an
/// ITickable, so ControlLoop needs to know nothing about it. Null entries are
/// skipped.
template <std::size_t N>
class Poller final : public ITickable
{
public:
    Poller(ITickable& motion, std::array<::owlet::hal::IPollable*, N> items)
        : m_motion(motion), m_items(items)
    {
    }

    Poller(const Poller&) = delete;
    Poller& operator=(const Poller&) = delete;

    [[nodiscard]] TickStatus tick(uint32_t dtMs) override
    {
        const TickStatus status = m_motion.tick(dtMs);
        for (::owlet::hal::IPollable* item : m_items)
        {
            if (item != nullptr)
            {
                item->poll(dtMs);
            }
        }
        return status;
    }

private:
    ITickable& m_motion;
    std::array<::owlet::hal::IPollable*, N> m_items;
};

} // namespace owlet::core
