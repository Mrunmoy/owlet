#pragma once

#include <cstdint>

namespace owlet::core
{

/// How well the control loop is keeping time. The loop writes it; whoever
/// reports on the head's health reads it.
struct LoopStats
{
    uint32_t lateTicks{0};   ///< Passes that began more than a millisecond late.
    uint32_t worstTickMs{0}; ///< The longest gap between two passes.
};

} // namespace owlet::core
