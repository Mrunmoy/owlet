#pragma once

#include <cstdint>

namespace owlet::hal
{

/// Background housekeeping that wants a regular slice of the control loop
/// (polling a motor driver's status, for instance). poll() must return
/// quickly: it runs between motion ticks.
class IPollable
{
public:
    virtual ~IPollable() = default;

    virtual void poll(uint32_t dtMs) = 0;
};

} // namespace owlet::hal
