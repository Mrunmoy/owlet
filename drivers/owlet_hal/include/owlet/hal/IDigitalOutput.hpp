#pragma once

namespace owlet::hal
{

// One logic-level output, in logical terms: set(true) asserts whatever the
// line does (enable a driver, light an LED). Electrical polarity belongs to
// the platform object, so an active-low enable needs no inversion upstream.
class IDigitalOutput
{
public:
    virtual ~IDigitalOutput() = default;

    virtual void set(bool on) = 0;
};

} // namespace owlet::hal
