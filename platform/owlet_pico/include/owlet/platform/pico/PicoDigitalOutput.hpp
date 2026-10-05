#pragma once

#include "owlet/hal/IDigitalOutput.hpp"

#include <cstdint>

namespace owlet::platform::pico
{

/// One GPIO as a logical output. The electrical polarity lives here, so an
/// active-low line (a stepper driver's EN) reads the same upstream as an
/// active-high one. The pin is driven to its *off* level before it becomes an
/// output, so nothing glitches on at boot.
class PicoDigitalOutput final : public ::owlet::hal::IDigitalOutput
{
public:
    enum class Polarity : uint8_t
    {
        ActiveHigh,
        ActiveLow,
    };

    PicoDigitalOutput(unsigned pin, Polarity polarity);

    PicoDigitalOutput(const PicoDigitalOutput&) = delete;
    PicoDigitalOutput& operator=(const PicoDigitalOutput&) = delete;

    void set(bool on) override;

private:
    const unsigned m_pin;
    const bool m_activeLow;
};

} // namespace owlet::platform::pico
