#pragma once

#include "owlet/hal/IDigitalInput.hpp"
#include "owlet/hal/IEdgeListener.hpp"

#include <cstdint>

namespace owlet::platform::pico
{

/// One GPIO as a logical input, for a limit switch or a driver status pin.
///
/// A switch is best wired normally-closed to ground and read ActiveHigh with
/// the pull-up on: the pin sits low while the switch is closed and rises when
/// it opens, so a broken wire or an unplugged switch reads as pressed and the
/// axis stops, instead of running on into the end of its travel.
class PicoDigitalInput final : public ::owlet::hal::IDigitalInput
{
public:
    enum class Polarity : uint8_t
    {
        ActiveHigh,
        ActiveLow,
    };

    enum class Pull : uint8_t
    {
        None,
        Up,
        Down,
    };

    /// Configures the pin as an input straight away.
    PicoDigitalInput(unsigned pin, Polarity polarity, Pull pull);

    PicoDigitalInput(const PicoDigitalInput&) = delete;
    PicoDigitalInput& operator=(const PicoDigitalInput&) = delete;

    [[nodiscard]] bool active() const override;

    /// Calls `listener` from this pin's interrupt each time the input
    /// becomes active. A switch that bounces calls it more than once.
    /// One listener per pin; a second call replaces the first.
    void onActive(::owlet::hal::IEdgeListener& listener);

private:
    const unsigned m_pin;
    const bool m_activeLow;
};

} // namespace owlet::platform::pico
