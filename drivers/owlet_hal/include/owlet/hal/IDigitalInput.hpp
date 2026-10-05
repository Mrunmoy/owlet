#pragma once

namespace owlet::hal
{

/// One logic-level input in logical terms: a limit switch, a driver's fault
/// pin. Electrical polarity and pull resistors belong to the platform object,
/// so a switch wired normally-closed reads the same upstream as one wired
/// normally-open.
class IDigitalInput
{
public:
    virtual ~IDigitalInput() = default;

    /// @return true while the input is asserted (the switch is pressed).
    [[nodiscard]] virtual bool active() const = 0;
};

} // namespace owlet::hal
