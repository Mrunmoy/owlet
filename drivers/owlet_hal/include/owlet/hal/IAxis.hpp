#pragma once

#include <cstdint>

namespace owlet::hal
{

/// One motor-driven axis, in degrees. This is the seam between the motion
/// policy (owlet::core::PanTilt) and whatever turns the shaft: a hobby servo,
/// a stepper, a DC motor with an encoder. Write one of these for a new motor
/// type and the core, every protocol and every client work unchanged.
//
/// Two motion modes, both advanced by tick(dtMs):
///   - position: travel toward a target at speed() deg/s;
///   - velocity: turn at a signed rate until told otherwise, stopping at the
///     soft limits. Any position command leaves velocity mode.
/// An axis starts *released* (unpowered); engaging is always an explicit act.
class IAxis
{
public:
    /// What this motor can do, deg/s. The head reports the intersection of
    /// its axes, so clients never command more than the slower one can give.
    struct Capabilities
    {
        float speedMin;    ///< slowest position-move speed
        float speedMax;    ///< fastest position-move speed
        float velocityMax; ///< |rate| limit in velocity mode
    };

    virtual ~IAxis() = default;

    // ── Power ───────────────────────────────────────────────────────────────

    /// Power the axis and take `deg` (clamped to the limits) as its position.
    /// A servo jumps there; a motor without absolute position adopts it as
    /// the name of where it already is. Position and target both become `deg`.
    [[nodiscard]] virtual bool engageAt(float deg) = 0;

    /// Cut power. The last position is kept as the best available estimate.
    [[nodiscard]] virtual bool release() = 0;

    // ── Motion ──────────────────────────────────────────────────────────────

    /// Store a position target (clamped to soft limits). Leaves velocity mode.
    virtual void setTarget(float deg) = 0;

    /// Come to rest and hold. Leaves velocity mode.
    virtual void stop() = 0;

    /// Stops where it is, at once, with no braking distance. For when going
    /// any further would do harm; an axis that cannot stop more sharply
    /// than stop() does the same as stop(). Leaves velocity mode.
    virtual void halt() = 0;

    /// Velocity mode at a signed rate (clamped to ±velocityMax). Zero is the
    /// same as stop().
    virtual void setVelocity(float degPerSec) = 0;

    /// Speed for position moves, clamped to [speedMin, speedMax].
    virtual void setSpeed(float degPerSec) = 0;

    /// Advance motion by `dtMs`. False if the hardware refused an update; the
    /// position estimate then stays where the hardware last confirmed it.
    [[nodiscard]] virtual bool tick(uint32_t dtMs) = 0;

    // ── Introspection ───────────────────────────────────────────────────────

    [[nodiscard]] virtual bool engaged() const = 0;
    [[nodiscard]] virtual bool moving() const = 0;
    [[nodiscard]] virtual bool inVelocityMode() const = 0;
    [[nodiscard]] virtual float positionDeg() const = 0;
    [[nodiscard]] virtual float targetDeg() const = 0;
    [[nodiscard]] virtual float speed() const = 0;
    [[nodiscard]] virtual float minDeg() const = 0;
    [[nodiscard]] virtual float maxDeg() const = 0;
    /// False if the shaft may be outside the limits, or its angle is unknown.
    [[nodiscard]] virtual bool positionInLimits() const = 0;
    /// @return true if motion was cut short by something physical (a limit
    ///         switch, or pulses stopped from outside), so the axis is not
    ///         where a completed move would have put it. Stays true until
    ///         the axis next moves or is engaged afresh. An axis that cannot
    ///         tell returns false.
    [[nodiscard]] virtual bool blocked() const = 0;
    [[nodiscard]] virtual Capabilities capabilities() const = 0;
};

} // namespace owlet::hal
