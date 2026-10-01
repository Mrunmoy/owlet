#pragma once

#include <cstdint>

namespace owlet::core
{

// Everything a protocol may ask of a two-axis head. Protocols depend on this,
// never on a concrete head, so a different motion backend (steppers, a
// simulator, a test fake) works under every protocol unchanged.
//
// Angles are degrees from calibrated center: pan + = right, tilt + = up.
class IPanTilt
{
public:
    enum class Result : uint8_t
    {
        Ok,
        Range,     // argument outside limits
        Disabled,  // motion requested while disabled
        Bus,       // hardware write failed
    };

    enum class State : uint8_t
    {
        Disabled,
        Moving,
        Holding,
    };

    struct Pose
    {
        float pan;
        float tilt;
    };

    struct Limits
    {
        float panMin;
        float panMax;
        float tiltMin;
        float tiltMax;
    };

    virtual ~IPanTilt() = default;

    // ── Power ───────────────────────────────────────────────────────────────

    // Energise at the last known position. No-op if already enabled.
    [[nodiscard]] virtual Result enable() = 0;
    // Energise at a pose (snap). If already enabled, a slewed move instead.
    [[nodiscard]] virtual Result enableAt(float panDeg, float tiltDeg) = 0;
    [[nodiscard]] virtual Result disable() = 0;

    // ── Motion ──────────────────────────────────────────────────────────────

    // Absolute; Range if outside limits. Replaces any current goal.
    [[nodiscard]] virtual Result move(float panDeg, float tiltDeg) = 0;
    // Relative to the current target; clamps to limits.
    [[nodiscard]] virtual Result nudge(float dPanDeg, float dTiltDeg) = 0;
    [[nodiscard]] virtual Result home() = 0;
    [[nodiscard]] virtual Result stop() = 0;
    // Slew speed for position moves, deg/s.
    [[nodiscard]] virtual Result setSpeed(float degPerSec) = 0;

    // ── Velocity mode ───────────────────────────────────────────────────────

    // Turn at signed rates (deg/s) until told otherwise; stops at soft limits.
    // Any position command ends it. Each call re-arms the watchdog.
    [[nodiscard]] virtual Result setVelocity(float panDegPerSec, float tiltDegPerSec) = 0;
    // Velocity mode stops itself if not refreshed within this period.
    // 0 disables the watchdog; otherwise kMinWatchdogMs..kMaxWatchdogMs.
    [[nodiscard]] virtual Result setWatchdog(uint32_t ms) = 0;
    [[nodiscard]] virtual uint32_t watchdogMs() const = 0;

    static constexpr uint32_t kDefaultWatchdogMs = 500;
    static constexpr uint32_t kMinWatchdogMs     = 50;
    static constexpr uint32_t kMaxWatchdogMs     = 5000;

    // ── Introspection ───────────────────────────────────────────────────────

    [[nodiscard]] virtual State state() const = 0;
    [[nodiscard]] virtual Pose position() const = 0;
    [[nodiscard]] virtual Pose target() const = 0;
    [[nodiscard]] virtual float speed() const = 0;
    [[nodiscard]] virtual Limits limits() const = 0;
};

} // namespace owlet::core
