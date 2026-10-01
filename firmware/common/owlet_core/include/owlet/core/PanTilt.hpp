#pragma once

#include "owlet/servo/Servo.hpp"

#include <cstdint>

namespace owlet::core
{

// Two-axis head: the motion model behind the protocol. Owns the
// enabled/disabled policy and range checks; Servo owns mapping and slew.
class PanTilt
{
public:
    enum class Result : uint8_t
    {
        Ok,
        Range,
        Disabled,
        Bus,
    };

    enum class State : uint8_t
    {
        Disabled,
        Moving,
        Holding,
    };

    PanTilt(::owlet::Servo& pan, ::owlet::Servo& tilt);

    PanTilt(const PanTilt&)            = delete;
    PanTilt& operator=(const PanTilt&) = delete;

    // ── Power ───────────────────────────────────────────────────────────────

    // Energise both axes at their last known position. No-op if enabled.
    [[nodiscard]] Result enable();
    // Energise both axes at the given pose (snap). If already enabled this is
    // a slewed move instead: a running head must never snap.
    [[nodiscard]] Result enableAt(float panDeg, float tiltDeg);
    [[nodiscard]] Result disable();

    // ── Motion ──────────────────────────────────────────────────────────────

    [[nodiscard]] Result move(float panDeg, float tiltDeg);
    // Relative to the current target; clamps instead of rejecting so jogging
    // against a limit just stops there.
    [[nodiscard]] Result nudge(float dPanDeg, float dTiltDeg);
    [[nodiscard]] Result home();
    [[nodiscard]] Result stop();
    [[nodiscard]] Result setSpeed(float degPerSec);

    [[nodiscard]] bool tick(uint32_t dtMs);

    // ── Introspection ───────────────────────────────────────────────────────

    [[nodiscard]] State state() const;
    float speed() const { return m_pan.speed(); }
    float panDeg() const { return m_pan.positionDeg(); }
    float tiltDeg() const { return m_tilt.positionDeg(); }
    float panTargetDeg() const { return m_pan.targetDeg(); }
    float tiltTargetDeg() const { return m_tilt.targetDeg(); }
    const ::owlet::Servo& panServo() const { return m_pan; }
    const ::owlet::Servo& tiltServo() const { return m_tilt; }

private:
    [[nodiscard]] static bool inLimits(const ::owlet::Servo& s, float deg);
    void releaseBoth();

    ::owlet::Servo& m_pan;
    ::owlet::Servo& m_tilt;
};

} // namespace owlet::core
