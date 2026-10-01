#pragma once

#include "owlet/core/IPanTilt.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/servo/Servo.hpp"

#include <cstdint>

namespace owlet::core
{

// Two-axis head on two Servo objects. Owns the enable/limit/motion policy;
// Servo owns mapping and slew.
//
// Partial engagement (one axis failed to release on DISABLE) is a fault
// state: new motion is refused, but STOP and VEL 0,0 still work on whatever
// is powered, and state() doesn't report Disabled while anything is.
class PanTilt final : public IPanTilt, public ITickable
{
public:
    // Longest motion step per tick: a starved loop resumes without lurching.
    static constexpr uint32_t kMaxStepMs = 100;

    PanTilt(::owlet::Servo& pan, ::owlet::Servo& tilt);

    PanTilt(const PanTilt&)            = delete;
    PanTilt& operator=(const PanTilt&) = delete;

    // ── IPanTilt ────────────────────────────────────────────────────────────

    [[nodiscard]] Result enable() override;
    [[nodiscard]] Result enableAt(float panDeg, float tiltDeg) override;
    [[nodiscard]] Result disable() override;

    [[nodiscard]] Result move(float panDeg, float tiltDeg) override;
    [[nodiscard]] Result nudge(float dPanDeg, float dTiltDeg) override;
    [[nodiscard]] Result home() override;
    [[nodiscard]] Result stop() override;
    [[nodiscard]] Result setSpeed(float degPerSec) override;

    [[nodiscard]] Result setVelocity(float panDegPerSec, float tiltDegPerSec) override;
    [[nodiscard]] Result setWatchdog(uint32_t ms) override;
    [[nodiscard]] uint32_t watchdogMs() const override { return m_watchdogMs; }

    [[nodiscard]] State state() const override;
    [[nodiscard]] Pose position() const override;
    [[nodiscard]] Pose target() const override;
    [[nodiscard]] float speed() const override;
    [[nodiscard]] Limits limits() const override;
    [[nodiscard]] Capabilities capabilities() const override;

    // ── ITickable ───────────────────────────────────────────────────────────

    [[nodiscard]] TickStatus tick(uint32_t dtMs) override;

private:
    [[nodiscard]] static bool inLimits(const ::owlet::Servo& s, float deg);
    void releaseBoth();

    [[nodiscard]] bool inVelocityMode() const;
    [[nodiscard]] bool bothEngaged() const { return m_pan.engaged() && m_tilt.engaged(); }
    [[nodiscard]] bool anyEngaged() const { return m_pan.engaged() || m_tilt.engaged(); }

    ::owlet::Servo& m_pan;
    ::owlet::Servo& m_tilt;
    uint32_t        m_watchdogMs{kDefaultWatchdogMs};
    uint32_t        m_velocityAgeMs{0};  // time since the last setVelocity()
};

} // namespace owlet::core
