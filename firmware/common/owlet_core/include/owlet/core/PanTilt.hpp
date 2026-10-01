#pragma once

#include "owlet/core/IPanTilt.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/servo/Servo.hpp"

#include <cstdint>

namespace owlet::core
{

// Two-axis head on two Servo objects. Owns the enable/limit/motion policy;
// Servo owns mapping and slew.
class PanTilt final : public IPanTilt, public ITickable
{
public:
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

    [[nodiscard]] State state() const override;
    [[nodiscard]] Pose position() const override;
    [[nodiscard]] Pose target() const override;
    [[nodiscard]] float speed() const override;
    [[nodiscard]] Limits limits() const override;

    // ── ITickable ───────────────────────────────────────────────────────────

    [[nodiscard]] TickStatus tick(uint32_t dtMs) override;

private:
    [[nodiscard]] static bool inLimits(const ::owlet::Servo& s, float deg);
    void releaseBoth();

    ::owlet::Servo& m_pan;
    ::owlet::Servo& m_tilt;
};

} // namespace owlet::core
