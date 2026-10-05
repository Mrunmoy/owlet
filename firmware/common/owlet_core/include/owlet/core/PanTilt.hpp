#pragma once

#include "owlet/core/IEventSink.hpp"
#include "owlet/core/IHoming.hpp"
#include "owlet/core/IPanTilt.hpp"
#include "owlet/core/ITickable.hpp"
#include "owlet/hal/IAxis.hpp"

#include <cstdint>

namespace owlet::core
{

/// Two-axis head on two hal::IAxis objects. Owns the enable/limit/motion
/// policy; each axis owns its own mapping and motion profile, so the motor
/// type (servo, stepper, ...) is invisible from here upward.
//
/// Partial engagement (one axis failed to release on DISABLE) is a fault
/// state: new motion is refused, but STOP and VEL 0,0 still work on whatever
/// is powered, and state() doesn't report Disabled while anything is.
//
/// Given an event sink it announces, from tick(): Arrived once for every
/// accepted position move (even one to where the head already is),
/// WatchdogExpired when velocity mode is stopped for lack of a command, and
/// BusFault / BusOk when commanding the motors starts or stops failing,
/// PositionLost when an axis is stopped by something physical, and Homed /
/// HomingFailed when homing ends.
///
/// Once the position is lost the head holds where it is and refuses motion
/// (Result::Untrusted) until it is homed, or disabled and enabled at a pose.
class PanTilt final : public IPanTilt, public ITickable
{
public:
    /// Longest motion step per tick: a starved loop resumes without lurching.
    static constexpr uint32_t kMaxStepMs = 100;

    /// @param homing Optional: how this head finds its reference by itself.
    PanTilt(::owlet::hal::IAxis& pan,
            ::owlet::hal::IAxis& tilt,
            IEventSink*          events = nullptr,
            IHoming*             homing = nullptr);

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
    [[nodiscard]] Result seekHome() override;
    [[nodiscard]] bool positionTrusted() const override { return m_trust == Trust::Trusted; }

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
    [[nodiscard]] static bool inLimits(const ::owlet::hal::IAxis& axis, float deg);
    void releaseBoth();
    void announce(EventId id, uint8_t axis = Event::kHead, uint16_t arg = 0);
    void losePosition(uint8_t axis);
    void serviceHoming();
    /// Ok if a motion command may run now; otherwise why not.
    [[nodiscard]] Result mayMove() const;

    enum class Trust : uint8_t
    {
        Trusted,
        Lost,
        Homing,
    };

    [[nodiscard]] bool inVelocityMode() const;
    [[nodiscard]] bool bothEngaged() const { return m_pan.engaged() && m_tilt.engaged(); }
    [[nodiscard]] bool anyEngaged() const { return m_pan.engaged() || m_tilt.engaged(); }

    ::owlet::hal::IAxis& m_pan;
    ::owlet::hal::IAxis& m_tilt;
    uint32_t             m_watchdogMs{kDefaultWatchdogMs};
    IEventSink*          m_events;
    IHoming*             m_homing;
    Trust                m_trust{Trust::Trusted};
    uint32_t             m_velocityAgeMs{0};  ///< time since the last setVelocity()
    bool                 m_arrivalOwed{false};  ///< a position move has not been announced yet
    bool                 m_busFault{false};
};

} // namespace owlet::core
