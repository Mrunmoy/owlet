#pragma once

#include "owlet/hal/IAxis.hpp"
#include "owlet/hal/IAxisCalibration.hpp"
#include "owlet/hal/IDigitalInput.hpp"

#include <cstdint>

namespace owlet::core
{

/// Finds an axis's reference position from its limit switches, so the head
/// does not have to be centred by hand and its travel need not be hard-coded.
///
/// With a switch at the low end only, the axis is driven onto it and that
/// point is given the angle from Config::minSwitchDeg. With a switch at each
/// end the distance between them is measured as well: zero becomes the
/// midpoint and the soft limits are set just inside the switches.
///
/// Each switch is found twice: quickly, then again slowly after backing off,
/// so the result does not depend on the seek speed.
///
/// The axis may be anywhere when homing starts. Nothing here assumes the
/// position it reports beforehand is right.
///
/// update() must be called once per control pass, after the axis has been
/// ticked. It only reads the switches and commands the axis, so it runs the
/// same under a simulated mechanism in tests.
class AxisHoming
{
public:
    struct Config
    {
        float seekSpeedDegPerSec = 20.0f;    ///< First, fast approach.
        float approachSpeedDegPerSec = 4.0f; ///< Second, slow approach that sets the reference.
        float backoffDeg = 3.0f;             ///< Distance to retreat between the two.
        float searchSpanDeg = 360.0f;        ///< Give up after looking this far for a switch.
        float marginDeg = 2.0f;              ///< Soft limits sit this far inside the switches.
        float minSwitchDeg = -90.0f; ///< Angle of the low switch; used when there is no high one.
    };

    enum class State : uint8_t
    {
        Idle,
        Seeking,     ///< Fast approach to a switch.
        Settling,    ///< Waiting for the axis to come to rest.
        BackingOff,  ///< Retreating from the switch.
        Approaching, ///< Slow approach that fixes the reference.
        Centring,    ///< Travelling to zero.
        Done,
        Failed,
    };

    enum class Failure : uint8_t
    {
        None,
        SwitchNotFound, ///< Nothing within Config::searchSpanDeg, or it did not trip again.
        SwitchStuck,    ///< Still pressed after backing off.
        OutOfTravel,    ///< The measured range does not fit what the axis accepts.
        Aborted,        ///< abort() was called, or the axis lost power.
    };

    /// @param axis        The axis to move.
    /// @param calibration The same axis's calibration, for the soft limits.
    /// @param minSwitch   Switch at the low end of travel.
    /// @param maxSwitch   Switch at the high end, or nullptr if there is none.
    /// @param config      Speeds and distances for the search.
    AxisHoming(::owlet::hal::IAxis& axis,
               ::owlet::hal::IAxisCalibration& calibration,
               const ::owlet::hal::IDigitalInput& minSwitch,
               const ::owlet::hal::IDigitalInput* maxSwitch,
               const Config& config);

    AxisHoming(const AxisHoming&) = delete;
    AxisHoming& operator=(const AxisHoming&) = delete;

    /// Powers the axis and begins. The axis's speed setting is borrowed and
    /// put back when homing ends, however it ends.
    /// @return false if homing is already running.
    [[nodiscard]] bool start();

    /// Brakes the axis and ends with Failure::Aborted. No effect unless busy.
    void abort();

    /// Advances homing by one control pass.
    void update();

    [[nodiscard]] State state() const
    {
        return m_state;
    }
    [[nodiscard]] Failure failure() const
    {
        return m_failure;
    }
    [[nodiscard]] bool busy() const
    {
        return m_state != State::Idle && m_state != State::Done && m_state != State::Failed;
    }

    /// @return The distance measured between the two switches, in degrees;
    ///         0 until a two-switch homing has completed.
    [[nodiscard]] float travelDeg() const
    {
        return m_travelDeg;
    }

private:
    enum class End : uint8_t
    {
        Min,
        Max,
    };

    /// What follows once the axis has come to rest.
    enum class Next : uint8_t
    {
        BackOff,
        Reference,
    };

    [[nodiscard]] const ::owlet::hal::IDigitalInput& switchAt(End end) const
    {
        return (end == End::Min) ? m_minSwitch : *m_maxSwitch;
    }
    [[nodiscard]] float towards(End end) const
    {
        return (end == End::Min) ? -1.0f : 1.0f;
    }

    void relabel(float deg);
    void beginSeek(End end);
    void continueSeek();
    void settle(Next next);
    void beginBackOff();
    void beginApproach();
    void referenceFound();
    void finishWithOneSwitch();
    void finishWithRange(float travelDeg);
    void centre();
    void fail(Failure failure);

    ::owlet::hal::IAxis& m_axis;
    ::owlet::hal::IAxisCalibration& m_calibration;
    const ::owlet::hal::IDigitalInput& m_minSwitch;
    const ::owlet::hal::IDigitalInput* m_maxSwitch;
    Config m_cfg;
    float m_savedSpeed{0.0f};
    float m_lastPositionDeg{0.0f};
    float m_odometerDeg{0.0f}; ///< Real travel since the low reference.
    float m_searchedDeg{0.0f};
    float m_travelDeg{0.0f};
    State m_state{State::Idle};
    Failure m_failure{Failure::None};
    End m_end{End::Min};
    Next m_next{Next::BackOff};
};

} // namespace owlet::core
