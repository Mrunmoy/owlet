#pragma once

#include "owlet/hal/IAxis.hpp"
#include "owlet/hal/IAxisCalibration.hpp"
#include "owlet/hal/IDigitalOutput.hpp"
#include "owlet/hal/IStepGenerator.hpp"

#include <cstdint>

namespace owlet
{

// A step/dir stepper as a hal::IAxis.
//
// Position is an exact integer step count; degrees are derived from it. Each
// tick the axis works out how many steps the next period needs and hands
// that burst to an IStepGenerator, which spaces the pulses in hardware.
//
// Unlike a servo, a stepper stalls if its rate jumps, so every change of
// speed is limited to the configured acceleration: moves ramp up, ramp down
// onto the target, and stop() brakes to rest instead of halting dead.
//
// There is no absolute position sensor. engageAt(deg) powers the driver and
// *names* the current shaft position `deg`; it never moves the motor.
class StepperAxis final : public ::owlet::hal::IAxis, public ::owlet::hal::IAxisCalibration
{
public:
    struct Config
    {
        // Microsteps per degree at the output, after any gearing.
        float stepsPerDeg = 9600.0f / 360.0f;
        bool reversed = false; // flip if + turns the wrong way
        float maxSpeedDegPerSec = 180.0f;
        float accelDegPerSec2 = 720.0f;
        // Factory soft limits.
        float minDeg = -90.0f;
        float maxDeg = +90.0f;
        // The widest soft limits a calibration may ask for (what the
        // mechanism and its cables allow).
        float travelMinDeg = -170.0f;
        float travelMaxDeg = +170.0f;
    };

    static constexpr float kMinSpeedDegPerSec = 1.0f;
    static constexpr float kDefaultSpeedDegPerSec = 90.0f;
    // Slack for "inside the limits": one reporting step (0.1°).
    static constexpr float kPositionToleranceDeg = 0.1f;
    // Longest period one tick will step for. A wrapped or corrupted dt then
    // can't turn into a lurch or overflow the step arithmetic.
    static constexpr uint32_t kMaxTickMs = 100;
    // A generator that refuses every burst for this long is a fault.
    static constexpr uint32_t kStallReportMs = 200;
    // Soft limits and travel are kept within this, so degrees × steps-per-
    // degree always fits the 32-bit step count.
    static constexpr float kMaxTravelDeg = 3600.0f;

    StepperAxis(::owlet::hal::IStepGenerator& steps,
                ::owlet::hal::IDigitalOutput& enable,
                const Config& config);

    StepperAxis(const StepperAxis&) = delete;
    StepperAxis& operator=(const StepperAxis&) = delete;

    // ── IAxis: power ────────────────────────────────────────────────────────

    [[nodiscard]] bool engageAt(float deg) override;
    [[nodiscard]] bool release() override;

    // ── IAxis: motion ───────────────────────────────────────────────────────

    void setTarget(float deg) override;
    // Brakes at the configured acceleration; the target becomes the point
    // where the axis will come to rest.
    void stop() override;
    void setVelocity(float degPerSec) override;
    void setSpeed(float degPerSec) override;
    // A burst the generator can't take yet is not lost: the profile simply
    // continues from the unchanged step count next tick. False only once the
    // generator has refused everything for kStallReportMs, which means the
    // pulse hardware isn't running and the axis is not really moving.
    [[nodiscard]] bool tick(uint32_t dtMs) override;

    // ── IAxis: introspection ────────────────────────────────────────────────

    [[nodiscard]] bool engaged() const override
    {
        return m_engaged;
    }
    [[nodiscard]] bool moving() const override;
    [[nodiscard]] bool inVelocityMode() const override
    {
        return m_velocityMode;
    }
    [[nodiscard]] float positionDeg() const override
    {
        return toDeg(m_positionSteps);
    }
    [[nodiscard]] float targetDeg() const override
    {
        return toDeg(m_velocityMode ? m_positionSteps : m_targetSteps);
    }
    [[nodiscard]] float speed() const override
    {
        return m_speedDegPerSec;
    }
    [[nodiscard]] float minDeg() const override
    {
        return m_minDeg;
    }
    [[nodiscard]] float maxDeg() const override
    {
        return m_maxDeg;
    }
    [[nodiscard]] bool positionInLimits() const override
    {
        const float deg = positionDeg();
        return deg >= m_minDeg - kPositionToleranceDeg && deg <= m_maxDeg + kPositionToleranceDeg;
    }
    [[nodiscard]] Capabilities capabilities() const override
    {
        return Capabilities{kMinSpeedDegPerSec, m_cfg.maxSpeedDegPerSec, m_cfg.maxSpeedDegPerSec};
    }

    // ── IAxisCalibration ────────────────────────────────────────────────────
    // Only the soft limits are adjustable; the drive numbers are always zero.
    // Steps per degree and direction are properties of the build (Config).

    [[nodiscard]] ::owlet::hal::AxisCalibration calibration() const override
    {
        return ::owlet::hal::AxisCalibration{0, 0, 0, m_minDeg, m_maxDeg};
    }
    [[nodiscard]] ::owlet::hal::AxisCalibration defaultCalibration() const override
    {
        return ::owlet::hal::AxisCalibration{0, 0, 0, m_cfg.minDeg, m_cfg.maxDeg};
    }
    [[nodiscard]] bool acceptsCalibration(const ::owlet::hal::AxisCalibration& cal) const override;
    // Limits never move the shaft, so nothing needs the axis released.
    [[nodiscard]] bool calibrationNeedsRelease(const ::owlet::hal::AxisCalibration&) const override
    {
        return false;
    }
    void applyCalibration(const ::owlet::hal::AxisCalibration& cal) override;

    [[nodiscard]] int32_t positionSteps() const
    {
        return m_positionSteps;
    }

private:
    [[nodiscard]] float toDeg(int32_t steps) const
    {
        return static_cast<float>(steps) / m_cfg.stepsPerDeg;
    }
    [[nodiscard]] int32_t toSteps(float deg) const;
    [[nodiscard]] float clampDeg(float deg) const;
    void setLimits(float minDeg, float maxDeg);
    [[nodiscard]] bool pinnedAtLimit() const;

    ::owlet::hal::IStepGenerator& m_steps;
    ::owlet::hal::IDigitalOutput& m_enable;
    Config m_cfg;
    float m_minDeg{0.0f};
    float m_maxDeg{0.0f};
    int32_t m_minSteps{0};
    int32_t m_maxSteps{0};
    int32_t m_positionSteps{0};
    int32_t m_targetSteps{0};
    float m_rate{0.0f};     // current rate, steps/s, signed
    float m_fraction{0.0f}; // part-step carried to the next tick
    float m_speedDegPerSec{kDefaultSpeedDegPerSec};
    float m_velocityDegPerSec{0.0f};
    uint32_t m_refusedMs{0}; // how long the generator has refused bursts
    bool m_engaged{false};
    bool m_velocityMode{false};
};

} // namespace owlet
