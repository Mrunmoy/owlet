#pragma once

#include "FakeDigitalOutput.hpp"
#include "FakeStepGenerator.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/stepper/Tmc2209Standalone.hpp"

#include <cstdint>

namespace owlet::test
{

// The head the firmware ships for: NEMA 17 at 16 microsteps through 3:1 belts,
// the limits verified on hardware. Mirrors referenceAxis() in host-sim, so the
// core tests run against the same mechanism the Python end-to-end tests do.
inline ::owlet::StepperAxis::Config referenceAxis(float limitDeg, float travelDeg)
{
    ::owlet::StepperAxis::Config c;
    c.stepsPerDeg = ::owlet::tmc2209::stepsPerDegree(200, ::owlet::tmc2209::Microsteps::k16, 3.0f);
    c.maxSpeedDegPerSec = 240.0f;
    c.accelDegPerSec2 = 1440.0f;
    c.minDeg = -limitDeg;
    c.maxDeg = limitDeg;
    c.travelMinDeg = -travelDeg;
    c.travelMaxDeg = travelDeg;
    return c;
}

inline ::owlet::StepperAxis::Config referencePan()
{
    return referenceAxis(90.0f, 180.0f);
}

inline ::owlet::StepperAxis::Config referenceTilt()
{
    return referenceAxis(60.0f, 135.0f);
}

/// One step, in degrees: how far apart two positions can be and still be the
/// same step count.
inline float referenceStepDeg()
{
    return 1.0f / referencePan().stepsPerDeg;
}

/// Both axes of the reference head with their pulse and enable fakes.
struct ReferenceAxes
{
    FakeStepGenerator panSteps;
    FakeStepGenerator tiltSteps;
    FakeDigitalOutput panEnable;
    FakeDigitalOutput tiltEnable;
    ::owlet::StepperAxis pan{panSteps, panEnable, referencePan()};
    ::owlet::StepperAxis tilt{tiltSteps, tiltEnable, referenceTilt()};

    [[nodiscard]] bool anyStepped() const
    {
        return !panSteps.bursts.empty() || !tiltSteps.bursts.empty();
    }
    [[nodiscard]] bool anyPowered() const
    {
        return panEnable.on || tiltEnable.on;
    }
};

} // namespace owlet::test
