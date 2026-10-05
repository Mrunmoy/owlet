#pragma once

#include "owlet/hal/IAxis.hpp"

namespace owlet::test
{

// A motor that is neither a servo nor a stepper: it reaches any target in one
// tick. Proves the motion policy needs nothing beyond hal::IAxis.
class FakeAxis final : public ::owlet::hal::IAxis
{
public:
    [[nodiscard]] bool engageAt(float deg) override
    {
        if (failEngage)
        {
            return false;
        }
        position = target = deg;
        on = true;
        velocity = 0.0f;
        return true;
    }
    [[nodiscard]] bool release() override
    {
        on = false;
        return true;
    }
    void setTarget(float deg) override
    {
        target = deg;
        velocity = 0.0f;
    }
    void stop() override
    {
        target = position;
        velocity = 0.0f;
    }
    void halt() override
    {
        stop();
        ++halts;
    }
    [[nodiscard]] bool blocked() const override
    {
        return isBlocked;
    }
    void setVelocity(float degPerSec) override
    {
        velocity = degPerSec;
    }
    void setSpeed(float degPerSec) override
    {
        slew = degPerSec;
    }
    [[nodiscard]] bool tick(uint32_t) override
    {
        ++ticks;
        position = target;
        return true;
    }

    [[nodiscard]] bool engaged() const override
    {
        return on;
    }
    [[nodiscard]] bool moving() const override
    {
        return on && (position != target || velocity != 0.0f);
    }
    [[nodiscard]] bool inVelocityMode() const override
    {
        return velocity != 0.0f;
    }
    [[nodiscard]] float positionDeg() const override
    {
        return position;
    }
    [[nodiscard]] float targetDeg() const override
    {
        return target;
    }
    [[nodiscard]] float speed() const override
    {
        return slew;
    }
    [[nodiscard]] float minDeg() const override
    {
        return lo;
    }
    [[nodiscard]] float maxDeg() const override
    {
        return hi;
    }
    [[nodiscard]] bool positionInLimits() const override
    {
        return position >= lo && position <= hi;
    }
    [[nodiscard]] Capabilities capabilities() const override
    {
        return caps;
    }

    Capabilities caps{1.0f, 100.0f, 100.0f};
    float position{0.0f};
    float target{0.0f};
    float velocity{0.0f};
    float slew{10.0f};
    bool isBlocked{false};
    int halts{0};
    float lo{-90.0f};
    float hi{90.0f};
    int ticks{0};
    bool on{false};
    bool failEngage{false};
};

} // namespace owlet::test
