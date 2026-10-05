#pragma once

#include "owlet/hal/IDigitalOutput.hpp"

namespace owlet::test
{

class FakeDigitalOutput final : public ::owlet::hal::IDigitalOutput
{
public:
    void set(bool value) override
    {
        on = value;
        ++writes;
    }

    bool on{false};
    int writes{0};
};

} // namespace owlet::test
