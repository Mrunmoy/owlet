#pragma once

#include <cstdint>

namespace owlet::core
{

// What happened during one motion step. Passed from the motion layer to every
// protocol so each can report it in its own dialect.
struct TickStatus
{
    bool busOk{true};             // false: a servo update failed to reach the hardware
    bool watchdogTripped{false};  // true once: velocity mode stopped for lack of refresh
};

// Something advanced by elapsed time. Only ControlLoop calls tick(); protocols
// never do, so they can't change timing behind the loop's back.
class ITickable
{
public:
    virtual ~ITickable() = default;

    [[nodiscard]] virtual TickStatus tick(uint32_t dtMs) = 0;
};

} // namespace owlet::core
