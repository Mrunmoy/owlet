#pragma once

#include <cstdint>

namespace owlet::hal
{

/// Step and direction pulses for one step/dir motor driver (TMC2209, A4988,
/// DRV8825, ...). The axis decides how many steps each control period needs;
/// the platform turns that into evenly spaced, jitter-free pulses (a PIO state
/// machine on the Pico, RMT on ESP32, a recorder in tests).
class IStepGenerator
{
public:
    virtual ~IStepGenerator() = default;

    /// Emits |steps| pulses spread evenly over the next `durationUs`, in the
    /// direction given by the sign. Bursts run back to back: one queued while
    /// another is still running starts the moment that one ends.
    /// @return false if the burst could not be queued, or the generator is
    ///         halted. Nothing is emitted then, so the caller's step count
    ///         stays exact and it simply asks again next period.
    [[nodiscard]] virtual bool emit(int32_t steps, uint32_t durationUs) = 0;

    /// Stops the pulses now, part-way through a burst if need be, and
    /// refuses further bursts until resume(). Safe to call from an interrupt
    /// handler; this is what a limit switch's interrupt calls.
    ///
    /// A burst cut short leaves the caller's step count wrong by an unknown
    /// amount, so the position must be treated as lost.
    virtual void halt() = 0;

    /// Clears a halt, discarding whatever was queued. Main loop only.
    virtual void resume() = 0;

    /// @return true between halt() and resume().
    [[nodiscard]] virtual bool halted() const = 0;
};

} // namespace owlet::hal
