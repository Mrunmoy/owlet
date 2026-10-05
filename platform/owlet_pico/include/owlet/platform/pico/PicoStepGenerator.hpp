#pragma once

#include "owlet/hal/IStepGenerator.hpp"

#include "hardware/pio.h"

#include <cstdint>

namespace owlet::platform::pico
{

// The step/dir pulse program, loaded once into a PIO block and shared by
// every PicoStepGenerator on that block (up to four axes).
//
// The program takes two words per burst, a delay and a count with the
// direction in bit 0, sets DIR, then emits the pulses with cycle-exact
// spacing. Nothing the CPU does (USB interrupts, flash writes, a late control
// loop) can move a pulse once the burst is queued.
class PicoStepProgram
{
public:
    // State-machine clock: 0.2 µs resolution, 0.8 µs STEP high time.
    static constexpr uint32_t kClockHz = 5'000'000;
    static constexpr uint32_t kCyclesPerStep = 8; // fixed part of one pulse period
    static constexpr unsigned kStepHighCycles = 4;

    explicit PicoStepProgram(PIO pio) : m_pio(pio) {}

    PicoStepProgram(const PicoStepProgram&) = delete;
    PicoStepProgram& operator=(const PicoStepProgram&) = delete;

    // False if the PIO block has no room for the program.
    [[nodiscard]] bool load();

    [[nodiscard]] PIO pio() const
    {
        return m_pio;
    }
    [[nodiscard]] unsigned offset() const
    {
        return m_offset;
    }
    [[nodiscard]] bool loaded() const
    {
        return m_loaded;
    }

private:
    const PIO m_pio;
    unsigned m_offset{0};
    bool m_loaded{false};
};

// IStepGenerator on one PIO state machine: one per motor.
class PicoStepGenerator final : public ::owlet::hal::IStepGenerator
{
public:
    PicoStepGenerator(PicoStepProgram& program,
                      unsigned stateMachine,
                      unsigned stepPin,
                      unsigned dirPin);

    PicoStepGenerator(const PicoStepGenerator&) = delete;
    PicoStepGenerator& operator=(const PicoStepGenerator&) = delete;

    // Claims the pins and starts the state machine, idle with STEP low.
    // Call after PicoStepProgram::load(). False (and nothing is touched) if
    // the program isn't loaded, a pin or state-machine number is out of
    // range, or the state machine is already in use.
    [[nodiscard]] bool init();

    // Refuses (false) while an earlier burst is still waiting in the FIFO, so
    // at most one burst is ever queued behind the one running: commands take
    // effect within a control period.
    //
    // Call from the control loop only. It reads the FIFO level and then
    // writes two words, which is not safe against an interrupt doing the same.
    [[nodiscard]] bool emit(int32_t steps, uint32_t durationUs) override;

    // Freezes the state machine where it is: no further edge leaves the
    // chip. Only a register write and a flag, so an interrupt may call it.
    void halt() override;
    // Empties the FIFO, puts STEP low and restarts the program from the top.
    void resume() override;
    [[nodiscard]] bool halted() const override
    {
        return m_halted;
    }


private:
    PicoStepProgram& m_program;
    const unsigned m_sm;
    const unsigned m_stepPin;
    const unsigned m_dirPin;
    bool m_ready{false};
    volatile bool m_halted{false}; // written by halt() in an interrupt
};

} // namespace owlet::platform::pico
