#include "owlet/platform/pico/PicoStepGenerator.hpp"

#include "hardware/clocks.h"
#include "hardware/pio_instructions.h"

#include <array>

namespace owlet::platform::pico
{

namespace
{

// Program layout (addresses relative to the load offset):
//   0  pull block        ; word 1: delay count
//   1  mov  isr, osr     ;         parked in ISR for the whole burst
//   2  pull block        ; word 2: bit 0 = direction, bits 31..1 = steps − 1
//   3  out  pins, 1      ; DIR, two cycles before the first STEP edge
//   4  out  y, 31
//   5  set  pins, 1 [3]  ; STEP high for kStepHighCycles
//   6  set  pins, 0
//   7  mov  x, isr
//   8  jmp  x-- 8        ; delay: x + 1 cycles
//   9  jmp  y-- 5        ; next pulse
// One pulse period is x + kCyclesPerStep cycles. With an empty FIFO the
// machine waits at 0 with STEP low.
constexpr unsigned kStepLoop = 5;
constexpr unsigned kDelayLoop = 8;
constexpr unsigned kLength = 10;

std::array<uint16_t, kLength> makeProgram()
{
    return {
        static_cast<uint16_t>(pio_encode_pull(false, true)),
        static_cast<uint16_t>(pio_encode_mov(pio_isr, pio_osr)),
        static_cast<uint16_t>(pio_encode_pull(false, true)),
        static_cast<uint16_t>(pio_encode_out(pio_pins, 1)),
        static_cast<uint16_t>(pio_encode_out(pio_y, 31)),
        static_cast<uint16_t>(pio_encode_set(pio_pins, 1) |
                              pio_encode_delay(PicoStepProgram::kStepHighCycles - 1)),
        static_cast<uint16_t>(pio_encode_set(pio_pins, 0)),
        static_cast<uint16_t>(pio_encode_mov(pio_x, pio_isr)),
        static_cast<uint16_t>(pio_encode_jmp_x_dec(kDelayLoop)),
        static_cast<uint16_t>(pio_encode_jmp_y_dec(kStepLoop)),
    };
}

static_assert(PicoStepProgram::kCyclesPerStep == PicoStepProgram::kStepHighCycles + 4,
              "set-high, set-low, mov, the delay loop's last pass and jmp y-- make up the period");

} // namespace

bool PicoStepProgram::load()
{
    if (m_loaded)
    {
        return true;
    }
    // pio_add_program keeps a pointer to nothing: it copies the instructions
    // into the PIO block, so the encoded array only has to outlive the call.
    const std::array<uint16_t, kLength> code = makeProgram();
    pio_program_t program{};
    program.instructions = code.data();
    program.length = static_cast<uint8_t>(code.size());
    program.origin = -1; // relocatable
    if (!pio_can_add_program(m_pio, &program))
    {
        return false;
    }
    m_offset = static_cast<unsigned>(pio_add_program(m_pio, &program));
    m_loaded = true;
    return true;
}

PicoStepGenerator::PicoStepGenerator(PicoStepProgram& program,
                                     unsigned stateMachine,
                                     unsigned stepPin,
                                     unsigned dirPin)
    : m_program(program), m_sm(stateMachine), m_stepPin(stepPin), m_dirPin(dirPin)
{
}

bool PicoStepGenerator::init()
{
    // The pin masks below shift by the pin number, and pio_sm_claim() panics
    // on a state machine that is taken, so check everything first.
    const PIO pio = m_program.pio();
    if (m_ready || !m_program.loaded() || m_sm >= NUM_PIO_STATE_MACHINES ||
        m_stepPin >= NUM_BANK0_GPIOS || m_dirPin >= NUM_BANK0_GPIOS || m_stepPin == m_dirPin ||
        pio_sm_is_claimed(pio, m_sm))
    {
        return false;
    }
    pio_sm_claim(pio, m_sm);

    pio_sm_config cfg = pio_get_default_sm_config();
    sm_config_set_wrap(&cfg, m_program.offset(), m_program.offset() + kLength - 1);
    sm_config_set_set_pins(&cfg, m_stepPin, 1);
    sm_config_set_out_pins(&cfg, m_dirPin, 1);
    sm_config_set_out_shift(&cfg, true, false, 32); // shift right, no autopull
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&cfg,
                         static_cast<float>(clock_get_hz(clk_sys)) /
                             static_cast<float>(PicoStepProgram::kClockHz));

    // Both pins low before the PIO takes them over, so the driver never sees
    // a stray edge.
    pio_sm_set_pins_with_mask(pio, m_sm, 0u, (1u << m_stepPin) | (1u << m_dirPin));
    pio_sm_set_consecutive_pindirs(pio, m_sm, m_stepPin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, m_sm, m_dirPin, 1, true);
    pio_gpio_init(pio, m_stepPin);
    pio_gpio_init(pio, m_dirPin);

    pio_sm_init(pio, m_sm, m_program.offset(), &cfg);
    pio_sm_set_enabled(pio, m_sm, true);
    m_ready = true;
    return true;
}

bool PicoStepGenerator::emit(int32_t steps, uint32_t durationUs)
{
    if (steps == 0)
    {
        return true;
    }
    const PIO pio = m_program.pio();
    if (!m_ready || m_halted || pio_sm_get_tx_fifo_level(pio, m_sm) != 0)
    {
        return false;
    }
    const bool forward = steps > 0;
    // Unsigned negation: −INT32_MIN does not fit an int32_t.
    const uint32_t count =
        forward ? static_cast<uint32_t>(steps) : 0u - static_cast<uint32_t>(steps);

    // Spread the pulses across the period. 64-bit: 5 cycles/µs times a long
    // period overflows 32 bits.
    const uint64_t cycles =
        static_cast<uint64_t>(durationUs) * (PicoStepProgram::kClockHz / 1'000'000u) / count;
    const uint32_t period = (cycles > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(cycles);
    const uint32_t delay =
        (period > PicoStepProgram::kCyclesPerStep) ? period - PicoStepProgram::kCyclesPerStep : 0u;

    // The FIFO is empty and eight deep, so both words always fit.
    pio_sm_put(pio, m_sm, delay);
    pio_sm_put(pio, m_sm, ((count - 1u) << 1) | (forward ? 1u : 0u));
    return true;
}

void PicoStepGenerator::halt()
{
    // The flag first: the loop must not queue a burst behind a machine that
    // is about to stop.
    m_halted = true;
    if (m_ready)
    {
        pio_sm_set_enabled(m_program.pio(), m_sm, false);
    }
}

void PicoStepGenerator::resume()
{
    if (!m_ready)
    {
        m_halted = false;
        return;
    }
    const PIO pio = m_program.pio();
    pio_sm_set_enabled(pio, m_sm, false);
    pio_sm_clear_fifos(pio, m_sm);
    pio_sm_restart(pio, m_sm);
    // The machine may have been frozen with STEP high.
    pio_sm_exec(pio, m_sm, pio_encode_set(pio_pins, 0));
    pio_sm_exec(pio, m_sm, pio_encode_jmp(m_program.offset()));
    pio_sm_set_enabled(pio, m_sm, true);
    m_halted = false;
}

} // namespace owlet::platform::pico
