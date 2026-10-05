#pragma once

#include "owlet/hal/IAxisDiagnostics.hpp"
#include "owlet/hal/IPollable.hpp"
#include "owlet/stepper/Tmc2209.hpp"

#include <cstdint>

namespace owlet
{

// Looks after one TMC2209 from inside the control loop without getting in
// its way: one short exchange every kPollPeriodMs, never more.
//
//  - Until the chip answers (its motor supply may be off) it is probed once
//    a second; the head keeps running in standalone step/dir mode meanwhile.
//  - When it answers it is configured, and from then on its status, load
//    and reset flag are read in turn.
//  - A chip that restarted (its supply dipped) is configured again; one
//    that stops answering goes back to being probed.
class Tmc2209Monitor final : public ::owlet::hal::IPollable, public ::owlet::hal::IAxisDiagnostics
{
public:
    static constexpr uint32_t kPollPeriodMs = 100;
    static constexpr uint32_t kProbePeriodMs = 1000;
    static constexpr uint8_t kMaxFailures = 3;

    // `phaseMs` delays this monitor's schedule, so two drivers don't both
    // take their slice in the same loop pass.
    explicit Tmc2209Monitor(Tmc2209& driver, uint32_t phaseMs = 0);

    Tmc2209Monitor(const Tmc2209Monitor&) = delete;
    Tmc2209Monitor& operator=(const Tmc2209Monitor&) = delete;

    void poll(uint32_t dtMs) override;

    [[nodiscard]] ::owlet::hal::AxisDiagnostics diagnostics() const override;
    [[nodiscard]] bool online() const
    {
        return m_online;
    }

private:
    enum class Step : uint8_t
    {
        Status,
        Load,
        ResetFlag,
    };

    void probe();
    void readNext();
    void noteFailure();

    Tmc2209& m_driver;
    Tmc2209::Status m_status{};
    uint32_t m_elapsedMs;
    uint16_t m_load{0};
    Step m_step{Step::Status};
    uint8_t m_failures{0};
    bool m_online{false};
};

} // namespace owlet
