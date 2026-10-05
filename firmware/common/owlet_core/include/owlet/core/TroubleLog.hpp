#pragma once

#include "owlet/core/Event.hpp"
#include "owlet/core/IEventObserver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::core
{

/// What can go wrong, as numbers that never change meaning. They go on the
/// wire, so a value is never reused for something else.
enum class TroubleCode : uint8_t
{
    MotorBus = 10,        ///< A motor update did not reach the hardware.
    WatchdogExpired = 11, ///< Velocity mode was stopped for lack of a command.
    DriverOffline = 20,   ///< A motor driver stopped answering.
    DriverFault = 21,     ///< A motor driver raised a fault. Detail: its fault bits.
    LimitHit = 30,        ///< A limit switch tripped. Detail: 0 = min end, 1 = max end.
    PositionLost = 31,    ///< The step count stopped being trustworthy.
    Stalled = 32,         ///< A motor stopped following its steps.
    HomingFailed = 40,    ///< Homing gave up. Detail: the reason.
    EventsLost = 50,      ///< Events were dropped. Detail: how many, last time.
};

/// One kind of trouble on one axis, and how often it has happened.
struct TroubleRecord
{
    TroubleCode code{TroubleCode::MotorBus};
    uint8_t axis{Event::kHead}; ///< 0 = pan, 1 = tilt, or Event::kHead.
    uint16_t count{0};          ///< Times seen since power-up or clear(); stops at 65535.
    uint16_t detail{0};         ///< From the most recent occurrence; meaning depends on `code`.
    uint32_t firstMs{0};        ///< Device clock at the first occurrence.
    uint32_t lastMs{0};         ///< Device clock at the most recent one.
};

/// Remembers what has gone wrong since power-up, so that "why did it stop?"
/// can be answered after the fact. It listens to the event bus and keeps one
/// record per kind of trouble per axis; a repeat raises the count. Nothing
/// is stored across a power cycle.
class TroubleLog final : public IEventObserver
{
public:
    /// One record for every code on each axis it can apply to.
    static constexpr std::size_t kCapacity = 16;

    void onEvent(const Event& event) override;

    /// @return The number of records held.
    [[nodiscard]] std::size_t size() const
    {
        return m_size;
    }

    /// @param index Below size(), in order of first occurrence.
    [[nodiscard]] const TroubleRecord& at(std::size_t index) const
    {
        return m_records[index];
    }

    /// Forgets everything recorded so far.
    void clear()
    {
        m_size = 0;
    }

private:
    std::array<TroubleRecord, kCapacity> m_records{};
    std::size_t m_size{0};
};

} // namespace owlet::core
