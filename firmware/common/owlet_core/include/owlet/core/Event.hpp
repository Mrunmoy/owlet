#pragma once

#include <cstdint>
#include <type_traits>

namespace owlet::core
{

/// What happened. Each value is a change of state that something noticed
/// once; none of them is a measurement to be read repeatedly.
enum class EventId : uint8_t
{
    Arrived,            ///< A position move reached its target.
    WatchdogExpired,    ///< Velocity mode was stopped for lack of a command.
    LimitHit,           ///< A limit switch tripped. `arg` is a LimitEnd.
    Homed,              ///< Homing finished; the position can be trusted.
    HomingFailed,       ///< Homing gave up. `arg` is the reason.
    PositionLost,       ///< The step count no longer says where the axis is.
    Stalled,            ///< The motor stopped following its steps.
    DriverFaultChanged, ///< A motor driver's fault bits changed. `arg` is the new set.
    DriverOnline,       ///< A motor driver started answering.
    DriverOffline,      ///< A motor driver stopped answering.
    BusFault,           ///< A motor could not be commanded.
    BusOk,              ///< Commands are getting through again.
    EventsLost,         ///< Events were dropped. `arg` is how many.
};

/// Which end of an axis's travel, as carried in a LimitHit's `arg`.
enum class LimitEnd : uint8_t
{
    Min = 0,
    Max = 1,
};

/// One notification. Small and plain on purpose: it is copied into a queue
/// from interrupts, compared in tests and written to the wire from a table.
struct Event
{
    /// `axis` for an event about the whole head rather than one axis.
    static constexpr uint8_t kHead = 0xFF;

    EventId id{EventId::Arrived};
    uint8_t axis{kHead}; ///< 0 = pan, 1 = tilt, or kHead.
    uint16_t arg{0};     ///< Meaning depends on `id`.
    uint32_t timeMs{0};  ///< When it was posted, on the device clock.
};

static_assert(sizeof(Event) == 8, "Event is meant to stay two words");
static_assert(std::is_trivially_copyable<Event>::value, "Event is copied in and out of a queue");

[[nodiscard]] constexpr bool operator==(const Event& a, const Event& b)
{
    return a.id == b.id && a.axis == b.axis && a.arg == b.arg && a.timeMs == b.timeMs;
}

[[nodiscard]] constexpr bool operator!=(const Event& a, const Event& b)
{
    return !(a == b);
}

} // namespace owlet::core
