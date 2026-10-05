#pragma once

#include "owlet/core/Event.hpp"

namespace owlet::core
{

/// Where an event source announces something. Sources hold only this, so
/// they know nothing about who is listening.
class IEventSink
{
public:
    virtual ~IEventSink() = default;

    /// Hands over an event for later delivery. Safe to call from an
    /// interrupt handler. The sink stamps the time.
    /// @param id   What happened.
    /// @param axis 0 = pan, 1 = tilt, or Event::kHead.
    /// @param arg  Detail whose meaning depends on `id`; 0 if it has none.
    /// @return false if the event could not be kept. The loss is counted and
    ///         reported to the listeners; the caller need do nothing.
    virtual bool post(EventId id, uint8_t axis, uint16_t arg) = 0;
};

} // namespace owlet::core
