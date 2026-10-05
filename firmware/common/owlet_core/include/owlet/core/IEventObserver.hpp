#pragma once

#include "owlet/core/Event.hpp"

namespace owlet::core
{

/// Something that reacts to events. Always called from the main loop, one
/// event at a time, never from an interrupt and never re-entrantly.
class IEventObserver
{
public:
    virtual ~IEventObserver() = default;

    /// Called once for each event, in the order observers were listed.
    /// A handler may post further events; they are delivered after this one.
    virtual void onEvent(const Event& event) = 0;
};

} // namespace owlet::core
