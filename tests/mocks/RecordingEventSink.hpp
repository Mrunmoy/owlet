#pragma once

#include "owlet/core/IEventSink.hpp"

#include <vector>

namespace owlet::test
{

/// An event sink that keeps what it is given, so a test can read a scenario
/// back as the list of things that were announced.
class RecordingEventSink final : public ::owlet::core::IEventSink
{
public:
    bool post(::owlet::core::EventId id, uint8_t axis, uint16_t arg) override
    {
        ids.push_back(id);
        ::owlet::core::Event e;
        e.id = id;
        e.axis = axis;
        e.arg = arg;
        events.push_back(e);
        return true;
    }

    /// Returns the ids posted since the last call, and forgets them.
    std::vector<::owlet::core::EventId> take()
    {
        std::vector<::owlet::core::EventId> out;
        out.swap(ids);
        return out;
    }

    std::vector<::owlet::core::EventId> ids;
    std::vector<::owlet::core::Event> events;
};

} // namespace owlet::test
