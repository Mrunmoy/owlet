#pragma once

#include "owlet/core/ICalibrationStore.hpp"

namespace owlet::test
{

class FakeCalibrationStore final : public ::owlet::core::ICalibrationStore
{
public:
    [[nodiscard]] bool load(::owlet::core::CalibrationRecord& out) override
    {
        ++loads;
        if (!hasRecord)
        {
            return false;
        }
        out = record;
        return true;
    }

    [[nodiscard]] bool save(const ::owlet::core::CalibrationRecord& in) override
    {
        if (failSave)
        {
            return false;
        }
        record    = in;
        hasRecord = true;
        ++saves;
        return true;
    }

    [[nodiscard]] bool erase() override
    {
        if (failErase)
        {
            return false;
        }
        hasRecord = false;
        ++erases;
        return true;
    }

    ::owlet::core::CalibrationRecord record{};
    bool                             hasRecord{false};
    bool                             failSave{false};
    bool                             failErase{false};
    int                              loads{0};
    int                              saves{0};
    int                              erases{0};
};

} // namespace owlet::test
