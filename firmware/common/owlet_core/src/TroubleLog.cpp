#include "owlet/core/TroubleLog.hpp"

#include <limits>

namespace owlet::core
{

void TroubleLog::record(TroubleCode code, uint8_t axis, uint16_t detail)
{
    const uint32_t now = eventTimeMs();
    for (std::size_t i = 0; i < m_size; ++i)
    {
        TroubleRecord& r = m_records[i];
        if (r.code == code && r.axis == axis)
        {
            r.count = (r.count < std::numeric_limits<uint16_t>::max())
                          ? static_cast<uint16_t>(r.count + 1)
                          : r.count;
            r.detail = detail;
            r.lastMs = now;
            return;
        }
    }
    if (m_size < kCapacity)
    {
        m_records[m_size++] = TroubleRecord{code, axis, 1, detail, now, now};
    }
}

} // namespace owlet::core
