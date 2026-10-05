#pragma once

#include "owlet/core/IProtocol.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace owlet::test
{

// Line-oriented test protocol: claims frames starting with `claimByte`
// (anywhere, or only at line start) and releases after '\n'.
class FakeProtocol final : public ::owlet::core::IProtocol
{
public:
    FakeProtocol(char claimByte, bool anywhere) : m_claimByte(claimByte), m_anywhere(anywhere) {}

    [[nodiscard]] bool claims(uint8_t byte, bool atLineStart) const override
    {
        return static_cast<char>(byte) == m_claimByte && (m_anywhere || atLineStart);
    }

    [[nodiscard]] ::owlet::core::ConsumeResult consume(const uint8_t* data,
                                                      std::size_t    len) override
    {
        ++consumeCalls;
        const std::size_t limit = (maxPerCall != 0 && maxPerCall < len) ? maxPerCall : len;
        for (std::size_t i = 0; i < limit; ++i)
        {
            const char c = static_cast<char>(data[i]);
            current.push_back(c);
            if (c == '\n')
            {
                frames.push_back(current);
                current.clear();
                return {i + 1, true};
            }
        }
        return {limit, false};
    }

    void begin(uint32_t nowMs) override
    {
        beginAt.push_back(nowMs);
    }

    void onTick(uint32_t nowMs) override
    {
        tickAt.push_back(nowMs);
        if (log != nullptr)
        {
            log->push_back("onTick");
        }
    }

    std::vector<std::string>              frames;
    std::string                           current;
    std::vector<uint32_t>                 beginAt;
    std::vector<uint32_t>                 tickAt;
    
    std::vector<std::string>*             log{nullptr};
    std::size_t                           maxPerCall{0};  // 0 = unlimited
    int                                   consumeCalls{0};

private:
    char m_claimByte;
    bool m_anywhere;
};

} // namespace owlet::test
