#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace owlet::text
{

inline constexpr uint8_t kProtocolVersion = 3;

/// Longest frame on the wire including '$' and the trailing CRLF.
inline constexpr std::size_t kMaxLineLen = 82;
/// Longest frame as seen by the parser: '$' .. checksum, CRLF stripped.
inline constexpr std::size_t kMaxFrameLen = kMaxLineLen - 2;

/// XOR of every byte of `body` (the text between '$' and '*').
[[nodiscard]] constexpr uint8_t checksum(std::string_view body)
{
    uint8_t ck = 0;
    for (char c : body)
    {
        ck = static_cast<uint8_t>(ck ^ static_cast<uint8_t>(c));
    }
    return ck;
}

static_assert(checksum("PING") == 0x10, "checksum must match the protocol vectors");

} // namespace owlet::text
