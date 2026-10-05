#pragma once

#include "owlet/text/Sentence.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace owlet::text
{

enum class Command : uint8_t
{
    Ping,
    Enable,
    Disable,
    Move,
    Nudge,
    Home,
    Stop,
    Speed,
    Pos,
    Stream,
    Limits,
    Vel,
    Watchdog,
    Clock,
    Caps,
    Cal,
    Diag,
};

enum class RequestStatus : uint8_t
{
    Ok,
    BadFormat,   // known command, wrong field count or malformed number → NAK FMT
    Unknown,     // command name not recognised → ERR UNK
};

// A parsed request. Numeric arguments land in `args` in field order; range
// checking is the executor's job because limits are configuration. `token`
// holds a leading word argument (CAL's axis or action) and points into the
// frame buffer, so it is only valid while that frame is.
struct Request
{
    static constexpr std::size_t kMaxArgs = 5;

    Command                         command{Command::Ping};
    RequestStatus                   status{RequestStatus::Unknown};
    std::size_t                     argCount{0};
    std::array<float, kMaxArgs>     args{};
    std::string_view                token;
};

[[nodiscard]] RequestStatus parseRequest(const Sentence& sentence, Request& out);

std::string_view commandName(Command command);

// Strict decimal: [+-]?D{1,4}(\.D{1,2})?  — no exponents, nan, inf or blanks.
[[nodiscard]] bool parseDecimal(std::string_view text, float& out);

// Strict unsigned integer: D{1,5}.
[[nodiscard]] bool parseUnsigned(std::string_view text, uint32_t& out);

} // namespace owlet::text
