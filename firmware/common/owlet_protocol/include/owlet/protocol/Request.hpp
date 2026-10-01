#pragma once

#include "owlet/protocol/Sentence.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace owlet::protocol
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
};

enum class RequestStatus : uint8_t
{
    Ok,
    BadFormat,   // known command, wrong field count or malformed number → NAK FMT
    Unknown,     // command name not recognised → ERR UNK
};

// A parsed request. Numeric arguments land in a/b in field order; range
// checking is the executor's job because limits are configuration.
struct Request
{
    Command       command{Command::Ping};
    RequestStatus status{RequestStatus::Unknown};
    std::size_t   argCount{0};
    float         a{0.0f};
    float         b{0.0f};
};

[[nodiscard]] RequestStatus parseRequest(const Sentence& sentence, Request& out);

std::string_view commandName(Command command);

// Strict decimal: [+-]?D{1,4}(\.D{1,2})?  — no exponents, nan, inf or blanks.
[[nodiscard]] bool parseDecimal(std::string_view text, float& out);

// Strict unsigned integer: D{1,5}.
[[nodiscard]] bool parseUnsigned(std::string_view text, uint32_t& out);

} // namespace owlet::protocol
