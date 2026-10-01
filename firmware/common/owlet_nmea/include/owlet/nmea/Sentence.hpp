#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace owlet::nmea
{

enum class FrameStatus : uint8_t
{
    Ok,
    Syntax,     // not of the form $NAME[,field]*HH
    Checksum,   // well-formed but checksum mismatch
};

// A decoded frame. Views point into the caller's frame buffer, so a Sentence
// is only valid while that buffer is unchanged.
struct Sentence
{
    static constexpr std::size_t kMaxFields  = 8;
    static constexpr std::size_t kMaxNameLen = 10;

    std::string_view                         name;
    std::array<std::string_view, kMaxFields> fields{};
    std::size_t                              fieldCount{0};
};

// `frame` is '$' through the two checksum digits, without CR/LF.
[[nodiscard]] FrameStatus decodeSentence(std::string_view frame, Sentence& out);

} // namespace owlet::nmea
