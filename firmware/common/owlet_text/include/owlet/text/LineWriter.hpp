#pragma once

#include "owlet/text/Checksum.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace owlet::text
{

/// Builds one outgoing "$NAME,f1,f2*HH\r\n" line in a fixed buffer.
//
///   LineWriter w;
///   w.begin("ACK");
///   w.field("MOVE");
///   port.write(w.finish());
//
/// A field that would push the line past kMaxLine is dropped whole and
/// overflowed() latches, so the result is always a valid, checksummed frame.
class LineWriter
{
public:
    static constexpr std::size_t kMaxLine = kMaxLineLen;

    void begin(std::string_view name);
    /// Test helper: start from an already-assembled body ("NAME,f1,f2").
    void beginRaw(std::string_view body);

    void field(std::string_view text);
    /// Degrees with exactly one decimal, rounded half away from zero.
    void fixed1(float value);
    void uint(uint32_t value);

    /// Appends "*HH\r\n" and returns the full line (valid until next begin()).
    std::string_view finish();

    [[nodiscard]] bool overflowed() const { return m_overflowed; }

private:
    static constexpr std::size_t kTailLen = 5;  ///< "*HH\r\n"

    void append(std::string_view text);

    std::array<char, kMaxLine> m_buf{};
    std::size_t                m_len{0};
    bool                       m_overflowed{false};
};

} // namespace owlet::text
