#pragma once

#include "owlet/protocol/Checksum.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace owlet::protocol
{

// Turns a raw byte stream into '$'-delimited frames (CR/LF stripped).
//
// Bytes outside a frame are dropped, which is what lets the ROM boot banner
// on the USB port pass harmlessly. A '$' always starts a fresh frame, so a
// line garbled mid-way resynchronises on the next sentence. A frame longer
// than kMaxFrameLen is reported once as Overflow and the rest of that line
// is discarded.
class LineFramer
{
public:
    enum class Event : uint8_t
    {
        None,
        Frame,     // frame() is valid until the next push()
        Overflow,
    };

    [[nodiscard]] Event push(uint8_t byte);

    std::string_view frame() const { return std::string_view(m_buf.data(), m_len); }

private:
    enum class State : uint8_t
    {
        Idle,
        InFrame,
        Discarding,
    };

    std::array<char, kMaxFrameLen> m_buf{};
    std::size_t                    m_len{0};
    State                          m_state{State::Idle};
};

} // namespace owlet::protocol
