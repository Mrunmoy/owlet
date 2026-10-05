#include "owlet/text/Checksum.hpp"
#include "owlet/text/LineFramer.hpp"
#include "owlet/text/LineWriter.hpp"
#include "owlet/text/Request.hpp"
#include "owlet/text/Sentence.hpp"

#include <array>
#include <cmath>
#include <cstdint>

namespace owlet::text
{

namespace
{

bool isDigit(char c)
{
    return c >= '0' && c <= '9';
}

bool isUpper(char c)
{
    return c >= 'A' && c <= 'Z';
}

bool isFieldChar(char c)
{
    return c >= 0x20 && c <= 0x7E && c != '$' && c != '*';
}

int hexValue(char c)
{
    if (isDigit(c)) return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Allowed argument counts per command, as a bitmask (bit n = n args OK).
struct CommandSpec
{
    std::string_view name;
    Command          command;
    uint8_t          argCountMask;
    bool             integerArgs;
};

constexpr uint8_t kArgs0  = 1U << 0;
constexpr uint8_t kArgs1  = 1U << 1;
constexpr uint8_t kArgs2  = 1U << 2;

constexpr std::array<CommandSpec, 17> kCommands{{
    {"PING", Command::Ping, kArgs0, false},
    {"ENABLE", Command::Enable, kArgs0 | kArgs2, false},
    {"DISABLE", Command::Disable, kArgs0, false},
    {"MOVE", Command::Move, kArgs2, false},
    {"NUDGE", Command::Nudge, kArgs2, false},
    {"HOME", Command::Home, kArgs0, false},
    {"STOP", Command::Stop, kArgs0, false},
    {"SPEED", Command::Speed, kArgs0 | kArgs1, false},
    {"POS", Command::Pos, kArgs0, false},
    {"STREAM", Command::Stream, kArgs1, true},
    {"LIMITS", Command::Limits, kArgs0, false},
    {"VEL", Command::Vel, kArgs2, false},
    {"WATCHDOG", Command::Watchdog, kArgs0 | kArgs1, true},
    {"CLOCK", Command::Clock, kArgs0, false},
    {"CAPS", Command::Caps, kArgs0, false},
    {"CAL", Command::Cal, 0, false},  // parsed by parseCal()
    {"DIAG", Command::Diag, 0, false},  // parsed by parseDiag()
}};

} // namespace

// ── LineFramer ──────────────────────────────────────────────────────────────

LineFramer::Event LineFramer::push(uint8_t byte)
{
    const char c = static_cast<char>(byte);
    if (c == '$')
    {
        m_buf[0] = c;
        m_len    = 1;
        m_state  = State::InFrame;
        return Event::None;
    }
    if (c == '\r' || c == '\n')
    {
        const bool complete = (m_state == State::InFrame) && (m_len > 1);
        m_state             = State::Idle;
        return complete ? Event::Frame : Event::None;
    }
    if (m_state != State::InFrame)
    {
        return Event::None;
    }
    if (m_len == m_buf.size())
    {
        m_state = State::Discarding;
        return Event::Overflow;
    }
    m_buf[m_len++] = c;
    return Event::None;
}

// ── Sentence ────────────────────────────────────────────────────────────────

FrameStatus decodeSentence(std::string_view frame, Sentence& out)
{
    // Shortest legal frame: "$X*HH".
    if (frame.size() < 5 || frame.size() > kMaxFrameLen || frame.front() != '$')
    {
        return FrameStatus::Syntax;
    }
    const std::size_t star = frame.size() - 3;
    if (frame[star] != '*')
    {
        return FrameStatus::Syntax;
    }
    const int hi = hexValue(frame[star + 1]);
    const int lo = hexValue(frame[star + 2]);
    if (hi < 0 || lo < 0)
    {
        return FrameStatus::Syntax;
    }
    const std::string_view body = frame.substr(1, star - 1);
    for (char c : body)
    {
        if (!isFieldChar(c))
        {
            return FrameStatus::Syntax;
        }
    }
    if (checksum(body) != static_cast<uint8_t>((hi << 4) | lo))
    {
        return FrameStatus::Checksum;
    }

    out            = Sentence{};
    std::size_t at = body.find(',');
    out.name       = body.substr(0, at);
    if (out.name.empty() || out.name.size() > Sentence::kMaxNameLen)
    {
        return FrameStatus::Syntax;
    }
    for (char c : out.name)
    {
        if (!isUpper(c))
        {
            return FrameStatus::Syntax;
        }
    }
    while (at != std::string_view::npos)
    {
        if (out.fieldCount == Sentence::kMaxFields)
        {
            return FrameStatus::Syntax;
        }
        const std::size_t start = at + 1;
        at                      = body.find(',', start);
        const std::size_t len   = (at == std::string_view::npos) ? body.size() - start
                                                                 : at - start;
        out.fields[out.fieldCount++] = body.substr(start, len);
    }
    return FrameStatus::Ok;
}

// ── Request ─────────────────────────────────────────────────────────────────

bool parseDecimal(std::string_view text, float& out)
{
    std::size_t i        = 0;
    bool        negative = false;
    if (i < text.size() && (text[i] == '+' || text[i] == '-'))
    {
        negative = text[i] == '-';
        ++i;
    }
    int32_t     scaled    = 0;  // value × 100
    std::size_t intDigits = 0;
    while (i < text.size() && isDigit(text[i]))
    {
        if (++intDigits > 4) return false;
        scaled = scaled * 10 + (text[i] - '0');
        ++i;
    }
    if (intDigits == 0) return false;
    scaled *= 100;
    if (i < text.size() && text[i] == '.')
    {
        ++i;
        std::size_t fracDigits = 0;
        int32_t     place      = 10;
        while (i < text.size() && isDigit(text[i]))
        {
            if (++fracDigits > 2) return false;
            scaled += (text[i] - '0') * place;
            place /= 10;
            ++i;
        }
        if (fracDigits == 0) return false;
    }
    if (i != text.size()) return false;
    out = static_cast<float>(negative ? -scaled : scaled) / 100.0f;
    return true;
}

bool parseUnsigned(std::string_view text, uint32_t& out)
{
    if (text.empty() || text.size() > 5) return false;
    uint32_t v = 0;
    for (char c : text)
    {
        if (!isDigit(c)) return false;
        v = v * 10U + static_cast<uint32_t>(c - '0');
    }
    out = v;
    return true;
}

// DIAG,<PAN|TILT>
static RequestStatus parseDiag(const Sentence& s, Request& out)
{
    if (s.fieldCount != 1 || (s.fields[0] != "PAN" && s.fields[0] != "TILT"))
    {
        return RequestStatus::BadFormat;
    }
    out.token = s.fields[0];
    return RequestStatus::Ok;
}

// CAL,<PAN|TILT|SAVE|RESET>  or  CAL,<PAN|TILT>,<min_us>,<center_us>,<max_us>,<min_deg>,<max_deg>
static RequestStatus parseCal(const Sentence& s, Request& out)
{
    if (s.fieldCount == 0)
    {
        return RequestStatus::BadFormat;
    }
    const std::string_view t    = s.fields[0];
    const bool             axis = (t == "PAN" || t == "TILT");
    const bool             verb = (t == "SAVE" || t == "RESET");
    out.token                   = t;
    if (s.fieldCount == 1 && (axis || verb))
    {
        return RequestStatus::Ok;
    }
    if (s.fieldCount != 6 || !axis)
    {
        return RequestStatus::BadFormat;
    }
    for (std::size_t i = 0; i < 3; ++i)  // pulses: unsigned integers
    {
        uint32_t v = 0;
        if (!parseUnsigned(s.fields[i + 1], v)) return RequestStatus::BadFormat;
        out.args[i] = static_cast<float>(v);
    }
    for (std::size_t i = 3; i < 5; ++i)  // limits: decimals
    {
        if (!parseDecimal(s.fields[i + 1], out.args[i])) return RequestStatus::BadFormat;
    }
    out.argCount = 5;
    return RequestStatus::Ok;
}

RequestStatus parseRequest(const Sentence& sentence, Request& out)
{
    out = Request{};
    for (const CommandSpec& spec : kCommands)
    {
        if (spec.name != sentence.name)
        {
            continue;
        }
        out.command = spec.command;
        if (spec.command == Command::Cal)
        {
            return parseCal(sentence, out);
        }
        if (spec.command == Command::Diag)
        {
            return parseDiag(sentence, out);
        }
        const std::size_t n = sentence.fieldCount;
        if (n > 2 || (spec.argCountMask & (1U << n)) == 0)
        {
            return RequestStatus::BadFormat;
        }
        for (std::size_t i = 0; i < n; ++i)
        {
            if (spec.integerArgs)
            {
                uint32_t v = 0;
                if (!parseUnsigned(sentence.fields[i], v)) return RequestStatus::BadFormat;
                out.args[i] = static_cast<float>(v);
            }
            else if (!parseDecimal(sentence.fields[i], out.args[i]))
            {
                return RequestStatus::BadFormat;
            }
        }
        out.argCount = n;
        return RequestStatus::Ok;
    }
    return RequestStatus::Unknown;
}

std::string_view commandName(Command command)
{
    for (const CommandSpec& spec : kCommands)
    {
        if (spec.command == command) return spec.name;
    }
    return "?";
}

// ── LineWriter ──────────────────────────────────────────────────────────────

void LineWriter::begin(std::string_view name)
{
    m_len        = 0;
    m_overflowed = false;
    append("$");
    append(name);
}

void LineWriter::beginRaw(std::string_view body)
{
    begin(body);
}

void LineWriter::append(std::string_view text)
{
    if (m_len + text.size() + kTailLen > m_buf.size())
    {
        m_overflowed = true;
        return;
    }
    for (char c : text)
    {
        m_buf[m_len++] = c;
    }
}

void LineWriter::field(std::string_view text)
{
    // Separator and text fit together or not at all, so a dropped field
    // never leaves a dangling comma.
    if (m_len + 1 + text.size() + kTailLen > m_buf.size())
    {
        m_overflowed = true;
        return;
    }
    m_buf[m_len++] = ',';
    append(text);
}

void LineWriter::fixed1(float value)
{
    // Double for the scale so values like -15.05f round consistently.
    const long tenths = std::lround(static_cast<double>(value) * 10.0);
    const unsigned long mag = static_cast<unsigned long>(tenths < 0 ? -tenths : tenths);

    std::array<char, 16> tmp{};
    std::size_t          n = tmp.size();
    tmp[--n] = static_cast<char>('0' + mag % 10);
    tmp[--n] = '.';
    unsigned long whole = mag / 10;
    do
    {
        tmp[--n] = static_cast<char>('0' + whole % 10);
        whole /= 10;
    } while (whole != 0 && n > 1);
    if (tenths < 0)
    {
        tmp[--n] = '-';
    }
    field(std::string_view(tmp.data() + n, tmp.size() - n));
}

void LineWriter::uint(uint32_t value)
{
    std::array<char, 10> tmp{};
    std::size_t          n = tmp.size();
    do
    {
        tmp[--n] = static_cast<char>('0' + value % 10U);
        value /= 10U;
    } while (value != 0);
    field(std::string_view(tmp.data() + n, tmp.size() - n));
}

std::string_view LineWriter::finish()
{
    static constexpr char kHex[] = "0123456789ABCDEF";
    const uint8_t ck = checksum(std::string_view(m_buf.data() + 1, m_len - 1));
    m_buf[m_len++] = '*';
    m_buf[m_len++] = kHex[ck >> 4];
    m_buf[m_len++] = kHex[ck & 0x0F];
    m_buf[m_len++] = '\r';
    m_buf[m_len++] = '\n';
    return std::string_view(m_buf.data(), m_len);
}

} // namespace owlet::text
