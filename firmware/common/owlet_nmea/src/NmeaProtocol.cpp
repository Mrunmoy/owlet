#include "owlet/nmea/NmeaProtocol.hpp"

#include "owlet/nmea/Sentence.hpp"

#include <array>

namespace owlet::nmea
{

using ::owlet::core::ConsumeResult;
using ::owlet::core::IPanTilt;
using ::owlet::core::TickStatus;

NmeaProtocol::NmeaProtocol(::owlet::hal::IByteSink&    out,
                           IPanTilt&                   head,
                           const ::owlet::hal::IClock& clock,
                           std::string_view            firmwareVersion)
    : m_out(out), m_head(head), m_clock(clock), m_firmwareVersion(firmwareVersion)
{
}

// ── IProtocol ───────────────────────────────────────────────────────────────

bool NmeaProtocol::claims(uint8_t byte, bool /*atLineStart*/) const
{
    // '$' starts a frame anywhere, so a garbled line resyncs on the next one.
    return byte == '$';
}

ConsumeResult NmeaProtocol::consume(const uint8_t* data, std::size_t len)
{
    for (std::size_t i = 0; i < len; ++i)
    {
        const uint8_t b = data[i];
        const bool    eol = (b == '\r' || b == '\n');
        if (!eol && (b < 0x20 || b > 0x7E))
        {
            // Not text. A request already under way was garbled: it is still
            // owed a reply. A lone '$' (or no frame at all) is just noise
            // before somebody else's binary frame: hand back silently. Either
            // way the byte goes back unconsumed for another protocol.
            const bool garbled = m_framer.inFrame() && m_framer.frame().size() > 1;
            m_framer.reset();
            if (garbled)
            {
                sendErr("SYN");
            }
            return {i, true};
        }
        switch (m_framer.push(b))
        {
        case LineFramer::Event::Frame:
            handleFrame(m_framer.frame());
            break;
        case LineFramer::Event::Overflow:
            sendErr("LEN");
            return {i + 1, true};  // let the mux drop the rest of the line
        case LineFramer::Event::None:
            break;
        }
        if (eol)
        {
            return {i + 1, true};
        }
    }
    return {len, false};
}

void NmeaProtocol::begin(uint32_t nowMs)
{
    m_nowMs = nowMs;
    sendVersioned("READY");
}

void NmeaProtocol::onTick(uint32_t nowMs, const TickStatus& status)
{
    m_nowMs = nowMs;

    if (!status.busOk && !m_busFault)
    {
        sendErr("BUS");
    }
    m_busFault = !status.busOk;

    if (status.watchdogTripped)
    {
        m_writer.begin("TIMEOUT");
        m_writer.field("VEL");
        flush();
    }

    if (m_awaitingArrival && m_head.state() == IPanTilt::State::Holding)
    {
        m_awaitingArrival = false;
        const IPanTilt::Pose p = m_head.position();
        m_writer.begin("ARRIVED");
        m_writer.fixed1(p.pan);
        m_writer.fixed1(p.tilt);
        flush();
    }

    // Signed difference so the schedule survives the 49-day millis wrap.
    if (m_streamPeriodMs != 0 && static_cast<int32_t>(nowMs - m_nextStreamMs) >= 0)
    {
        m_nextStreamMs += m_streamPeriodMs;
        if (static_cast<int32_t>(nowMs - m_nextStreamMs) >= 0)
        {
            m_nextStreamMs = nowMs + m_streamPeriodMs;  // fell behind: don't burst
        }
        sendPos();
    }
}

// ── Request handling ────────────────────────────────────────────────────────

void NmeaProtocol::handleFrame(std::string_view frame)
{
    Sentence sentence;
    switch (decodeSentence(frame, sentence))
    {
    case FrameStatus::Ok:
        break;
    case FrameStatus::Checksum:
        sendErr("CHK");
        return;
    case FrameStatus::Syntax:
        sendErr("SYN");
        return;
    }

    Request req;
    switch (parseRequest(sentence, req))
    {
    case RequestStatus::Ok:
        execute(req);
        break;
    case RequestStatus::BadFormat:
        sendNak(req.command, "FMT");
        break;
    case RequestStatus::Unknown:
        sendErr("UNK");
        break;
    }
}

void NmeaProtocol::execute(const Request& req)
{
    switch (req.command)
    {
    case Command::Ping:
        sendVersioned("PONG");
        return;
    case Command::Enable:
    {
        // Only ENABLE,<pan>,<tilt> on an already-powered head is a move (and
        // earns ARRIVED); from disabled it is a snap, and bare ENABLE moves nothing.
        const bool   wasEnabled = m_head.state() != IPanTilt::State::Disabled;
        const Result r = req.argCount == 2 ? m_head.enableAt(req.a, req.b) : m_head.enable();
        if (r == Result::Ok && req.argCount == 2 && wasEnabled)
        {
            m_awaitingArrival = true;
        }
        sendResult(req.command, r);
        return;
    }
    case Command::Disable:
        m_awaitingArrival = false;
        sendResult(req.command, m_head.disable());
        return;
    case Command::Move:
        sendResult(req.command, m_head.move(req.a, req.b));
        return;
    case Command::Nudge:
        sendResult(req.command, m_head.nudge(req.a, req.b));
        return;
    case Command::Home:
        sendResult(req.command, m_head.home());
        return;
    case Command::Stop:
    {
        const Result r = m_head.stop();
        if (r == Result::Ok)
        {
            m_awaitingArrival = false;
        }
        sendResult(req.command, r);
        return;
    }
    case Command::Speed:
        if (req.argCount == 0)
        {
            m_writer.begin("SPEED");
            m_writer.fixed1(m_head.speed());
            flush();
            return;
        }
        sendResult(req.command, m_head.setSpeed(req.a));
        return;
    case Command::Pos:
        sendPos();
        return;
    case Command::Stream:
    {
        const uint32_t hz = static_cast<uint32_t>(req.a);
        if (hz > kMaxStreamHz)
        {
            sendNak(req.command, "RNG");
            return;
        }
        m_streamPeriodMs = (hz == 0) ? 0 : 1000U / hz;
        m_nextStreamMs   = m_nowMs + m_streamPeriodMs;
        sendAck(req.command);
        return;
    }
    case Command::Vel:
    {
        const Result r = m_head.setVelocity(req.a, req.b);
        if (r == Result::Ok)
        {
            m_awaitingArrival = false;  // velocity mode never "arrives"
        }
        sendResult(req.command, r);
        return;
    }
    case Command::Watchdog:
        if (req.argCount == 0)
        {
            m_writer.begin("WATCHDOG");
            m_writer.uint(m_head.watchdogMs());
            flush();
            return;
        }
        sendResult(req.command, m_head.setWatchdog(static_cast<uint32_t>(req.a)));
        return;
    case Command::Clock:
        // The live clock, not the tick time: this is the sample a host uses
        // to estimate the clock offset, so it must be as fresh as possible.
        m_writer.begin("CLOCK");
        m_writer.uint(m_clock.nowMs());
        flush();
        return;
    case Command::Limits:
    {
        const IPanTilt::Limits lim = m_head.limits();
        m_writer.begin("LIMITS");
        m_writer.fixed1(lim.panMin);
        m_writer.fixed1(lim.panMax);
        m_writer.fixed1(lim.tiltMin);
        m_writer.fixed1(lim.tiltMax);
        flush();
        return;
    }
    }
}

// ── Replies ─────────────────────────────────────────────────────────────────

void NmeaProtocol::sendResult(Command cmd, Result result)
{
    switch (result)
    {
    case Result::Ok:
        // Every accepted motion request earns exactly one ARRIVED, even when
        // the target equals the position, so clients can always wait on it.
        if (cmd == Command::Move || cmd == Command::Nudge || cmd == Command::Home)
        {
            m_awaitingArrival = true;
        }
        sendAck(cmd);
        return;
    case Result::Range:
        sendNak(cmd, "RNG");
        return;
    case Result::Disabled:
        sendNak(cmd, "DIS");
        return;
    case Result::Bus:
        sendNak(cmd, "BUS");
        return;
    }
}

void NmeaProtocol::sendPos()
{
    static constexpr std::array<std::string_view, 3> kStateCodes{"D", "M", "H"};
    const IPanTilt::Pose p = m_head.position();
    const IPanTilt::Pose t = m_head.target();
    m_writer.begin("POS");
    m_writer.fixed1(p.pan);
    m_writer.fixed1(p.tilt);
    m_writer.fixed1(t.pan);
    m_writer.fixed1(t.tilt);
    m_writer.field(kStateCodes[static_cast<std::size_t>(m_head.state())]);
    m_writer.uint(m_nowMs);
    flush();
}

void NmeaProtocol::sendAck(Command cmd)
{
    m_writer.begin("ACK");
    m_writer.field(commandName(cmd));
    flush();
}

void NmeaProtocol::sendNak(Command cmd, std::string_view code)
{
    m_writer.begin("NAK");
    m_writer.field(commandName(cmd));
    m_writer.field(code);
    flush();
}

void NmeaProtocol::sendErr(std::string_view code)
{
    m_writer.begin("ERR");
    m_writer.field(code);
    flush();
}

void NmeaProtocol::sendVersioned(std::string_view name)
{
    m_writer.begin(name);
    m_writer.uint(kProtocolVersion);
    m_writer.field(m_firmwareVersion);
    flush();
}

void NmeaProtocol::flush()
{
    const std::string_view line = m_writer.finish();
    // A refused write means no host is listening; dropping the line is the
    // correct behaviour for a stateless line protocol.
    (void) m_out.write(line.data(), line.size());
}

} // namespace owlet::nmea
