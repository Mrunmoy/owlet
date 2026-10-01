#include "owlet/core/CommandProcessor.hpp"

#include "owlet/protocol/Sentence.hpp"

#include <array>

namespace owlet::core
{

using ::owlet::protocol::Command;
using ::owlet::protocol::FrameStatus;
using ::owlet::protocol::LineFramer;
using ::owlet::protocol::Request;
using ::owlet::protocol::RequestStatus;
using ::owlet::protocol::Sentence;

CommandProcessor::CommandProcessor(::owlet::hal::ISerialPort& port,
                                   PanTilt&                   head,
                                   std::string_view           firmwareVersion)
    : m_port(port), m_head(head), m_firmwareVersion(firmwareVersion)
{
}

void CommandProcessor::begin(uint32_t nowMs)
{
    m_lastTickMs = nowMs;
    sendVersioned("READY");
}

void CommandProcessor::service(uint32_t nowMs)
{
    tickMotion(nowMs);
    pumpInput();
}

// ── Motion + unsolicited events ─────────────────────────────────────────────

void CommandProcessor::tickMotion(uint32_t nowMs)
{
    uint32_t dt = nowMs - m_lastTickMs;  // unsigned: wraps correctly
    m_lastTickMs = nowMs;
    if (dt > kMaxTickMs)
    {
        dt = kMaxTickMs;
    }

    const bool ok = m_head.tick(dt);
    if (!ok && !m_busFault)
    {
        sendErr("BUS");
    }
    m_busFault = !ok;

    if (m_awaitingArrival && m_head.state() == PanTilt::State::Holding)
    {
        m_awaitingArrival = false;
        m_writer.begin("ARRIVED");
        m_writer.fixed1(m_head.panDeg());
        m_writer.fixed1(m_head.tiltDeg());
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

// ── Input ───────────────────────────────────────────────────────────────────

void CommandProcessor::pumpInput()
{
    std::array<uint8_t, kReadChunk> chunk{};
    for (std::size_t n = m_port.read(chunk.data(), chunk.size()); n != 0;
         n             = m_port.read(chunk.data(), chunk.size()))
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            switch (m_framer.push(chunk[i]))
            {
            case LineFramer::Event::Frame:
                handleFrame(m_framer.frame());
                break;
            case LineFramer::Event::Overflow:
                sendErr("LEN");
                break;
            case LineFramer::Event::None:
                break;
            }
        }
    }
}

void CommandProcessor::handleFrame(std::string_view frame)
{
    Sentence sentence;
    switch (::owlet::protocol::decodeSentence(frame, sentence))
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
    switch (::owlet::protocol::parseRequest(sentence, req))
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

void CommandProcessor::execute(const Request& req)
{
    switch (req.command)
    {
    case Command::Ping:
        sendVersioned("PONG");
        return;
    case Command::Enable:
        sendResult(req.command,
                   req.argCount == 2 ? m_head.enableAt(req.a, req.b) : m_head.enable());
        return;
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
        const PanTilt::Result r = m_head.stop();
        if (r == PanTilt::Result::Ok)
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
        m_nextStreamMs   = m_lastTickMs + m_streamPeriodMs;
        sendAck(req.command);
        return;
    }
    case Command::Limits:
        m_writer.begin("LIMITS");
        m_writer.fixed1(m_head.panServo().minDeg());
        m_writer.fixed1(m_head.panServo().maxDeg());
        m_writer.fixed1(m_head.tiltServo().minDeg());
        m_writer.fixed1(m_head.tiltServo().maxDeg());
        flush();
        return;
    }
}

// ── Replies ─────────────────────────────────────────────────────────────────

void CommandProcessor::sendResult(Command cmd, PanTilt::Result result)
{
    switch (result)
    {
    case PanTilt::Result::Ok:
        // Every accepted motion request earns exactly one ARRIVED, even when
        // the target equals the position, so clients can always wait on it.
        if (cmd == Command::Move || cmd == Command::Nudge || cmd == Command::Home ||
            (cmd == Command::Enable && m_head.state() == PanTilt::State::Moving))
        {
            m_awaitingArrival = true;
        }
        sendAck(cmd);
        return;
    case PanTilt::Result::Range:
        sendNak(cmd, "RNG");
        return;
    case PanTilt::Result::Disabled:
        sendNak(cmd, "DIS");
        return;
    case PanTilt::Result::Bus:
        sendNak(cmd, "BUS");
        return;
    }
}

void CommandProcessor::sendPos()
{
    static constexpr std::array<std::string_view, 3> kStateCodes{"D", "M", "H"};
    m_writer.begin("POS");
    m_writer.fixed1(m_head.panDeg());
    m_writer.fixed1(m_head.tiltDeg());
    m_writer.fixed1(m_head.panTargetDeg());
    m_writer.fixed1(m_head.tiltTargetDeg());
    m_writer.field(kStateCodes[static_cast<std::size_t>(m_head.state())]);
    flush();
}

void CommandProcessor::sendAck(Command cmd)
{
    m_writer.begin("ACK");
    m_writer.field(::owlet::protocol::commandName(cmd));
    flush();
}

void CommandProcessor::sendNak(Command cmd, std::string_view code)
{
    m_writer.begin("NAK");
    m_writer.field(::owlet::protocol::commandName(cmd));
    m_writer.field(code);
    flush();
}

void CommandProcessor::sendErr(std::string_view code)
{
    m_writer.begin("ERR");
    m_writer.field(code);
    flush();
}

void CommandProcessor::sendVersioned(std::string_view name)
{
    m_writer.begin(name);
    m_writer.uint(::owlet::protocol::kProtocolVersion);
    m_writer.field(m_firmwareVersion);
    flush();
}

void CommandProcessor::flush()
{
    const std::string_view line = m_writer.finish();
    // A refused write means no host is listening; dropping the line is the
    // correct behaviour for a stateless line protocol.
    (void) m_port.write(line.data(), line.size());
}

} // namespace owlet::core
