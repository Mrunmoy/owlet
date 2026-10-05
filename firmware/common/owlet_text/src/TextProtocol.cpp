#include "owlet/text/TextProtocol.hpp"

#include "owlet/text/Sentence.hpp"

#include <array>

namespace owlet::text
{

using ::owlet::core::ConsumeResult;
using ::owlet::core::IPanTilt;

TextProtocol::TextProtocol(::owlet::hal::IByteSink&     out,
                           IPanTilt&                    head,
                           ::owlet::core::ICalibration& calibration,
                           const ::owlet::hal::IClock&  clock,
                           std::string_view             firmwareVersion,
                           const ::owlet::core::IDiagnostics* diagnostics,
                           const ::owlet::core::EventBus*     events,
                           ::owlet::core::TroubleLog*         trouble,
                           const ::owlet::core::LoopStats*    loop)
    : m_out(out),
      m_head(head),
      m_calibration(calibration),
      m_clock(clock),
      m_firmwareVersion(firmwareVersion),
      m_diagnostics(diagnostics),
      m_events(events),
      m_trouble(trouble),
      m_loop(loop)
{
}

// ── IProtocol ───────────────────────────────────────────────────────────────

bool TextProtocol::claims(uint8_t byte, bool /*atLineStart*/) const
{
    // '$' starts a frame anywhere, so a garbled line resyncs on the next one.
    return byte == '$';
}

ConsumeResult TextProtocol::consume(const uint8_t* data, std::size_t len)
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

void TextProtocol::begin(uint32_t nowMs)
{
    m_nowMs = nowMs;
    sendVersioned("READY");
}

void TextProtocol::onEvent(const ::owlet::core::Event& event)
{
    using ::owlet::core::EventId;
    switch (event.id)
    {
    case EventId::BusFault:
        sendErr("BUS");
        break;
    case EventId::WatchdogExpired:
        m_writer.begin("TIMEOUT");
        m_writer.field("VEL");
        flush();
        break;
    case EventId::Arrived:
        // Only the link that asked for the move is told it finished.
        if (m_awaitingArrival)
        {
            m_awaitingArrival      = false;
            const IPanTilt::Pose p = m_head.position();
            m_writer.begin("ARRIVED");
            m_writer.fixed1(p.pan);
            m_writer.fixed1(p.tilt);
            flush();
        }
        break;
    case EventId::BusOk:
    case EventId::LimitHit:
    case EventId::Homed:
    case EventId::HomingFailed:
    case EventId::PositionLost:
    case EventId::Stalled:
    case EventId::DriverFaultChanged:
    case EventId::DriverOnline:
    case EventId::DriverOffline:
    case EventId::EventsLost:
        if (m_eventLines)
        {
            sendEvent(event);
        }
        break;
    }
}

void TextProtocol::executeDtc(const Request& req)
{
    // $DTC          → $DTC,<count>
    // $DTC,<index>  → $DTC,<index>,<code>,<PAN|TILT|->,<times>,<detail>,<first_ms>,<last_ms>
    // $DTC,CLEAR    → $ACK,DTC
    const std::size_t count = (m_trouble != nullptr) ? m_trouble->size() : 0;
    if (req.token == "CLEAR")
    {
        if (m_trouble != nullptr)
        {
            m_trouble->clear();
        }
        sendAck(req.command);
        return;
    }
    if (req.argCount == 0)
    {
        m_writer.begin("DTC");
        m_writer.uint(static_cast<uint32_t>(count));
        flush();
        return;
    }
    const std::size_t index = static_cast<std::size_t>(req.args[0]);
    if (index >= count)
    {
        sendNak(req.command, "RNG");
        return;
    }
    const ::owlet::core::TroubleRecord& r = m_trouble->at(index);
    m_writer.begin("DTC");
    m_writer.uint(static_cast<uint32_t>(index));
    m_writer.uint(static_cast<uint32_t>(r.code));
    m_writer.field(r.axis == 0 ? "PAN" : (r.axis == 1 ? "TILT" : "-"));
    m_writer.uint(r.count);
    m_writer.uint(r.detail);
    m_writer.uint(r.firstMs);
    m_writer.uint(r.lastMs);
    flush();
}

void TextProtocol::sendEvent(const ::owlet::core::Event& event)
{
    using ::owlet::core::EventId;
    // $EVT,<name>,<PAN|TILT|->,<arg>
    std::string_view name = "?";
    switch (event.id)
    {
    case EventId::BusOk:              name = "BUSOK";    break;
    case EventId::LimitHit:           name = "LIMIT";    break;
    case EventId::Homed:              name = "HOMED";    break;
    case EventId::HomingFailed:       name = "HOMEFAIL"; break;
    case EventId::PositionLost:       name = "LOST";     break;
    case EventId::Stalled:            name = "STALL";    break;
    case EventId::DriverFaultChanged: name = "FAULT";    break;
    case EventId::DriverOnline:       name = "ONLINE";   break;
    case EventId::DriverOffline:      name = "OFFLINE";  break;
    case EventId::EventsLost:         name = "OVERFLOW"; break;
    case EventId::Arrived:
    case EventId::WatchdogExpired:
    case EventId::BusFault:
        return;  // these have lines of their own
    }
    m_writer.begin("EVT");
    m_writer.field(name);
    m_writer.field(event.axis == 0 ? "PAN" : (event.axis == 1 ? "TILT" : "-"));
    if (event.id == EventId::LimitHit)
    {
        m_writer.field(event.arg == static_cast<uint16_t>(::owlet::core::LimitEnd::Min) ? "MIN" : "MAX");
    }
    else
    {
        m_writer.uint(event.arg);
    }
    flush();
}

void TextProtocol::onTick(uint32_t nowMs)
{
    m_nowMs = nowMs;

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

void TextProtocol::handleFrame(std::string_view frame)
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

void TextProtocol::execute(const Request& req)
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
        const Result r = req.argCount == 2 ? m_head.enableAt(req.args[0], req.args[1]) : m_head.enable();
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
        sendResult(req.command, m_head.move(req.args[0], req.args[1]));
        return;
    case Command::Nudge:
        sendResult(req.command, m_head.nudge(req.args[0], req.args[1]));
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
        sendResult(req.command, m_head.setSpeed(req.args[0]));
        return;
    case Command::Pos:
        sendPos();
        return;
    case Command::Stream:
    {
        const uint32_t hz = static_cast<uint32_t>(req.args[0]);
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
        const Result r = m_head.setVelocity(req.args[0], req.args[1]);
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
        sendResult(req.command, m_head.setWatchdog(static_cast<uint32_t>(req.args[0])));
        return;
    case Command::Clock:
        // The live clock, not the tick time: this is the sample a host uses
        // to estimate the clock offset, so it must be as fresh as possible.
        m_writer.begin("CLOCK");
        m_writer.uint(m_clock.nowMs());
        flush();
        return;
    case Command::Caps:
    {
        const IPanTilt::Capabilities c = m_head.capabilities();
        m_writer.begin("CAPS");
        m_writer.fixed1(c.speedMin);
        m_writer.fixed1(c.speedMax);
        m_writer.fixed1(c.velocityMax);
        m_writer.uint(c.watchdogMinMs);
        m_writer.uint(c.watchdogMaxMs);
        m_writer.uint(kMaxStreamHz);
        flush();
        return;
    }
    case Command::Cal:
        executeCal(req);
        return;
    case Command::Diag:
        sendDiag(req.token == "PAN" ? ::owlet::core::Axis::Pan : ::owlet::core::Axis::Tilt,
                 req.token);
        return;
    case Command::Events:
        if (req.args[0] > 1.0f)
        {
            sendNak(req.command, "RNG");
            return;
        }
        m_eventLines = req.args[0] != 0.0f;
        sendAck(req.command);
        return;
    case Command::Dtc:
        executeDtc(req);
        return;
    case Command::Seek:
        sendResult(req.command, m_head.seekHome());
        return;
    case Command::Stat:
    {
        // $STAT,<events posted>,<events dropped>,<most ever waiting>,<late passes>,<longest pass ms>
        const ::owlet::core::EventBus::Stats stats =
            (m_events != nullptr) ? m_events->stats() : ::owlet::core::EventBus::Stats{};
        m_writer.begin("STAT");
        m_writer.uint(stats.posted);
        m_writer.uint(stats.dropped);
        m_writer.uint(stats.highWater);
        m_writer.uint((m_loop != nullptr) ? m_loop->lateTicks : 0U);
        m_writer.uint((m_loop != nullptr) ? m_loop->worstTickMs : 0U);
        flush();
        return;
    }
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

// ── Calibration (v3) ────────────────────────────────────────────────────────

void TextProtocol::executeCal(const Request& req)
{
    using ::owlet::core::Axis;
    using CalResult = ::owlet::core::ICalibration::Result;

    CalResult r = CalResult::Ok;
    if (req.token == "SAVE")
    {
        // Writing to flash stops everything for tens of milliseconds, and
        // a moving stepper would lose steps over it.
        if (m_head.state() == IPanTilt::State::Moving)
        {
            sendNak(req.command, "MOV");
            return;
        }
        r = m_calibration.save();
    }
    else if (req.token == "RESET")
    {
        r = m_calibration.reset();
    }
    else
    {
        const Axis axis = (req.token == "PAN") ? Axis::Pan : Axis::Tilt;
        if (req.argCount == 0)
        {
            sendCal(axis, req.token);
            return;
        }
        // Range-check before narrowing: 66036 would wrap to a valid 500.
        for (std::size_t i = 0; i < 3; ++i)
        {
            if (req.args[i] > 65535.0f)
            {
                sendNak(Command::Cal, "RNG");
                return;
            }
        }
        ::owlet::hal::AxisCalibration c;
        c.driveMin    = static_cast<uint16_t>(req.args[0]);
        c.driveCenter = static_cast<uint16_t>(req.args[1]);
        c.driveMax    = static_cast<uint16_t>(req.args[2]);
        c.minDeg      = req.args[3];
        c.maxDeg      = req.args[4];
        r             = m_calibration.set(axis, c);
    }

    switch (r)
    {
    case CalResult::Ok:
        sendAck(Command::Cal);
        return;
    case CalResult::Range:
        sendNak(Command::Cal, "RNG");
        return;
    case CalResult::Enabled:
        sendNak(Command::Cal, "ENA");
        return;
    case CalResult::Storage:
        sendNak(Command::Cal, "STO");
        return;
    }
}

void TextProtocol::sendCal(::owlet::core::Axis axis, std::string_view name)
{
    using Source = ::owlet::core::ICalibration::Source;
    static constexpr std::array<std::string_view, 3> kSourceCodes{"F", "S", "M"};
    static_assert(static_cast<std::size_t>(Source::Factory) == 0 &&
                      static_cast<std::size_t>(Source::Saved) == 1 &&
                      static_cast<std::size_t>(Source::Modified) == 2,
                  "kSourceCodes must follow ICalibration::Source");

    const ::owlet::hal::AxisCalibration c = m_calibration.get(axis);
    m_writer.begin("CAL");
    m_writer.field(name);
    m_writer.uint(c.driveMin);
    m_writer.uint(c.driveCenter);
    m_writer.uint(c.driveMax);
    m_writer.fixed1(c.minDeg);
    m_writer.fixed1(c.maxDeg);
    m_writer.field(c.driveMin > c.driveMax ? "R" : "N");
    m_writer.field(kSourceCodes[static_cast<std::size_t>(m_calibration.source())]);
    flush();
}

void TextProtocol::sendDiag(::owlet::core::Axis axis, std::string_view name)
{
    // $DIAG,<AXIS>,<U|N>,<faults>,<stalled>,<load>,<current_mA>
    // U = the driver is answering and the numbers are live; N = nothing to
    // report (no data link to the driver, or it is not answering).
    const ::owlet::hal::AxisDiagnostics d =
        (m_diagnostics != nullptr) ? m_diagnostics->read(axis) : ::owlet::hal::AxisDiagnostics{};
    m_writer.begin("DIAG");
    m_writer.field(name);
    m_writer.field(d.available ? "U" : "N");
    m_writer.uint(d.faults);
    m_writer.uint(d.stalled ? 1U : 0U);
    m_writer.uint(d.load);
    m_writer.uint(d.currentMa);
    flush();
}

// ── Replies ─────────────────────────────────────────────────────────────────

void TextProtocol::sendResult(Command cmd, Result result)
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
    case Result::Untrusted:
        sendNak(cmd, "LOST");
        return;
    case Result::Busy:
        sendNak(cmd, "BSY");
        return;
    case Result::Unsupported:
        sendNak(cmd, "UNS");
        return;
    }
}

void TextProtocol::sendPos()
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

void TextProtocol::sendAck(Command cmd)
{
    m_writer.begin("ACK");
    m_writer.field(commandName(cmd));
    flush();
}

void TextProtocol::sendNak(Command cmd, std::string_view code)
{
    m_writer.begin("NAK");
    m_writer.field(commandName(cmd));
    m_writer.field(code);
    flush();
}

void TextProtocol::sendErr(std::string_view code)
{
    m_writer.begin("ERR");
    m_writer.field(code);
    flush();
}

void TextProtocol::sendVersioned(std::string_view name)
{
    m_writer.begin(name);
    m_writer.uint(kProtocolVersion);
    m_writer.field(m_firmwareVersion);
    flush();
}

void TextProtocol::flush()
{
    const std::string_view line = m_writer.finish();
    // A refused write means no host is listening; dropping the line is the
    // correct behaviour for a stateless line protocol.
    (void) m_out.write(line.data(), line.size());
}

} // namespace owlet::text
