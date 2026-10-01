#pragma once

#include "owlet/core/PanTilt.hpp"
#include "owlet/hal/ISerialPort.hpp"
#include "owlet/protocol/LineFramer.hpp"
#include "owlet/protocol/LineWriter.hpp"
#include "owlet/protocol/Request.hpp"

#include <cstdint>
#include <string_view>

namespace owlet::core
{

// The whole device behaviour behind the wire protocol (docs/DESIGN.md §5).
// Platform code only has to call begin() once and service() periodically.
class CommandProcessor
{
public:
    // Upper bound on the slew step taken per service() call, so a starved
    // loop can't make the head lurch when it resumes.
    static constexpr uint32_t kMaxTickMs       = 100;
    static constexpr uint32_t kMaxStreamHz     = 50;
    static constexpr std::size_t kReadChunk    = 64;

    CommandProcessor(::owlet::hal::ISerialPort& port,
                     PanTilt&                   head,
                     std::string_view           firmwareVersion);

    CommandProcessor(const CommandProcessor&)            = delete;
    CommandProcessor& operator=(const CommandProcessor&) = delete;

    // Announce $READY. Call once the hardware is initialised.
    void begin(uint32_t nowMs);

    // One loop iteration: advance motion, emit events, then answer input.
    void service(uint32_t nowMs);

private:
    void tickMotion(uint32_t nowMs);
    void pumpInput();
    void handleFrame(std::string_view frame);
    void execute(const ::owlet::protocol::Request& req);

    void sendPos();
    void sendAck(::owlet::protocol::Command cmd);
    void sendNak(::owlet::protocol::Command cmd, std::string_view code);
    void sendErr(std::string_view code);
    void sendResult(::owlet::protocol::Command cmd, PanTilt::Result result);
    void sendVersioned(std::string_view name);
    void flush();

    ::owlet::hal::ISerialPort&   m_port;
    PanTilt&                     m_head;
    std::string_view             m_firmwareVersion;
    ::owlet::protocol::LineFramer m_framer;
    ::owlet::protocol::LineWriter m_writer;

    uint32_t m_lastTickMs{0};
    uint32_t m_streamPeriodMs{0};   // 0 = streaming off
    uint32_t m_nextStreamMs{0};
    bool     m_awaitingArrival{false};
    bool     m_busFault{false};
};

} // namespace owlet::core
