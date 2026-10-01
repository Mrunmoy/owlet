#pragma once

#include "owlet/core/IPanTilt.hpp"
#include "owlet/core/IProtocol.hpp"
#include "owlet/hal/IByteSink.hpp"
#include "owlet/hal/IClock.hpp"
#include "owlet/nmea/LineFramer.hpp"
#include "owlet/nmea/LineWriter.hpp"
#include "owlet/nmea/Request.hpp"

#include <cstdint>
#include <string_view>

namespace owlet::nmea
{

// owlet's native "$BODY*HH" protocol (docs/DESIGN.md §5) as an IProtocol.
// Depends only on IPanTilt, IByteSink and IClock, so it runs on any head and
// any transport.
class NmeaProtocol final : public ::owlet::core::IProtocol
{
public:
    static constexpr uint32_t kMaxStreamHz = 50;

    // `clock` stamps $CLOCK replies; $POS is stamped with the tick time of
    // the pose it reports.
    NmeaProtocol(::owlet::hal::IByteSink&    out,
                 ::owlet::core::IPanTilt&    head,
                 const ::owlet::hal::IClock& clock,
                 std::string_view            firmwareVersion);

    NmeaProtocol(const NmeaProtocol&)            = delete;
    NmeaProtocol& operator=(const NmeaProtocol&) = delete;

    // ── IProtocol ───────────────────────────────────────────────────────────

    [[nodiscard]] bool claims(uint8_t byte, bool atLineStart) const override;
    [[nodiscard]] ::owlet::core::ConsumeResult consume(const uint8_t* data,
                                                       std::size_t    len) override;
    void begin(uint32_t nowMs) override;
    void onTick(uint32_t nowMs, const ::owlet::core::TickStatus& status) override;

private:
    using Result = ::owlet::core::IPanTilt::Result;

    void handleFrame(std::string_view frame);
    void execute(const Request& req);

    void sendPos();
    void sendAck(Command cmd);
    void sendNak(Command cmd, std::string_view code);
    void sendErr(std::string_view code);
    void sendResult(Command cmd, Result result);
    void sendVersioned(std::string_view name);
    void flush();

    ::owlet::hal::IByteSink& m_out;
    ::owlet::core::IPanTilt& m_head;
    const ::owlet::hal::IClock& m_clock;
    std::string_view         m_firmwareVersion;
    LineFramer               m_framer;
    LineWriter               m_writer;

    uint32_t m_nowMs{0};  // time of the last tick: the pose's timestamp
    uint32_t m_streamPeriodMs{0};  // 0 = streaming off
    uint32_t m_nextStreamMs{0};
    bool     m_awaitingArrival{false};
    bool     m_busFault{false};
};

} // namespace owlet::nmea
