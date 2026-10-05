#pragma once

#include "owlet/core/ICalibration.hpp"
#include "owlet/core/EventBus.hpp"
#include "owlet/core/IDiagnostics.hpp"
#include "owlet/core/IEventObserver.hpp"
#include "owlet/core/IPanTilt.hpp"
#include "owlet/core/IProtocol.hpp"
#include "owlet/core/LoopStats.hpp"
#include "owlet/core/TroubleLog.hpp"
#include "owlet/hal/IByteSink.hpp"
#include "owlet/hal/IClock.hpp"
#include "owlet/text/LineFramer.hpp"
#include "owlet/text/LineWriter.hpp"
#include "owlet/text/Request.hpp"

#include <cstdint>
#include <string_view>

namespace owlet::text
{

/// owlet's native "$BODY*HH" protocol (docs/DESIGN.md §5) as an IProtocol.
/// Depends only on IPanTilt, IByteSink and IClock, so it runs on any head and
/// any transport.
//
/// Unsolicited lines are written in answer to events, so the protocol must
/// be attached to the head's bus: $ARRIVED, $TIMEOUT and $ERR,BUS as always,
/// and $EVT,<name>,<axis>,<arg> for everything else unless $EVENTS,0 has
/// turned those off.
class TextProtocol final : public ::owlet::core::IProtocol, public ::owlet::core::IEventObserver
{
public:
    static constexpr uint32_t kMaxStreamHz = 50;

    /// `clock` stamps $CLOCK replies; $POS is stamped with the tick time of
    /// the pose it reports. `diagnostics` is optional: a head without any
    /// answers $DIAG with "nothing available". `events` is optional too: it
    /// supplies the counters for $STAT. So is `trouble`: it is what $DTC
    /// reads and clears.
    TextProtocol(::owlet::hal::IByteSink&      out,
                 ::owlet::core::IPanTilt&      head,
                 ::owlet::core::ICalibration&  calibration,
                 const ::owlet::hal::IClock&   clock,
                 std::string_view              firmwareVersion,
                 const ::owlet::core::IDiagnostics* diagnostics = nullptr,
                 const ::owlet::core::EventBus*     events      = nullptr,
                 ::owlet::core::TroubleLog*         trouble     = nullptr,
                 const ::owlet::core::LoopStats*    loop        = nullptr);

    TextProtocol(const TextProtocol&)            = delete;
    TextProtocol& operator=(const TextProtocol&) = delete;

    // ── IProtocol ───────────────────────────────────────────────────────────

    [[nodiscard]] bool claims(uint8_t byte, bool atLineStart) const override;
    [[nodiscard]] ::owlet::core::ConsumeResult consume(const uint8_t* data,
                                                       std::size_t    len) override;
    void begin(uint32_t nowMs) override;
    void onTick(uint32_t nowMs) override;

    // ── IEventObserver ──────────────────────────────────────────────────────

    void onEvent(const ::owlet::core::Event& event) override;

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
    void executeCal(const Request& req);
    void sendCal(::owlet::core::Axis axis, std::string_view name);
    void sendDiag(::owlet::core::Axis axis, std::string_view name);
    void sendEvent(const ::owlet::core::Event& event);
    void executeDtc(const Request& req);
    void flush();

    ::owlet::hal::IByteSink& m_out;
    ::owlet::core::IPanTilt& m_head;
    ::owlet::core::ICalibration& m_calibration;
    const ::owlet::hal::IClock& m_clock;
    std::string_view         m_firmwareVersion;
    const ::owlet::core::IDiagnostics* m_diagnostics;
    const ::owlet::core::EventBus* m_events;
    ::owlet::core::TroubleLog* m_trouble;
    const ::owlet::core::LoopStats* m_loop;
    LineFramer               m_framer;
    LineWriter               m_writer;

    uint32_t m_nowMs{0};  ///< time of the last tick: the pose's timestamp
    uint32_t m_streamPeriodMs{0};  ///< 0 = streaming off
    uint32_t m_nextStreamMs{0};
    bool     m_eventLines{true};  ///< $EVT lines are wanted
    bool     m_awaitingArrival{false};  ///< a move asked for on this link has not been reported
};

} // namespace owlet::text
