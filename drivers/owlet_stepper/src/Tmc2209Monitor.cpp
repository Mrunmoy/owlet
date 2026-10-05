#include "owlet/stepper/Tmc2209Monitor.hpp"

namespace owlet
{

using ::owlet::hal::AxisDiagnostics;

Tmc2209Monitor::Tmc2209Monitor(Tmc2209& driver,
                               uint32_t phaseMs,
                               ::owlet::hal::IAxisDiagnosticsListener* listener)
    // Start one probe period "in", less the phase, so the first probe comes
    // straight away (or `phaseMs` later).
    : m_driver(driver), m_listener(listener), m_elapsedMs(phaseMs < kProbePeriodMs ? kProbePeriodMs - phaseMs : 0)
{
}

void Tmc2209Monitor::poll(uint32_t dtMs)
{
    // Saturating: a long stall must not wrap the timer back to "just ran".
    m_elapsedMs = (dtMs > kProbePeriodMs || m_elapsedMs > kProbePeriodMs) ? kProbePeriodMs + 1
                                                                          : m_elapsedMs + dtMs;
    if (!m_online)
    {
        if (m_elapsedMs >= kProbePeriodMs)
        {
            m_elapsedMs = 0;
            probe();
            notifyIfChanged();
        }
        return;
    }
    if (m_elapsedMs >= kPollPeriodMs)
    {
        m_elapsedMs = 0;
        readNext();
        notifyIfChanged();
    }
}

void Tmc2209Monitor::notifyIfChanged()
{
    if (m_listener == nullptr)
    {
        return;
    }
    const AxisDiagnostics now = diagnostics();
    if (now.available != m_told.available || now.faults != m_told.faults ||
        now.stalled != m_told.stalled)
    {
        m_told = now;
        m_listener->onDiagnosticsChanged(now);
    }
}

void Tmc2209Monitor::probe()
{
    if (m_driver.probe() && m_driver.configure())
    {
        m_online = true;
        m_failures = 0;
        m_step = Step::Status;
        m_status = Tmc2209::Status{};
        m_load = 0;
        m_loadKnown = false;
    }
}

void Tmc2209Monitor::readNext()
{
    bool ok = false;
    switch (m_step)
    {
    case Step::Status:
        ok = m_driver.readStatus(m_status);
        m_step = Step::Load;
        break;
    case Step::Load:
        ok = m_driver.readLoad(m_load);
        m_loadKnown = m_loadKnown || ok;
        m_step = Step::ResetFlag;
        break;
    case Step::ResetFlag:
    {
        bool wasReset = false;
        ok = m_driver.readResetFlag(wasReset);
        // Back on its power-up defaults: put our settings back.
        if (ok && wasReset)
        {
            ok = m_driver.configure();
        }
        m_step = Step::Status;
        break;
    }
    }
    if (ok)
    {
        m_failures = 0;
    }
    else
    {
        noteFailure();
    }
}

void Tmc2209Monitor::noteFailure()
{
    if (++m_failures >= kMaxFailures)
    {
        m_online = false;
        m_failures = 0;
        m_elapsedMs = 0; // a full probe period before trying again
    }
}

AxisDiagnostics Tmc2209Monitor::diagnostics() const
{
    AxisDiagnostics d;
    if (!m_online)
    {
        return d;
    }
    d.available = true;
    d.faults =
        static_cast<uint16_t>((m_status.overTempWarning ? AxisDiagnostics::kOverTempWarning : 0) |
                              (m_status.overTemp ? AxisDiagnostics::kOverTemp : 0) |
                              (m_status.shortA ? AxisDiagnostics::kShortA : 0) |
                              (m_status.shortB ? AxisDiagnostics::kShortB : 0) |
                              (m_status.openA ? AxisDiagnostics::kOpenA : 0) |
                              (m_status.openB ? AxisDiagnostics::kOpenB : 0));
    d.load = m_load;
    d.currentMa = m_driver.runCurrentMa();
    // The load value is only a measurement while the motor is turning.
    const uint16_t threshold = static_cast<uint16_t>(2U * m_driver.stallThreshold());
    // Nor is it one before it has been read: zero would look like a stall.
    d.stalled = threshold != 0 && m_loadKnown && !m_status.standstill && m_load <= threshold;
    return d;
}

} // namespace owlet
