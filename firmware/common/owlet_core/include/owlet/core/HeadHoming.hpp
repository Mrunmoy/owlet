#pragma once

#include "owlet/core/AxisHoming.hpp"
#include "owlet/core/IHoming.hpp"

#include <cstdint>

namespace owlet::core
{

/// Homes a two-axis head one axis at a time: tilt first, then pan, so only
/// one motor is hunting for a switch at any moment and a failure names the
/// axis it happened on.
class HeadHoming final : public IHoming
{
public:
    HeadHoming(AxisHoming& pan, AxisHoming& tilt) : m_pan(pan), m_tilt(tilt) {}

    HeadHoming(const HeadHoming&) = delete;
    HeadHoming& operator=(const HeadHoming&) = delete;

    [[nodiscard]] bool start() override
    {
        if (busy() || !m_tilt.start())
        {
            return false;
        }
        m_current = &m_tilt;
        m_outcome = Outcome::None;
        return true;
    }

    void abort() override
    {
        if (m_current != nullptr)
        {
            m_current->abort();
            finishIfEnded();
        }
    }

    void update() override
    {
        if (m_current == nullptr)
        {
            return;
        }
        m_current->update();
        finishIfEnded();
    }

    [[nodiscard]] bool busy() const override
    {
        return m_current != nullptr;
    }
    [[nodiscard]] Outcome outcome() const override
    {
        return m_outcome;
    }
    [[nodiscard]] uint8_t failedAxis() const override
    {
        return m_failedAxis;
    }
    [[nodiscard]] uint16_t failureCode() const override
    {
        return m_failureCode;
    }

private:
    void finishIfEnded()
    {
        if (m_current->busy())
        {
            return;
        }
        if (m_current->state() == AxisHoming::State::Failed)
        {
            m_failedAxis = (m_current == &m_pan) ? uint8_t{0} : uint8_t{1};
            m_failureCode = static_cast<uint16_t>(m_current->failure());
            m_outcome = Outcome::Failed;
            m_current = nullptr;
            return;
        }
        if (m_current == &m_tilt && m_pan.start())
        {
            m_current = &m_pan;
            return;
        }
        m_outcome = Outcome::Done;
        m_current = nullptr;
    }

    AxisHoming& m_pan;
    AxisHoming& m_tilt;
    AxisHoming* m_current{nullptr};
    Outcome m_outcome{Outcome::None};
    uint16_t m_failureCode{0};
    uint8_t m_failedAxis{0};
};

} // namespace owlet::core
