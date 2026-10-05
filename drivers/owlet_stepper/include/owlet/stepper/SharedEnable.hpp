#pragma once

#include "owlet/hal/IDigitalOutput.hpp"

namespace owlet
{

// One enable line wired to several motor drivers. Each axis gets its own
// User; the line is asserted while any of them wants it, so an axis can't
// cut power from under another that is still moving.
class SharedEnable
{
public:
    class User final : public ::owlet::hal::IDigitalOutput
    {
    public:
        explicit User(SharedEnable& shared) : m_shared(shared) {}

        User(const User&) = delete;
        User& operator=(const User&) = delete;

        void set(bool on) override
        {
            if (on == m_on)
            {
                return;
            }
            m_on = on;
            m_shared.change(on ? 1 : -1);
        }

    private:
        SharedEnable& m_shared;
        bool m_on{false};
    };

    explicit SharedEnable(::owlet::hal::IDigitalOutput& line) : m_line(line) {}

    SharedEnable(const SharedEnable&) = delete;
    SharedEnable& operator=(const SharedEnable&) = delete;

private:
    void change(int delta)
    {
        m_users += delta;
        m_line.set(m_users > 0);
    }

    ::owlet::hal::IDigitalOutput& m_line;
    int m_users{0};
};

} // namespace owlet
