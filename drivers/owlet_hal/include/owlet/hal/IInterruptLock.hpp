#pragma once

#include <cstdint>

namespace owlet::hal
{

/// Keeps interrupts out of a few instructions that share data with them.
///
/// The saved state is handed back to unlock(), so locks nest correctly and a
/// lock taken inside an interrupt handler leaves interrupts as it found them.
class IInterruptLock
{
public:
    virtual ~IInterruptLock() = default;

    /// Masks interrupts.
    /// @return The previous state, to be passed to unlock().
    [[nodiscard]] virtual uint32_t lock() = 0;

    /// Restores the state returned by the matching lock().
    virtual void unlock(uint32_t saved) = 0;
};

/// A lock for code that has no interrupts to keep out: host tests, and
/// platforms where everything that touches the data runs in one context.
class NullInterruptLock final : public IInterruptLock
{
public:
    [[nodiscard]] uint32_t lock() override
    {
        return 0;
    }
    void unlock(uint32_t) override {}
};

/// Holds an IInterruptLock for the length of a scope.
class InterruptGuard
{
public:
    explicit InterruptGuard(IInterruptLock& lock) : m_lock(lock), m_saved(lock.lock()) {}
    ~InterruptGuard()
    {
        m_lock.unlock(m_saved);
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    IInterruptLock& m_lock;
    uint32_t m_saved;
};

} // namespace owlet::hal
