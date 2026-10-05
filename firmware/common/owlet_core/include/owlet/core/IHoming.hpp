#pragma once

#include <cstdint>

namespace owlet::core
{

/// Finding the head's reference position by itself, as the motion policy
/// sees it. A head with no means of doing so simply has none of these.
class IHoming
{
public:
    enum class Outcome : uint8_t
    {
        None,   ///< Never run, or still running.
        Done,   ///< Every axis found its reference.
        Failed, ///< An axis gave up; see failedAxis() and failureCode().
    };

    virtual ~IHoming() = default;

    /// Begins homing every axis.
    /// @return false if it is already running.
    [[nodiscard]] virtual bool start() = 0;

    /// Stops homing where it is. The outcome becomes Failed.
    virtual void abort() = 0;

    /// Advances homing by one control pass, after the axes have been ticked.
    virtual void update() = 0;

    [[nodiscard]] virtual bool busy() const = 0;
    [[nodiscard]] virtual Outcome outcome() const = 0;

    /// @return The axis that failed (0 = pan, 1 = tilt); meaningful after Failed.
    [[nodiscard]] virtual uint8_t failedAxis() const = 0;

    /// @return Why it failed, as a number for reports; meaningful after Failed.
    [[nodiscard]] virtual uint16_t failureCode() const = 0;
};

} // namespace owlet::core
