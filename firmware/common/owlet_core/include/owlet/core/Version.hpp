#pragma once

#include <string_view>

namespace owlet::core
{

// Reported in $READY / $PONG. Bump on every tagged release.
inline constexpr std::string_view kFirmwareVersion = "0.1.0";

} // namespace owlet::core
