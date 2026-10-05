#pragma once

#include "owlet/hal/IByteSink.hpp"
#include "owlet/hal/IByteSource.hpp"

namespace owlet::hal
{

/// A bidirectional byte stream. On ESP32-S3 this is the USB-Serial-JTAG CDC
/// port; on the host simulator a pseudo-terminal; in tests an in-memory fake.
class ISerialPort : public IByteSource, public IByteSink
{
};

} // namespace owlet::hal
