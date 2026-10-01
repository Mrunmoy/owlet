#pragma once

#include "owlet/hal/ISerialPort.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace owlet::test
{

// In-memory serial port. Tests push request bytes into `rx` and pop the
// device's reply lines (CRLF stripped) from `tx`.
class FakeSerialPort final : public ::owlet::hal::ISerialPort
{
public:
    [[nodiscard]] std::size_t read(uint8_t* out, std::size_t maxLen) override
    {
        const std::size_t n = std::min(maxLen, rx.size() - m_rxPos);
        for (std::size_t i = 0; i < n; ++i)
        {
            out[i] = static_cast<uint8_t>(rx[m_rxPos + i]);
        }
        m_rxPos += n;
        return n;
    }

    [[nodiscard]] bool write(const char* data, std::size_t len) override
    {
        if (failWrites)
        {
            return false;
        }
        tx.append(data, len);
        return true;
    }

    // All complete lines written so far, CRLF stripped; clears the buffer.
    std::vector<std::string> takeLines()
    {
        std::vector<std::string> lines;
        std::size_t              start = 0;
        for (std::size_t nl = tx.find("\r\n"); nl != std::string::npos;
             nl             = tx.find("\r\n", start))
        {
            lines.push_back(tx.substr(start, nl - start));
            start = nl + 2;
        }
        tx.erase(0, start);
        return lines;
    }

    std::string rx;
    std::string tx;
    bool        failWrites{false};

private:
    std::size_t m_rxPos{0};
};

} // namespace owlet::test
