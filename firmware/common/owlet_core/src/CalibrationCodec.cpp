#include "owlet/core/CalibrationCodec.hpp"

#include <cstring>

namespace owlet::core
{

namespace
{

constexpr std::array<uint32_t, 256> makeCrcTable()
{
    std::array<uint32_t, 256> table{};
    for (uint32_t i = 0; i < 256; ++i)
    {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k)
        {
            c = (c & 1U) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        table[i] = c;
    }
    return table;
}

constexpr std::array<uint32_t, 256> kCrcTable = makeCrcTable();

void put16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put32(uint8_t* p, uint32_t v)
{
    for (int i = 0; i < 4; ++i)
    {
        p[i] = static_cast<uint8_t>(v >> (8 * i));
    }
}

uint16_t get16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t get32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void putFloat(uint8_t* p, float f)
{
    static_assert(sizeof(float) == 4, "IEEE-754 binary32 expected");
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    put32(p, bits);
}

float getFloat(const uint8_t* p)
{
    const uint32_t bits = get32(p);
    float          f    = 0.0f;
    std::memcpy(&f, &bits, sizeof f);
    return f;
}

void putAxis(uint8_t* p, const ::owlet::hal::AxisCalibration& c)
{
    put16(p + 0, c.driveMin);
    put16(p + 2, c.driveCenter);
    put16(p + 4, c.driveMax);
    put16(p + 6, 0);
    putFloat(p + 8, c.minDeg);
    putFloat(p + 12, c.maxDeg);
}

::owlet::hal::AxisCalibration getAxis(const uint8_t* p)
{
    ::owlet::hal::AxisCalibration c;
    c.driveMin    = get16(p + 0);
    c.driveCenter = get16(p + 2);
    c.driveMax    = get16(p + 4);
    c.minDeg      = getFloat(p + 8);
    c.maxDeg      = getFloat(p + 12);
    return c;
}

constexpr std::size_t kCrcOffset = CalibrationCodec::kEncodedSize - 4;

} // namespace

uint32_t CalibrationCodec::crc32(const uint8_t* data, std::size_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i)
    {
        c = kCrcTable[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

CalibrationCodec::Bytes CalibrationCodec::encode(const CalibrationRecord& record)
{
    Bytes b{};
    put32(b.data() + 0, kMagic);
    put16(b.data() + 4, kVersion);
    put16(b.data() + 6, 0);
    putAxis(b.data() + 8, record.pan);
    putAxis(b.data() + 24, record.tilt);
    put32(b.data() + kCrcOffset, crc32(b.data(), kCrcOffset));
    return b;
}

bool CalibrationCodec::decode(const uint8_t* data, std::size_t len, CalibrationRecord& out)
{
    if (data == nullptr || len != kEncodedSize)
    {
        return false;
    }
    if (get32(data + kCrcOffset) != crc32(data, kCrcOffset) || get32(data) != kMagic ||
        get16(data + 4) != kVersion)
    {
        return false;
    }
    out.pan  = getAxis(data + 8);
    out.tilt = getAxis(data + 24);
    return true;
}

} // namespace owlet::core
