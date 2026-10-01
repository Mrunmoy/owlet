#pragma once

#include "owlet/core/ICalibrationStore.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::core
{

// Fixed, versioned, CRC-protected byte layout for CalibrationRecord, so the
// persisted format is host-tested and platform stores only move bytes.
//
//   0  u32 magic 'OWLC'   4  u16 version   6  u16 reserved (0)
//   8  pan  : u16 min_us, u16 center_us, u16 max_us, u16 pad, f32 min_deg, f32 max_deg
//   24 tilt : same
//   40 u32 CRC-32 (IEEE) of bytes 0..39
// All little-endian.
class CalibrationCodec
{
public:
    static constexpr uint32_t    kMagic       = 0x434C574Fu;  // "OWLC" little-endian
    static constexpr uint16_t    kVersion     = 1;
    static constexpr std::size_t kEncodedSize = 44;

    using Bytes = std::array<uint8_t, kEncodedSize>;

    static Bytes encode(const CalibrationRecord& record);
    [[nodiscard]] static bool decode(const uint8_t* data, std::size_t len, CalibrationRecord& out);

    static uint32_t crc32(const uint8_t* data, std::size_t len);
};

} // namespace owlet::core
