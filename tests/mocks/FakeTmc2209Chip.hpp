#pragma once

#include "owlet/hal/IHalfDuplexLink.hpp"
#include "owlet/stepper/Tmc2209.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::test
{

// A TMC2209 on the far end of the wire: checks sync, address and CRC as the
// real chip does, keeps a register file, counts accepted writes in IFCNT and
// answers reads. Unpowered, it says nothing.
class FakeTmc2209Chip final : public ::owlet::hal::IHalfDuplexLink
{
public:
    FakeTmc2209Chip()
    {
        reset();
    }

    // What the chip looks like after its supply comes up.
    void reset()
    {
        reg.fill(0);
        reg[::owlet::Tmc2209::kIoin] = static_cast<uint32_t>(version) << 24;
        reg[::owlet::Tmc2209::kGconf] = 0x00000101u;    // I_scale_analog, multistep_filt
        reg[::owlet::Tmc2209::kChopconf] = 0x10010053u; // as read from a real chip
        reg[::owlet::Tmc2209::kGstat] = 0x00000001u;    // reset flag
        reg[::owlet::Tmc2209::kIfcnt] = 0;
    }

    [[nodiscard]] bool
    exchange(const uint8_t* tx, std::size_t txLen, uint8_t* rx, std::size_t rxLen) override
    {
        ++exchanges;
        if (!powered)
        {
            return rxLen == 0;
        }
        const bool isWrite = txLen == 8;
        const bool isRead = txLen == 4;
        if ((!isWrite && !isRead) || tx[0] != 0x05 || tx[1] != address ||
            ::owlet::Tmc2209::crc8(tx, txLen - 1) != tx[txLen - 1])
        {
            ++rejected;
            return rxLen == 0; // the chip ignores it; a waiting master times out
        }
        const uint8_t r = tx[2] & 0x7Fu;
        if (isWrite)
        {
            if ((tx[2] & 0x80u) == 0)
            {
                ++rejected;
                return rxLen == 0;
            }
            const uint32_t value = (static_cast<uint32_t>(tx[3]) << 24) |
                                   (static_cast<uint32_t>(tx[4]) << 16) |
                                   (static_cast<uint32_t>(tx[5]) << 8) | tx[6];
            if (r == ::owlet::Tmc2209::kGstat)
            {
                reg[r] &= ~value; // write 1 to clear
            }
            else if (r != ::owlet::Tmc2209::kIoin && r != ::owlet::Tmc2209::kIfcnt &&
                     r != ::owlet::Tmc2209::kDrvStatus && r != ::owlet::Tmc2209::kSgResult)
            {
                reg[r] = value;
            }
            reg[::owlet::Tmc2209::kIfcnt] = (reg[::owlet::Tmc2209::kIfcnt] + 1) & 0xFFu;
            ++writes;
            return rxLen == 0;
        }
        if (rxLen != 8)
        {
            return false;
        }
        const uint32_t v = reg[r];
        rx[0] = 0x05;
        rx[1] = 0xFF;
        rx[2] = r;
        rx[3] = static_cast<uint8_t>(v >> 24);
        rx[4] = static_cast<uint8_t>(v >> 16);
        rx[5] = static_cast<uint8_t>(v >> 8);
        rx[6] = static_cast<uint8_t>(v);
        rx[7] = ::owlet::Tmc2209::crc8(rx, 7);
        if (corruptReplies)
        {
            rx[7] = static_cast<uint8_t>(rx[7] ^ 0x5Au);
        }
        return true;
    }

    std::array<uint32_t, 128> reg{};
    uint8_t address{0};
    uint8_t version{0x21};
    bool powered{true};
    bool corruptReplies{false};
    int exchanges{0};
    int writes{0};
    int rejected{0};
};

} // namespace owlet::test
