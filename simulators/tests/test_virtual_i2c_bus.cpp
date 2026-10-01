// Tests for the host-side VirtualI2CBus: address routing, NACK propagation,
// transaction counters, and multi-device dispatch.

#include "owlet/sim/II2CDevice.hpp"
#include "owlet/sim/VirtualI2CBus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>

using owlet::sim::II2CDevice;
using owlet::sim::VirtualI2CBus;

namespace
{

// Minimal II2CDevice that records the last transaction for assertions.
class RecordingDevice : public II2CDevice
{
public:
    explicit RecordingDevice(uint8_t addr) : m_addr(addr) {}

    uint8_t address() const override { return m_addr; }

    bool onWrite(uint8_t reg, const uint8_t* data, std::size_t len) override
    {
        m_lastWriteReg = reg;
        m_lastWriteLen = len;
        for (std::size_t i = 0; i < len && i < m_scratch.size(); ++i)
        {
            m_scratch[i] = data[i];
        }
        return m_ackWrite;
    }

    bool onRead(uint8_t reg, uint8_t* out, std::size_t len) override
    {
        m_lastReadReg = reg;
        m_lastReadLen = len;
        // Synthesize deterministic bytes so reads can be verified.
        for (std::size_t i = 0; i < len; ++i)
        {
            out[i] = static_cast<uint8_t>(reg + i);
        }
        return m_ackRead;
    }

    bool onProbe() override { return m_ackProbe; }

    uint8_t m_addr;
    bool m_ackWrite{true};
    bool m_ackRead{true};
    bool m_ackProbe{true};

    uint8_t m_lastWriteReg{0};
    std::size_t m_lastWriteLen{0};
    uint8_t m_lastReadReg{0};
    std::size_t m_lastReadLen{0};
    std::array<uint8_t, 64> m_scratch{};
};

} // namespace

TEST(VirtualI2CBus, WriteRoutesByAddress)
{
    RecordingDevice dev(0x40);
    VirtualI2CBus bus;
    bus.attach(dev);

    const uint8_t payload[3] = {0xAA, 0xBB, 0xCC};
    EXPECT_TRUE(bus.writeRegister(0x40, 0x06, payload, 3));
    EXPECT_EQ(dev.m_lastWriteReg, 0x06);
    EXPECT_EQ(dev.m_lastWriteLen, 3u);
    EXPECT_EQ(dev.m_scratch[0], 0xAA);
    EXPECT_EQ(dev.m_scratch[2], 0xCC);
}

TEST(VirtualI2CBus, ReadRoutesByAddress)
{
    RecordingDevice dev(0x40);
    VirtualI2CBus bus;
    bus.attach(dev);

    uint8_t buf[4] = {};
    EXPECT_TRUE(bus.readRegister(0x40, 0x10, buf, 4));
    EXPECT_EQ(buf[0], 0x10);
    EXPECT_EQ(buf[1], 0x11);
    EXPECT_EQ(buf[2], 0x12);
    EXPECT_EQ(buf[3], 0x13);
    EXPECT_EQ(dev.m_lastReadLen, 4u);
}

TEST(VirtualI2CBus, NackOnUnknownAddress)
{
    RecordingDevice dev(0x40);
    VirtualI2CBus bus;
    bus.attach(dev);

    const uint8_t val = 0xAA;
    uint8_t out = 0;
    EXPECT_FALSE(bus.writeRegister(0x50, 0x00, &val, 1));
    EXPECT_FALSE(bus.readRegister(0x50, 0x00, &out, 1));
    EXPECT_FALSE(bus.probe(0x50));
}

TEST(VirtualI2CBus, MultipleDevicesRoutedIndependently)
{
    RecordingDevice dev1(0x40);
    RecordingDevice dev2(0x41);
    VirtualI2CBus bus;
    bus.attach(dev1);
    bus.attach(dev2);

    const uint8_t a = 0xAA;
    const uint8_t b = 0xBB;
    EXPECT_TRUE(bus.writeRegister(0x40, 0x01, &a, 1));
    EXPECT_TRUE(bus.writeRegister(0x41, 0x02, &b, 1));
    EXPECT_EQ(dev1.m_scratch[0], 0xAA);
    EXPECT_EQ(dev2.m_scratch[0], 0xBB);
    EXPECT_EQ(dev1.m_lastWriteReg, 0x01);
    EXPECT_EQ(dev2.m_lastWriteReg, 0x02);
}

TEST(VirtualI2CBus, PropagatesDeviceNack)
{
    RecordingDevice dev(0x40);
    dev.m_ackWrite = false;
    VirtualI2CBus bus;
    bus.attach(dev);

    const uint8_t val = 0x00;
    EXPECT_FALSE(bus.writeRegister(0x40, 0x00, &val, 1));
}

TEST(VirtualI2CBus, ProbeUsesDeviceAck)
{
    RecordingDevice present(0x40);
    RecordingDevice rude(0x41);
    rude.m_ackProbe = false;

    VirtualI2CBus bus;
    bus.attach(present);
    bus.attach(rude);

    EXPECT_TRUE(bus.probe(0x40));
    EXPECT_FALSE(bus.probe(0x41));
}

TEST(VirtualI2CBus, CountersTrackTransactions)
{
    RecordingDevice dev(0x40);
    VirtualI2CBus bus;
    bus.attach(dev);

    const uint8_t val = 0xAA;
    uint8_t out = 0;
    EXPECT_TRUE(bus.writeRegister(0x40, 0x00, &val, 1));
    EXPECT_TRUE(bus.writeRegister(0x40, 0x00, &val, 1));
    EXPECT_TRUE(bus.readRegister(0x40, 0x00, &out, 1));
    EXPECT_TRUE(bus.probe(0x40));

    EXPECT_EQ(bus.writeCount(), 2u);
    EXPECT_EQ(bus.readCount(), 1u);
    EXPECT_EQ(bus.probeCount(), 1u);
}
