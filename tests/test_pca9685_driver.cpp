// PCA9685 driver tests — two suites.
//
//   PCA9685DriverSimTest exercises the driver end-to-end through
//   VirtualI2CBus + PCA9685Simulator and asserts register-level state
//   (datasheet-correct pulse encoding, SLEEP guard, full-OFF semantics).
//
//   PCA9685DriverMockTest uses MockI2CBus to assert the *exact* I²C
//   transactions the driver emits, including ordering.

#include "MockI2CBus.hpp"
#include "owlet/hal/IDelayProvider.hpp"
#include "owlet/pca9685/PCA9685.hpp"
#include "owlet/pca9685/Registers.hpp"
#include "owlet/sim/PCA9685Simulator.hpp"
#include "owlet/sim/VirtualI2CBus.hpp"

#include <cstdint>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace regs = ::owlet::pca9685;
using owlet::PCA9685;
using owlet::sim::PCA9685Simulator;
using owlet::sim::VirtualI2CBus;
using owlet::test::MockI2CBus;

namespace
{

// Host-side fake for IDelayProvider — records call count and cumulative µs.
class FakeDelay final : public ::owlet::hal::IDelayProvider
{
public:
    void delayUs(uint32_t us) override
    {
        m_totalUs += us;
        ++m_usCalls;
    }
    void delayMs(uint32_t ms) override
    {
        m_totalUs += ms * 1000U;
        ++m_msCalls;
    }

    uint32_t m_totalUs{0};
    std::size_t m_usCalls{0};
    std::size_t m_msCalls{0};
};

} // namespace

// ── End-to-end suite: driver against the simulator ──────────────────────────

class PCA9685DriverSimTest : public ::testing::Test
{
protected:
    PCA9685Simulator sim;
    VirtualI2CBus    bus;
    FakeDelay        delay;

    void SetUp() override { bus.attach(sim); }
};

TEST_F(PCA9685DriverSimTest, InitProbesAndBringsUpAt50Hz)
{
    PCA9685 drv(bus, delay);
    EXPECT_TRUE(drv.init());

    EXPECT_FALSE(sim.isSleeping());
    EXPECT_TRUE(sim.autoIncrement());
    EXPECT_NEAR(sim.frequencyHz(), 50.0f, 0.5f);
    EXPECT_EQ(drv.frequencyHz(), 50u);
    // Datasheet §7.3.1 [3]: oscillator needs ≥500 µs after wake.
    EXPECT_GE(delay.m_totalUs, 500u);
    // SLEEP guard was observed: no prescale writes dropped.
    EXPECT_EQ(sim.blockedPrescaleWrites(), 0u);
}

TEST_F(PCA9685DriverSimTest, InitFailsIfDeviceNotPresent)
{
    PCA9685::Config cfg;
    cfg.address = 0x50;  // nothing attached at this address
    PCA9685 drv(bus, delay, cfg);
    EXPECT_FALSE(drv.init());
}

TEST_F(PCA9685DriverSimTest, Mode2MatchesConfig)
{
    PCA9685::Config cfg;
    cfg.totemPoleOutput = true;
    cfg.invertOutput    = false;
    PCA9685 drv(bus, delay, cfg);
    ASSERT_TRUE(drv.init());

    const uint8_t mode2 = sim.regValue(regs::kRegMode2);
    EXPECT_NE(mode2 & regs::kMode2OutDrv, 0u);
    EXPECT_EQ(mode2 & regs::kMode2Invrt, 0u);
}

TEST_F(PCA9685DriverSimTest, SetChannelPulse1500UsEncodesOffCount307)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());

    ASSERT_TRUE(drv.setChannelPulseUs(0, 1500));
    // 1500 µs / 20 000 µs × 4096 = 307.2 → 307
    EXPECT_EQ(sim.onCount(0), 0u);
    EXPECT_EQ(sim.offCount(0), 307u);
}

TEST_F(PCA9685DriverSimTest, SetChannelPulseRejectsOutOfRange)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());

    // Period at 50 Hz is 20 000 µs; equal or greater is invalid.
    EXPECT_FALSE(drv.setChannelPulseUs(0, 20000));
    EXPECT_FALSE(drv.setChannelPulseUs(0, 65535));
    EXPECT_FALSE(drv.setChannelPulseUs(16, 1500));  // out-of-range channel
}

TEST_F(PCA9685DriverSimTest, SetChannelPulseBeforeInitReturnsFalse)
{
    PCA9685 drv(bus, delay);
    EXPECT_FALSE(drv.setChannelPulseUs(0, 1500));
}

TEST_F(PCA9685DriverSimTest, SetChannelIdleSetsFullOffBit)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());
    ASSERT_TRUE(drv.setChannelPulseUs(3, 1500));
    ASSERT_FALSE(sim.fullOff(3));

    EXPECT_TRUE(drv.setChannelIdle(3));
    EXPECT_TRUE(sim.fullOff(3));
}

TEST_F(PCA9685DriverSimTest, SetAllIdleSetsFullOffOnEveryChannel)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());

    EXPECT_TRUE(drv.setAllIdle());
    for (uint8_t ch = 0; ch < 16; ++ch)
    {
        EXPECT_TRUE(sim.fullOff(ch)) << "channel " << int{ch};
    }
}

TEST_F(PCA9685DriverSimTest, ChannelProxyRoutesThroughDriver)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());

    auto ch2 = drv.channel(2);
    EXPECT_TRUE(ch2.setPulseUs(2000));
    // 2000 µs / 20 000 µs × 4096 = 409.6 → 409
    EXPECT_EQ(sim.offCount(2), 409u);
    EXPECT_FALSE(sim.fullOff(2));

    EXPECT_TRUE(ch2.setIdle());
    EXPECT_TRUE(sim.fullOff(2));
}

TEST_F(PCA9685DriverSimTest, SetFrequencyReconfiguresPrescaleWithoutLoss)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());

    EXPECT_TRUE(drv.setFrequency(200));
    EXPECT_NEAR(sim.frequencyHz(), 200.0f, 5.0f);
    EXPECT_FALSE(sim.isSleeping());
    EXPECT_EQ(sim.blockedPrescaleWrites(), 0u);

    // Channel math must follow the new period. At ~200 Hz the period is ~5 ms,
    // so a 2.5 ms pulse is accepted while a 10 ms pulse (comfortably beyond
    // any realised period) is rejected. Prescale quantisation means the exact
    // period may be slightly more than 5 ms, so avoid asserting against the
    // boundary.
    EXPECT_TRUE(drv.setChannelPulseUs(0, 2500));
    EXPECT_FALSE(drv.setChannelPulseUs(0, 10000));
}

TEST_F(PCA9685DriverSimTest, SleepAndWakeTogglesMode1SleepBit)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());
    ASSERT_FALSE(sim.isSleeping());

    EXPECT_TRUE(drv.sleep());
    EXPECT_TRUE(sim.isSleeping());

    EXPECT_TRUE(drv.wake());
    EXPECT_FALSE(sim.isSleeping());
}

TEST_F(PCA9685DriverSimTest, MultiChannelIndependence)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());

    EXPECT_TRUE(drv.setChannelPulseUs(0, 1000));
    EXPECT_TRUE(drv.setChannelPulseUs(1, 1500));
    EXPECT_TRUE(drv.setChannelPulseUs(2, 2000));

    EXPECT_EQ(sim.offCount(0), 204u);  // 1000/20000*4096 = 204.8
    EXPECT_EQ(sim.offCount(1), 307u);
    EXPECT_EQ(sim.offCount(2), 409u);
    // Channels we did not touch stay at reset defaults.
    EXPECT_EQ(sim.offCount(3), 0u);
    EXPECT_EQ(sim.offCount(15), 0u);
}

TEST_F(PCA9685DriverSimTest, RestartBitSelfClearsAfterInit)
{
    PCA9685 drv(bus, delay);
    ASSERT_TRUE(drv.init());
    // Driver writes RESTART=1 to restart any in-flight PWM; simulator clears it.
    EXPECT_EQ(sim.regValue(regs::kRegMode1) & regs::kMode1Restart, 0u);
}

TEST_F(PCA9685DriverSimTest, CustomAddressIsHonored)
{
    PCA9685Simulator alt(0x55);
    VirtualI2CBus    altBus;
    altBus.attach(alt);

    PCA9685::Config cfg;
    cfg.address = 0x55;
    PCA9685 drv(altBus, delay, cfg);
    EXPECT_TRUE(drv.init());

    EXPECT_TRUE(drv.setChannelPulseUs(4, 1500));
    EXPECT_EQ(alt.offCount(4), 307u);
}

// ── Mock suite: exact transaction assertions ────────────────────────────────

class PCA9685DriverMockTest : public ::testing::Test
{
protected:
    MockI2CBus bus;
    FakeDelay  delay;

    // Satisfy the init() transaction sequence with minimal expectations.
    void expectSuccessfulInit(uint8_t addr = regs::kDefaultAddress)
    {
        using ::testing::AnyNumber;
        EXPECT_CALL(bus, probe(addr)).WillOnce(Return(true));
        EXPECT_CALL(bus, writeRegister(addr, _, _, _))
            .Times(AnyNumber())
            .WillRepeatedly(Return(true));
    }
};

TEST_F(PCA9685DriverMockTest, InitFailsIfProbeFails)
{
    PCA9685 drv(bus, delay);
    EXPECT_CALL(bus, probe(regs::kDefaultAddress)).WillOnce(Return(false));
    EXPECT_CALL(bus, writeRegister(_, _, _, _)).Times(0);
    EXPECT_FALSE(drv.init());
}

TEST_F(PCA9685DriverMockTest, SetChannelPulseIssuesSingleFourByteBurst)
{
    PCA9685 drv(bus, delay);
    expectSuccessfulInit();
    ASSERT_TRUE(drv.init());

    // Exactly one write to LEDn_ON_L with length 4. No other traffic.
    EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                   regs::ledOnLReg(0), _, 4))
        .WillOnce(Return(true));
    EXPECT_TRUE(drv.setChannelPulseUs(0, 1500));
}

TEST_F(PCA9685DriverMockTest, SetChannelIdleIssuesSingleFourByteBurst)
{
    PCA9685 drv(bus, delay);
    expectSuccessfulInit();
    ASSERT_TRUE(drv.init());

    EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                   regs::ledOnLReg(2), _, 4))
        .WillOnce(Return(true));
    EXPECT_TRUE(drv.setChannelIdle(2));
}

TEST_F(PCA9685DriverMockTest, SetAllIdleUsesAllLedOffH)
{
    PCA9685 drv(bus, delay);
    expectSuccessfulInit();
    ASSERT_TRUE(drv.init());

    EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                   regs::kRegAllLedOffH, _, 1))
        .WillOnce(Return(true));
    EXPECT_TRUE(drv.setAllIdle());
}

TEST_F(PCA9685DriverMockTest, BusNackPropagates)
{
    PCA9685 drv(bus, delay);
    expectSuccessfulInit();
    ASSERT_TRUE(drv.init());

    EXPECT_CALL(bus, writeRegister(_, _, _, 4)).WillOnce(Return(false));
    EXPECT_FALSE(drv.setChannelPulseUs(0, 1500));
}

TEST_F(PCA9685DriverMockTest, InvalidChannelRejectedWithoutBusTraffic)
{
    PCA9685 drv(bus, delay);
    expectSuccessfulInit();
    ASSERT_TRUE(drv.init());

    // After init, no further calls on the bus must happen for invalid ops.
    ::testing::Mock::VerifyAndClearExpectations(&bus);
    EXPECT_CALL(bus, probe(_)).Times(0);
    EXPECT_CALL(bus, writeRegister(_, _, _, _)).Times(0);
    EXPECT_CALL(bus, readRegister(_, _, _, _)).Times(0);

    EXPECT_FALSE(drv.setChannelPulseUs(16, 1500));
    EXPECT_FALSE(drv.setChannelIdle(255));
}

TEST_F(PCA9685DriverMockTest, InitWritesPrescaleBeforeWake)
{
    // The driver must have written MODE1 with SLEEP=1 before writing PRE_SCALE
    // (datasheet footnote), and PRE_SCALE before the subsequent MODE1 wake.
    PCA9685 drv(bus, delay);

    EXPECT_CALL(bus, probe(regs::kDefaultAddress)).WillOnce(Return(true));

    {
        InSequence seq;
        // MODE1 sleep (SLEEP bit set, AI+ALLCALL present).
        EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                       regs::kRegMode1, _, 1))
            .WillOnce(Return(true));
        // PRE_SCALE write.
        EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                       regs::kRegPreScale, _, 1))
            .WillOnce(Return(true));
        // MODE1 wake (SLEEP cleared).
        EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                       regs::kRegMode1, _, 1))
            .WillOnce(Return(true));
        // MODE1 restart.
        EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                       regs::kRegMode1, _, 1))
            .WillOnce(Return(true));
        // MODE2 configure.
        EXPECT_CALL(bus, writeRegister(regs::kDefaultAddress,
                                       regs::kRegMode2, _, 1))
            .WillOnce(Return(true));
    }

    EXPECT_TRUE(drv.init());
    EXPECT_GE(delay.m_totalUs, 500u);
}
