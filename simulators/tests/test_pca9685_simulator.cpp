// Tests for the PCA9685 simulator against the NXP PCA9685 datasheet
// (Rev. 4 — 16 April 2015). These tests are authoritative for what the
// simulator claims about chip behavior; the PCA9685 driver will
// later be tested against both this simulator and MockI2CBus.

#include "owlet/pca9685/Registers.hpp"
#include "owlet/sim/PCA9685Simulator.hpp"
#include "owlet/sim/VirtualI2CBus.hpp"

#include <cstdint>
#include <gtest/gtest.h>

namespace regs = ::owlet::pca9685;
using owlet::sim::PCA9685Simulator;
using owlet::sim::VirtualI2CBus;

namespace
{

// Convenience: write a single byte to `reg` using the simulator directly
// (bypassing VirtualI2CBus for tests that only care about the chip state).
[[nodiscard]] bool writeByte(PCA9685Simulator& sim, uint8_t reg, uint8_t val)
{
    return sim.onWrite(reg, &val, 1);
}

// Wake the simulator up. Used by tests that want to exercise awake-mode
// behaviors after construction (which leaves SLEEP=1 per datasheet default).
void wake(PCA9685Simulator& sim)
{
    const uint8_t wakeByte = regs::kMode1AllCall; // clear SLEEP, keep ALLCALL
    ASSERT_TRUE(writeByte(sim, regs::kRegMode1, wakeByte));
}

// Enable register auto-increment without also waking, so that tests wanting
// to subsequently program PRE_SCALE still satisfy the SLEEP requirement.
void enableAutoIncrementAsleep(PCA9685Simulator& sim)
{
    const uint8_t v = regs::kMode1AutoInc | regs::kMode1Sleep | regs::kMode1AllCall;
    ASSERT_TRUE(writeByte(sim, regs::kRegMode1, v));
}

} // namespace

// ── Defaults ────────────────────────────────────────────────────────────────

TEST(Pca9685Simulator, ResetMatchesDatasheetDefaults)
{
    PCA9685Simulator sim;

    EXPECT_EQ(sim.regValue(regs::kRegMode1),        regs::kMode1Default);
    EXPECT_EQ(sim.regValue(regs::kRegMode2),        regs::kMode2Default);
    EXPECT_EQ(sim.regValue(regs::kRegSubAddr1),     regs::kSubAddr1Default);
    EXPECT_EQ(sim.regValue(regs::kRegSubAddr2),     regs::kSubAddr2Default);
    EXPECT_EQ(sim.regValue(regs::kRegSubAddr3),     regs::kSubAddr3Default);
    EXPECT_EQ(sim.regValue(regs::kRegAllCallAddr),  regs::kAllCallAdrDefault);
    EXPECT_EQ(sim.regValue(regs::kRegPreScale),     regs::kPreScaleDefault);

    EXPECT_TRUE(sim.isSleeping());
    EXPECT_FALSE(sim.autoIncrement());
    EXPECT_EQ(sim.address(), regs::kDefaultAddress);
}

TEST(Pca9685Simulator, ProbeAcks)
{
    PCA9685Simulator sim;
    EXPECT_TRUE(sim.onProbe());
}

// ── Single-byte writes (AI off) ─────────────────────────────────────────────

TEST(Pca9685Simulator, SingleByteWriteUpdatesRegister)
{
    PCA9685Simulator sim;
    EXPECT_TRUE(writeByte(sim, regs::kRegMode2, 0x12));
    EXPECT_EQ(sim.regValue(regs::kRegMode2), 0x12);
}

TEST(Pca9685Simulator, WithoutAutoIncrementBurstHitsSameRegister)
{
    PCA9685Simulator sim;
    // AI defaults to 0.
    const uint8_t burst[] = {0x11, 0x22, 0x33};
    EXPECT_TRUE(sim.onWrite(regs::kRegMode2, burst, 3));
    EXPECT_EQ(sim.regValue(regs::kRegMode2), 0x33); // last-write-wins
}

// ── Auto-increment bursts ───────────────────────────────────────────────────

TEST(Pca9685Simulator, AutoIncrementFillsConsecutiveRegisters)
{
    PCA9685Simulator sim;
    enableAutoIncrementAsleep(sim);
    ASSERT_TRUE(sim.autoIncrement());

    // Program channel 0: ON=0, OFF=307 (≈1.5 ms at 50 Hz).
    const uint16_t offCount = 307;
    const uint8_t led0[] = {
        0x00, 0x00,                                    // ON_L, ON_H
        static_cast<uint8_t>(offCount & 0xFF),
        static_cast<uint8_t>((offCount >> 8) & 0x0F),  // OFF_L, OFF_H
    };
    EXPECT_TRUE(sim.onWrite(regs::kRegLed0OnL, led0, 4));

    EXPECT_EQ(sim.onCount(0),  0u);
    EXPECT_EQ(sim.offCount(0), offCount);
}

TEST(Pca9685Simulator, AutoIncrementWrapsPastLed15)
{
    PCA9685Simulator sim;
    enableAutoIncrementAsleep(sim);

    // Write from LED15_OFF_H through one more byte. The next pointer should
    // wrap to MODE1 per the datasheet footnote. Use a distinctive MODE1 value
    // we can detect (keeping SLEEP=1 and AI=1 so state stays consistent).
    const uint8_t mode1After = regs::kMode1AutoInc | regs::kMode1Sleep;
    const uint8_t two[] = {0xAB, mode1After};
    EXPECT_TRUE(sim.onWrite(regs::kRegLed15OffH, two, 2));

    EXPECT_EQ(sim.regValue(regs::kRegLed15OffH), 0xAB);
    EXPECT_EQ(sim.regValue(regs::kRegMode1), mode1After);
}

TEST(Pca9685Simulator, ReadBackAfterBurstWrite)
{
    PCA9685Simulator sim;
    enableAutoIncrementAsleep(sim);

    const uint8_t base = regs::ledOnLReg(5);           // LED5 block
    const uint8_t written[] = {0x55, 0x05, 0xAA, 0x0A};
    EXPECT_TRUE(sim.onWrite(base, written, 4));

    uint8_t readBack[4] = {};
    EXPECT_TRUE(sim.onRead(base, readBack, 4));
    EXPECT_EQ(readBack[0], 0x55);
    EXPECT_EQ(readBack[1], 0x05);
    EXPECT_EQ(readBack[2], 0xAA);
    EXPECT_EQ(readBack[3], 0x0A);
}

// ── PRE_SCALE / SLEEP interaction ───────────────────────────────────────────

TEST(Pca9685Simulator, PrescaleBlockedWhenAwake)
{
    PCA9685Simulator sim;
    wake(sim);
    EXPECT_FALSE(sim.isSleeping());

    const uint8_t newPrescale = 0x79;  // 50 Hz
    EXPECT_TRUE(writeByte(sim, regs::kRegPreScale, newPrescale));  // ACK, but ignored
    EXPECT_EQ(sim.regValue(regs::kRegPreScale), regs::kPreScaleDefault);
    EXPECT_EQ(sim.blockedPrescaleWrites(), 1u);
}

TEST(Pca9685Simulator, PrescaleAcceptedWhenSleeping)
{
    PCA9685Simulator sim;
    ASSERT_TRUE(sim.isSleeping());

    EXPECT_TRUE(writeByte(sim, regs::kRegPreScale, 0x79));
    EXPECT_EQ(sim.regValue(regs::kRegPreScale), 0x79);
    EXPECT_EQ(sim.blockedPrescaleWrites(), 0u);
    EXPECT_NEAR(sim.frequencyHz(), 50.0f, 0.5f);
}

TEST(Pca9685Simulator, PrescaleComputationForKnownFrequencies)
{
    // prescaleForFrequency returns values that feed back to ~target frequency.
    struct Case { uint32_t target; float tolerance; };
    constexpr Case cases[] = {
        {   50u, 1.0f },   // servos
        {  200u, 5.0f },   // near default
        { 1000u, 40.0f },  // Fm+ LED use
    };
    for (const auto& c : cases)
    {
        const uint8_t ps = regs::prescaleForFrequency(c.target);
        const float observed = regs::frequencyForPrescale(ps);
        EXPECT_NEAR(observed, static_cast<float>(c.target), c.tolerance)
            << "target " << c.target << " Hz, prescale=0x" << std::hex << int{ps};
    }
}

// ── ALL_LED_* mirroring ─────────────────────────────────────────────────────

TEST(Pca9685Simulator, AllLedOffHFullOffBitAppliesToEveryChannel)
{
    PCA9685Simulator sim;
    const uint8_t fullOff = regs::kLedFullOff;
    EXPECT_TRUE(writeByte(sim, regs::kRegAllLedOffH, fullOff));

    for (uint8_t ch = 0; ch < 16; ++ch)
    {
        EXPECT_TRUE(sim.fullOff(ch)) << "channel " << int{ch};
    }
    EXPECT_EQ(sim.regValue(regs::kRegAllLedOffH), fullOff);
}

TEST(Pca9685Simulator, AllLedOnLMirrorsIntoEveryChannel)
{
    PCA9685Simulator sim;
    const uint8_t pattern = 0x42;
    EXPECT_TRUE(writeByte(sim, regs::kRegAllLedOnL, pattern));

    for (uint8_t ch = 0; ch < 16; ++ch)
    {
        EXPECT_EQ(sim.regValue(regs::ledOnLReg(ch)), pattern) << "channel " << int{ch};
    }
}

// ── Reserved + MODE1.RESTART semantics ──────────────────────────────────────

TEST(Pca9685Simulator, ReservedRegisterNotAcknowledged)
{
    PCA9685Simulator sim;
    const uint8_t val = 0xAA;
    uint8_t out = 0;
    EXPECT_FALSE(sim.onWrite(0x80, &val, 1));
    EXPECT_FALSE(sim.onRead(0x80, &out, 1));
}

TEST(Pca9685Simulator, RestartBitSelfClearsOnWriteOne)
{
    PCA9685Simulator sim;
    const uint8_t arm = regs::kMode1Restart | regs::kMode1Sleep | regs::kMode1AllCall;
    EXPECT_TRUE(writeByte(sim, regs::kRegMode1, arm));
    EXPECT_EQ(sim.regValue(regs::kRegMode1) & regs::kMode1Restart, 0u);
    EXPECT_NE(sim.regValue(regs::kRegMode1) & regs::kMode1Sleep, 0u);
}

// ── End-to-end through VirtualI2CBus ────────────────────────────────────────

TEST(Pca9685Simulator, DriverReachesSimulatorThroughVirtualBus)
{
    PCA9685Simulator sim;
    VirtualI2CBus bus;
    bus.attach(sim);

    // Master puts chip to sleep (already the default) + AI on, programs
    // PRE_SCALE for 50 Hz, wakes it, then writes LED2 pulse 1500 µs.
    EXPECT_TRUE(bus.writeByte(regs::kDefaultAddress, regs::kRegMode1,
                              regs::kMode1Sleep | regs::kMode1AutoInc | regs::kMode1AllCall));
    EXPECT_TRUE(bus.writeByte(regs::kDefaultAddress, regs::kRegPreScale, 0x79));
    EXPECT_TRUE(bus.writeByte(regs::kDefaultAddress, regs::kRegMode1,
                              regs::kMode1AutoInc | regs::kMode1AllCall));

    // 1.5 ms pulse at 50 Hz → offCount ≈ 307.
    const uint16_t offCount = 307;
    const uint8_t led2[] = {
        0x00, 0x00,
        static_cast<uint8_t>(offCount & 0xFF),
        static_cast<uint8_t>((offCount >> 8) & 0x0F),
    };
    EXPECT_TRUE(bus.writeRegister(regs::kDefaultAddress, regs::ledOnLReg(2), led2, 4));

    EXPECT_NEAR(sim.frequencyHz(), 50.0f, 0.5f);
    EXPECT_EQ(sim.onCount(2),  0u);
    EXPECT_EQ(sim.offCount(2), offCount);
    EXPECT_FALSE(sim.isSleeping());
    EXPECT_EQ(bus.writeCount(), 4u);
}
