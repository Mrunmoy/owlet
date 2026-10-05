#include "FakeAxis.hpp"
#include "FakeTmc2209Chip.hpp"
#include "owlet/core/HeadDiagnostics.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Poller.hpp"
#include "owlet/stepper/Tmc2209.hpp"
#include "owlet/stepper/Tmc2209Monitor.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

using ::owlet::Tmc2209;
using ::owlet::Tmc2209Monitor;
using ::owlet::hal::AxisDiagnostics;
using ::owlet::test::FakeTmc2209Chip;

namespace
{

Tmc2209::Config config()
{
    Tmc2209::Config c;
    c.address = 3;
    c.runCurrentMa = 450;
    c.holdCurrentMa = 250;
    c.microsteps = 16;
    c.stallThreshold = 20;
    return c;
}

struct Rig
{
    Rig() : driver(chip, config())
    {
        chip.address = 3;
    }

    FakeTmc2209Chip chip;
    Tmc2209 driver;
};

// Run the monitor for `ms` of 2 ms ticks.
void run(Tmc2209Monitor& monitor, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += 2)
    {
        monitor.poll(2);
    }
}

} // namespace

// ── Datagrams ───────────────────────────────────────────────────────────────

TEST(Tmc2209, CrcMatchesKnownDatagrams)
{
    // Read requests as published for the TMC220x family: sync, address, register, CRC.
    const uint8_t ioin[] = {0x05, 0x00, 0x06};
    const uint8_t gconf[] = {0x05, 0x00, 0x00};
    const uint8_t ifcnt[] = {0x05, 0x00, 0x02};
    EXPECT_EQ(Tmc2209::crc8(ioin, 3), 0x6F);
    EXPECT_EQ(Tmc2209::crc8(gconf, 3), 0x48);
    EXPECT_EQ(Tmc2209::crc8(ifcnt, 3), 0x8F);
}

TEST(Tmc2209, ReadsAndWritesRegisters)
{
    Rig r;
    uint32_t v = 0;
    ASSERT_TRUE(r.driver.readRegister(Tmc2209::kChopconf, v));
    EXPECT_EQ(v, 0x10010053u);
    ASSERT_TRUE(r.driver.writeRegister(Tmc2209::kSgthrs, 0x12345678u));
    EXPECT_EQ(r.chip.reg[Tmc2209::kSgthrs], 0x12345678u);
    EXPECT_EQ(r.chip.rejected, 0);
}

TEST(Tmc2209, ACorruptOrMissingReplyIsAFailureNotData)
{
    Rig r;
    uint32_t v = 0xAAAAAAAAu;
    r.chip.corruptReplies = true;
    EXPECT_FALSE(r.driver.readRegister(Tmc2209::kChopconf, v));
    r.chip.corruptReplies = false;
    r.chip.powered = false;
    EXPECT_FALSE(r.driver.readRegister(Tmc2209::kChopconf, v));
    EXPECT_EQ(v, 0xAAAAAAAAu) << "a failed read must not touch the output";
}

TEST(Tmc2209, ProbeRecognisesTheChipByItsVersion)
{
    Rig r;
    EXPECT_TRUE(r.driver.probe());
    r.chip.version = 0x20; // a TMC2208: no StallGuard
    r.chip.reset();
    EXPECT_FALSE(r.driver.probe());
    r.chip.version = 0x21;
    r.chip.reset();
    r.chip.address = 0; // answers a different address than we use
    EXPECT_FALSE(r.driver.probe());
}

// ── Configuration ───────────────────────────────────────────────────────────

TEST(Tmc2209, CurrentScaleFollowsTheDatasheetFormula)
{
    // 0.11 Ω sense resistors. High-sensitivity range (vsense) tops out at
    // 180 mV / 130 mΩ / √2 = 979 mA; the other at 325 / 130 / √2 = 1768 mA.
    const Tmc2209::CurrentSetting low = Tmc2209::currentFor(450, 110);
    EXPECT_TRUE(low.vsense);
    EXPECT_EQ(low.scale, 14); // 15/32 of 979 mA
    EXPECT_NEAR(low.actualMa, 459, 3);

    const Tmc2209::CurrentSetting high = Tmc2209::currentFor(1500, 110);
    EXPECT_FALSE(high.vsense);
    EXPECT_EQ(high.scale, 26);
    EXPECT_NEAR(high.actualMa, 1492, 5);

    EXPECT_EQ(Tmc2209::currentFor(0, 110).scale, 0);
    EXPECT_EQ(Tmc2209::currentFor(60000, 110).scale, 31) << "clamped, never wrapped";
    EXPECT_EQ(Tmc2209::currentFor(450, 0).scale, Tmc2209::currentFor(450, 110).scale)
        << "a nonsense resistor value falls back to the usual one";
}

TEST(Tmc2209, MicrostepsMapToTheRegisterCode)
{
    EXPECT_EQ(Tmc2209::microstepCode(256), 0);
    EXPECT_EQ(Tmc2209::microstepCode(16), 4);
    EXPECT_EQ(Tmc2209::microstepCode(8), 5);
    EXPECT_EQ(Tmc2209::microstepCode(1), 8);
    EXPECT_EQ(Tmc2209::microstepCode(17), 4) << "anything that is not a power of two means 16";
}

TEST(Tmc2209, ConfigureTakesControlFromThePinsAndThePot)
{
    Rig r;
    ASSERT_TRUE(r.driver.configure());
    const uint32_t gconf = r.chip.reg[Tmc2209::kGconf];
    EXPECT_TRUE(gconf & (1u << 6)) << "pdn_disable: the PDN pin is now only a UART";
    EXPECT_TRUE(gconf & (1u << 7)) << "mstep_reg_select: microsteps from the register, not MS1/MS2";
    EXPECT_FALSE(gconf & (1u << 0))
        << "I_scale_analog off: the potentiometer no longer sets current";
    EXPECT_FALSE(gconf & (1u << 2)) << "StealthChop stays on; StallGuard needs it";
    EXPECT_TRUE(gconf & (1u << 8)) << "bits we don't own are preserved";

    const uint32_t chop = r.chip.reg[Tmc2209::kChopconf];
    EXPECT_EQ((chop >> 24) & 0xFu, 4u) << "1/16";
    EXPECT_TRUE(chop & (1u << 28)) << "interpolation to 1/256";
    EXPECT_TRUE(chop & (1u << 17)) << "vsense for the low current range";
    EXPECT_EQ(chop & 0xFu, 3u) << "TOFF preserved: zero would switch the driver off";

    const uint32_t current = r.chip.reg[Tmc2209::kIholdIrun];
    EXPECT_EQ((current >> 8) & 0x1Fu, 14u);
    EXPECT_LT(current & 0x1Fu, 14u) << "hold current below run current";
    EXPECT_EQ(r.chip.reg[Tmc2209::kSgthrs], 20u);
    EXPECT_EQ(r.chip.reg[Tmc2209::kTcoolthrs], 0xFFFFFu);
    EXPECT_EQ(r.chip.reg[Tmc2209::kGstat] & 1u, 0u) << "reset flag cleared";
    EXPECT_NEAR(r.driver.runCurrentMa(), 459, 3);
}

TEST(Tmc2209, ConfigureFailsIfTheChipDidNotTakeEveryWrite)
{
    Rig r;
    r.chip.powered = false;
    EXPECT_FALSE(r.driver.configure());
    r.chip.powered = true;
    r.chip.address = 1; // writes are ignored: the write counter never moves
    EXPECT_FALSE(r.driver.configure());
}

// ── Status ──────────────────────────────────────────────────────────────────

TEST(Tmc2209, StatusDecodesTheFaultBits)
{
    Rig r;
    Tmc2209::Status s;
    r.chip.reg[Tmc2209::kDrvStatus] = (1u << 0) | (1u << 3) | (1u << 6) | (17u << 16) | (1u << 31);
    ASSERT_TRUE(r.driver.readStatus(s));
    EXPECT_TRUE(s.overTempWarning);
    EXPECT_FALSE(s.overTemp);
    EXPECT_FALSE(s.shortA);
    EXPECT_TRUE(s.shortB);
    EXPECT_TRUE(s.openA);
    EXPECT_FALSE(s.openB);
    EXPECT_TRUE(s.standstill);
    EXPECT_EQ(s.currentScale, 17);

    uint16_t load = 0;
    r.chip.reg[Tmc2209::kSgResult] = 312;
    ASSERT_TRUE(r.driver.readLoad(load));
    EXPECT_EQ(load, 312);
}

// ── Monitor ─────────────────────────────────────────────────────────────────

TEST(Tmc2209Monitor, ReportsNothingUntilTheChipAnswersThenConfiguresIt)
{
    Rig r;
    Tmc2209Monitor monitor(r.driver);
    r.chip.powered = false; // motor supply off: the chip is silent
    run(monitor, 3000);
    EXPECT_FALSE(monitor.diagnostics().available);
    EXPECT_FALSE(monitor.online());

    r.chip.powered = true; // supply comes up, chip in its reset state
    r.chip.reset();
    run(monitor, 1500);
    EXPECT_TRUE(monitor.online());
    EXPECT_TRUE(r.chip.reg[Tmc2209::kGconf] & (1u << 7)) << "configured as soon as it was found";
    const AxisDiagnostics d = monitor.diagnostics();
    EXPECT_TRUE(d.available);
    EXPECT_EQ(d.faults, 0);
    EXPECT_NEAR(d.currentMa, 459, 3);
}

TEST(Tmc2209Monitor, PollsRarelySoTheControlLoopIsNotStarved)
{
    Rig r;
    Tmc2209Monitor monitor(r.driver);
    run(monitor, 1000); // found and configured
    const int before = r.chip.exchanges;
    run(monitor, 1000);
    EXPECT_LE(r.chip.exchanges - before, 12) << "about one short exchange per 100 ms";
    EXPECT_GE(r.chip.exchanges - before, 8);
}

TEST(Tmc2209Monitor, CarriesFaultsLoadAndStallToTheDiagnostics)
{
    Rig r;
    Tmc2209Monitor monitor(r.driver);
    run(monitor, 1000);
    r.chip.reg[Tmc2209::kDrvStatus] = (1u << 1) | (1u << 2); // over-temperature, short on A
    r.chip.reg[Tmc2209::kSgResult] = 30; // at or below 2 x threshold (40): stalled
    run(monitor, 600);
    AxisDiagnostics d = monitor.diagnostics();
    EXPECT_EQ(d.faults, AxisDiagnostics::kOverTemp | AxisDiagnostics::kShortA);
    EXPECT_EQ(d.load, 30);
    EXPECT_TRUE(d.stalled);

    r.chip.reg[Tmc2209::kDrvStatus] = (1u << 31); // at standstill the load value means nothing
    run(monitor, 600);
    d = monitor.diagnostics();
    EXPECT_EQ(d.faults, 0);
    EXPECT_FALSE(d.stalled);
}

TEST(Tmc2209Monitor, ReconfiguresAfterTheChipLosesPower)
{
    Rig r;
    Tmc2209Monitor monitor(r.driver);
    run(monitor, 1000);
    ASSERT_TRUE(monitor.online());

    r.chip.reset(); // a brown-out: registers back to defaults, reset flag set
    ASSERT_FALSE(r.chip.reg[Tmc2209::kGconf] & (1u << 7));
    run(monitor, 1000);
    EXPECT_TRUE(r.chip.reg[Tmc2209::kGconf] & (1u << 7)) << "settings restored without a reboot";
    EXPECT_TRUE(monitor.online());

    r.chip.powered = false; // and gone altogether
    run(monitor, 1500);
    EXPECT_FALSE(monitor.online());
    EXPECT_FALSE(monitor.diagnostics().available);
}

// ── Into the core ───────────────────────────────────────────────────────────

TEST(HeadDiagnostics, RoutesEachAxisToItsSource)
{
    using ::owlet::core::Axis;
    Rig pan;
    Rig tilt;
    Tmc2209Monitor panMonitor(pan.driver);
    Tmc2209Monitor tiltMonitor(tilt.driver);
    tilt.chip.powered = false;
    run(panMonitor, 1000);
    run(tiltMonitor, 1000);
    const ::owlet::core::HeadDiagnostics head(&panMonitor, &tiltMonitor);
    EXPECT_TRUE(head.read(Axis::Pan).available);
    EXPECT_FALSE(head.read(Axis::Tilt).available);

    const ::owlet::core::HeadDiagnostics none(nullptr, nullptr); // a head with nothing to report
    EXPECT_FALSE(none.read(Axis::Pan).available);
}

TEST(Poller, TicksMotionThenPollsEveryItem)
{
    struct Counter final : ::owlet::hal::IPollable
    {
        void poll(uint32_t dtMs) override
        {
            total += dtMs;
            ++calls;
        }
        uint32_t total{0};
        int calls{0};
    };
    ::owlet::test::FakeAxis pan;
    ::owlet::test::FakeAxis tilt;
    ::owlet::core::PanTilt head(pan, tilt);
    Counter a;
    Counter b;
    ::owlet::core::Poller<3> poller(head, {&a, nullptr, &b});
    EXPECT_TRUE(poller.tick(2).busOk);
    EXPECT_TRUE(poller.tick(3).busOk);
    EXPECT_EQ(pan.ticks, 2) << "motion is still ticked";
    EXPECT_EQ(a.calls, 2);
    EXPECT_EQ(b.total, 5u);
}
