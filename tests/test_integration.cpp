// Whole firmware stack on the host: ControlLoop → ProtocolMux → NmeaProtocol →
// PanTilt → Servo →
// PCA9685 driver → VirtualI2CBus → PCA9685Simulator. Asserts on the PWM
// counts the real chip would be driving.

#include "FakeCalibrationStore.hpp"
#include "FakeClock.hpp"
#include "FakeSerialPort.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/ProtocolMux.hpp"
#include "owlet/nmea/NmeaProtocol.hpp"
#include "owlet/hal/IDelayProvider.hpp"
#include "owlet/pca9685/PCA9685.hpp"
#include "owlet/nmea/LineWriter.hpp"
#include "owlet/sim/PCA9685Simulator.hpp"
#include "owlet/sim/VirtualI2CBus.hpp"

#include <gtest/gtest.h>

#include <string>

using ::owlet::PCA9685;
using ::owlet::Servo;
using ::owlet::core::ControlLoop;
using ::owlet::core::ProtocolMux;
using ::owlet::nmea::NmeaProtocol;
using ::owlet::core::PanTilt;

namespace
{

class NoDelay final : public ::owlet::hal::IDelayProvider
{
public:
    void delayUs(uint32_t) override {}
    void delayMs(uint32_t) override {}
};

std::string frame(const std::string& body)
{
    ::owlet::nmea::LineWriter w;
    w.beginRaw(body);
    return std::string(w.finish());
}

class IntegrationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        bus.attach(chip);
        ASSERT_TRUE(pwm.init());
        ASSERT_TRUE(pwm.setAllIdle());
        calib.begin();
        proc.begin(now);
        (void) port.takeLines();
    }

    void send(const std::string& body)
    {
        port.rx += frame(body);
        proc.service(now);
    }

    void run(uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 10)
        {
            now += 10;
            proc.service(now);
        }
    }

    // Pulse width the chip is generating on `ch`, in µs.
    double pulseUs(uint8_t ch) const
    {
        const double periodUs = 1e6 / static_cast<double>(pwm.frequencyHz());
        return (chip.offCount(ch) - chip.onCount(ch)) * periodUs / 4096.0;
    }

    ::owlet::sim::PCA9685Simulator chip;
    ::owlet::sim::VirtualI2CBus    bus;
    NoDelay                        delay;
    PCA9685                        pwm{bus, delay};
    PCA9685::Channel               panCh{pwm.channel(0)};
    PCA9685::Channel               tiltCh{pwm.channel(1)};
    Servo                          pan{panCh, Servo::Calibration{}};
    Servo                          tilt{tiltCh, Servo::Calibration{}};
    PanTilt                        head{pan, tilt};
    ::owlet::test::FakeSerialPort  port;
    ::owlet::test::FakeClock       clock;
    ::owlet::test::FakeCalibrationStore store;
    ::owlet::core::Calibrator      calib{pan, tilt, head, store,
                                    ::owlet::core::CalibrationRecord{Servo::Calibration{},
                                                                     Servo::Calibration{}}};
    NmeaProtocol                   nmea{port, head, calib, clock, "test"};
    ProtocolMux<1>                 mux{{&nmea}};
    ControlLoop                    proc{port, head, mux};
    uint32_t                       now{0};
};

} // namespace

TEST_F(IntegrationTest, BootsWithBothChannelsFullOff)
{
    EXPECT_TRUE(chip.fullOff(0));
    EXPECT_TRUE(chip.fullOff(1));
}

TEST_F(IntegrationTest, EnableDrivesCenterPulse)
{
    send("ENABLE");
    EXPECT_FALSE(chip.fullOff(0));
    EXPECT_NEAR(pulseUs(0), 1500.0, 10.0);
    EXPECT_NEAR(pulseUs(1), 1500.0, 10.0);
}

TEST_F(IntegrationTest, MoveReachesEndpointPulses)
{
    send("ENABLE");
    send("SPEED,500");
    send("MOVE,90,-90");
    run(500);
    EXPECT_NEAR(pulseUs(0), 2500.0, 10.0);
    EXPECT_NEAR(pulseUs(1), 500.0, 10.0);
}

TEST_F(IntegrationTest, DisableReturnsChannelsToFullOff)
{
    send("ENABLE,30,30");
    send("DISABLE");
    EXPECT_TRUE(chip.fullOff(0));
    EXPECT_TRUE(chip.fullOff(1));
}

TEST_F(IntegrationTest, ReversedAxisDrivesTheMirroredPulse)
{
    send("CAL,TILT,2500,1500,500,-90,90");
    send("ENABLE,0,45");
    EXPECT_NEAR(pulseUs(1), 1000.0, 10.0);  // +45° on a reversed axis
}
