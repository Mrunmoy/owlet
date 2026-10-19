// owlet_sim — runs the device core on a simulated stepper head (the reference
// head's gearing, speeds and limits) and exposes the protocol on a
// pseudo-terminal, exactly as the Pico exposes it on USB.
//
//   ./owlet_sim                 # prints the pty path, e.g. /dev/pts/7
//   python -m owlet -p /dev/pts/7 ping
//
// With --link PATH the pty is also symlinked to PATH for a stable name.
// With --cal-file PATH the calibration is persisted to PATH (as the Pico keeps
// it in flash); without it, $CAL,SAVE keeps it in memory only.
// With --inject, lines typed on standard input raise events the simulated
// head has no hardware to produce, so clients can be tested against
// them:  limit pan max | lost pan | stall tilt | fault pan 6 | online pan |
//        offline tilt | homed | homefail tilt 1

#include "owlet/core/CalibrationCodec.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/EventBus.hpp"
#include "owlet/core/TroubleLog.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Version.hpp"
#include "owlet/hal/IClock.hpp"
#include "owlet/hal/IDigitalOutput.hpp"
#include "owlet/hal/ISerialPort.hpp"
#include "owlet/hal/IStepGenerator.hpp"
#include "owlet/stepper/SharedEnable.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/stepper/Tmc2209Standalone.hpp"
#include "owlet/text/TextProtocol.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace
{

volatile std::sig_atomic_t g_stop = 0;  // signal handlers can only touch globals

void onSignal(int)
{
    g_stop = 1;
}

// A step generator that takes every burst at once: the simulated motor
// never stalls, so the step count is the shaft position.
class SimStepGenerator final : public ::owlet::hal::IStepGenerator
{
public:
    [[nodiscard]] bool emit(int32_t, uint32_t) override { return !m_halted; }
    void halt() override { m_halted = true; }
    void resume() override { m_halted = false; }
    [[nodiscard]] bool halted() const override { return m_halted; }

private:
    bool m_halted{false};
};

// The drivers' shared enable line, with nothing attached.
class NullOutput final : public ::owlet::hal::IDigitalOutput
{
public:
    void set(bool) override {}
};

// The reference head (apps/pico-owlet/board.hpp): 1.8° motors at 1/16 through
// 3:1 belts, 240 °/s, 1440 °/s², factory limits pan ±90 and tilt ±60, travel
// up to pan ±180 and tilt ±135.
::owlet::StepperAxis::Config referenceAxis(float limitDeg, float travelDeg)
{
    ::owlet::StepperAxis::Config c;
    c.stepsPerDeg = ::owlet::tmc2209::stepsPerDegree(200, ::owlet::tmc2209::Microsteps::k16, 3.0f);
    c.maxSpeedDegPerSec = 240.0f;
    c.accelDegPerSec2 = 1440.0f;
    c.minDeg = -limitDeg;
    c.maxDeg = limitDeg;
    c.travelMinDeg = -travelDeg;
    c.travelMaxDeg = travelDeg;
    return c;
}

// ISerialPort over the master side of a pty.
class PtySerialPort final : public ::owlet::hal::ISerialPort
{
public:
    ~PtySerialPort()
    {
        if (m_fd >= 0)
        {
            ::close(m_fd);
        }
    }

    [[nodiscard]] bool open()
    {
        m_fd = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (m_fd < 0 || ::grantpt(m_fd) != 0 || ::unlockpt(m_fd) != 0)
        {
            return false;
        }
        // Raw mode on the slave so the line discipline doesn't echo or
        // translate CR/LF behind the client's back.
        const int slave = ::open(::ptsname(m_fd), O_RDWR | O_NOCTTY);
        if (slave < 0)
        {
            return false;
        }
        termios tio{};
        ::tcgetattr(slave, &tio);
        ::cfmakeraw(&tio);
        ::tcsetattr(slave, TCSANOW, &tio);
        ::close(slave);
        return ::fcntl(m_fd, F_SETFL, O_NONBLOCK) == 0;
    }

    const char* slavePath() const { return ::ptsname(m_fd); }

    [[nodiscard]] std::size_t read(uint8_t* out, std::size_t maxLen) override
    {
        const ssize_t n = ::read(m_fd, out, maxLen);
        return n > 0 ? static_cast<std::size_t>(n) : 0;
    }

    [[nodiscard]] bool write(const char* data, std::size_t len) override
    {
        // EIO when no client has the slave open: drop, like a real USB host
        // that isn't listening.
        return ::write(m_fd, data, len) == static_cast<ssize_t>(len);
    }

private:
    int m_fd{-1};
};

// steady_clock is CLOCK_MONOTONIC on Linux, the same clock Python's
// time.monotonic() reads, which keeps clock-sync tests meaningful.
class SteadyClock final : public ::owlet::hal::IClock
{
public:
    [[nodiscard]] uint32_t nowMs() const override
    {
        using namespace std::chrono;
        return static_cast<uint32_t>(
            duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
    }
};

// ICalibrationStore on a file (or memory when no path is given), using the
// same byte format the Pico writes to its flash.
class FileCalibrationStore final : public ::owlet::core::ICalibrationStore
{
public:
    explicit FileCalibrationStore(std::string path) : m_path(std::move(path)) {}

    [[nodiscard]] bool load(::owlet::core::CalibrationRecord& out) override
    {
        std::vector<uint8_t> bytes = m_memory;
        if (!m_path.empty())
        {
            std::ifstream in(m_path, std::ios::binary);
            if (!in)
            {
                return false;
            }
            bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        }
        return ::owlet::core::CalibrationCodec::decode(bytes.data(), bytes.size(), out);
    }

    [[nodiscard]] bool save(const ::owlet::core::CalibrationRecord& record) override
    {
        const auto bytes = ::owlet::core::CalibrationCodec::encode(record);
        if (m_path.empty())
        {
            m_memory.assign(bytes.begin(), bytes.end());
            return true;
        }
        std::ofstream out(m_path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
        return static_cast<bool>(out);
    }

    [[nodiscard]] bool erase() override
    {
        m_memory.clear();
        return m_path.empty() || std::remove(m_path.c_str()) == 0 || errno == ENOENT;
    }

private:
    std::string          m_path;
    std::vector<uint8_t> m_memory;
};

// Turns one typed line into an event. Unknown lines are ignored.
void inject(const std::string& line, ::owlet::core::IEventSink& events)
{
    using ::owlet::core::Event;
    using ::owlet::core::EventId;
    std::istringstream in(line);
    std::string        what;
    std::string        where;
    std::string        extra;
    in >> what >> where >> extra;
    const uint8_t axis = (where == "pan") ? uint8_t{0} : ((where == "tilt") ? uint8_t{1} : Event::kHead);
    uint16_t      arg  = 0;
    if (extra == "max")
    {
        arg = 1;
    }
    else if (!extra.empty() && extra != "min")
    {
        arg = static_cast<uint16_t>(std::strtoul(extra.c_str(), nullptr, 10));
    }
    struct Named
    {
        const char* name;
        EventId     id;
    };
    static const Named kNames[] = {
        {"limit", EventId::LimitHit},       {"lost", EventId::PositionLost},
        {"stall", EventId::Stalled},        {"fault", EventId::DriverFaultChanged},
        {"online", EventId::DriverOnline},  {"offline", EventId::DriverOffline},
        {"homed", EventId::Homed},          {"homefail", EventId::HomingFailed},
    };
    for (const Named& n : kNames)
    {
        if (what == n.name)
        {
            (void) events.post(n.id, axis, arg);
        }
    }
}

// Reads whatever complete lines are waiting on standard input, without
// blocking. Returns false once input has ended.
bool pumpInjections(std::string& pending, ::owlet::core::IEventSink& events)
{
    pollfd fd{STDIN_FILENO, POLLIN, 0};
    while (::poll(&fd, 1, 0) > 0)
    {
        char          buf[128];
        const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0)
        {
            return false;
        }
        pending.append(buf, static_cast<std::size_t>(n));
        std::size_t eol = 0;
        while ((eol = pending.find('\n')) != std::string::npos)
        {
            inject(pending.substr(0, eol), events);
            pending.erase(0, eol + 1);
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    std::string link;
    std::string calFile;
    bool        injecting = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--inject") == 0)
        {
            injecting = true;
            continue;
        }
        if (std::strcmp(argv[i], "--link") == 0 && i + 1 < argc)
        {
            link = argv[++i];
        }
        else if (std::strcmp(argv[i], "--cal-file") == 0 && i + 1 < argc)
        {
            calFile = argv[++i];
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    SimStepGenerator            panSteps;
    SimStepGenerator            tiltSteps;
    NullOutput                  enableLine;
    ::owlet::SharedEnable       enable(enableLine);
    ::owlet::SharedEnable::User panEnable(enable);
    ::owlet::SharedEnable::User tiltEnable(enable);
    ::owlet::StepperAxis        pan(panSteps, panEnable, referenceAxis(90.0f, 180.0f));
    ::owlet::StepperAxis        tilt(tiltSteps, tiltEnable, referenceAxis(60.0f, 135.0f));
    SteadyClock                          clock;
    ::owlet::hal::NullInterruptLock      eventLock;  // one thread, no interrupts
    ::owlet::core::StaticEventBus<16, 2> events(eventLock, clock);
    ::owlet::core::TroubleLog            trouble;
    (void) events.attach(trouble);
    ::owlet::core::PanTilt               head(pan, tilt, &events);

    PtySerialPort port;
    if (!port.open())
    {
        std::fprintf(stderr, "owlet_sim: pty setup failed: %s\n", std::strerror(errno));
        return 1;
    }
    if (!link.empty())
    {
        ::unlink(link.c_str());
        if (::symlink(port.slavePath(), link.c_str()) != 0)
        {
            std::fprintf(stderr, "owlet_sim: symlink %s failed\n", link.c_str());
            return 1;
        }
    }
    std::printf("%s\n", link.empty() ? port.slavePath() : link.c_str());
    std::fflush(stdout);

    FileCalibrationStore      store(calFile);
    ::owlet::core::Calibrator calibrator(
        pan, tilt, head, store,
        ::owlet::core::CalibrationRecord{pan.defaultCalibration(), tilt.defaultCalibration()});
    calibrator.begin();
    ::owlet::text::TextProtocol protocol(port, head, calibrator, clock,
                                         ::owlet::core::kFirmwareVersion, nullptr, &events, &trouble);
    (void) events.attach(protocol);
    ::owlet::core::ControlLoop  loop(port, head, protocol, &events);
    loop.begin(clock.nowMs());
    std::string typed;
    while (g_stop == 0)
    {
        if (injecting)
        {
            injecting = pumpInjections(typed, events);
        }
        loop.service(clock.nowMs());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!link.empty())
    {
        ::unlink(link.c_str());
    }
    return 0;
}
