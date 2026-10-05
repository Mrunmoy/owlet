// owlet_sim — runs the device core against a simulated PCA9685 and exposes
// the protocol on a pseudo-terminal, exactly as the ESP exposes it on USB.
//
//   ./owlet_sim                 # prints the pty path, e.g. /dev/pts/7
//   python -m owlet -p /dev/pts/7 ping
//
// With --link PATH the pty is also symlinked to PATH for a stable name.
// With --cal-file PATH the calibration is persisted to PATH (as NVS would be
// on the device); without it, $CAL,SAVE keeps it in memory only.

#include "owlet/core/CalibrationCodec.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/EventBus.hpp"
#include "owlet/core/TroubleLog.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Version.hpp"
#include "owlet/hal/IClock.hpp"
#include "owlet/hal/IDelayProvider.hpp"
#include "owlet/hal/ISerialPort.hpp"
#include "owlet/servo/Servo.hpp"
#include "owlet/text/TextProtocol.hpp"
#include "owlet/pca9685/PCA9685.hpp"
#include "owlet/sim/PCA9685Simulator.hpp"
#include "owlet/sim/VirtualI2CBus.hpp"

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

class NoDelay final : public ::owlet::hal::IDelayProvider
{
public:
    void delayUs(uint32_t) override {}
    void delayMs(uint32_t) override {}
};

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
// same byte format the ESP32 writes to NVS.
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

} // namespace

int main(int argc, char** argv)
{
    std::string link;
    std::string calFile;
    for (int i = 1; i < argc; ++i)
    {
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

    ::owlet::sim::PCA9685Simulator chip;
    ::owlet::sim::VirtualI2CBus    bus;
    bus.attach(chip);
    NoDelay        delay;
    ::owlet::PCA9685 pwm(bus, delay);
    if (!pwm.init() || !pwm.setAllIdle())
    {
        std::fprintf(stderr, "owlet_sim: PCA9685 simulator init failed\n");
        return 1;
    }

    ::owlet::PCA9685::Channel panCh  = pwm.channel(0);
    ::owlet::PCA9685::Channel tiltCh = pwm.channel(1);
    ::owlet::Servo            pan(panCh, ::owlet::Servo::Calibration{});
    ::owlet::Servo            tilt(tiltCh, ::owlet::Servo::Calibration{});
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
        ::owlet::core::CalibrationRecord{::owlet::Servo::toAxisCalibration({}), ::owlet::Servo::toAxisCalibration({})});
    calibrator.begin();
    ::owlet::text::TextProtocol protocol(port, head, calibrator, clock,
                                         ::owlet::core::kFirmwareVersion, nullptr, &events, &trouble);
    (void) events.attach(protocol);
    ::owlet::core::ControlLoop  loop(port, head, protocol, &events);
    loop.begin(clock.nowMs());
    while (g_stop == 0)
    {
        loop.service(clock.nowMs());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!link.empty())
    {
        ::unlink(link.c_str());
    }
    return 0;
}
