#include "owlet/platform/pico/PicoFlashCalibrationStore.hpp"

#include "owlet/core/CalibrationCodec.hpp"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/platform.h"

#include <algorithm>
#include <array>

namespace owlet::platform::pico
{

namespace
{

using ::owlet::core::CalibrationCodec;

constexpr uint32_t kSectorOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;

static_assert(CalibrationCodec::kEncodedSize <= FLASH_PAGE_SIZE,
              "the record must fit in one flash page");

[[nodiscard]] const uint8_t* stored()
{
    return reinterpret_cast<const uint8_t*>(XIP_BASE + kSectorOffset);
}

// RAII: flash operations must not be interrupted by code that runs from flash.
// Enough on one core; a second core would also need pico-sdk's lockout.
class InterruptsOff
{
public:
    InterruptsOff() : m_saved(save_and_disable_interrupts()) {}
    ~InterruptsOff()
    {
        restore_interrupts(m_saved);
    }
    InterruptsOff(const InterruptsOff&) = delete;
    InterruptsOff& operator=(const InterruptsOff&) = delete;

private:
    uint32_t m_saved;
};

} // namespace

bool PicoFlashCalibrationStore::load(::owlet::core::CalibrationRecord& out)
{
    return CalibrationCodec::decode(stored(), CalibrationCodec::kEncodedSize, out);
}

bool PicoFlashCalibrationStore::save(const ::owlet::core::CalibrationRecord& record)
{
    const CalibrationCodec::Bytes bytes = CalibrationCodec::encode(record);
    // Static: a whole flash page on a 2 KB stack, at the bottom of the
    // deepest call chain in the firmware, is space better not spent there.
    static std::array<uint8_t, FLASH_PAGE_SIZE> page;
    page.fill(0xFF); // erased-flash value, so the rest of the page stays blank
    std::copy(bytes.begin(), bytes.end(), page.begin());
    {
        InterruptsOff guard;
        flash_range_erase(kSectorOffset, FLASH_SECTOR_SIZE);
        flash_range_program(kSectorOffset, page.data(), page.size());
    }
    return std::equal(bytes.begin(), bytes.end(), stored());
}

bool PicoFlashCalibrationStore::erase()
{
    {
        InterruptsOff guard;
        flash_range_erase(kSectorOffset, FLASH_SECTOR_SIZE);
    }
    ::owlet::core::CalibrationRecord scratch{};
    return !load(scratch);
}

} // namespace owlet::platform::pico
