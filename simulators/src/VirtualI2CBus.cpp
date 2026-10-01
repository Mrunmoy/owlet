#include "owlet/sim/VirtualI2CBus.hpp"

namespace owlet::sim
{

void VirtualI2CBus::attach(II2CDevice& device)
{
    m_devices.push_back(&device);
}

II2CDevice* VirtualI2CBus::find(uint8_t addr) const
{
    for (II2CDevice* d : m_devices)
    {
        if (d != nullptr && d->address() == addr)
        {
            return d;
        }
    }
    return nullptr;
}

bool VirtualI2CBus::writeRegister(uint8_t devAddr, uint8_t reg,
                                  const uint8_t* data, std::size_t len)
{
    ++m_writeCount;
    II2CDevice* const d = find(devAddr);
    if (d == nullptr)
    {
        return false;
    }
    return d->onWrite(reg, data, len);
}

bool VirtualI2CBus::readRegister(uint8_t devAddr, uint8_t reg,
                                 uint8_t* out, std::size_t len)
{
    ++m_readCount;
    II2CDevice* const d = find(devAddr);
    if (d == nullptr)
    {
        return false;
    }
    return d->onRead(reg, out, len);
}

bool VirtualI2CBus::probe(uint8_t devAddr)
{
    ++m_probeCount;
    II2CDevice* const d = find(devAddr);
    if (d == nullptr)
    {
        return false;
    }
    return d->onProbe();
}

} // namespace owlet::sim
