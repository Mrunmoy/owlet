#include "owlet/platform/pico/PicoDigitalOutput.hpp"

#include "hardware/gpio.h"

namespace owlet::platform::pico
{

PicoDigitalOutput::PicoDigitalOutput(unsigned pin, Polarity polarity)
    : m_pin(pin), m_activeLow(polarity == Polarity::ActiveLow)
{
    gpio_init(m_pin);
    gpio_put(m_pin, m_activeLow); // off level first, then drive
    gpio_set_dir(m_pin, GPIO_OUT);
}

void PicoDigitalOutput::set(bool on)
{
    gpio_put(m_pin, on != m_activeLow);
}

} // namespace owlet::platform::pico
