#include "owlet/platform/pico/PicoDigitalInput.hpp"

#include "hardware/gpio.h"

namespace owlet::platform::pico
{

namespace
{

// Who to call for each pin. Written before that pin's interrupt is enabled
// and not changed while it is, so the handler reads it without a lock.
::owlet::hal::IEdgeListener* g_listeners[NUM_BANK0_GPIOS] = {};

void onGpioInterrupt(unsigned gpio, uint32_t /*events*/)
{
    if (gpio < NUM_BANK0_GPIOS && g_listeners[gpio] != nullptr)
    {
        g_listeners[gpio]->onEdge();
    }
}

} // namespace

PicoDigitalInput::PicoDigitalInput(unsigned pin, Polarity polarity, Pull pull)
    : m_pin(pin), m_activeLow(polarity == Polarity::ActiveLow)
{
    gpio_init(m_pin);
    gpio_set_dir(m_pin, GPIO_IN);
    gpio_set_pulls(m_pin, pull == Pull::Up, pull == Pull::Down);
}

void PicoDigitalInput::onActive(::owlet::hal::IEdgeListener& listener)
{
    if (m_pin >= NUM_BANK0_GPIOS)
    {
        return;
    }
    const uint32_t edge = m_activeLow ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE;
    gpio_set_irq_enabled(m_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
    g_listeners[m_pin] = &listener;
    gpio_set_irq_enabled_with_callback(m_pin, edge, true, &onGpioInterrupt);
}

bool PicoDigitalInput::active() const
{
    return gpio_get(m_pin) != m_activeLow;
}

} // namespace owlet::platform::pico
