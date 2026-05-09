#include "spi_bus.h"

#include "pin_map.h"

bool spi_bus_init(spi_bus_state_t *state, const spi_bus_config_t *config)
{
    if ((state == nullptr) || (config == nullptr))
    {
        return false;
    }

    state->config = *config;
    SPI.begin();
    state->initialized = true;
    return true;
}

void spi_bus_select(int32_t cs_pin)
{
    if (cs_pin == KAMOD_PIN_DISABLED)
    {
        return;
    }

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, LOW);
}

void spi_bus_deselect(int32_t cs_pin)
{
    if (cs_pin == KAMOD_PIN_DISABLED)
    {
        return;
    }

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
}

void spi_bus_begin(spi_bus_state_t *state)
{
    if ((state == nullptr) || !state->initialized)
    {
        return;
    }

    SPI.beginTransaction(SPISettings(state->config.clock_hz, MSBFIRST, state->config.data_mode));
}

void spi_bus_end(void)
{
    SPI.endTransaction();
}

uint8_t spi_bus_transfer8(uint8_t value)
{
    return SPI.transfer(value);
}
