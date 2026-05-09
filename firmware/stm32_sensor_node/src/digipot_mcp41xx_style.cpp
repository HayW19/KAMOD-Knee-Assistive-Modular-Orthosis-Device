#include "digipot_mcp41xx_style.h"

#include "kamod_sensor_config.h"

namespace
{
uint8_t select_command(digipot_channel_t channel)
{
    if (channel == DIGIPOT_CHANNEL_B)
    {
        return 0x12U;
    }

    return 0x11U;
}
}

bool digipot_mcp41xx_style_init(
    digipot_mcp41xx_style_state_t *state,
    spi_bus_state_t *spi_bus,
    const digipot_mcp41xx_style_config_t *config)
{
    (void) spi_bus;

    if ((state == nullptr) || (spi_bus == nullptr) || (config == nullptr))
    {
        return false;
    }

    pinMode(config->cs_pin, OUTPUT);
    digitalWrite(config->cs_pin, HIGH);

    state->config = *config;
    state->last_tap_a = KAMOD_DIGIPOT_START_TAP;
    state->last_tap_b = KAMOD_DIGIPOT_START_TAP;
    state->initialized = true;
    return true;
}

uint8_t digipot_mcp41xx_style_clamp_tap(uint16_t tap_value)
{
    if (tap_value > KAMOD_DIGIPOT_MAX_TAP)
    {
        return KAMOD_DIGIPOT_MAX_TAP;
    }

    return static_cast<uint8_t>(tap_value);
}

bool digipot_mcp41xx_style_write_tap(
    digipot_mcp41xx_style_state_t *state,
    spi_bus_state_t *spi_bus,
    digipot_channel_t channel,
    uint8_t tap_value)
{
    if ((state == nullptr) || (spi_bus == nullptr) || !state->initialized)
    {
        return false;
    }

    const uint8_t clamped_tap = digipot_mcp41xx_style_clamp_tap(tap_value);

    spi_bus_begin(spi_bus);
    spi_bus_select(state->config.cs_pin);
    spi_bus_transfer8(select_command(channel));
    spi_bus_transfer8(clamped_tap);
    spi_bus_deselect(state->config.cs_pin);
    spi_bus_end();

    if (channel == DIGIPOT_CHANNEL_B)
    {
        state->last_tap_b = clamped_tap;
    }
    else
    {
        state->last_tap_a = clamped_tap;
    }

    return true;
}

uint8_t digipot_mcp41xx_style_get_tap(
    const digipot_mcp41xx_style_state_t *state,
    digipot_channel_t channel)
{
    if (state == nullptr)
    {
        return KAMOD_DIGIPOT_START_TAP;
    }

    if (channel == DIGIPOT_CHANNEL_B)
    {
        return state->last_tap_b;
    }

    return state->last_tap_a;
}
