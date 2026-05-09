#include "adc_24bit_interface.h"

#include "kamod_sensor_config.h"
#include "pin_map.h"

namespace
{
bool channel_valid(const adc_24bit_interface_state_t *state, uint8_t channel)
{
    return (state != nullptr) && (channel < state->config.channel_count) && (channel < 4U);
}

int32_t mock_counts_from_state(const adc_24bit_interface_state_t *state, uint8_t channel)
{
    if (!channel_valid(state, channel))
    {
        return 0L;
    }

    if (state->mock_manual_valid[channel])
    {
        return state->mock_manual_counts[channel] - state->zero_offset[channel];
    }

    const int32_t tap_delta = state->mock_tap_hint[channel] - static_cast<int32_t>(KAMOD_DIGIPOT_START_TAP);
    const int32_t value = state->mock_base_counts[channel] + (tap_delta * KAMOD_MOCK_ADC_TAP_GAIN_COUNTS);
    return value - state->zero_offset[channel];
}
}

bool adc_24bit_interface_init(
    adc_24bit_interface_state_t *state,
    spi_bus_state_t *spi_bus,
    const adc_24bit_config_t *config)
{
    (void) spi_bus;

    if ((state == nullptr) || (spi_bus == nullptr) || (config == nullptr))
    {
        return false;
    }

    pinMode(config->cs_pin, OUTPUT);
    digitalWrite(config->cs_pin, HIGH);

    if (config->drdy_pin != KAMOD_PIN_DISABLED)
    {
        pinMode(config->drdy_pin, INPUT_PULLUP);
    }

    if (config->reset_pin != KAMOD_PIN_DISABLED)
    {
        pinMode(config->reset_pin, OUTPUT);
        digitalWrite(config->reset_pin, HIGH);
    }

    if (config->sync_pin != KAMOD_PIN_DISABLED)
    {
        pinMode(config->sync_pin, OUTPUT);
        digitalWrite(config->sync_pin, HIGH);
    }

    state->config = *config;
    state->initialized = true;
#if KAMOD_USE_MOCK_ADC
    state->backend = ADC_24BIT_BACKEND_MOCK;
#else
    state->backend = ADC_24BIT_BACKEND_GENERIC_SPI;
#endif

    for (uint8_t channel = 0U; channel < 4U; ++channel)
    {
        state->zero_offset[channel] = 0L;
        state->mock_base_counts[channel] = KAMOD_MOCK_ADC_BASE_COUNTS;
        state->mock_manual_counts[channel] = KAMOD_MOCK_ADC_BASE_COUNTS;
        state->mock_manual_valid[channel] = false;
        state->mock_tap_hint[channel] = KAMOD_DIGIPOT_START_TAP;
    }

    return true;
}

bool adc_24bit_interface_is_ready(const adc_24bit_interface_state_t *state)
{
    if ((state == nullptr) || !state->initialized)
    {
        return false;
    }

    if (state->backend == ADC_24BIT_BACKEND_MOCK)
    {
        return true;
    }

    if (state->config.drdy_pin == KAMOD_PIN_DISABLED)
    {
        return false;
    }

    return (digitalRead(state->config.drdy_pin) == LOW);
}

bool adc_24bit_interface_read_raw(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    int32_t *raw_out)
{
    if ((state == nullptr) || (raw_out == nullptr) || !channel_valid(state, channel) || !state->initialized)
    {
        return false;
    }

    if (state->backend == ADC_24BIT_BACKEND_MOCK)
    {
        *raw_out = mock_counts_from_state(state, channel);
        return true;
    }

    // todo: bind exact adc part
    *raw_out = 0L;
    return false;
}

bool adc_24bit_interface_read_voltage(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    float *voltage_out)
{
    if ((state == nullptr) || (voltage_out == nullptr))
    {
        return false;
    }

    int32_t raw_value = 0L;
    if (!adc_24bit_interface_read_raw(state, channel, &raw_value))
    {
        return false;
    }

    *voltage_out = (static_cast<float>(raw_value) / 8388607.0f) * state->config.reference_voltage;
    return true;
}

bool adc_24bit_interface_tare_or_zero(
    adc_24bit_interface_state_t *state,
    uint8_t channel)
{
    if (!channel_valid(state, channel))
    {
        return false;
    }

    int32_t raw_value = 0L;
    if (!adc_24bit_interface_read_raw(state, channel, &raw_value))
    {
        return false;
    }

    state->zero_offset[channel] = raw_value;
    return true;
}

void adc_24bit_interface_set_mock_base(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    int32_t base_counts)
{
    if (!channel_valid(state, channel))
    {
        return;
    }

    state->mock_base_counts[channel] = base_counts;
}

void adc_24bit_interface_set_mock_raw(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    int32_t raw_counts)
{
    if (!channel_valid(state, channel))
    {
        return;
    }

    state->mock_manual_counts[channel] = raw_counts;
    state->mock_manual_valid[channel] = true;
}

void adc_24bit_interface_clear_mock_raw(
    adc_24bit_interface_state_t *state,
    uint8_t channel)
{
    if (!channel_valid(state, channel))
    {
        return;
    }

    state->mock_manual_valid[channel] = false;
}

void adc_24bit_interface_set_mock_tap_hint(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    uint8_t tap_value)
{
    if (!channel_valid(state, channel))
    {
        return;
    }

    state->mock_tap_hint[channel] = tap_value;
}
