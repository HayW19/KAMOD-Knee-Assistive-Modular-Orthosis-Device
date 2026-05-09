#include "fsr_channel.h"

#include "kamod_sensor_config.h"

namespace
{
int32_t read_average(
    adc_24bit_interface_state_t *adc_state,
    uint8_t adc_channel,
    uint16_t sample_count)
{
    int64_t accumulator = 0;
    int32_t sample_value = 0L;

    for (uint16_t sample_index = 0U; sample_index < sample_count; ++sample_index)
    {
        if (!adc_24bit_interface_read_raw(adc_state, adc_channel, &sample_value))
        {
            return 0L;
        }
        accumulator += sample_value;
    }

    return static_cast<int32_t>(accumulator / static_cast<int64_t>(sample_count));
}

bool read_saturation(const fsr_channel_state_t *state)
{
    if (state == nullptr)
    {
        return true;
    }

    return (state->raw_adc <= state->config.adc_saturation_low) ||
           (state->raw_adc >= state->config.adc_saturation_high);
}
}

bool fsr_channel_init(
    fsr_channel_state_t *state,
    const fsr_channel_config_t *config,
    const rolling_baseline_config_t *baseline_config)
{
    if ((state == nullptr) || (config == nullptr) || (baseline_config == nullptr))
    {
        return false;
    }

    state->initialized = true;
    state->calibrated = false;
    state->config = *config;
    state->rolling_state = {};
    state->channel_id = config->channel_id;
    state->raw_adc = 0L;
    state->voltage_or_counts = 0.0f;
    state->digipot_tap = KAMOD_DIGIPOT_START_TAP;
    state->baseline = 0.0f;
    state->corrected_value = 0.0f;
    state->filtered_value = 0.0f;
    state->status_flags = 0U;
    return rolling_baseline_init(&state->rolling_state, baseline_config, 0.0f);
}

bool fsr_channel_startup_calibrate(
    fsr_channel_state_t *state,
    spi_bus_state_t *spi_bus,
    digipot_mcp41xx_style_state_t *digipots,
    uint8_t digipot_count,
    adc_24bit_interface_state_t *adc_state,
    const rolling_baseline_config_t *baseline_config,
    uint32_t now_ms,
    uint32_t timeout_ms,
    uint8_t start_tap,
    uint8_t tap_step)
{
    (void) now_ms;

    if ((state == nullptr) || (spi_bus == nullptr) || (digipots == nullptr) ||
        (adc_state == nullptr) || (baseline_config == nullptr) || !state->initialized)
    {
        return false;
    }

    if (state->config.digipot_device_index >= digipot_count)
    {
        return false;
    }

    digipot_mcp41xx_style_state_t *digipot = &digipots[state->config.digipot_device_index];
    uint8_t current_tap = digipot_mcp41xx_style_clamp_tap(start_tap);
    const uint32_t start_ms = millis();

    while ((millis() - start_ms) <= timeout_ms)
    {
        digipot_mcp41xx_style_write_tap(digipot, spi_bus, state->config.digipot_channel, current_tap);
        adc_24bit_interface_set_mock_tap_hint(adc_state, state->config.adc_channel, current_tap);

        const int32_t averaged_adc = read_average(
            adc_state,
            state->config.adc_channel,
            state->config.startup_sample_count);

        state->raw_adc = averaged_adc;
        state->voltage_or_counts = static_cast<float>(averaged_adc);
        state->digipot_tap = current_tap;

        if ((averaged_adc >= state->config.adc_target_min) &&
            (averaged_adc <= state->config.adc_target_max))
        {
            state->calibrated = true;
            state->status_flags = FSR_CHANNEL_STATUS_PRESENT | FSR_CHANNEL_STATUS_CALIBRATED;
            state->filtered_value = static_cast<float>(averaged_adc);
            state->baseline = static_cast<float>(averaged_adc);
            state->corrected_value = 0.0f;
            rolling_baseline_init(&state->rolling_state, baseline_config, state->baseline);
            return true;
        }

        if (averaged_adc < state->config.adc_target_min)
        {
            if (current_tap >= (KAMOD_DIGIPOT_MAX_TAP - tap_step))
            {
                break;
            }
            current_tap = static_cast<uint8_t>(current_tap + tap_step);
            continue;
        }

        if (current_tap <= tap_step)
        {
            break;
        }
        current_tap = static_cast<uint8_t>(current_tap - tap_step);
    }

    state->calibrated = false;
    state->status_flags = FSR_CHANNEL_STATUS_PRESENT | FSR_CHANNEL_STATUS_CALIBRATION_FAILED;
    return false;
}

bool fsr_channel_sample(
    fsr_channel_state_t *state,
    adc_24bit_interface_state_t *adc_state,
    const rolling_baseline_config_t *baseline_config,
    bool movement_state_active)
{
    if ((state == nullptr) || (adc_state == nullptr) || (baseline_config == nullptr) || !state->initialized)
    {
        return false;
    }

    int32_t raw_value = 0L;
    float voltage_value = 0.0f;

    if (!adc_24bit_interface_read_raw(adc_state, state->config.adc_channel, &raw_value))
    {
        return false;
    }

    adc_24bit_interface_read_voltage(adc_state, state->config.adc_channel, &voltage_value);

    state->raw_adc = raw_value;
    state->voltage_or_counts = voltage_value;

    if (!state->calibrated)
    {
        state->filtered_value = static_cast<float>(raw_value);
    }
    else
    {
        state->filtered_value += state->config.filter_alpha * (static_cast<float>(raw_value) - state->filtered_value);
    }

    const bool saturated = read_saturation(state);
    rolling_baseline_step(
        &state->rolling_state,
        baseline_config,
        state->filtered_value,
        saturated,
        movement_state_active);

    state->baseline = state->rolling_state.baseline;
    state->corrected_value = state->rolling_state.corrected_value;
    state->status_flags = FSR_CHANNEL_STATUS_PRESENT;

    if (state->calibrated)
    {
        state->status_flags |= FSR_CHANNEL_STATUS_CALIBRATED;
    }

    if (state->rolling_state.adapting)
    {
        state->status_flags |= FSR_CHANNEL_STATUS_BASELINE_ADAPTING;
    }

    if (state->rolling_state.movement_detected)
    {
        state->status_flags |= FSR_CHANNEL_STATUS_MOVEMENT_ACTIVE;
    }

    if (state->raw_adc <= state->config.adc_saturation_low)
    {
        state->status_flags |= FSR_CHANNEL_STATUS_SATURATED_LOW;
    }

    if (state->raw_adc >= state->config.adc_saturation_high)
    {
        state->status_flags |= FSR_CHANNEL_STATUS_SATURATED_HIGH;
    }

    if (!state->calibrated)
    {
        state->status_flags |= FSR_CHANNEL_STATUS_CALIBRATION_FAILED;
    }

    return true;
}
