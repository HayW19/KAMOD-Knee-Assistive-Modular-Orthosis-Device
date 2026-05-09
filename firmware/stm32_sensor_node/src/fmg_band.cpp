#include "fmg_band.h"

#include <math.h>

bool fmg_band_init(
    fmg_band_state_t *state,
    const fmg_band_config_t *config)
{
    if ((state == nullptr) || (config == nullptr))
    {
        return false;
    }

    state->initialized = true;
    state->feature_sum = 0.0f;
    state->feature_mean = 0.0f;
    state->movement_active = false;
    return true;
}

void fmg_band_update(
    fmg_band_state_t *state,
    const fmg_band_config_t *config,
    const fsr_channel_state_t *channels,
    uint8_t channel_count)
{
    if ((state == nullptr) || (config == nullptr) || (channels == nullptr) || !state->initialized || (channel_count == 0U))
    {
        return;
    }

    float feature_sum = 0.0f;
    bool movement_active = false;

    for (uint8_t channel_index = 0U; channel_index < channel_count; ++channel_index)
    {
        feature_sum += channels[channel_index].corrected_value;
        if ((channels[channel_index].status_flags & FSR_CHANNEL_STATUS_MOVEMENT_ACTIVE) != 0U)
        {
            movement_active = true;
        }
        if (fabsf(channels[channel_index].corrected_value) >= config->active_threshold)
        {
            movement_active = true;
        }
    }

    state->feature_sum = feature_sum;
    state->feature_mean = feature_sum / static_cast<float>(channel_count);
    state->movement_active = movement_active;
}
