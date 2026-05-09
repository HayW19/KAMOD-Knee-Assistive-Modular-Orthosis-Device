#include "rolling_baseline.h"

#include <math.h>

bool rolling_baseline_init(
    rolling_baseline_state_t *state,
    const rolling_baseline_config_t *config,
    float initial_baseline)
{
    if ((state == nullptr) || (config == nullptr))
    {
        return false;
    }

    state->initialized = true;
    state->baseline = initial_baseline;
    state->corrected_value = 0.0f;
    state->adapting = false;
    state->movement_detected = false;
    return true;
}

void rolling_baseline_step(
    rolling_baseline_state_t *state,
    const rolling_baseline_config_t *config,
    float filtered_value,
    bool saturated,
    bool movement_state_active)
{
    if ((state == nullptr) || (config == nullptr) || !state->initialized)
    {
        return;
    }

    const float delta = filtered_value - state->baseline;
    state->movement_detected = (fabsf(delta) >= config->movement_threshold) || movement_state_active;
    state->adapting = false;

    if (!saturated && !state->movement_detected)
    {
        state->baseline += config->alpha_slow * delta;
        state->adapting = true;
    }

    state->corrected_value = filtered_value - state->baseline;
}
