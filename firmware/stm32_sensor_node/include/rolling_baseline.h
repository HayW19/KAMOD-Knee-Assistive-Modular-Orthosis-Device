#ifndef ROLLING_BASELINE_H
#define ROLLING_BASELINE_H

#include <Arduino.h>

typedef struct
{
    float alpha_slow;
    float alpha_fast;
    float movement_threshold;
} rolling_baseline_config_t;

typedef struct
{
    bool initialized;
    float baseline;
    float corrected_value;
    bool adapting;
    bool movement_detected;
} rolling_baseline_state_t;

bool rolling_baseline_init(
    rolling_baseline_state_t *state,
    const rolling_baseline_config_t *config,
    float initial_baseline);
void rolling_baseline_step(
    rolling_baseline_state_t *state,
    const rolling_baseline_config_t *config,
    float filtered_value,
    bool saturated,
    bool movement_state_active);

#endif /* ROLLING_BASELINE_H */
