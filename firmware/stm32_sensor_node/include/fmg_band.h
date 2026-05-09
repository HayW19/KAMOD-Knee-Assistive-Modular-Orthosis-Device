#ifndef FMG_BAND_H
#define FMG_BAND_H

#include <Arduino.h>

#include "fsr_channel.h"

typedef struct
{
    uint8_t band_id;
    float active_threshold;
} fmg_band_config_t;

typedef struct
{
    bool initialized;
    float feature_sum;
    float feature_mean;
    bool movement_active;
} fmg_band_state_t;

bool fmg_band_init(
    fmg_band_state_t *state,
    const fmg_band_config_t *config);
void fmg_band_update(
    fmg_band_state_t *state,
    const fmg_band_config_t *config,
    const fsr_channel_state_t *channels,
    uint8_t channel_count);

#endif /* FMG_BAND_H */
