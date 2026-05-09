#ifndef FSR_CHANNEL_H
#define FSR_CHANNEL_H

#include <Arduino.h>

#include "adc_24bit_interface.h"
#include "digipot_mcp41xx_style.h"
#include "rolling_baseline.h"
#include "spi_bus.h"

typedef enum
{
    FSR_CHANNEL_STATUS_PRESENT = 1U << 0,
    FSR_CHANNEL_STATUS_CALIBRATED = 1U << 1,
    FSR_CHANNEL_STATUS_BASELINE_ADAPTING = 1U << 2,
    FSR_CHANNEL_STATUS_MOVEMENT_ACTIVE = 1U << 3,
    FSR_CHANNEL_STATUS_SATURATED_LOW = 1U << 4,
    FSR_CHANNEL_STATUS_SATURATED_HIGH = 1U << 5,
    FSR_CHANNEL_STATUS_CALIBRATION_FAILED = 1U << 6
} fsr_channel_status_flags_t;

typedef struct
{
    uint8_t channel_id;
    uint8_t adc_channel;
    uint8_t digipot_device_index;
    digipot_channel_t digipot_channel;
    uint16_t startup_sample_count;
    int32_t adc_target_min;
    int32_t adc_target_max;
    int32_t adc_saturation_low;
    int32_t adc_saturation_high;
    float filter_alpha;
} fsr_channel_config_t;

typedef struct
{
    bool initialized;
    bool calibrated;
    fsr_channel_config_t config;
    rolling_baseline_state_t rolling_state;
    int32_t channel_id;
    int32_t raw_adc;
    float voltage_or_counts;
    uint8_t digipot_tap;
    float baseline;
    float corrected_value;
    float filtered_value;
    uint16_t status_flags;
} fsr_channel_state_t;

bool fsr_channel_init(
    fsr_channel_state_t *state,
    const fsr_channel_config_t *config,
    const rolling_baseline_config_t *baseline_config);
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
    uint8_t tap_step);
bool fsr_channel_sample(
    fsr_channel_state_t *state,
    adc_24bit_interface_state_t *adc_state,
    const rolling_baseline_config_t *baseline_config,
    bool movement_state_active);

#endif /* FSR_CHANNEL_H */
