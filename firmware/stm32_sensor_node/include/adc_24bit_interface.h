#ifndef ADC_24BIT_INTERFACE_H
#define ADC_24BIT_INTERFACE_H

#include <Arduino.h>

#include "spi_bus.h"

typedef enum
{
    ADC_24BIT_BACKEND_UNKNOWN = 0,
    ADC_24BIT_BACKEND_GENERIC_SPI = 1,
    ADC_24BIT_BACKEND_MOCK = 2
} adc_24bit_backend_t;

typedef struct
{
    int32_t cs_pin;
    int32_t drdy_pin;
    int32_t reset_pin;
    int32_t sync_pin;
    float reference_voltage;
    uint8_t channel_count;
} adc_24bit_config_t;

typedef struct
{
    bool initialized;
    adc_24bit_backend_t backend;
    adc_24bit_config_t config;
    int32_t zero_offset[4];
    int32_t mock_base_counts[4];
    int32_t mock_manual_counts[4];
    bool mock_manual_valid[4];
    int32_t mock_tap_hint[4];
} adc_24bit_interface_state_t;

bool adc_24bit_interface_init(
    adc_24bit_interface_state_t *state,
    spi_bus_state_t *spi_bus,
    const adc_24bit_config_t *config);
bool adc_24bit_interface_is_ready(const adc_24bit_interface_state_t *state);
bool adc_24bit_interface_read_raw(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    int32_t *raw_out);
bool adc_24bit_interface_read_voltage(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    float *voltage_out);
bool adc_24bit_interface_tare_or_zero(
    adc_24bit_interface_state_t *state,
    uint8_t channel);
void adc_24bit_interface_set_mock_base(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    int32_t base_counts);
void adc_24bit_interface_set_mock_raw(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    int32_t raw_counts);
void adc_24bit_interface_clear_mock_raw(
    adc_24bit_interface_state_t *state,
    uint8_t channel);
void adc_24bit_interface_set_mock_tap_hint(
    adc_24bit_interface_state_t *state,
    uint8_t channel,
    uint8_t tap_value);

#endif /* ADC_24BIT_INTERFACE_H */
