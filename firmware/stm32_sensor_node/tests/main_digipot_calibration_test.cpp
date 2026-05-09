#include <Arduino.h>

#include "../include/adc_24bit_interface.h"
#include "../include/digipot_mcp41xx_style.h"
#include "../include/fsr_channel.h"
#include "../include/kamod_sensor_config.h"
#include "../include/pin_map.h"
#include "../include/rolling_baseline.h"
#include "../include/sensor_logger.h"
#include "../include/spi_bus.h"

namespace
{
spi_bus_state_t g_spi_bus = {};
digipot_mcp41xx_style_state_t g_digipots[2] = {};
adc_24bit_interface_state_t g_adc = {};
rolling_baseline_config_t g_baseline_config = {};
fsr_channel_state_t g_channel = {};
bool g_done = false;
}

void setup()
{
    sensor_logger_begin(115200UL);
    const spi_bus_config_t spi_config = {
        KAMOD_PIN_SPI_SCK,
        KAMOD_PIN_SPI_MISO,
        KAMOD_PIN_SPI_MOSI,
        1000000UL,
        SPI_MODE0,
    };
    const digipot_mcp41xx_style_config_t digipot_0_config = {
        KAMOD_PIN_DIGIPOT_0_CS,
        KAMOD_DIGIPOT_NOMINAL_OHMS,
    };
    const adc_24bit_config_t adc_config = {
        KAMOD_PIN_ADC_0_CS,
        KAMOD_PIN_ADC_DRDY,
        KAMOD_PIN_ADC_RESET,
        KAMOD_PIN_ADC_SYNC,
        KAMOD_ADC_REFERENCE_VOLTAGE,
        1U,
    };
    const fsr_channel_config_t channel_config = {
        0U,
        0U,
        0U,
        DIGIPOT_CHANNEL_A,
        static_cast<uint16_t>(KAMOD_STARTUP_SAMPLE_COUNT),
        KAMOD_ADC_TARGET_MIN,
        KAMOD_ADC_TARGET_MAX,
        KAMOD_ADC_SATURATION_LOW,
        KAMOD_ADC_SATURATION_HIGH,
        KAMOD_FSR_FILTER_ALPHA,
    };

    g_baseline_config.alpha_slow = KAMOD_BASELINE_ALPHA_SLOW;
    g_baseline_config.alpha_fast = KAMOD_BASELINE_ALPHA_FAST;
    g_baseline_config.movement_threshold = KAMOD_MOVEMENT_THRESHOLD;

    spi_bus_init(&g_spi_bus, &spi_config);
    digipot_mcp41xx_style_init(&g_digipots[0], &g_spi_bus, &digipot_0_config);
    adc_24bit_interface_init(&g_adc, &g_spi_bus, &adc_config);
    adc_24bit_interface_set_mock_base(&g_adc, 0U, KAMOD_MOCK_ADC_BASE_COUNTS);
    fsr_channel_init(&g_channel, &channel_config, &g_baseline_config);
    sensor_logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if (g_done)
    {
        return;
    }

    g_done = true;
    const bool calibrated = fsr_channel_startup_calibrate(
        &g_channel,
        &g_spi_bus,
        g_digipots,
        1U,
        &g_adc,
        &g_baseline_config,
        millis(),
        KAMOD_CALIBRATION_TIMEOUT_MS,
        KAMOD_DIGIPOT_START_TAP,
        KAMOD_DIGIPOT_CALIBRATION_STEP);

    sensor_logger_log_calibration(
        millis(),
        0U,
        g_channel.digipot_tap,
        g_channel.raw_adc,
        calibrated);
}
