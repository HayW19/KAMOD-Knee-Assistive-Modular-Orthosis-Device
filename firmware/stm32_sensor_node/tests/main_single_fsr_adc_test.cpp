#include <Arduino.h>

#include "../include/adc_24bit_interface.h"
#include "../include/kamod_sensor_config.h"
#include "../include/pin_map.h"
#include "../include/sensor_logger.h"
#include "../include/spi_bus.h"

namespace
{
spi_bus_state_t g_spi_bus = {};
adc_24bit_interface_state_t g_adc = {};
uint32_t g_last_log_ms = 0U;
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
    const adc_24bit_config_t adc_config = {
        KAMOD_PIN_ADC_0_CS,
        KAMOD_PIN_ADC_DRDY,
        KAMOD_PIN_ADC_RESET,
        KAMOD_PIN_ADC_SYNC,
        KAMOD_ADC_REFERENCE_VOLTAGE,
        1U,
    };
    spi_bus_init(&g_spi_bus, &spi_config);
    adc_24bit_interface_init(&g_adc, &g_spi_bus, &adc_config);
    adc_24bit_interface_set_mock_base(&g_adc, 0U, KAMOD_MOCK_ADC_BASE_COUNTS);
    sensor_logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if ((millis() - g_last_log_ms) < 100U)
    {
        return;
    }

    g_last_log_ms = millis();

    int32_t raw_adc = 0L;
    float voltage = 0.0f;
    adc_24bit_interface_read_raw(&g_adc, 0U, &raw_adc);
    adc_24bit_interface_read_voltage(&g_adc, 0U, &voltage);

    Serial.print("raw_adc,");
    Serial.print(millis());
    Serial.print(",");
    Serial.println(raw_adc);

    Serial.print("voltage,");
    Serial.print(millis());
    Serial.print(",");
    Serial.println(voltage, 6);
}
